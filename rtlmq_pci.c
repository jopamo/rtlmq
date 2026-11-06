// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/filter.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/rcupdate.h>
#include <linux/reboot.h>
#include <linux/rtnetlink.h>
#include <linux/sched.h>

#include "rtlmq_chip.h"
#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq_rx.h"
#include "rtlmq_xdp.h"

/* K1 rtl_csi_read/write: 100 completion checks, 10 us between checks. */
static int rtlmq_csi_wait(struct rtlmq_private *tp, bool read)
{
	unsigned int i;
	u32 value;

	for (i = 0; i < 100; i++) {
		if (!rtlmq_mmio_ordinary_authorized(tp))
			return -ENODEV;
		value = readl(tp->mmio_addr + RTLMQ_CSIAR);
		if (value == U32_MAX)
			return -ENODEV;
		if (!!(value & RTLMQ_CSIAR_FLAG) == read)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

static int rtlmq_pci_config_read(struct rtlmq_private *tp, u16 reg, u32 *value)
{
	int ret;

	if (tp->pdev->bus && tp->pdev->cfg_size >= reg + sizeof(*value) &&
	    pci_read_config_dword(tp->pdev, reg, value) == PCIBIOS_SUCCESSFUL)
		return *value == U32_MAX ? -ENODEV : 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	writel(reg | RTLMQ_CSIAR_BYTE_ENABLE | PCI_FUNC(tp->pdev->devfn) << 16,
	       tp->mmio_addr + RTLMQ_CSIAR);
	ret = rtlmq_csi_wait(tp, true);
	if (ret)
		return ret;
	*value = readl(tp->mmio_addr + RTLMQ_CSIDR);
	return *value == U32_MAX ? -ENODEV : 0;
}

static int rtlmq_pci_config_write(struct rtlmq_private *tp, u16 reg, u32 value)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (tp->pdev->bus && tp->pdev->cfg_size >= reg + sizeof(value) &&
	    pci_write_config_dword(tp->pdev, reg, value) == PCIBIOS_SUCCESSFUL)
		return 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	writel(value, tp->mmio_addr + RTLMQ_CSIDR);
	writel(reg | RTLMQ_CSIAR_FLAG | RTLMQ_CSIAR_BYTE_ENABLE |
	       PCI_FUNC(tp->pdev->devfn) << 16, tp->mmio_addr + RTLMQ_CSIAR);
	return rtlmq_csi_wait(tp, false);
}

int rtlmq_pci_config_modify(struct rtlmq_private *tp, u32 reg, u32 mask,
			    u32 set)
{
	u32 value;
	int ret;

	/* Only aligned extended configuration DWORDs use this cold RMW owner. */
	if (reg < PCI_CFG_SPACE_SIZE || reg > PCI_CFG_SPACE_EXP_SIZE - sizeof(u32) ||
	    !IS_ALIGNED(reg, sizeof(u32)) || (set & ~mask))
		return -EINVAL;
	if (!tp || !tp->pdev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	lockdep_assert_held(&tp->control_lock);
	/* control_lock serializes the whole RMW, including the CSI mailbox. */
	ret = rtlmq_pci_config_read(tp, reg, &value);
	if (ret)
		return ret;
	return rtlmq_pci_config_write(tp, reg, (value & ~mask) | set);
}

static const struct pci_device_id rtlmq_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8125) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8126) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x5000) },
	{ 0, }
};

/*
 * PCI IDs alone do not establish a supported method. Probe checks the exact
 * XID and config method before allowing either family to program hardware.
 */
MODULE_DEVICE_TABLE(pci, rtlmq_pci_ids);

int rtlmq_pci_enable_bus_master(struct rtlmq_private *tp)
{
	int revoke_ret;
	u16 command;
	int ret;

	if (!tp || !tp->pdev || !tp->pdev->bus)
		return -ENODEV;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Validate the chip profile before enabling bus mastering. Rollback
	 * must disable it before freeing any candidate DMA storage.
	 */
	if (!rtlmq_chip_profile_is_authoritative(tp))
		return -ENODEV;

	/* Admission may take effect even if the confirmation read fails. */
	WRITE_ONCE(tp->dma_quiesced, false);
	WRITE_ONCE(tp->quiesce_phase,
		   READ_ONCE(tp->quiesce_phase) &
			   ~(RTLMQ_QUIESCE_HARDWARE_STOPPED |
			     RTLMQ_QUIESCE_DMA_REVOKED));
	pci_set_master(tp->pdev);
	ret = pci_read_config_word(tp->pdev, PCI_COMMAND, &command);
	if (ret != PCIBIOS_SUCCESSFUL || command == U16_MAX ||
	    !(command & PCI_COMMAND_MASTER)) {
		revoke_ret = rtlmq_pci_disable_bus_master(tp, true);
		pci_err(tp->pdev,
			"rtlmq: failed to confirm PCI bus-master admission command=0x%04x status=%d revoke=%d\n",
			ret == PCIBIOS_SUCCESSFUL ? command : 0, ret,
			revoke_ret);
		return revoke_ret ? revoke_ret : -EIO;
	}

	return 0;
}

static int rtlmq_pci_read_command_checked(struct pci_dev *pdev, u16 *command)
{
	int ret;

	ret = pci_read_config_word(pdev, PCI_COMMAND, command);
	if (ret != PCIBIOS_SUCCESSFUL)
		return pcibios_err_to_errno(ret);
	if (*command == U16_MAX)
		return -ENODEV;

	return 0;
}

static int rtlmq_pci_clear_master_checked(struct pci_dev *pdev)
{
	u16 command;
	int ret;

	/* pci_clear_master() discards config-access errors. */
	ret = rtlmq_pci_read_command_checked(pdev, &command);
	if (ret)
		return ret;

	if (command & PCI_COMMAND_MASTER) {
		ret = pci_write_config_word(pdev, PCI_COMMAND,
					    command & ~PCI_COMMAND_MASTER);
		if (ret != PCIBIOS_SUCCESSFUL)
			return pcibios_err_to_errno(ret);
		ret = rtlmq_pci_read_command_checked(pdev, &command);
		if (ret)
			return ret;
		if (command & PCI_COMMAND_MASTER)
			return -EIO;
	}

	pdev->is_busmaster = 0;
	return 0;
}

static int rtlmq_pci_wait_pending_checked(struct pci_dev *pdev)
{
	u16 status;
	int ret;
	int i;

	if (!pci_is_pcie(pdev))
		return -EOPNOTSUPP;

	/*
	 * Keep the PCI core's 100/200/400 ms retry delays, but propagate config
	 * errors. TRPND covers non-posted requests, not outstanding DMA writes.
	 */
	for (i = 0; i < 4; i++) {
		if (i)
			msleep((1 << (i - 1)) * 100);
		ret = pci_read_config_word(pdev, pci_pcie_cap(pdev) + PCI_EXP_DEVSTA, &status);
		if (ret != PCIBIOS_SUCCESSFUL)
			return pcibios_err_to_errno(ret);
		if (status == U16_MAX)
			return -ENODEV;
		if (!(status & PCI_EXP_DEVSTA_TRPND))
			return 0;
	}

	return -ETIMEDOUT;
}

int rtlmq_pci_disable_bus_master(struct rtlmq_private *tp,
				 bool wait_for_pending)
{
	bool was_quiesced;
	int ret = -ENODEV;

	if (!tp)
		return -EINVAL;
	was_quiesced = READ_ONCE(tp->dma_quiesced);
	if (!was_quiesced)
		WRITE_ONCE(tp->quiesce_phase,
			   READ_ONCE(tp->quiesce_phase) &
				   ~RTLMQ_QUIESCE_DMA_REVOKED);

	if (tp->pdev && tp->pdev->bus) {
		ret = rtlmq_pci_clear_master_checked(tp->pdev);
		if (!ret && wait_for_pending)
			ret = rtlmq_pci_wait_pending_checked(tp->pdev);
		if (ret == -ETIMEDOUT)
			pci_warn(tp->pdev,
				 "rtlmq: timed out waiting for pending DMA transaction\n");
	}

	/*
	 * Revocation cannot admit DMA, so a failed recheck does not invalidate
	 * prior retirement or the proof that storage was never exposed.
	 * New shutdown proof still needs both a MAC stop and PCI revocation.
	 * Return the PCI error even when prior retirement remains valid.
	 */
	WRITE_ONCE(tp->dma_quiesced,
		   was_quiesced ||
			   (!ret && (READ_ONCE(tp->quiesce_phase) &
				     RTLMQ_QUIESCE_HARDWARE_STOPPED)));
	if (!ret)
		rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_DMA_REVOKED);

	return ret;
}

static int rtlmq_pci_validate_mmio_bar(struct pci_dev *pdev, int region)
{
	unsigned long flags;
	resource_size_t start;
	resource_size_t end;
	resource_size_t len;

	if (!pdev || region < 0 || region >= PCI_NUM_RESOURCES)
		return -EINVAL;

	flags = pci_resource_flags(pdev, region);
	if (!(flags & IORESOURCE_MEM))
		return -ENODEV;

	start = pci_resource_start(pdev, region);
	end = pci_resource_end(pdev, region);
	if (start > end)
		return -ENODEV;

	len = pci_resource_len(pdev, region);
	if (len < RTLMQ_MMIO_REQUIRED_SIZE)
		return -ENODEV;

	return 0;
}

static int rtlmq_pci_select_mmio_region(const struct pci_dev *pdev,
					unsigned long bars)
{
	if (!pdev || !bars)
		return -ENODEV;

	/* sbwml r8127 11.016.00 requires BAR2, even if BAR0 is memory. */
	if (pdev->vendor == PCI_VENDOR_ID_REALTEK && pdev->device == 0x8127)
		return bars & BIT(2) ? 2 : -ENODEV;

	return ffs(bars) - 1;
}

static bool rtlmq_pci_disable_unsafe_link_states(struct rtlmq_private *tp)
{
	struct pci_dev *pdev = tp->pdev;
	int ret;

	if (!pdev || !pci_is_pcie(pdev) ||
	    !rtlmq_chip_needs_conservative_aspm(tp))
		return false;

	ret = pci_disable_link_state(pdev, PCIE_LINK_STATE_L1);
	if (ret)
		pci_warn(pdev,
			 "rtlmq: failed to disable PCIe L1 state: %d\n",
			 ret);

	return !ret;
}

static void rtlmq_log_probe_state(struct rtlmq_private *tp, bool l1_disabled)
{
	const char *config_method = "unknown";
	const char *mac_name = "unknown";
	struct pci_dev *pdev = tp->pdev;
	bool clkpm_enabled = false;
	bool ltr_enabled = false;
	u16 max_rx_queues = 0;
	u16 max_tx_queues = 0;
	u32 aspm_support = 0;
	u32 aspm_enabled = 0;
	const char *fw_name;
	u16 devctl2 = 0;
	u32 lnkcap = 0;
	u16 lnkctl = 0;

	if (!pdev)
		return;

	fw_name = rtlmq_firmware_name(tp);
	if (!fw_name)
		fw_name = "none";
	if (tp->profile)
		mac_name = rtlmq_chip_profile_name(tp->profile);
	config_method = rtlmq_config_method_name(tp->config_method);
	max_rx_queues = rtlmq_advertised_max_rx_queues(tp);
	max_tx_queues = rtlmq_advertised_max_tx_queues(tp);

	pci_info(pdev,
		 "rtlmq: profile=%s pci=%04x:%04x xid=0x%x cfg_method=%s rev=%02x queue_ceiling=%urx/%utx fw=%s subsystem=%04x:%04x\n",
		 mac_name, pdev->vendor, pdev->device, tp->xid,
		 config_method, pdev->revision, max_rx_queues, max_tx_queues,
		 fw_name, pdev->subsystem_vendor, pdev->subsystem_device);

	if (pci_is_pcie(pdev)) {
		pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &lnkcap);
		pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &lnkctl);
		pcie_capability_read_word(pdev, PCI_EXP_DEVCTL2, &devctl2);

		aspm_support = (lnkcap & PCI_EXP_LNKCAP_ASPMS) >> 10;
		aspm_enabled = lnkctl & PCI_EXP_LNKCTL_ASPMC;
		clkpm_enabled = !!(lnkctl & PCI_EXP_LNKCTL_CLKREQ_EN);
		ltr_enabled = !!(devctl2 & PCI_EXP_DEVCTL2_LTR_EN);
	}

	pci_info(pdev,
		 "rtlmq: pcie aspm_support=0x%x aspm_enabled=0x%x clkpm=%u ltr=%u l1_disabled=%u\n",
		 aspm_support, aspm_enabled, clkpm_enabled, ltr_enabled,
		 l1_disabled);
}

static void rtlmq_drain_control_work(struct rtlmq_private *tp)
{
	lockdep_assert_not_held(&tp->control_lock);

	/* Callers must also drop RTNL before waiting for either callback. */
	cancel_delayed_work_sync(&tp->reset_work);
	cancel_work_sync(&tp->link_work);
}

static int
rtlmq_finish_platform_quiesce(struct rtlmq_private *tp,
			      enum rtlmq_platform_quiesce_owner owner,
			      bool prepare_wol, bool restore_mmio)
{
	bool owner_matches = false;
	int ret = 0;

	if (!tp)
		return -EINVAL;

	rtnl_lock();
	mutex_lock(&tp->control_lock);
	if (tp->state == RTLMQ_STATE_DEAD) {
		ret = -ENODEV;
		goto out_close_unlock;
	}
	if (tp->platform_quiesce_owner != owner) {
		ret = -EINVAL;
		goto out_close_unlock;
	}
	owner_matches = true;
	if (restore_mmio) {
		ret = rtlmq_grant_ordinary_mmio_locked(tp);
		if (ret)
			goto out_close_unlock;
	} else if (!rtlmq_mmio_ordinary_authorized(tp)) {
		ret = -ENODEV;
		goto out_close_unlock;
	}
	WRITE_ONCE(tp->platform_access_task, current);
	ret = rtlmq_detach_and_close_locked(tp, prepare_wol);
	WRITE_ONCE(tp->platform_access_task, NULL);
	if (!ret && owner == RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM &&
	    !prepare_wol)
		ret = rtlmq_prepare_no_wake_suspend(tp);
	if (!ret) {
		WARN_ON_ONCE(rtlmq_published_runtime_config(tp));
		WARN_ON_ONCE(READ_ONCE(tp->state) != RTLMQ_STATE_PROBED);
	}
out_close_unlock:
	/*
	 * PM will change power state after this callback, while PCI recovery
	 * does not rebuild until ->resume(). Keep normal MMIO blocked across
	 * both intervals, including failed close.
	 */
	if (owner_matches)
		rtlmq_revoke_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
	/*
	 * close_device() stops PHY/IRQ callbacks under control_lock. Drain any
	 * final link_work, which takes RTNL, before changing device access again.
	 */
	cancel_work_sync(&tp->link_work);
	WRITE_ONCE(tp->link_work_epoch, 0);

	return ret;
}

static void rtlmq_recover_failed_system_suspend(struct rtlmq_private *tp,
						int suspend_ret);

static int
rtlmq_quiesce_and_close_for_pm_or_pci(struct rtlmq_private *tp,
				      enum rtlmq_platform_quiesce_owner owner,
				      bool prepare_wol, bool frozen_channel)
{
	int dma_revoke_ret = 0;
	int ret = 0;

	if (!tp)
		return -EINVAL;

	rtnl_lock();
	mutex_lock(&tp->control_lock);
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE) {
		ret = -EBUSY;
		goto out_unlock;
	}
	ret = rtlmq_capture_platform_recovery_committed_locked(tp);
	if (ret)
		goto out_unlock;
	tp->platform_quiesce_owner = owner;
	tp->platform_quiesce_was_running = netif_running(tp->ndev);
	rtlmq_serial_tx_pause(tp);
	if (frozen_channel) {
		/*
		 * Drain TX producers that may hold cached doorbells before
		 * blocking MMIO. AER already made registers unsafe; stop PTP
		 * work without issuing hardware commands.
		 */
		netif_tx_disable(tp->ndev);
		rtlmq_ptp_freeze_locked(tp);
		rtlmq_revoke_mmio_locked(tp);
	}
	rtlmq_revoke_platform_datapath_locked(tp);
	if (frozen_channel) {
		/*
		 * The PCI channel is frozen: revoke the command bit without
		 * another config-space pending-transaction probe. Rely on PCI
		 * error recovery to quiesce the frozen channel.
		 */
		dma_revoke_ret = rtlmq_pci_disable_bus_master(tp, false);
	}
out_unlock:
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
	if (ret)
		return ret;

	/*
	 * Publication withdrawal does not establish DMA quiescence. Drop
	 * lifecycle locks before draining workers that may acquire them.
	 */
	rtlmq_drain_control_work(tp);
	WRITE_ONCE(tp->link_work_epoch, 0);

	if (dma_revoke_ret) {
		netdev_err(tp->ndev,
			   "platform quiesce retained resources after ambiguous DMA revoke: %d\n",
			   dma_revoke_ret);
		return dma_revoke_ret;
	}

	if (!frozen_channel) {
		ret = rtlmq_finish_platform_quiesce(tp, owner, prepare_wol, false);
		/*
		 * pm_runtime_force_suspend() does not call resume on failure.
		 * Only the invocation that acquired this owner may roll it back.
		 */
		if (ret && owner == RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM)
			rtlmq_recover_failed_system_suspend(tp, ret);
		return ret;
	}

	/*
	 * Do not enter ordinary close while MMIO is frozen. Remove only host
	 * IRQ/NAPI callbacks; slot_reset() finishes hardware and
	 * ring teardown after PCI core restores MMIO access.
	 */
	rtnl_lock();
	mutex_lock(&tp->control_lock);
	if (tp->platform_quiesce_owner != owner) {
		ret = -EINVAL;
		goto out_frozen_unlock;
	}
	ret = rtlmq_quiesce_irqs_frozen(tp);
	WARN_ON_ONCE(rtlmq_published_runtime_config(tp));
out_frozen_unlock:
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();

	return ret;
}

static int rtlmq_reopen_or_republish_locked(struct rtlmq_private *tp, u32 reopen_reasons)
{
	lockdep_assert_held(&tp->control_lock);

	return rtlmq_apply_recovery_transaction_locked(tp, reopen_reasons, NULL, false);
}

static int rtlmq_reopen_or_republish_for_pm_or_pci(struct rtlmq_private *tp,
						   enum rtlmq_platform_quiesce_owner owner,
						   u32 reopen_reasons,
						   u32 failure_reasons,
						   const char *failure_err,
						   bool request_reset_if_running)
{
	bool mmio_granted = false;
	int ret;

	if (!tp)
		return -EINVAL;

	rtnl_lock();
	mutex_lock(&tp->control_lock);
	if (tp->state == RTLMQ_STATE_DEAD) {
		ret = -ENODEV;
		goto err_detach;
	}
	if (tp->platform_quiesce_owner != owner ||
	    tp->platform_quiesce_was_running != netif_running(tp->ndev)) {
		ret = -EINVAL;
		goto err_detach;
	}
	ret = rtlmq_grant_ordinary_mmio_locked(tp);
	if (ret)
		goto err_detach;
	mmio_granted = true;
	if (!tp->platform_recovery_committed_valid) {
		ret = -EINVAL;
		goto err_detach;
	}
	ret = rtlmq_restore_platform_recovery_committed_locked(tp);
	if (ret)
		goto out_unlock;
	WRITE_ONCE(tp->platform_access_task, current);
	ret = rtlmq_reopen_or_republish_locked(tp, reopen_reasons);
	WRITE_ONCE(tp->platform_access_task, NULL);
	if (!ret) {
		tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_NONE;
		tp->platform_quiesce_was_running = false;
		rtlmq_clear_platform_recovery_committed_locked(tp);
		if (owner == RTLMQ_PLATFORM_QUIESCE_PCI_ERROR &&
		    netif_running(tp->ndev)) {
			rtlmq_start_phy(tp);
			rtlmq_sync_link_state(tp);
		}
	} else {
		rtlmq_revoke_mmio_locked(tp);
		mmio_granted = false;
		if (request_reset_if_running && netif_running(tp->ndev))
			rtlmq_request_reset(tp, failure_reasons);
		netif_device_detach(tp->ndev);
		netdev_err(tp->ndev, failure_err, ret);
	}
	goto out_unlock;

err_detach:
	netif_device_detach(tp->ndev);
out_unlock:
	if (ret && mmio_granted)
		rtlmq_revoke_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();

	return ret;
}

static void rtlmq_recover_failed_system_suspend(struct rtlmq_private *tp,
						int suspend_ret)
{
	int ret;

	ret = rtlmq_reopen_or_republish_for_pm_or_pci(tp,
						      RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM, 0,
						      0,
						      "suspend rollback failed: %d\n",
						      false);
	if (ret)
		netdev_err(tp->ndev,
			   "suspend failed: %d; rollback failed: %d; device remains unavailable\n",
			   suspend_ret, ret);
}

static bool
rtlmq_runtime_pm_idle_eligible_locked(const struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	return READ_ONCE(tp->runtime_pm_ready) &&
	       !tp->runtime_pm_suspended &&
	       !READ_ONCE(tp->runtime_pm_netdev_ref) &&
	       tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE &&
	       tp->state == RTLMQ_STATE_PROBED &&
	       !netif_running(tp->ndev) &&
	       rtlmq_quiesce_contract_satisfied(tp);
}

int rtlmq_runtime_pm_netdev_open(struct rtlmq_private *tp)
{
	struct device *device;
	int ret;

	if (!tp || !tp->pdev)
		return -ENODEV;
	if (!READ_ONCE(tp->runtime_pm_ready))
		return 0;

	device = &tp->pdev->dev;
	/*
	 * dev_open() sets __LINK_STATE_START before ndo_open(). Set our matching
	 * flag before resume so the runtime callback recognizes this open.
	 * On failure clear the flag without dropping an unacquired PM reference.
	 */
	if (xchg(&tp->runtime_pm_netdev_ref, true))
		return -EALREADY;
	ret = pm_runtime_resume_and_get(device);
	if (ret < 0) {
		WRITE_ONCE(tp->runtime_pm_netdev_ref, false);
		return ret;
	}
	return 0;
}

void rtlmq_runtime_pm_netdev_close(struct rtlmq_private *tp)
{
	struct device *device;

	if (!tp || !tp->pdev ||
	    !xchg(&tp->runtime_pm_netdev_ref, false))
		return;

	device = &tp->pdev->dev;
	pm_runtime_mark_last_busy(device);
	pm_runtime_put_noidle(device);
	if (READ_ONCE(tp->runtime_pm_ready))
		pm_runtime_idle(device);
}

int rtlmq_runtime_pm_access_get(struct rtlmq_private *tp)
{
	enum rtlmq_platform_quiesce_owner owner;
	int ret;

	if (!tp || !tp->pdev)
		return -ENODEV;
	if (!READ_ONCE(tp->runtime_pm_ready))
		return 0;

	owner = READ_ONCE(tp->platform_quiesce_owner);
	/* PHY callbacks run before PCI recovery relinquishes its owner. */
	if (owner == RTLMQ_PLATFORM_QUIESCE_PCI_ERROR &&
	    READ_ONCE(tp->platform_access_task) == current &&
	    rtlmq_mmio_ordinary_authorized(tp))
		return 0;
	if (owner == RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM &&
	    READ_ONCE(tp->system_sleep_transition))
		return 0;
	if (owner != RTLMQ_PLATFORM_QUIESCE_NONE)
		return -EBUSY;

	ret = pm_runtime_resume_and_get(&tp->pdev->dev);
	return ret < 0 ? ret : 1;
}

void rtlmq_runtime_pm_access_put(struct rtlmq_private *tp, int token)
{
	if (!tp || !tp->pdev || token <= 0)
		return;

	pm_runtime_mark_last_busy(&tp->pdev->dev);
	pm_runtime_put_autosuspend(&tp->pdev->dev);
}

static int rtlmq_runtime_suspend_down(struct rtlmq_private *tp)
{
	int ret = 0;

	/* An RTNL-held open/control caller may be waiting for this callback. */
	if (!rtnl_trylock())
		return -EBUSY;
	mutex_lock(&tp->control_lock);
	if (!rtlmq_runtime_pm_idle_eligible_locked(tp)) {
		ret = -EBUSY;
		goto out_unlock;
	}
	if (device_may_wakeup(&tp->pdev->dev))
		ret = rtlmq_prepare_wol_suspend(tp);
	else
		ret = rtlmq_prepare_no_wake_suspend(tp);
	if (ret)
		goto out_unlock;
	tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_RUNTIME_PM;
	tp->platform_quiesce_was_running = false;
	rtlmq_revoke_platform_datapath_locked(tp);
	/*
	 * Keep the down netdev present during runtime sleep. Detaching it would
	 * reject control operations before their NDO could acquire a PM reference.
	 * PM references resume hardware before control access; MMIO authority
	 * remains revoked while suspended.
	 */
	netif_device_attach(tp->ndev);

	/*
	 * Do not wait for workers: reset_work may be waiting for RTNL held by
	 * a synchronous resume caller. Nothing is freed here. Both workers
	 * serialize hardware access with control_lock; revocation invalidated
	 * their epoch, and the down-state gates prevent later hardware access.
	 * Keep RTNL through commit so no open can enter a worker-drain gap.
	 * Requests may change while suspended; do not save a recovery snapshot.
	 */
	cancel_delayed_work(&tp->reset_work);
	cancel_work(&tp->link_work);
	WRITE_ONCE(tp->link_work_epoch, 0);

	rtlmq_revoke_mmio_locked(tp);
	tp->runtime_pm_suspended = true;
	tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_NONE;
	tp->platform_quiesce_was_running = false;
out_unlock:
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();

	return ret;
}

static int rtlmq_runtime_resume_down(struct rtlmq_private *tp)
{
	bool owner_set = false;
	int ret = 0;

	mutex_lock(&tp->control_lock);
	if (!tp->runtime_pm_suspended)
		goto out_unlock;
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE ||
	    tp->state != RTLMQ_STATE_PROBED ||
	    (netif_running(tp->ndev) &&
	     !READ_ONCE(tp->runtime_pm_netdev_ref))) {
		ret = -EBUSY;
		goto out_unlock;
	}

	tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_RUNTIME_PM;
	tp->platform_quiesce_was_running = false;
	owner_set = true;
	ret = rtlmq_grant_ordinary_mmio_locked(tp);
	if (ret)
		goto out_failure;
	ret = rtlmq_program_mac_address(tp, tp->ndev->dev_addr);
	if (ret)
		goto out_failure;
	/*
	 * Runtime resume may be entered by ndo_open() while RTNL is already
	 * held, so do not take RTNL or call feature-notifying recovery helpers.
	 * Runtime suspend preserved TSO/XDP capabilities; restore hardware
	 * availability only.
	 */
	ret = rtlmq_apply_wol(tp);
	if (ret)
		goto out_failure;
	netif_device_attach(tp->ndev);
	rtlmq_sync_link_state(tp);

	tp->runtime_pm_suspended = false;
	tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_NONE;
	tp->platform_quiesce_was_running = false;
	owner_set = false;
	goto out_unlock;

out_failure:
	rtlmq_revoke_mmio_locked(tp);
	netif_device_detach(tp->ndev);
	if (owner_set) {
		tp->platform_quiesce_owner = RTLMQ_PLATFORM_QUIESCE_NONE;
		tp->platform_quiesce_was_running = false;
	}
out_unlock:
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int __maybe_unused rtlmq_runtime_suspend(struct device *device)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct rtlmq_private *tp;

	if (!ndev)
		return 0;

	tp = netdev_priv(ndev);
	if (!netif_running(ndev))
		return rtlmq_runtime_suspend_down(tp);
	if (!READ_ONCE(tp->system_sleep_transition))
		return -EBUSY;

	return rtlmq_quiesce_and_close_for_pm_or_pci(tp, RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM,
						     device_may_wakeup(device), false);
}

static int __maybe_unused rtlmq_runtime_resume(struct device *device)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct rtlmq_private *tp;

	if (!ndev)
		return 0;

	tp = netdev_priv(ndev);
	if (READ_ONCE(tp->platform_quiesce_owner) ==
	    RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM)
		return rtlmq_reopen_or_republish_for_pm_or_pci(tp,
							       RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM, 0,
							       BIT(RTLMQ_RESET_REASON_PM_RESUME),
							       "resume failed: %d\n",
							       true);

	return rtlmq_runtime_resume_down(tp);
}

static int __maybe_unused rtlmq_runtime_idle(struct device *device)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct rtlmq_private *tp;
	bool eligible;

	if (!ndev)
		return -ENODEV;

	tp = netdev_priv(ndev);
	mutex_lock(&tp->control_lock);
	eligible = rtlmq_runtime_pm_idle_eligible_locked(tp);
	mutex_unlock(&tp->control_lock);
	if (!eligible)
		return -EBUSY;

	pm_schedule_suspend(device, RTLMQ_RUNTIME_AUTOSUSPEND_DELAY_MS);
	return -EBUSY;
}

static void rtlmq_release_probe_storage(void *data)
{
	struct rtlmq_private *tp = data;

	/*
	 * Devres runs even when remove or probe unwind cannot retire DMA.
	 * Keep the owner with its retained storage and optional XSK DMA refs.
	 */
	if (!READ_ONCE(tp->dma_quiesced) ||
	    !rtlmq_ring_storage_release_allowed(tp) ||
	    tp->rx_rings || tp->tx_rings || tp->rx_ring_cold || tp->vectors) {
		netdev_crit(tp->ndev,
			    "retaining probe storage after incomplete retirement; reboot required for reclamation\n");
		return;
	}

	if (tp->counters)
		dma_free_coherent(&tp->pdev->dev, sizeof(*tp->counters),
				  tp->counters, tp->counters_phys_addr);
	free_netdev(tp->ndev);
}

static struct net_device *rtlmq_alloc_probe_storage(struct pci_dev *pdev)
{
	struct rtlmq_private *tp;
	struct net_device *ndev;
	int ret;

	ndev = alloc_etherdev_mqs(sizeof(*tp), RTLMQ_IMPL_MAX_TX_QUEUES,
				  RTLMQ_IMPL_MAX_RX_QUEUES);
	if (!ndev)
		return ERR_PTR(-ENOMEM);
	tp = netdev_priv(ndev);
	tp->pdev = pdev;
	tp->ndev = ndev;
	/* No driver storage has been exposed to the device yet. */
	WRITE_ONCE(tp->dma_quiesced, true);

	/* Register first so MDIO and other devres users retire before their owner. */
	ret = devm_add_action_or_reset(&pdev->dev, rtlmq_release_probe_storage, tp);
	if (ret)
		return ERR_PTR(ret);
	return ndev;
}

static int rtlmq_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	bool vector_storage_allocated = false;
	netdev_features_t qualified_features;
	netdev_features_t qualified_defaults;
	bool ring_storage_allocated = false;
	bool netdev_was_registered = false;
	bool phy_work_initialized = false;
	bool highdma_capable = false;
	struct rtlmq_private *tp;
	struct net_device *ndev;
	u8 mac_addr[ETH_ALEN];
	void __iomem *mmio;
	unsigned long bars;
	bool l1_disabled;
	int region;
	int ret;

	(void)id;

	ndev = rtlmq_alloc_probe_storage(pdev);
	if (IS_ERR(ndev))
		return PTR_ERR(ndev);

	/*
	 * Open binds persistent NAPI identities to the allocated IRQs.
	 * Net-core retains and restores their IRQ affinity configuration.
	 */
	netif_set_affinity_auto(ndev);

	tp = netdev_priv(ndev);
	mutex_init(&tp->control_lock);
	mutex_init(&tp->led_lock);
	spin_lock_init(&tp->state_lock);
	spin_lock_init(&tp->mac_ocp_lock);
	spin_lock_init(&tp->phy_ocp_lock);
	u64_stats_init(&tp->queue_stats_syncp);
	INIT_DELAYED_WORK(&tp->reset_work, rtlmq_reset_work);
	tp->tx_timeout_pending_events = 0;
	tp->last_tx_timeout_valid = false;
	tp->wol_opts = 0;
	tp->msg_enable = RTLMQ_DEFAULT_MSG_ENABLE;
	tp->state = RTLMQ_STATE_NEW;

	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable PCI device\n");

	/*
	 * pci_enable_device() may leave inherited bus mastering enabled.
	 * Disable it before reading the BAR or checking chip identity.
	 * No RTLMQ DMA storage has been exposed at this point.
	 */
	ret = rtlmq_pci_disable_bus_master(tp, true);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to revoke inherited DMA authority\n");

	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	if (!bars)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "no MMIO BAR available\n");

	region = rtlmq_pci_select_mmio_region(pdev, bars);
	if (region < 0)
		return dev_err_probe(&pdev->dev, region,
				     "required MMIO BAR unavailable\n");
	ret = rtlmq_pci_validate_mmio_bar(pdev, region);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "BAR%d is too small or not MMIO (need %#x bytes)\n",
				     region, RTLMQ_MMIO_REQUIRED_SIZE);

	mmio = pcim_iomap_region(pdev, region, KBUILD_MODNAME);
	if (IS_ERR(mmio))
		return dev_err_probe(&pdev->dev, PTR_ERR(mmio),
				     "failed to map MMIO BAR%d\n", region);
	WRITE_ONCE(tp->mmio_addr, mmio);
	mutex_lock(&tp->control_lock);
	ret = rtlmq_grant_ordinary_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to grant probe MMIO authority\n");

	ret = rtlmq_identify_chip(tp);
	if (ret)
		goto err_cancel_work;

	if (!rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp)) {
		ret = -EINVAL;
		goto err_cancel_work;
	}
	/* Probe must reject fiber before registering the copper PHY. */
	if (rtlmq_selected_hw_contract(tp)->mac_version == RTLMQ_MAC_8126A) {
		ret = rtlmq_8126_admit_board_mode(tp);
		if (ret) {
			dev_err_probe(&pdev->dev, ret,
				      "RTL8126 board-mode admission failed\n");
			goto err_cancel_work;
		}
	}

	/*
	 * Identity and board admission must precede DMA-mask selection. A
	 * profile without qualified high-address DMA must constrain both
	 * coherent rings and streaming buffers, not just NETIF_F_HIGHDMA.
	 * No DMA storage is allocated before this point.
	 */
	if (sizeof(dma_addr_t) > 4 && rtlmq_qualified_supports_highdma(tp)) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
		if (!ret)
			highdma_capable = true;
		else
			ret = dma_set_mask_and_coherent(&pdev->dev,
							DMA_BIT_MASK(32));
	} else {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	}
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to set DMA mask\n");
		goto err_cancel_work;
	}

	l1_disabled = rtlmq_pci_disable_unsafe_link_states(tp);
	tp->aspm_manageable = l1_disabled;
	rtlmq_log_probe_state(tp, l1_disabled);

	device_set_wakeup_capable(&pdev->dev,
				  !!rtlmq_qualified_wol_opts(tp));
	if (rtlmq_qualified_wol_opts(tp)) {
		device_set_wakeup_enable(&pdev->dev, false);
		pci_wake_from_d3(pdev, false);
	}

	ret = rtlmq_read_mac_address(tp, mac_addr);
	if (ret)
		goto err_cancel_work;

	rtlmq_init_rss(tp);

	ret = rtlmq_alloc_counter_memory(tp);
	if (ret)
		goto err_cancel_work;

	ret = rtlmq_withdraw_netdev_queue_counts(tp);
	if (ret)
		goto err_cancel_work;

	ret = rtlmq_alloc_rings(tp);
	if (ret)
		goto err_cancel_work;
	ring_storage_allocated = true;

	ret = rtlmq_alloc_vectors(tp);
	if (ret)
		goto err_cancel_work;
	vector_storage_allocated = true;

	/*
	 * rtlmq_prepare_phy() initializes link_work before it enters phylib, so
	 * the common unwind below must drain that work even when registration
	 * itself fails.
	 */
	phy_work_initialized = true;
	ret = rtlmq_prepare_phy(tp);
	if (ret)
		goto err_cancel_work;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_PROBED);
	if (!ret)
		ret = rtlmq_apply_requested_runtime_transaction(tp, NULL, false);
	mutex_unlock(&tp->control_lock);
	if (ret)
		goto err_cancel_work;

	eth_hw_addr_set(ndev, mac_addr);
	ether_addr_copy(ndev->perm_addr, mac_addr);
	ndev->netdev_ops = &rtlmq_netdev_ops;
	ndev->stat_ops = &rtlmq_stat_ops;
	ndev->ethtool_ops = rtlmq_get_ethtool_ops(tp);
	ndev->watchdog_timeo = 5 * HZ;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = rtlmq_max_supported_mtu();
	if (rtlmq_qualified_mtu_limit(tp))
		ndev->max_mtu = min_t(u32, ndev->max_mtu,
				      rtlmq_qualified_mtu_limit(tp));
	qualified_features = rtlmq_qualified_netdev_hw_features(tp);
	qualified_defaults = rtlmq_qualified_netdev_default_features(tp);
	ndev->hw_features |= qualified_features;
	ndev->features |= qualified_defaults;
	if (highdma_capable && rtlmq_qualified_supports_highdma(tp))
		ndev->features |= NETIF_F_HIGHDMA;
	if (rtlmq_qualified_supports_live_mac_change(tp))
		ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
	ndev->wanted_features = ndev->features;
	rtlmq_sync_vlan_features(ndev, ndev->features);
	mutex_lock(&tp->control_lock);
	rtlmq_sync_netdev_tso_limits(tp);
	mutex_unlock(&tp->control_lock);
	/*
	 * Set profile capabilities before registration so userspace can query
	 * support and bind XSK before open. Add NDO_XMIT only with live TX
	 * resources after runtime commit.
	 */
	ndev->xdp_features = rtlmq_xdp_feature_mask(tp);
	ndev->xdp_metadata_ops = rtlmq_get_xdp_metadata_ops(tp);
	ndev->xsk_tx_metadata_ops = rtlmq_get_xsk_tx_metadata_ops(tp);
	ndev->xdp_zc_max_segs = rtlmq_xdp_zc_max_segs(tp);
	/*
	 * On non-RT kernels, RTL8125B uses the net-core 20 us repoll. A 160 us
	 * repoll holds IRQs masked after productive polls and can delay the
	 * next packet in a sparse request/response stream until that timer.
	 * Keep the existing override for other chips, including persistent
	 * configs restored on NAPI re-enable. Sysfs overrides remain available.
	 */
	netdev_sw_irq_coalesce_default_on(ndev);
	if (!IS_ENABLED(CONFIG_PREEMPT_RT) &&
	    !rtlmq_qualified_use_core_napi_defaults(tp)) {
		unsigned int i;
		unsigned int count = max(ndev->num_rx_queues,
					 ndev->num_tx_queues);

		WRITE_ONCE(ndev->gro_flush_timeout,
			   RTLMQ_DEFAULT_GRO_FLUSH_TIMEOUT_NS);
		for (i = 0; i < count; i++)
			ndev->napi_config[i].gro_flush_timeout =
				RTLMQ_DEFAULT_GRO_FLUSH_TIMEOUT_NS;
	}
	SET_NETDEV_DEV(ndev, &pdev->dev);
	netif_carrier_off(ndev);

	ret = rtlmq_ptp_register(tp);
	if (ret)
		goto err_cancel_work;

	ret = register_netdev(ndev);
	if (ret)
		goto err_unregister_ptp;
	netdev_was_registered = true;

	ret = rtlmq_leds_register(tp);
	if (ret) {
		unregister_netdev(ndev);
		goto err_unregister_ptp;
	}

	pci_set_drvdata(pdev, ndev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, RTLMQ_RUNTIME_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	WRITE_ONCE(tp->runtime_pm_ready, true);
	pm_runtime_mark_last_busy(&pdev->dev);
	/* Drop the PCI core's probe reference, separate from userspace policy. */
	pm_runtime_put_noidle(&pdev->dev);
	/*
	 * driver_override handoff may retain a userspace PM forbid reference.
	 * Release it after PM setup. pm_runtime_allow() drops the reference
	 * only if the device was not already automatic.
	 */
	pm_runtime_allow(&pdev->dev);
	pm_runtime_idle(&pdev->dev);
	dev_info(&pdev->dev,
		 "registered %s XID %#x cfg_rxq=%u cfg_txq=%u vec=%u routing=%u requested_rxq=%u requested_txq=%u; datapath follows live link state\n",
		 rtlmq_chip_profile_name(tp->profile), tp->xid,
		 rtlmq_published_num_rx_queues(tp),
		 rtlmq_published_num_tx_queues(tp),
		 rtlmq_published_num_vectors(tp),
		 rtlmq_published_routing_mode(tp),
		 rtlmq_requested_num_rx_queues(tp),
		 rtlmq_requested_num_tx_queues(tp));
	return 0;

	/*
	 * Drain callbacks before freeing probe allocations. NAPI and IRQs may
	 * have been registered by an early open after register_netdev().
	 */
err_unregister_ptp:
	rtlmq_ptp_unregister(tp);
err_cancel_work:
	cancel_delayed_work_sync(&tp->reset_work);
	if (phy_work_initialized)
		rtlmq_cleanup_phy(tp);
	rtlmq_release_firmware(tp);
	if (netdev_was_registered) {
		int cleanup_ret = 0;

		/* An early open may retain runtime ownership after failed stop. */
		rtnl_lock();
		mutex_lock(&tp->control_lock);
		if (!rtlmq_release_allowed(tp))
			cleanup_ret = rtlmq_finalize_failed_runtime_locked(tp);
		if (!cleanup_ret && READ_ONCE(tp->state) != RTLMQ_STATE_DEAD)
			cleanup_ret = rtlmq_finish_removal_locked(tp);
		mutex_unlock(&tp->control_lock);
		rtnl_unlock();
		if (cleanup_ret)
			netdev_crit(ndev,
				    "failed probe software cleanup incomplete: %d; core managed-resource release will continue\n",
				    cleanup_ret);
	}
	/*
	 * Constructors free their own partial allocations. Call release helpers
	 * only for completed constructors; their guards require complete setup.
	 */
	if (vector_storage_allocated)
		rtlmq_free_vectors(tp);
	if (ring_storage_allocated)
		rtlmq_free_rings(tp);
	mutex_lock(&tp->control_lock);
	rtlmq_terminalize_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);

	return ret;
}

static int __maybe_unused rtlmq_suspend(struct device *device)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct rtlmq_private *tp;
	int ret;

	if (!ndev)
		return 0;

	tp = netdev_priv(ndev);
	WRITE_ONCE(tp->system_sleep_transition, true);
	pm_runtime_get_noresume(device);
	ret = pm_runtime_force_suspend(device);
	if (ret) {
		pm_runtime_put_noidle(device);
		WRITE_ONCE(tp->system_sleep_transition, false);
	}
	return ret;
}

static int __maybe_unused rtlmq_resume(struct device *device)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct rtlmq_private *tp;
	int ret;

	if (!ndev)
		return 0;

	tp = netdev_priv(ndev);
	ret = pm_runtime_force_resume(device);
	pm_runtime_put_noidle(device);
	WRITE_ONCE(tp->system_sleep_transition, false);
	return ret;
}

static pci_ers_result_t rtlmq_pci_error_detected(struct pci_dev *pdev,
						 pci_channel_state_t state)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct rtlmq_private *tp;

	if (!ndev)
		return PCI_ERS_RESULT_DISCONNECT;

	tp = netdev_priv(ndev);
	if (rtlmq_quiesce_and_close_for_pm_or_pci(tp, RTLMQ_PLATFORM_QUIESCE_PCI_ERROR, false,
						  true))
		return PCI_ERS_RESULT_DISCONNECT;

	if (state == pci_channel_io_perm_failure) {
		int ret;

		rtnl_lock();
		mutex_lock(&tp->control_lock);
		ret = rtlmq_terminalize_frozen_pci_failure_locked(tp);
		mutex_unlock(&tp->control_lock);
		rtnl_unlock();
		if (ret)
			netdev_err(ndev,
				   "terminal PCI failure teardown failed: %d\n",
				   ret);
	}

	pci_disable_device(pdev);

	return state == pci_channel_io_perm_failure ?
	       PCI_ERS_RESULT_DISCONNECT : PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t rtlmq_pci_mmio_enabled_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * MMIO recovery does not restart the datapath. Keep runtime unpublished
	 * and bus mastering disabled. Mask stale interrupt sources only, then
	 * require PCI reset before rebuilding hardware state.
	 */
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_PCI_ERROR ||
	    READ_ONCE(tp->state) == RTLMQ_STATE_DEAD ||
	    READ_ONCE(tp->runtime_published))
		return PCI_ERS_RESULT_DISCONNECT;

	if (rtlmq_grant_recovery_mmio_locked(tp))
		return PCI_ERS_RESULT_DISCONNECT;
	if (rtlmq_mask_and_ack_intrs_recovery(tp)) {
		rtlmq_revoke_mmio_locked(tp);
		return PCI_ERS_RESULT_DISCONNECT;
	}
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t rtlmq_pci_mmio_enabled(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct rtlmq_private *tp;
	pci_ers_result_t result;

	if (!ndev)
		return PCI_ERS_RESULT_DISCONNECT;

	tp = netdev_priv(ndev);
	rtnl_lock();
	mutex_lock(&tp->control_lock);
	result = rtlmq_pci_mmio_enabled_locked(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();

	return result;
}

static pci_ers_result_t rtlmq_pci_slot_reset(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct rtlmq_private *tp;
	int ret;

	if (!ndev)
		return PCI_ERS_RESULT_DISCONNECT;

	ret = pci_enable_device(pdev);
	if (ret) {
		netdev_err(ndev,
			   "Cannot re-enable PCI device after reset: %d\n",
			   ret);
		return PCI_ERS_RESULT_DISCONNECT;
	}

	tp = netdev_priv(ndev);
	ret = rtlmq_finish_platform_quiesce(tp, RTLMQ_PLATFORM_QUIESCE_PCI_ERROR, false, true);
	if (ret) {
		netdev_err(ndev,
			   "Cannot finish PCI recovery teardown after reset: %d\n",
			   ret);
		pci_disable_device(pdev);
		return PCI_ERS_RESULT_DISCONNECT;
	}

	return PCI_ERS_RESULT_RECOVERED;
}

static void rtlmq_pci_resume(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct rtlmq_private *tp;
	int ret;

	if (!ndev)
		return;

	tp = netdev_priv(ndev);
	ret = rtlmq_reopen_or_republish_for_pm_or_pci(tp,
						      RTLMQ_PLATFORM_QUIESCE_PCI_ERROR,
						      BIT(RTLMQ_RESET_REASON_PCI_RECOVERY),
						      BIT(RTLMQ_RESET_REASON_PCI_RECOVERY),
						      "PCI recovery resume failed: %d\n",
						      false);
	(void)ret;
}

static const struct dev_pm_ops rtlmq_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rtlmq_suspend, rtlmq_resume)
	RUNTIME_PM_OPS(rtlmq_runtime_suspend, rtlmq_runtime_resume,
		       rtlmq_runtime_idle)
};

static const struct pci_error_handlers rtlmq_err_handler = {
	.error_detected = rtlmq_pci_error_detected,
	.mmio_enabled = rtlmq_pci_mmio_enabled,
	.slot_reset = rtlmq_pci_slot_reset,
	.resume = rtlmq_pci_resume,
};

static int rtlmq_shutdown_device(struct rtlmq_private *tp, bool poweroff)
{
	int close_ret;
	int pm_token;
	int ret;

	/* PCI shutdown attempts resume, but does not report its error to us. */
	pm_token = rtlmq_runtime_pm_access_get(tp);
	WRITE_ONCE(tp->runtime_pm_ready, false);
	rtnl_lock();
	mutex_lock(&tp->control_lock);
	ret = rtlmq_begin_removal_locked(tp);
	if (pm_token < 0) {
		ret = pm_token;
		rtlmq_terminalize_mmio_locked(tp);
	}
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
	rtlmq_leds_unregister(tp);
	rtlmq_drain_control_work(tp);

	rtnl_lock();
	mutex_lock(&tp->control_lock);
	/*
	 * REMOVING blocks rebuild and new control requests. Close retains DMA
	 * storage if its fence fails; shutdown must never attempt a reopen.
	 */
	close_ret = rtlmq_close_device(tp);
	if (!ret)
		ret = close_ret;
	else if (close_ret)
		netdev_err(tp->ndev, "shutdown close failed: %d\n", close_ret);
	if (!ret)
		ret = rtlmq_program_mac_address(tp, tp->ndev->perm_addr);
	if (!ret && poweroff && device_may_wakeup(&tp->pdev->dev))
		ret = rtlmq_prepare_wol_suspend(tp);
	else if (!ret && poweroff)
		ret = rtlmq_prepare_no_wake_suspend(tp);
	rtlmq_terminalize_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
	/* PHY/IRQ shutdown can queue one last link notification. */
	cancel_work_sync(&tp->link_work);
	WRITE_ONCE(tp->link_work_epoch, 0);

	if (!ret && poweroff) {
		ret = pci_prepare_to_sleep(tp->pdev);
	}
	if (pm_token > 0)
		pm_runtime_put_noidle(&tp->pdev->dev);
	return ret;
}

static void rtlmq_shutdown(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	int ret;

	if (!ndev)
		return;
	ret = rtlmq_shutdown_device(netdev_priv(ndev),
				    system_state == SYSTEM_POWER_OFF);
	if (ret)
		netdev_err(ndev, "shutdown failed: %d; device will not be restarted\n",
			   ret);
}

static void rtlmq_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct rtlmq_private *tp;
	int dma_ret;
	int ret;

	if (!ndev)
		return;

	tp = netdev_priv(ndev);
	/* Restore the probe reference consumed by the PCI core after remove. */
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_barrier(&pdev->dev);
	WRITE_ONCE(tp->runtime_pm_ready, false);
	mutex_lock(&tp->control_lock);
	ret = 0;
	if (READ_ONCE(tp->state) != RTLMQ_STATE_DEAD)
		ret = rtlmq_begin_removal_locked(tp);
	mutex_unlock(&tp->control_lock);
	if (ret) {
		netdev_err(ndev,
			   "remove ownership transition failed: %d\n",
			   ret);
		WARN_ON_ONCE(ret);
	}
	pci_set_drvdata(pdev, NULL);
	rtlmq_leds_unregister(tp);
	unregister_netdev(ndev);
	rtlmq_ptp_unregister(tp);
	/*
	 * unregister_netdev()/ndo_stop() stop PHY/IRQ sources. Drain queued
	 * control work before entering DEAD and freeing probe allocations.
	 */
	rtlmq_drain_control_work(tp);
	WRITE_ONCE(tp->link_work_epoch, 0);
	/*
	 * If ndo_stop() could not fully quiesce, terminal cleanup still resets
	 * queue counts and removes queue/NAPI mappings. Take RTNL before
	 * control_lock, as in normal lifecycle calls.
	 */
	rtnl_lock();
	mutex_lock(&tp->control_lock);
	/*
	 * Unregister/stop should already have disabled bus mastering. Check it
	 * again before DEAD or any probe allocation is freed. A failed recheck
	 * does not prevent a subsequent normal close from completing shutdown.
	 */
	dma_ret = rtlmq_pci_disable_bus_master(
		tp,
		tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_PCI_ERROR);
	ret = 0;
	if (READ_ONCE(tp->state) != RTLMQ_STATE_DEAD &&
	    !rtlmq_quiesce_contract_satisfied(tp))
		ret = rtlmq_close_device(tp);
	if (ret || (!rtlmq_quiesce_contract_satisfied(tp) &&
		    !rtlmq_final_release_allowed(tp))) {
		netdev_crit(ndev,
			    "remove normal cleanup incomplete dma=%d close=%d; core-owned lifetimes require final cleanup\n",
			    dma_ret, ret);
		ret = rtlmq_finalize_failed_runtime_locked(tp);
	}
	if (!ret && READ_ONCE(tp->state) != RTLMQ_STATE_DEAD)
		ret = rtlmq_finish_removal_locked(tp);
	rtlmq_terminalize_mmio_locked(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
	if (ret)
		netdev_crit(ndev,
			    "remove software cleanup incomplete: %d; core managed-resource release will continue\n",
			    ret);
	synchronize_rcu();
	rtlmq_cleanup_phy(tp);
	rtlmq_release_firmware(tp);
	rtlmq_free_vectors(tp);
	rtlmq_free_rings(tp);
	if (tp->requested_xdp_prog)
		bpf_prog_put(xchg(&tp->requested_xdp_prog, NULL));
	if (tp->xdp_prog)
		bpf_prog_put(xchg(&tp->xdp_prog, NULL));
}

static struct pci_driver rtlmq_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= rtlmq_pci_ids,
	.probe		= rtlmq_probe,
	.remove		= rtlmq_remove,
	.shutdown	= rtlmq_shutdown,
	.err_handler	= &rtlmq_err_handler,
	.driver.pm	= pm_ptr(&rtlmq_pm_ops),
};

static int __init rtlmq_init_module(void)
{
	return pci_register_driver(&rtlmq_driver);
}

static void __exit rtlmq_exit_module(void)
{
	pci_unregister_driver(&rtlmq_driver);
}

module_init(rtlmq_init_module);
module_exit(rtlmq_exit_module);

MODULE_AUTHOR("rtlmq contributors");
MODULE_DESCRIPTION("Realtek RTL8125B/RTL8126A Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RTLMQ_DRIVER_VERSION);
