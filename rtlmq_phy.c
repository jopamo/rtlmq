// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/moduleparam.h>

#include "rtlmq_chip.h"
#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_runtime.h"
#include "rtlmq_ring.h"
#include "rtlmq_regs.h"
#include "rtlmq_tx.h"

static bool rtlmq_eee_default_enabled;
/* Boot default only: runtime EEE state remains an administrative setting. */
module_param_named(eee_default_enabled, rtlmq_eee_default_enabled, bool, 0400);
MODULE_PARM_DESC(eee_default_enabled,
		 "enable Energy Efficient Ethernet by default (default: false for low-latency)");

/* Keep the fixed read/write operation out of the PTP mailbox poll loop. */
static __always_inline int
rtlmq_phy_ocp_access(struct rtlmq_private *tp, u32 reg, u16 value, bool write)
{
	unsigned long flags;
	u32 command, ocp;
	int ret;

	if (reg & 0xffff0001)
		return -EINVAL;
	if (!tp)
		return -ENODEV;

	spin_lock_irqsave(&tp->phy_ocp_lock, flags);
	/* Recheck after waiting for the mailbox; removal may revoke access. */
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		ret = -ENODEV;
		goto out_unlock;
	}
	command = reg << RTLMQ_OCP_REG_SHIFT;
	if (write)
		command |= RTLMQ_OCP_FLAG | value;
	writel(command, tp->mmio_addr + RTLMQ_GPHY_OCP);
	/* Read completion sets FLAG; write completion clears it. */
	ret = readl_poll_timeout_atomic(tp->mmio_addr + RTLMQ_GPHY_OCP, ocp,
					ocp == U32_MAX ||
					(write ? !(ocp & RTLMQ_OCP_FLAG) :
					 ocp & RTLMQ_OCP_FLAG), 1, 250);
	if (ocp == U32_MAX)
		ret = -ENODEV;
	else if (!ret)
		ret = write ? 0 : ocp & RTLMQ_OCP_DATA_MASK;

out_unlock:
	spin_unlock_irqrestore(&tp->phy_ocp_lock, flags);
	return ret;
}

int rtlmq_phy_ocp_write(struct rtlmq_private *tp, u32 reg, u16 value)
{
	return rtlmq_phy_ocp_access(tp, reg, value, true);
}

int rtlmq_phy_ocp_read(struct rtlmq_private *tp, u32 reg)
{
	return rtlmq_phy_ocp_access(tp, reg, 0, false);
}

static int rtlmq_mdio_ocp_address(u16 base, int reg)
{
	if (reg < 0)
		return -EINVAL;
	if (base != RTLMQ_PHY_OCP_STD_BASE) {
		if (reg < 0x10)
			return -EINVAL;
		reg -= 0x10;
	}
	/* Firmware can use extended offsets: bound the address, not just C22. */
	if (reg > (U16_MAX - base) / 2)
		return -EINVAL;
	return base + reg * 2;
}

int rtlmq_mdio_write(struct rtlmq_private *tp, int reg, int value)
{
	int ocp_reg;

	if (reg < 0 || value < 0 || value > U16_MAX)
		return -EINVAL;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	if (reg == 0x1f) {
		/* Validate before narrowing to the cached 16-bit OCP byte base. */
		ocp_reg = rtlmq_ocp_page_to_base(value);
		if (ocp_reg < 0)
			return ocp_reg;
		tp->ocp_base = ocp_reg ?: RTLMQ_PHY_OCP_STD_BASE;
		return 0;
	}

	ocp_reg = rtlmq_mdio_ocp_address(tp->ocp_base, reg);
	if (ocp_reg < 0)
		return ocp_reg;
	return rtlmq_phy_ocp_write(tp, ocp_reg, value);
}

int rtlmq_mdio_read(struct rtlmq_private *tp, int reg)
{
	int ocp_reg;

	if (reg < 0)
		return -EINVAL;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	if (reg == 0x1f)
		return tp->ocp_base == RTLMQ_PHY_OCP_STD_BASE ?
		       0 : tp->ocp_base >> 4;

	ocp_reg = rtlmq_mdio_ocp_address(tp->ocp_base, reg);
	if (ocp_reg < 0)
		return ocp_reg;
	return rtlmq_phy_ocp_read(tp, ocp_reg);
}

static bool rtlmq_phy_is_connected(const struct rtlmq_private *tp)
{
	return tp->phydev && tp->ndev && tp->ndev->phydev == tp->phydev;
}

static u8 rtlmq_read_phy_status(struct rtlmq_private *tp)
{
	u8 status;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return 0;

	status = readb(tp->mmio_addr + RTLMQ_PHY_STATUS);
	return status == U8_MAX ? 0 : status;
}

static int rtlmq_mdio_read_reg(struct mii_bus *mii_bus, int phyaddr, int phyreg)
{
	struct rtlmq_private *tp = mii_bus->priv;
	int pm_token;
	int ret;

	if (phyaddr > 0)
		return -ENODEV;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	ret = rtlmq_mdio_read(tp, phyreg);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_mdio_write_reg(struct mii_bus *mii_bus, int phyaddr,
				int phyreg, u16 val)
{
	struct rtlmq_private *tp = mii_bus->priv;
	int pm_token;
	int ret;

	if (phyaddr > 0)
		return -ENODEV;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	ret = rtlmq_mdio_write(tp, phyreg, val);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_mdio_read_reg_c45(struct mii_bus *mii_bus, int addr,
				   int devnum, int regnum)
{
	struct rtlmq_private *tp = mii_bus->priv;
	int pm_token;
	int ret;

	if (addr > 0 || devnum != MDIO_MMD_VEND2 || regnum <= MDIO_STAT2)
		return -ENODEV;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	ret = rtlmq_phy_ocp_read(tp, regnum);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_mdio_write_reg_c45(struct mii_bus *mii_bus, int addr,
				    int devnum, int regnum, u16 val)
{
	struct rtlmq_private *tp = mii_bus->priv;
	int pm_token;
	int ret;

	if (addr > 0 || devnum != MDIO_MMD_VEND2 || regnum <= MDIO_STAT2)
		return -ENODEV;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	ret = rtlmq_phy_ocp_write(tp, regnum, val);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_mdio_register(struct rtlmq_private *tp)
{
	struct pci_dev *pdev = tp->pdev;
	struct phy_device *phydev;
	struct mii_bus *new_bus;
	int ret;

	new_bus = devm_mdiobus_alloc(&pdev->dev);
	if (!new_bus)
		return -ENOMEM;

	/*
	 * Register only the internal PHY at address 0 on this MDIO bus.
	 * The MAC driver handles its power and lifecycle sequencing.
	 */
	new_bus->name = KBUILD_MODNAME;
	new_bus->priv = tp;
	new_bus->parent = &pdev->dev;
	/*
	 * The MAC already routes link-change events into the driver IRQ path,
	 * and rtlmq_irq.c feeds those events back into phylib through
	 * phy_mac_interrupt(). Set PHY_MAC_INTERRUPT so phylib uses
	 * interrupt-driven state changes instead of a parallel polling loop.
	 */
	new_bus->irq[0] = PHY_MAC_INTERRUPT;
	new_bus->phy_mask = GENMASK(31, 1);
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%x-%x", KBUILD_MODNAME,
		 pci_domain_nr(pdev->bus), pci_dev_id(pdev));

	new_bus->read = rtlmq_mdio_read_reg;
	new_bus->write = rtlmq_mdio_write_reg;
	new_bus->read_c45 = rtlmq_mdio_read_reg_c45;
	new_bus->write_c45 = rtlmq_mdio_write_reg_c45;

	ret = devm_mdiobus_register(&pdev->dev, new_bus);
	if (ret)
		return ret;

	phydev = mdiobus_get_phy(new_bus, 0);
	if (!phydev)
		return -ENODEV;

	if (!phydev->drv) {
		dev_err(&pdev->dev,
			"no dedicated PHY driver found for PHY ID 0x%08x\n",
			phydev->phy_id);
		return -EUNATCH;
	}

	phydev->mac_managed_pm = true;
	phydev->irq = PHY_MAC_INTERRUPT;
	phy_support_asym_pause(phydev);

	tp->mii_bus = new_bus;
	tp->phydev = phydev;

	return 0;
}

static void rtlmq_clear_phy_eee_state(struct phy_device *phydev)
{
	phydev->eee_cfg.eee_enabled = false;
	phydev->eee_cfg.tx_lpi_enabled = false;
	phydev->eee_active = false;
	phydev->enable_tx_lpi = false;
}

int rtlmq_apply_eee_policy(struct rtlmq_private *tp)
{
	int ret;

	if (!tp || !tp->phydev)
		return -ENODEV;
	if (!rtlmq_qualified_supports_eee(tp)) {
		/*
		 * phylib may have inherited EEE advertisement from the PHY
		 * strap. An unqualified MAC must not negotiate it even when
		 * ethtool EEE controls are hidden; replay the disable on reopen.
		 */
		rtlmq_clear_phy_eee_state(tp->phydev);
		ret = genphy_c45_an_config_eee_aneg(tp->phydev);
		return ret < 0 ? ret : 0;
	}

	/*
	 * Apply the module-selected default exactly once. Later PHY opens must
	 * preserve the setting stored by phylib and merely replay
	 * it after hardware reset.
	 */
	if (!tp->eee_default_applied) {
		if (READ_ONCE(rtlmq_eee_default_enabled)) {
			tp->eee_default_applied = true;
			return 0;
		}

		/*
		 * Disable EEE by default for latency, but keep advertising_eee
		 * intact so userspace can enable it later through ethtool.
		 */
		rtlmq_clear_phy_eee_state(tp->phydev);
	}

	ret = genphy_c45_an_config_eee_aneg(tp->phydev);
	if (ret < 0)
		return ret;

	tp->eee_default_applied = true;
	return 0;
}

static bool rtlmq_link_is_up(struct rtlmq_private *tp)
{
	if (rtlmq_phy_is_connected(tp))
		return !!tp->phydev->link;

	return rtlmq_read_phy_status(tp) & RTLMQ_PHY_STATUS_LINK;
}

static bool rtlmq_link_health_is_wedged(struct rtlmq_private *tp)
{
	u8 phy_status;
	u8 chipcmd;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !tp->ndev || !rtlmq_mmio_ordinary_authorized(tp))
		return false;

	if (tp->state != RTLMQ_STATE_OPEN ||
	    !netif_running(tp->ndev) ||
	    !netif_device_present(tp->ndev))
		return false;

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	phy_status = readb(tp->mmio_addr + RTLMQ_PHY_STATUS);
	if (chipcmd != 0xff && phy_status != 0xff)
		return false;

	netdev_warn(tp->ndev,
		    "link-path wedge detected chipcmd=%#02x phy_status=%#02x, queuing owned reset\n",
		    chipcmd, phy_status);
	netif_carrier_off(tp->ndev);
	rtlmq_sync_tx_subqueues(tp);
	netif_device_detach(tp->ndev);
	rtlmq_request_reset(tp,
			    BIT(RTLMQ_RESET_REASON_LINK_WEDGE)
	);

	return true;
}

void rtlmq_sync_link_state(struct rtlmq_private *tp)
{
	int duplex = DUPLEX_UNKNOWN;
	int speed = SPEED_UNKNOWN;
	bool published_link;
	u8 phy_status;
	bool link_up;
	int eee_ret;

	lockdep_assert_held(&tp->control_lock);

	if (rtlmq_link_health_is_wedged(tp))
		return;

	rtlmq_tc_sync_link_locked(tp);
	published_link = netif_carrier_ok(tp->ndev);
	if (tp->state != RTLMQ_STATE_OPEN ||
	    !netif_running(tp->ndev) ||
	    !netif_device_present(tp->ndev)) {
		netif_carrier_off(tp->ndev);
		rtlmq_sync_tx_subqueues(tp);
		return;
	}

	phy_status = rtlmq_read_phy_status(tp);
	link_up = rtlmq_link_is_up(tp);
	if (rtlmq_phy_is_connected(tp)) {
		speed = READ_ONCE(tp->phydev->speed);
		duplex = READ_ONCE(tp->phydev->duplex);
	}
	/*
	 * Follow phylib's negotiated enable_tx_lpi state. Hardware handles LPI;
	 * no packet-path wake or software idle timer is needed.
	 */
	eee_ret = rtlmq_set_eee_tx_lpi_locked(tp,
					      link_up && rtlmq_phy_is_connected(tp) &&
					      READ_ONCE(tp->phydev->enable_tx_lpi));
	if (eee_ret)
		netdev_warn(tp->ndev,
			    "failed to %s EEE Tx LPI: %d\n",
			    link_up && rtlmq_phy_is_connected(tp) &&
			    READ_ONCE(tp->phydev->enable_tx_lpi) ?
			    "enable" : "disable",
			    eee_ret);

	if (link_up)
		netif_carrier_on(tp->ndev);
	else
		netif_carrier_off(tp->ndev);
	rtlmq_sync_tx_subqueues(tp);

	if (link_up != published_link)
		netif_info(tp, link, tp->ndev,
			   "link %s speed=%d duplex=%s phy_status=%#02x\n",
			   link_up ? "up" : "down", speed,
			   duplex == DUPLEX_FULL ? "full" :
			   duplex == DUPLEX_HALF ? "half" : "unknown",
			   phy_status);
}

static void rtlmq_link_work(struct work_struct *work)
{
	struct rtlmq_private *tp;

	tp = container_of(work, struct rtlmq_private, link_work);

	rtnl_lock();
	mutex_lock(&tp->control_lock);
	if (READ_ONCE(tp->link_work_epoch) == READ_ONCE(tp->reset_epoch))
		rtlmq_sync_link_state(tp);
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
}

void rtlmq_request_link_state_sync(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	WRITE_ONCE(tp->link_work_epoch, READ_ONCE(tp->reset_epoch));
	schedule_work(&tp->link_work);
}

static void rtlmq_adjust_link(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	rtlmq_request_link_state_sync(tp);
}

int rtlmq_prepare_phy(struct rtlmq_private *tp)
{
	struct pci_dev *pdev = tp->pdev;
	int ret;

	INIT_WORK(&tp->link_work, rtlmq_link_work);
	WRITE_ONCE(tp->link_work_epoch, 0);
	tp->mii_bus = NULL;
	tp->phydev = NULL;
	tp->ocp_base = RTLMQ_PHY_OCP_STD_BASE;

	if (!pdev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	ret = rtlmq_mdio_register(tp);
	if (ret)
		return ret;

	return 0;
}

int rtlmq_open_phy(struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp->phydev)
		return -ENODEV;

	if (rtlmq_phy_is_connected(tp))
		return 0;

	hw = rtlmq_chip_profile_hw(tp->profile);
	if (!hw)
		return -EOPNOTSUPP;
	ret = phy_connect_direct(tp->ndev, tp->phydev, rtlmq_adjust_link,
				 PHY_INTERFACE_MODE_GMII);
	if (ret) {
		netdev_err(tp->ndev, "open_phy failed at phy_connect_direct: %d\n",
			   ret);
		return ret;
	}

	phy_attached_info(tp->phydev);

	/*
	 * phy_connect_direct() initializes and resumes the PHY through
	 * phy_attach_direct(). Repeating those lifecycle operations here can
	 * re-enter the PHY driver immediately after a completed attach.
	 */

	ret = rtlmq_chip_configure_phy(tp);
	if (ret) {
		netdev_err(tp->ndev, "open_phy failed at init_phy: %d\n", ret);
		goto err_disconnect;
	}

	return 0;

err_disconnect:
	phy_disconnect(tp->phydev);
	return ret;
}

int rtlmq_resume_wol_phy(struct rtlmq_private *tp)
{
	struct phy_device *phydev = tp->phydev;
	int ret = 0;

	lockdep_assert_held(&tp->control_lock);
	if (!tp->wol_opts || tp->state != RTLMQ_STATE_PROBED)
		return 0;
	if (!phydev)
		return -EOPNOTSUPP;

	/*
	 * Close disconnects and may suspend the PHY. Enable wake while the
	 * caller holds a PM reference, before autosuspend can revoke MDIO.
	 * Keep phylib's power state authoritative; no second link state machine.
	 */
	mutex_lock(&phydev->lock);
	if (!phydev->drv || !phydev->drv->resume)
		ret = -EOPNOTSUPP;
	else if (phydev->suspended)
		ret = __phy_resume(phydev);
	mutex_unlock(&phydev->lock);
	return ret;
}

void rtlmq_start_phy(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_phy_is_connected(tp))
		return;

	/*
	 * Configure the PHY before final MAC setup, but start its state machine
	 * only after OPEN. phylib sets carrier before adjust_link(); starting
	 * earlier could report link-up before rings, IRQs, and TX are ready.
	 * Keep connect/init/config in rtlmq_open_phy().
	 */
	if (!phy_is_started(tp->phydev))
		phy_start(tp->phydev);

	rtlmq_request_link_state_sync(tp);
}

void rtlmq_close_phy(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	if (rtlmq_phy_is_connected(tp)) {
		if (phy_is_started(tp->phydev))
			phy_stop(tp->phydev);
		phy_disconnect(tp->phydev);
	}
}

void rtlmq_cleanup_phy(struct rtlmq_private *tp)
{
	cancel_work_sync(&tp->link_work);
	WRITE_ONCE(tp->link_work_epoch, 0);
	tp->phydev = NULL;
	tp->mii_bus = NULL;
	tp->ocp_base = RTLMQ_PHY_OCP_STD_BASE;
}
