// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/pci_ids.h>

#include "rtlmq.h"
#include "rtlmq_chip.h"
#include "rtlmq_hw.h"
#include "rtlmq_diag.h"
#include "rtlmq_regs.h"

#define RTLMQ_8126_EEPROM_CLOCK_US	3
#define RTLMQ_8126_EEPROM_EEM1		BIT(7)
#define RTLMQ_8126_EEPROM_EECS		BIT(3)
#define RTLMQ_8126_EEPROM_EESK		BIT(2)
#define RTLMQ_8126_EEPROM_EEDI		BIT(1)
#define RTLMQ_8126_EEPROM_EEDO		BIT(0)

static void rtlmq_8126_eeprom_clock(struct rtlmq_private *tp, u8 *cfg)
{
	*cfg |= RTLMQ_8126_EEPROM_EESK;
	writeb(*cfg, tp->mmio_addr + RTLMQ_CFG9346);
	udelay(RTLMQ_8126_EEPROM_CLOCK_US);
	*cfg &= ~RTLMQ_8126_EEPROM_EESK;
	writeb(*cfg, tp->mmio_addr + RTLMQ_CFG9346);
	udelay(RTLMQ_8126_EEPROM_CLOCK_US);
}

static int rtlmq_8126_eeprom_size(u16 magic, bool extended)
{
	if (magic != 0x8128 && magic != 0x8129)
		return 0;
	return extended ? 256 : 128;
}

/* V7 detects serial storage via its word-zero signature, not VPD contents. */
static int rtlmq_8126_detect_eeprom_locked(struct rtlmq_private *tp)
{
	u8 cfg, original, twsi;
	int addr_bits, i;
	u32 rx_config;
	u16 magic = 0;

	/* Only the checked PCI configuration path is admitted for data reads. */
	if (!tp->pdev || !tp->pdev->bus ||
	    tp->pdev->cfg_size < RTLMQ_8126_EEPROM_VPD_DATA + sizeof(u32))
		return 0;
	lockdep_assert_held(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	twsi = readb(tp->mmio_addr + RTLMQ_8126_EEPROM_TWSI);
	rx_config = readl(tp->mmio_addr + RTLMQ_RX_CONFIG);
	original = readb(tp->mmio_addr + RTLMQ_CFG9346);
	if (twsi == U8_MAX || rx_config == U32_MAX || original == U8_MAX)
		return -ENODEV;
	if (twsi & BIT(2))
		return 0;

	addr_bits = rx_config & BIT(6) ? 8 : 6;
	cfg = RTLMQ_8126_EEPROM_EEM1 | RTLMQ_8126_EEPROM_EECS;
	writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
	/* READ opcode 110, followed by the all-zero word address. */
	for (i = 2; i >= 0; i--) {
		if (6 & BIT(i))
			cfg |= RTLMQ_8126_EEPROM_EEDI;
		else
			cfg &= ~RTLMQ_8126_EEPROM_EEDI;
		writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
		udelay(RTLMQ_8126_EEPROM_CLOCK_US);
		rtlmq_8126_eeprom_clock(tp, &cfg);
	}
	cfg &= ~RTLMQ_8126_EEPROM_EEDI;
	writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
	for (i = 0; i < addr_bits; i++)
		rtlmq_8126_eeprom_clock(tp, &cfg);
	for (i = 0; i < 16; i++) {
		u8 bit;

		cfg |= RTLMQ_8126_EEPROM_EESK;
		writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
		udelay(RTLMQ_8126_EEPROM_CLOCK_US);
		bit = readb(tp->mmio_addr + RTLMQ_CFG9346);
		if (bit == U8_MAX)
			break;
		magic = (magic << 1) | (bit & RTLMQ_8126_EEPROM_EEDO);
		cfg &= ~RTLMQ_8126_EEPROM_EESK;
		writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
		udelay(RTLMQ_8126_EEPROM_CLOCK_US);
	}
	cfg &= ~(RTLMQ_8126_EEPROM_EEDI | RTLMQ_8126_EEPROM_EECS);
	writeb(cfg, tp->mmio_addr + RTLMQ_CFG9346);
	rtlmq_8126_eeprom_clock(tp, &cfg);
	writeb(original, tp->mmio_addr + RTLMQ_CFG9346);
	if (i != 16)
		return -ENODEV;
	return rtlmq_8126_eeprom_size(magic, addr_bits == 8);
}

static int rtlmq_8126_wait_link_list(struct rtlmq_private *tp)
{
	unsigned int i;
	u16 status;

	/* r8126 10.018.00 rtl8126_wait_ll_share_fifo_ready. */
	for (i = 0; i < 10; i++) {
		fsleep(100);
		if (!rtlmq_mmio_ordinary_authorized(tp))
			return -ENODEV;
		status = readw(tp->mmio_addr + RTLMQ_8126_LINK_LIST_STATUS);
		if (status == U16_MAX)
			return -ENODEV;
		if (status & RTLMQ_8126_LINK_LIST_READY)
			return 0;
	}
	return -ETIMEDOUT;
}

static int
rtlmq_8126_bootstrap_values(struct rtlmq_private *tp, u16 *nf, u16 *ne)
{
	u32 mtu;

	if (!tp || !tp->ndev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (tp->config_method != RTLMQ_CFG_METHOD_2 &&
	    tp->config_method != RTLMQ_CFG_METHOD_3)
		return -EOPNOTSUPP;

	mtu = tp->ndev->mtu;
	/* Validate the arithmetic before the first programming operation. */
	if (!mtu || mtu > ((U16_MAX + 1U) * 16U - 1U - 4947U) / 2U)
		return -ERANGE;
	if (mtu < 9500) {
		*nf = (14447 + mtu) >> 4;
		*ne = 0x107c;
	} else {
		*nf = (4947 + 2 * mtu) >> 4;
		*ne = mtu < 77033 ? (77033 - mtu) >> 4 : 0;
	}
	return 0;
}

/*
 * Reset-only portion of V1 rtl8126_exit_oob, with ENABLE_LIB_SUPPORT off.
 * The lifecycle caller must have completed CmdReset and drained callbacks;
 * ChipCmd below is a guard against live use, not proof of DMA retirement.
 * Keep these MTU-derived thresholds instead of the reference's single-queue values.
 */
static int rtlmq_8126_reinitialize_link_list(struct rtlmq_private *tp, u16 nf,
					     u16 ne)
{
	u8 chipcmd, mcu;
	int ret;

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	if (chipcmd & (RTLMQ_CHIP_CMD_RESET | RTLMQ_CHIP_CMD_TX_ENABLE |
		       RTLMQ_CHIP_CMD_RX_ENABLE | RTLMQ_CHIP_CMD_STOP_REQ))
		return -EBUSY;
	mcu = readb(tp->mmio_addr + RTLMQ_MCU);
	if (mcu == U8_MAX)
		return -ENODEV;
	writeb(mcu & ~RTLMQ_MCU_NOW_IS_OOB, tp->mmio_addr + RTLMQ_MCU);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_LINK_LIST_CTRL,
					   RTLMQ_MAC_OCP_LINK_LIST_OOB, 0);
	if (ret)
		return ret;
	ret = rtlmq_8126_wait_link_list(tp);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_FIFO_NF, nf);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_FIFO_NE, ne);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_LINK_LIST_REINIT,
					  0x5555);
	if (ret)
		return ret;
	return rtlmq_8126_wait_link_list(tp);
}

static int rtlmq_8126_clear_phy_ups(struct rtlmq_private *tp, u16 reg, u16 mask)
{
	int value = rtlmq_phy_ocp_read(tp, reg);

	/* These registers belong to bootstrap until phylib is started. */
	if (value < 0)
		return value;
	return rtlmq_phy_ocp_write(tp, reg, value & ~mask);
}

static int rtlmq_8126_exit_ups(struct rtlmq_private *tp)
{
	unsigned int i;
	int value, ret;

	value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_8126_MAC_OCP_UPS);
	if (value < 0)
		return value;
	if (!(value & RTLMQ_8126_MAC_OCP_UPS_RESUME) &&
	    !tp->phy_ups_cleanup_pending)
		return 0;
	/*
	 * V1 clears the MAC flag before the PHY controls. A failed PHY access
	 * must not make the next attempt skip the still-required cleanup.
	 * Recheck PHY readiness on every attempt, including after MAC reset.
	 */
	tp->phy_ups_cleanup_pending = true;
	/* V1 wait_phy_ups_resume: do not clear resume state on exhaustion. */
	for (i = 0; i < 100; i++) {
		value = rtlmq_phy_ocp_read(tp, RTLMQ_8126_PHY_OCP_STATE);
		if (value < 0)
			return value;
		if ((value & RTLMQ_8126_PHY_STATE_MASK) ==
		    RTLMQ_8126_PHY_STATE_UPS_READY)
			break;
		fsleep(1000);
	}
	if (i == 100)
		return -ETIMEDOUT;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_8126_MAC_OCP_UPS,
					   RTLMQ_8126_MAC_OCP_UPS_RESUME, 0);
	if (ret)
		return ret;
	ret = rtlmq_8126_clear_phy_ups(tp, RTLMQ_8126_PHY_OCP_UPS_CTRL, BIT(0));
	if (ret)
		return ret;
	ret = rtlmq_8126_clear_phy_ups(tp, RTLMQ_8126_PHY_OCP_UPS_CONFIG,
				       BIT(3) | BIT(1));
	if (!ret)
		tp->phy_ups_cleanup_pending = false;
	return ret;
}

static int rtlmq_8126_prepare_clocks(struct rtlmq_private *tp)
{
	u8 config, int_cfg0, config5, value;
	bool unlock;
	int ret;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	/*
	 * V1 hw_init: disable the ASPM/CLKREQ locks and forced CLKREQ before
	 * disabling UPS. Preserve the caller's configuration-write ownership.
	 * Read protected bytes before unlocking, so a failed read cannot leak
	 * write access. These writes do not enable either DMA engine.
	 */
	config = readb(tp->mmio_addr + RTLMQ_CFG9346);
	if (config == U8_MAX)
		return -ENODEV;
	int_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
	if (int_cfg0 == U8_MAX)
		return -ENODEV;
	config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
	if (config5 == U8_MAX)
		return -ENODEV;
	unlock = (config & RTLMQ_CFG9346_UNLOCK) != RTLMQ_CFG9346_UNLOCK;
	if (unlock)
		rtlmq_unlock_config_regs(tp);
	writeb(int_cfg0 & ~RTLMQ_INT_CFG0_CLKREQEN,
	       tp->mmio_addr + RTLMQ_INT_CFG0);
	writeb(config5 & ~RTLMQ_CONFIG5_ASPM_ENABLE,
	       tp->mmio_addr + RTLMQ_CONFIG5);
	if (unlock)
		rtlmq_lock_config_regs(tp);

	value = readb(tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value & ~RTLMQ_8126_FORCE_CLKREQ,
	       tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_8126_MAC_OCP_UPS_ENABLE,
					   RTLMQ_8126_UPS_ENABLE, 0);
	if (ret)
		return ret;

	/* V1 powerup_pll. phylib owns PHY resume and negotiation after this. */
	value = readb(tp->mmio_addr + RTLMQ_PMCH);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value | RTLMQ_8126_PLL_UP, tp->mmio_addr + RTLMQ_PMCH);
	return 0;
}

int rtlmq_configure_mac_8126(struct rtlmq_private *tp)
{
	u8 command, config;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	lockdep_assert_held(&tp->control_lock);
	/* hw_start owns the configuration window; never open it implicitly. */
	command = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (command == U8_MAX)
		return -ENODEV;
	if (command & (RTLMQ_CHIP_CMD_TX_ENABLE | RTLMQ_CHIP_CMD_RX_ENABLE |
		       RTLMQ_CHIP_CMD_RESET))
		return -EBUSY;
	config = readb(tp->mmio_addr + RTLMQ_CFG9346);
	if (config == U8_MAX)
		return -ENODEV;
	if ((config & RTLMQ_CFG9346_UNLOCK) != RTLMQ_CFG9346_UNLOCK)
		return -EACCES;

	/*
	 * K1 rtl_set_eee_txidle_timer and V1 rtl8126_set_eee_lpi_timer use the
	 * shared 16-bit byte timer. Start MAC LPI disabled; negotiated link state
	 * owns the later E040[1:0] transition.
	 */
	ret = rtlmq_program_eee_mac_locked(tp);
	if (ret)
		return ret;
	/* V1 rtl8126_hw_config keeps only magic-packet wake before RX setup. */
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_WOL_MAGIC_OCP_REG,
					   (u16)~RTLMQ_WOL_MAGIC_BIT, 0);
	if (ret)
		return ret;
	ret = rtlmq_disable_double_vlan(tp);
	if (ret)
		return ret;
	/* V1 preserves 70f[7:6]; K1's full-byte RTL8125 setting is separate. */
	ret = rtlmq_pci_config_modify(tp, RTLMQ_PCI_ASPM_LATENCY,
				      0x3f000000, 0x27000000);
	if (ret)
		return ret;
	ret = rtlmq_pci_config_modify(tp, RTLMQ_8126_PCI_L1_TIMEOUT,
				      RTLMQ_8126_L1_TIMEOUT_ENABLE, 0);
	if (ret)
		return ret;
	/* The late family latch owns descriptor mode and the sole MCU pulse. */
	return rtlmq_configure_mac_common(tp);
}

static int rtlmq_8126_wait_fifo(struct rtlmq_private *tp)
{
	unsigned int i;
	int value;

	/* K1 rtl_hw_start_8125_common for MAC_VER_70: 10 reads, 1 ms apart. */
	for (i = 0; i < 10; i++) {
		value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_8126_MAC_OCP_FIFO_STATUS);
		if (value < 0)
			return value;
		if (!(value & RTLMQ_8126_FIFO_BUSY))
			return 0;
		fsleep(1000);
	}
	return -ETIMEDOUT;
}

static int rtlmq_8126_program_tx_engine(struct rtlmq_private *tp)
{
	u8 value;
	int ret;

	/*
	 * V1 rtl8126_hw_config, descriptor-close branch. Do not substitute K1's
	 * legacy-descriptor E614/E0C0/D430 values into this TX-V3 sequence.
	 * TxConfig, descriptor bases and queue counts precede the MCU pulse.
	 */
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_TX_DESC_FORMAT_OCP_REG,
					   RTLMQ_8126_TX_DESC_FORMAT_MASK,
					   RTLMQ_8126_TX_DESC_V3);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_8126_MAC_OCP_TX_MODE,
					   RTLMQ_8126_TX_MODE_MASK,
					   RTLMQ_8126_TX_MODE_CLOSE);
	if (ret)
		return ret;
	ret = rtlmq_prepare_tx_engine_fifo(tp, true);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_VENDOR_E0C0,
					  RTLMQ_8126_TX_FIFO_CONFIG);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_TX_ENGINE_E052, 0,
					   BIT(6) | BIT(5));
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_TX_ENGINE_E052, BIT(3) | BIT(7),
					   0);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp,
					   RTLMQ_MAC_OCP_VENDOR_TX_ENGINE_D430,
					   0x0fff, RTLMQ_8126_TX_ENGINE_CONFIG);
	if (ret)
		return ret;
	value = readb(tp->mmio_addr + RTLMQ_8126_MISC_D0);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value | BIT(6), tp->mmio_addr + RTLMQ_8126_MISC_D0);
	value = readb(tp->mmio_addr + RTLMQ_PPSW);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value | BIT(6), tp->mmio_addr + RTLMQ_PPSW);
	value = readb(tp->mmio_addr + RTLMQ_8126_MISC_D0);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value | BIT(7), tp->mmio_addr + RTLMQ_8126_MISC_D0);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_E080, BIT(1), 0);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EA1C,
					   BIT(2) | BIT(9) | BIT(8), 0);
	if (ret)
		return ret;
	ret = rtlmq_finish_tx_engine_fifo(tp);
	if (ret)
		return ret;
	ret = rtlmq_8126_wait_fifo(tp);
	if (ret)
		return ret;
	/* V1 releases only F2[3]; RTL8125 owns its additional MISC gate. */
	return rtlmq_clear_rxdv_gate(tp);
}

/* The caller owns configuration-register write access. */
static void rtlmq_8126_disable_pm_controls(struct rtlmq_private *tp,
					   u8 force_clkreq, u8 int_cfg0,
					   u8 config5)
{
	writeb(force_clkreq & ~RTLMQ_8126_FORCE_CLKREQ,
	       tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	writeb(int_cfg0 & ~RTLMQ_INT_CFG0_CLKREQEN,
	       tp->mmio_addr + RTLMQ_INT_CFG0);
	writeb(config5 & ~RTLMQ_CONFIG5_ASPM_ENABLE,
	       tp->mmio_addr + RTLMQ_CONFIG5);
}

static int rtlmq_8126_finalize_pm(struct rtlmq_private *tp)
{
	u8 read_force_clkreq, read_int_cfg0, read_config5;
	u8 expected_int_cfg0, expected_config5;
	u8 force_clkreq, int_cfg0, config5;

	/*
	 * K1 enables INT_CFG0[3]/Config5[0] only alongside its LTR and
	 * E092/E094 setup. PCI admission attempts to disable L1, but may fail.
	 * Keep chip-side ASPM disabled without its full setup.
	 * Read all bytes before changing the pair.
	 */
	force_clkreq = readb(tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	if (force_clkreq == U8_MAX)
		return -ENODEV;
	int_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
	if (int_cfg0 == U8_MAX)
		return -ENODEV;
	config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
	if (config5 == U8_MAX)
		return -ENODEV;

	expected_int_cfg0 = int_cfg0 & ~RTLMQ_INT_CFG0_CLKREQEN;
	expected_config5 = config5 & ~RTLMQ_CONFIG5_ASPM_ENABLE;

	rtlmq_8126_disable_pm_controls(tp, force_clkreq, int_cfg0, config5);

	read_force_clkreq = readb(tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	read_int_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
	read_config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
	if (read_force_clkreq == U8_MAX || read_int_cfg0 == U8_MAX ||
	    read_config5 == U8_MAX)
		return -ENODEV;
	if (read_force_clkreq !=
	    (force_clkreq & ~RTLMQ_8126_FORCE_CLKREQ) ||
	    read_int_cfg0 != expected_int_cfg0 ||
	    read_config5 != expected_config5) {
		/*
		 * A rejected final policy must remain safely disabled. Preserve
		 * unrelated readback bits rather than restoring inherited policy.
		 */
		rtlmq_8126_disable_pm_controls(tp, read_force_clkreq,
					       read_int_cfg0, read_config5);
		return -EIO;
	}

	return 0;
}

int rtlmq_prepare_d3_8126(struct rtlmq_private *tp)
{
	u8 config, force_clkreq, int_cfg0, config5;
	bool unlock;

	lockdep_assert_held(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/* V1 hw_d3_para drops these locks and the exit-L1 event mask in D3. */
	config = readb(tp->mmio_addr + RTLMQ_CFG9346);
	force_clkreq = readb(tp->mmio_addr + RTLMQ_8126_CLKREQ_CTRL);
	int_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
	config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
	if (config == U8_MAX || force_clkreq == U8_MAX ||
	    int_cfg0 == U8_MAX || config5 == U8_MAX)
		return -ENODEV;

	unlock = (config & RTLMQ_CFG9346_UNLOCK) != RTLMQ_CFG9346_UNLOCK;
	if (unlock)
		rtlmq_unlock_config_regs(tp);
	rtlmq_8126_disable_pm_controls(tp, force_clkreq, int_cfg0, config5);
	if (unlock)
		rtlmq_lock_config_regs(tp);

	return rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_C0AC,
					    GENMASK(12, 7), 0);
}

int rtlmq_latch_tx_engine_8126(struct rtlmq_private *tp, const char *where)
{
	u8 command, config;
	u32 tx_config;
	bool unlock;
	int ret;

	/* The selected-family gateway holds control_lock and checks authority. */
	command = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (command == U8_MAX)
		return -ENODEV;
	tx_config = readl(tp->mmio_addr + RTLMQ_TX_CONFIG);
	if (tx_config == U32_MAX)
		return -ENODEV;
	config = readb(tp->mmio_addr + RTLMQ_CFG9346);
	if (config == U8_MAX)
		return -ENODEV;
	if (command & (RTLMQ_CHIP_CMD_TX_ENABLE | RTLMQ_CHIP_CMD_RX_ENABLE |
		       RTLMQ_CHIP_CMD_RESET))
		return -EBUSY;
	if (tx_config & RTLMQ_8126_TX_NO_CLOSE)
		return -EINVAL;
	unlock = (config & RTLMQ_CFG9346_UNLOCK) != RTLMQ_CFG9346_UNLOCK;
	if (unlock)
		rtlmq_unlock_config_regs(tp);
	ret = rtlmq_8126_program_tx_engine(tp);
	if (!ret)
		ret = rtlmq_validate_tx_engine_queue_count_latch(tp, where);
	if (!ret)
		ret = rtlmq_8126_finalize_pm(tp);
	if (unlock && ret != -ENODEV)
		rtlmq_lock_config_regs(tp);
	return ret;
}

int rtlmq_8126_admit_board_mode(struct rtlmq_private *tp)
{
	int value;

	if (tp->config_method == RTLMQ_CFG_METHOD_2)
		return 0;
	if (tp->config_method != RTLMQ_CFG_METHOD_3)
		return -EOPNOTSUPP;

	value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_8126_MAC_OCP_BOARD_MODE);
	if (value < 0)
		return value;
	return (value & U8_MAX) == RTLMQ_8126_BOARD_MODE_RTL8251B ?
	       -EOPNOTSUPP : 0;
}

int rtlmq_hw_prepare_8126(struct rtlmq_private *tp)
{
	u16 nf, ne;
	int ret;

	ret = rtlmq_8126_bootstrap_values(tp, &nf, &ne);
	if (ret)
		return ret;
	/* A failed reset must not leave a previously detected size advertised. */
	tp->eeprom_len = 0;
	/* Recheck the probe admission before any open-time copper setup. */
	ret = rtlmq_8126_admit_board_mode(tp);
	if (ret)
		return ret;
	/* V1 exit_oob: filter, RealWoW off, reset, link list, then UPS exit. */
	ret = rtlmq_clear_rx_acceptance(tp);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_8126_MAC_OCP_REALWOW, 0x00ff);
	if (ret)
		return ret;
	ret = rtlmq_hw_reset(tp, "hw_init");
	if (ret)
		return ret;
	/* Probe-time MAC reads precede reset; detect only in the reset owner. */
	ret = rtlmq_8126_detect_eeprom_locked(tp);
	if (ret < 0)
		return ret;
	tp->eeprom_len = ret;
	ret = rtlmq_8126_reinitialize_link_list(tp, nf, ne);
	if (ret)
		return ret;
	ret = rtlmq_8126_exit_ups(tp);
	if (ret)
		return ret;
	return rtlmq_8126_prepare_clocks(tp);
}

/*
 * Passive direct-MMIO state used by the RTL8126 setup paths. K1 and V1 read
 * the legacy MAC window for diagnostics; this narrower allowlist excludes
 * interrupt status/masks, doorbells, DMA addresses/cursors, and every indirect
 * PHY/OCP/CSI command window.
 */
static const struct rtlmq_regdump_register rtlmq_8126a_reg_dump[] = {
	{ RTLMQ_INT_CFG0, sizeof(u8) },
	{ RTLMQ_CHIP_CMD, sizeof(u8) },
	{ RTLMQ_TX_CONFIG, sizeof(u32) },
	{ RTLMQ_RX_CONFIG, sizeof(u32) },
	{ RTLMQ_CFG9346, sizeof(u8) },
	{ RTLMQ_CONFIG1, sizeof(u8) },
	{ RTLMQ_CONFIG2, sizeof(u8) },
	{ RTLMQ_CONFIG3, sizeof(u8) },
	{ RTLMQ_CONFIG5, sizeof(u8) },
	{ RTLMQ_PHY_STATUS, sizeof(u8) },
	{ RTLMQ_PMCH, sizeof(u8) },
	{ RTLMQ_8126_MISC_D0, sizeof(u8) },
	{ RTLMQ_8126_LINK_LIST_STATUS, sizeof(u16) },
	{ RTLMQ_RX_MAX_SIZE, sizeof(u16) },
	{ RTLMQ_CPLUS_CMD, sizeof(u16) },
	{ RTLMQ_INTR_MITIGATE, sizeof(u16) },
	{ RTLMQ_8126_CLKREQ_CTRL, sizeof(u8) },
	{ RTLMQ_PPSW, sizeof(u8) },
	{ RTLMQ_DOUBLE_VLAN_CONFIG, sizeof(u16) },
	{ RTLMQ_VENDOR_TX_ENGINE_MMIO_1880, sizeof(u16) },
	{ RTLMQ_RSS_CTRL, sizeof(u32) },
	{ RTLMQ_Q_NUM_CTRL, sizeof(u16) },
	{ RTLMQ_EEE_TXIDLE_TIMER, sizeof(u16) },
};

/* V5 disable programs and verifies the whole a00..a7c bank and cfg0/cfg1. */
#define RTLMQ_8126_MOD_SLOT(_i) \
	{ RTLMQ_INT_MITI_V2_0_RX + (_i) * sizeof(u32), sizeof(u32) }
static const struct rtlmq_regdump_register rtlmq_8126a_moderation[] = {
	RTLMQ_8126_MOD_SLOT(0), RTLMQ_8126_MOD_SLOT(1),
	RTLMQ_8126_MOD_SLOT(2), RTLMQ_8126_MOD_SLOT(3),
	RTLMQ_8126_MOD_SLOT(4), RTLMQ_8126_MOD_SLOT(5),
	RTLMQ_8126_MOD_SLOT(6), RTLMQ_8126_MOD_SLOT(7),
	RTLMQ_8126_MOD_SLOT(8), RTLMQ_8126_MOD_SLOT(9),
	RTLMQ_8126_MOD_SLOT(10), RTLMQ_8126_MOD_SLOT(11),
	RTLMQ_8126_MOD_SLOT(12), RTLMQ_8126_MOD_SLOT(13),
	RTLMQ_8126_MOD_SLOT(14), RTLMQ_8126_MOD_SLOT(15),
	RTLMQ_8126_MOD_SLOT(16), RTLMQ_8126_MOD_SLOT(17),
	RTLMQ_8126_MOD_SLOT(18), RTLMQ_8126_MOD_SLOT(19),
	RTLMQ_8126_MOD_SLOT(20), RTLMQ_8126_MOD_SLOT(21),
	RTLMQ_8126_MOD_SLOT(22), RTLMQ_8126_MOD_SLOT(23),
	RTLMQ_8126_MOD_SLOT(24), RTLMQ_8126_MOD_SLOT(25),
	RTLMQ_8126_MOD_SLOT(26), RTLMQ_8126_MOD_SLOT(27),
	RTLMQ_8126_MOD_SLOT(28), RTLMQ_8126_MOD_SLOT(29),
	RTLMQ_8126_MOD_SLOT(30), RTLMQ_8126_MOD_SLOT(31),
#undef RTLMQ_8126_MOD_SLOT
	{ RTLMQ_INT_CFG0, sizeof(u8) },
	{ RTLMQ_INT_CFG1, sizeof(u16) },
};

static_assert(ARRAY_SIZE(rtlmq_8126a_moderation) ==
	      RTLMQ_REG_DUMP_MAX_MODERATION_SLOTS);
static_assert(RTLMQ_INT_MITI_V2_0_RX + 32 * sizeof(u32) ==
	      RTLMQ_INT_MITI_V5_BANK_END);

#define RTLMQ_8126_REG_DUMP_ENTRY_COUNT \
	ARRAY_SIZE(rtlmq_8126a_reg_dump)

static_assert(sizeof(struct rtlmq_reg_dump_header) +
	      RTLMQ_8126_REG_DUMP_ENTRY_COUNT *
		      sizeof(struct rtlmq_reg_dump_entry) <= U16_MAX);
static_assert(RTLMQ_8126_REG_DUMP_ENTRY_COUNT <= U16_MAX);
static_assert(RTLMQ_EEE_TXIDLE_TIMER + sizeof(u16) <=
	      RTLMQ_MMIO_REQUIRED_SIZE);

const struct rtlmq_regdump_schema rtlmq_8126a_regdump_schema = {
	.registers = rtlmq_8126a_reg_dump,
	.register_count = RTLMQ_8126_REG_DUMP_ENTRY_COUNT,
	.moderation = rtlmq_8126a_moderation,
	.moderation_count = ARRAY_SIZE(rtlmq_8126a_moderation),
};

/*
 * r8169 at d7e5db215839 and r8126 10.018.00 identify A2/A3
 * separately. Bit 31 of TxConfig is outside the RTL8126 XID match mask.
 * Unknown IC versions must not inherit the vendor's method-3 fallback.
 *
 * PCI aliases alone never authorize hardware access.
 */
#define RTLMQ_8126_MATCH(device, version, method) {		\
	.pci_vendor = PCI_VENDOR_ID_REALTEK,			\
	.pci_device = (device),					\
	.xid = (version),					\
	.xid_mask = RTLMQ_8126_TX_CONFIG_XID_MASK,		\
	.accepted_config_methods = BIT(RTLMQ_CFG_METHOD_##method),	\
}

const struct rtlmq_chip_match rtlmq_8126_matches[RTLMQ_8126_MATCH_COUNT] = {
	[RTLMQ_8126_A2] = RTLMQ_8126_MATCH(0x8126, 0x649, 2),
	[RTLMQ_8126_A3] = RTLMQ_8126_MATCH(0x8126, 0x64a, 3),
	[RTLMQ_5000_A2] = RTLMQ_8126_MATCH(0x5000, 0x649, 2),
	[RTLMQ_5000_A3] = RTLMQ_8126_MATCH(0x5000, 0x64a, 3),
};

static const struct rtlmq_hw_contract rtlmq_8126a_hw = {
	.mac_version = RTLMQ_MAC_8126A,
	.rss_rx_desc_fmt = RX_DESC_FMT_V4,
	.tx_desc_fmt = RTLMQ_TX_DESC_FORMAT_V3,
	.rss_version = RTLMQ_RSS_VERSION_5,
	.irq_moderation_version = RTLMQ_IRQ_MODERATION_V5,
	.eee_tx_lpi_layout = RTLMQ_EEE_TX_LPI_E040,
	.max_rx_queues = 4,
	.max_tx_queues = 2,
	.max_irq_vectors = 22,
	.max_runtime_vectors = 4,
	.requires_tx_engine_runtime_latch = true,
	.has_q1_tx_desc_base_regs = true,
	.has_msi = true,
	.has_msix = true,
	.has_rss = true,
	.has_msix_message_routing = true,
	.requires_conservative_aspm = true,
	.msix_rx_message_ids = { 0, 1, 2, 3 },
	.msix_tx_message_ids = { 0, 1 },
	.msix_link_message_id = 21,
	.msix_int_cfg0_clear_mask = 0,
};

static const struct rtlmq_topology rtlmq_8126a_topologies[] = {
	{
		.rx_queues = 1,
		.tx_queues = 1,
		.vectors = 1,
		.routing = RTLMQ_ROUTING_SINGLE_VECTOR,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_INTX) |
			     BIT(RTLMQ_IRQ_MODE_MSI),
	},
	{
		.rx_queues = 2,
		.tx_queues = 1,
		.vectors = 2,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 4,
		.tx_queues = 1,
		.vectors = 4,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 2,
		.tx_queues = 2,
		.vectors = 1,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 4,
		.tx_queues = 2,
		.vectors = 3,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
};

/*
 * PTP RX and CBS remain unavailable without complete hardware contracts.
 */
static const struct rtlmq_qualified_policy rtlmq_8126a_qualified = {
	.base_netdev_features = NETIF_F_SG | NETIF_F_IP_CSUM |
				NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM |
				NETIF_F_TSO | NETIF_F_TSO6,
	.ptp_backend = RTLMQ_PTP_BACKEND_PHY_V2,
	.topologies = rtlmq_8126a_topologies,
	.default_topology = &rtlmq_8126a_topologies[2],
	.num_topologies = ARRAY_SIZE(rtlmq_8126a_topologies),
	.max_xsk_tx_queues = 2,
	.supports_rss = true,
	.supports_xdp = true,
	.supports_af_xdp = true,
	.supports_tc = true,
	.supports_vlan_ctag_tx = true,
	.supports_vlan_ctag_rx = true,
	.supports_rx_fcs = true,
	.supports_rx_all = true,
	.wol_opts = WAKE_MAGIC,
	.wol_link_v3 = true,
	.supports_highdma = true,
	.supports_hw_tally = true,
	.supports_eee = true,
	.supports_leds = true,
	.supports_live_mac_change = true,
	.supports_inplace_ring_resize = true,
};

static const struct rtlmq_chip_ops rtlmq_8126a_ops = {
	.prepare_mac = rtlmq_hw_prepare_8126,
	.prepare_reset = rtlmq_prepare_mac_reset,
	.configure_mac = rtlmq_configure_mac_8126,
	.configure_phy = rtlmq_configure_phy_8126,
	.latch_tx_engine = rtlmq_latch_tx_engine_8126,
};

#define RTLMQ_8126_PROFILE(_name, _match, _firmware) {	\
	.name = (_name),					\
	.match = &rtlmq_8126_matches[_match],			\
	.hw = &rtlmq_8126a_hw,					\
	.qualified = &rtlmq_8126a_qualified,			\
	.ops = &rtlmq_8126a_ops,				\
	.firmware = (_firmware),				\
	.regdump = &rtlmq_8126a_regdump_schema,			\
}

const struct rtlmq_chip_profile rtlmq_8126a2_profile =
	RTLMQ_8126_PROFILE("RTL8126A (A2)", RTLMQ_8126_A2,
			   &rtlmq_8126a2_firmware);
const struct rtlmq_chip_profile rtlmq_8126a3_profile =
	RTLMQ_8126_PROFILE("RTL8126A (A3)", RTLMQ_8126_A3,
			   &rtlmq_8126a3_firmware);
const struct rtlmq_chip_profile rtlmq_5000a2_profile =
	RTLMQ_8126_PROFILE("RTL8126A (A2, 5000 alias)", RTLMQ_5000_A2,
			   &rtlmq_8126a2_firmware);
const struct rtlmq_chip_profile rtlmq_5000a3_profile =
	RTLMQ_8126_PROFILE("RTL8126A (A3, 5000 alias)", RTLMQ_5000_A3,
			   &rtlmq_8126a3_firmware);

#undef RTLMQ_8126_PROFILE
