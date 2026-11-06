// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include "rtlmq.h"
#include "rtlmq_diag.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_hw.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"

#define RTLMQ_PTP_HW_WAIT_MS		20
#define RTLMQ_PTP_HW_ATOMIC_WAIT_US	500
#define RTLMQ_PTP_PHY_CMD_WAIT_US	20000
#define RTLMQ_PTP_PHY_CMD_ATOMIC_US	500
#define RTLMQ_PTP_PHY_LINK_SETTLE_MS	5000

#define RTLMQ_PHY_PTP_CLOCK_EXEC	BIT(0)
#define RTLMQ_PHY_PTP_CLOCK_MODE_MASK	GENMASK(3, 1)

#define RTLMQ_PHY_PTP_ISR_RX		BIT(2)
#define RTLMQ_PHY_PTP_GEN_W1C		BIT(0)
#define RTLMQ_PHY_PTP_TRX_BUSY		BIT(0)
#define RTLMQ_PHY_PTP_TRX_MSG_MASK	GENMASK(3, 2)
#define RTLMQ_PHY_PTP_PATH_APPLY	BIT(15)

/*
 * Realtek 9.018.00's RTL8125B PHY-v3 and 10.018.00's RTL8126A PHY-v2 use
 * this register bank with different detector control/status policy. Keep that
 * policy in the immutable backend contract. Neither backend admits SyncE or
 * one-step modes. Neither backend provides a Linux PPS event source.
 */
#define RTLMQ_PHY_PTP_CTRL_CLOCK_ENABLE	BIT(0)
#define RTLMQ_PHY_PTP_CTRL_PACKET_ENABLE	(GENMASK(7, 0) | BIT(12))
#define RTLMQ_PHY_PTP_V2_CTRL_PACKET_ENABLE	(GENMASK(5, 0) | BIT(12))

static const struct rtlmq_ptp_backend_contract rtlmq_ptp_backends[] = {
	[RTLMQ_PTP_BACKEND_PHY_V2] = {
		.type = RTLMQ_PTP_BACKEND_PHY_V2,
		.idle_ctrl = 0,
		.packet_ctrl = RTLMQ_PHY_PTP_V2_CTRL_PACKET_ENABLE,
		.tx_status_mac_ocp = true,
		.status_w1c = false,
		/*
		 * V5 reads one of four shared message-type slots after parsing
		 * the delivered packet, but provides no packet/sequence identity
		 * or overwrite indication. Do not publish unprovable association.
		 */
		.supports_rx = false,
		.clear_mac_timestamp_source = false,
	},
	[RTLMQ_PTP_BACKEND_PHY_V3] = {
		.type = RTLMQ_PTP_BACKEND_PHY_V3,
		.idle_ctrl = RTLMQ_PHY_PTP_CTRL_CLOCK_ENABLE,
		.packet_ctrl = RTLMQ_PHY_PTP_CTRL_PACKET_ENABLE,
		.tx_status_mac_ocp = false,
		.status_w1c = true,
		.supports_rx = true,
		.clear_mac_timestamp_source = true,
	},
};

const struct rtlmq_ptp_backend_contract *
rtlmq_ptp_backend_contract(enum rtlmq_ptp_backend type)
{
	const struct rtlmq_ptp_backend_contract *backend;

	if (type <= RTLMQ_PTP_BACKEND_NONE ||
	    type >= ARRAY_SIZE(rtlmq_ptp_backends))
		return NULL;

	backend = &rtlmq_ptp_backends[type];
	return backend->type == type ? backend : NULL;
}

bool rtlmq_ptp_backend_matches(const struct rtlmq_private *tp,
			       const struct rtlmq_ptp *ptp)
{
	return tp && ptp && ptp->backend &&
	       ptp->backend->type == rtlmq_qualified_ptp_backend(tp);
}

static bool rtlmq_ptp_hw_try_acquire(struct rtlmq_ptp *ptp)
{
	return atomic_cmpxchg(&ptp->hw_busy, 0, 1) == 0;
}

int rtlmq_ptp_hw_acquire(struct rtlmq_ptp *ptp, bool atomic_context)
{
	unsigned int i;

	if (rtlmq_ptp_hw_try_acquire(ptp))
		return 0;

	if (atomic_context) {
		for (i = 0; i < RTLMQ_PTP_HW_ATOMIC_WAIT_US; i++) {
			udelay(1);
			if (rtlmq_ptp_hw_try_acquire(ptp))
				return 0;
		}
		return -EBUSY;
	}

	if (!wait_event_timeout(ptp->hw_wait, rtlmq_ptp_hw_try_acquire(ptp),
				msecs_to_jiffies(RTLMQ_PTP_HW_WAIT_MS)))
		return -EBUSY;

	return 0;
}

void rtlmq_ptp_hw_release(struct rtlmq_ptp *ptp)
{
	atomic_set(&ptp->hw_busy, 0);
	wake_up_all(&ptp->hw_wait);
}

int rtlmq_ptp_phy_read(struct rtlmq_ptp *ptp, u16 reg, u16 *value)
{
	int ret;

	ret = rtlmq_phy_ocp_read(ptp->tp, reg);
	if (ret < 0)
		return ret;

	*value = ret;
	return 0;
}

int rtlmq_ptp_phy_write(struct rtlmq_ptp *ptp, u16 reg, u16 value)
{
	return rtlmq_phy_ocp_write(ptp->tp, reg, value);
}

static int rtlmq_ptp_phy_modify(struct rtlmq_ptp *ptp, u16 reg,
				u16 clear_mask, u16 set_mask)
{
	u16 value;
	int ret;

	ret = rtlmq_ptp_phy_read(ptp, reg, &value);
	if (ret)
		return ret;
	value &= ~clear_mask;
	value |= set_mask;
	return rtlmq_ptp_phy_write(ptp, reg, value);
}

static int rtlmq_ptp_phy_wait_clear(struct rtlmq_ptp *ptp, u16 reg, u16 mask,
				    bool atomic_context)
{
	unsigned int timeout = atomic_context ?
		RTLMQ_PTP_PHY_CMD_ATOMIC_US : RTLMQ_PTP_PHY_CMD_WAIT_US;
	u16 value;
	int ret;

	while (timeout--) {
		ret = rtlmq_ptp_phy_read(ptp, reg, &value);
		if (ret)
			return ret;
		if (!(value & mask))
			return 0;
		if (atomic_context)
			udelay(1);
		else
			usleep_range(1, 10);
	}

	return -ETIMEDOUT;
}

int rtlmq_ptp_phy_clock_command(struct rtlmq_ptp *ptp, u16 command)
{
	int ret;

	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_CLOCK_CFG, RTLMQ_PHY_PTP_CLOCK_MODE_MASK,
				   RTLMQ_PHY_PTP_CLOCK_EXEC | command);
	if (ret)
		return ret;

	return rtlmq_ptp_phy_wait_clear(ptp, RTLMQ_PHY_PTP_CLOCK_CFG, RTLMQ_PHY_PTP_CLOCK_EXEC,
					false);
}

static int rtlmq_ptp_phy_apply_path_owned(struct rtlmq_ptp *ptp)
{
	unsigned long settle =
		msecs_to_jiffies(RTLMQ_PTP_PHY_LINK_SETTLE_MS);
	int ret;

	/*
	 * A640 bit 15 is a self-clearing path-apply command, not persistent
	 * enable state. A working detector reads back 0x3000; 0xb000 means the
	 * command is still in flight. Do not enable packets until hardware has
	 * consumed the command or its timestamp can be lost permanently.
	 *
	 * The same command raises a delayed false-down event through the MAC
	 * link interrupt. Arm the quarantine before issuing it. The deferred
	 * worker performs one phylib status read after the hardware's
	 * internal path has settled.
	 */
	WRITE_ONCE(ptp->link_quarantine_until, jiffies + settle);
	mod_delayed_work(system_dfl_wq, &ptp->link_reconcile_work, settle);

	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_PATH_CTRL, 0, RTLMQ_PHY_PTP_PATH_APPLY);
	if (ret)
		return ret;

	return rtlmq_ptp_phy_wait_clear(ptp, RTLMQ_PHY_PTP_PATH_CTRL, RTLMQ_PHY_PTP_PATH_APPLY,
					false);
}

int rtlmq_ptp_phy_trx_command(struct rtlmq_ptp *ptp, u16 command,
			      bool atomic_context)
{
	int ret;

	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_TRX_STATUS,
				   RTLMQ_PHY_PTP_TRX_RX_SELECT | RTLMQ_PHY_PTP_TRX_MSG_MASK,
				   RTLMQ_PHY_PTP_TRX_BUSY | command);
	if (ret)
		return ret;

	return rtlmq_ptp_phy_wait_clear(ptp, RTLMQ_PHY_PTP_TRX_STATUS, RTLMQ_PHY_PTP_TRX_BUSY,
					atomic_context);
}

int rtlmq_ptp_phy_read_timestamp(struct rtlmq_ptp *ptp, ktime_t *timestamp)
{
	u16 sec_mid;
	u64 seconds;
	u16 sec_hi;
	u16 sec_lo;
	u16 ns_hi;
	u16 ns_lo;
	u32 nsec;
	int ret;

	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_NS_HI, &ns_hi);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_NS_LO, &ns_lo);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_SEC_HI, &sec_hi);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_SEC_MID, &sec_mid);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_SEC_LO, &sec_lo);
	if (ret)
		return ret;

	nsec = (((u32)ns_hi << 16) | ns_lo) & RTLMQ_PTP_NSEC_MASK;
	seconds = (u64)sec_hi << 32 | (u64)sec_mid << 16 | sec_lo;
	if (nsec >= NSEC_PER_SEC || seconds >= KTIME_SEC_MAX)
		return -ERANGE;

	*timestamp = ktime_set(seconds, nsec);
	return 0;
}

int rtlmq_ptp_clear_tx_status_owned(struct rtlmq_ptp *ptp)
{
	if (ptp->backend->tx_status_mac_ocp) {
		return rtlmq_mac_ocp_write_checked(ptp->tp,
			RTLMQ_8126_PTP_STATUS_OCP,
			RTLMQ_8126_PTP_STATUS_TX);
	}

	return rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_ISR, RTLMQ_PHY_PTP_ISR_TX);
}

int rtlmq_ptp_read_tx_status_owned(struct rtlmq_ptp *ptp, u16 *status)
{
	int value;

	if (!status)
		return -EINVAL;
	if (!ptp->backend->tx_status_mac_ocp)
		return rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_ISR, status);

	value = rtlmq_mac_ocp_read_checked(ptp->tp, RTLMQ_8126_PTP_STATUS_OCP);
	if (value < 0)
		return value;
	*status = value;
	return 0;
}

int rtlmq_ptp_phy_program_modes_owned(struct rtlmq_ptp *ptp,
				      u8 old_modes, u8 modes,
				      bool phc_active)
{
	u16 ier_set = modes & RTLMQ_PTP_MODE_TX ?
		RTLMQ_PHY_PTP_ISR_TX : 0;
	int ret;

	/*
	 * SyncE is unsupported. Clear vendor E406[0] before programming any
	 * detector mode so firmware or a previous driver cannot leave clock
	 * output enabled.
	 */
	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_SYNCE_CTRL, BIT(0), 0);
	if (ret)
		return ret;

	if (!phc_active || !modes) {
		ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_IER,
					   RTLMQ_PHY_PTP_ISR_RX | RTLMQ_PHY_PTP_ISR_TX, 0);
		if (ret)
			return ret;
		/*
		 * The PHC clock and packet detector share PTP_CTRL, but they do
		 * not share lifecycle. Use the backend's idle value while the
		 * active PHC has no packet timestamp mode. Full teardown clears
		 * the whole register.
		 */
		ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CTRL,
					  phc_active ? ptp->backend->idle_ctrl : 0);
		return ret;
	}

	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_IER,
				   RTLMQ_PHY_PTP_ISR_RX | RTLMQ_PHY_PTP_ISR_TX, ier_set);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_modify(ptp, RTLMQ_PHY_PTP_GEN_CFG, RTLMQ_PHY_PTP_GEN_W1C,
				   ptp->backend->status_w1c ? RTLMQ_PHY_PTP_GEN_W1C : 0);
	if (ret)
		return ret;
	if (!old_modes)
		ret = ptp->backend->tx_status_mac_ocp ?
			rtlmq_ptp_clear_tx_status_owned(ptp) :
			rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_ISR,
					    U16_MAX);
	else if (!(old_modes & RTLMQ_PTP_MODE_TX) &&
		 modes & RTLMQ_PTP_MODE_TX)
		ret = rtlmq_ptp_clear_tx_status_owned(ptp);
	if (ret)
		return ret;
	return rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CTRL, ptp->backend->packet_ctrl);
}

int rtlmq_ptp_phy_config_modes_owned(struct rtlmq_ptp *ptp,
				     u8 old_modes, u8 modes,
				     bool phc_active)
{
	int ret;

	ret = rtlmq_ptp_phy_program_modes_owned(ptp, old_modes, modes, phc_active);
	if (ret)
		return ret;

	return rtlmq_ptp_phy_apply_path_owned(ptp);
}

int rtlmq_ptp_phy_live_pause_owned(struct rtlmq_ptp *ptp, u8 old_modes)
{
	/*
	 * StopReq leaves the PHY timestamp path selected. Disable packet
	 * detection without A640, which starts a multi-second link-status
	 * sequence and is needed only when selecting the timestamp path.
	 */
	return rtlmq_ptp_phy_program_modes_owned(ptp, old_modes, 0, true);
}

int rtlmq_ptp_phy_live_resume_owned(struct rtlmq_ptp *ptp, u8 modes)
{
	if (!modes)
		return 0;

	/*
	 * Restore detector registers on the unchanged PHY path after StopReq.
	 * Clear stale status before enabling detection. Do not issue A640 or
	 * generate another link event.
	 */
	return rtlmq_ptp_phy_program_modes_owned(ptp, 0, modes, true);
}
