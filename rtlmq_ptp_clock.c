// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/time64.h>

#include "rtlmq.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"

#define RTLMQ_PTP_MAX_ADJ_PPB		488281
#define RTLMQ_PTP_SECONDS_MASK		GENMASK_ULL(47, 0)

static struct rtlmq_ptp *rtlmq_ptp_from_info(struct ptp_clock_info *info)
{
	return container_of(info, struct rtlmq_ptp, info);
}

static int rtlmq_ptp_admit_clock_locked(struct rtlmq_ptp *ptp)
{
	struct rtlmq_private *tp;

	if (!ptp || !ptp->tp)
		return -ENODEV;

	tp = ptp->tp;
	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_ptp_backend_matches(tp, ptp) || !ptp->active ||
	    !rtlmq_mmio_ordinary_authorized(tp))
		return -EBUSY;
	if (READ_ONCE(tp->state) == RTLMQ_STATE_REMOVING ||
	    READ_ONCE(tp->state) == RTLMQ_STATE_DEAD)
		return -ENODEV;

	return 0;
}

static int rtlmq_ptp_acquire_clock_locked(struct rtlmq_ptp *ptp)
{
	int ret;

	ret = rtlmq_ptp_admit_clock_locked(ptp);
	if (ret)
		return ret;
	return rtlmq_ptp_hw_acquire(ptp, false);
}

static int rtlmq_ptp_read_time_locked(struct rtlmq_ptp *ptp,
				      struct timespec64 *ts)
{
	u16 sec_mid;
	u64 seconds;
	u16 sec_hi;
	u16 sec_lo;
	u16 ns_hi;
	u16 ns_lo;
	u32 nsec;
	int ret;

	ret = rtlmq_ptp_acquire_clock_locked(ptp);
	if (ret)
		return ret;

	ret = rtlmq_ptp_phy_clock_command(ptp, RTLMQ_PHY_PTP_CLOCK_DIRECT_READ);
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_CFG_NS_HI, &ns_hi);
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_CFG_NS_LO, &ns_lo);
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_CFG_SEC_HI, &sec_hi);
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_CFG_SEC_MID, &sec_mid);
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_CFG_SEC_LO, &sec_lo);
	if (ret)
		goto out_release;

	nsec = (((u32)ns_hi << 16) | ns_lo) & RTLMQ_PTP_NSEC_MASK;
	seconds = (u64)sec_hi << 32 | (u64)sec_mid << 16 | sec_lo;
	if (nsec >= NSEC_PER_SEC) {
		ret = -EIO;
	} else {
		ts->tv_sec = seconds;
		ts->tv_nsec = nsec;
		ret = 0;
	}

out_release:
	rtlmq_ptp_hw_release(ptp);
	return ret;
}

static int rtlmq_ptp_validate_time(const struct timespec64 *ts)
{
	if (!ts || ts->tv_sec < 0 ||
	    (u64)ts->tv_sec > RTLMQ_PTP_SECONDS_MASK ||
	    ts->tv_nsec < 0 || ts->tv_nsec >= NSEC_PER_SEC)
		return -ERANGE;

	return 0;
}

int rtlmq_ptp_write_time_locked(struct rtlmq_ptp *ptp,
				const struct timespec64 *ts)
{
	u8 modes;
	int ret;

	ret = rtlmq_ptp_admit_clock_locked(ptp);
	if (ret)
		return ret;
	ret = rtlmq_ptp_validate_time(ts);
	if (ret)
		return ret;
	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		return ret;

	/*
	 * Setting absolute time invalidates an outstanding TX timestamp.
	 * Acquire before cancellation so a busy mailbox leaves admission intact.
	 * A racing worker's mailbox wait is bounded; cancellation invalidates
	 * its generation before waiting for it. Rearming clears capture status.
	 */
	rtlmq_ptp_tx_cancel_request(ptp, false);

	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_NS_LO, lower_16_bits(ts->tv_nsec));
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_NS_HI,
				  (ts->tv_nsec >> 16) & GENMASK(13, 0));
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_SEC_LO, lower_16_bits((u64)ts->tv_sec));
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_SEC_MID,
				  lower_16_bits((u64)ts->tv_sec >> 16));
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_SEC_HI,
				  lower_16_bits((u64)ts->tv_sec >> 32));
	if (ret)
		goto out_release;
	ret = rtlmq_ptp_phy_clock_command(ptp, RTLMQ_PHY_PTP_CLOCK_DIRECT_WRITE);

out_release:
	rtlmq_ptp_hw_release(ptp);
	if (!ret) {
		/* Pairs with active-mode publication after detector programming. */
		modes = smp_load_acquire(&ptp->active_modes);
		ret = rtlmq_ptp_tx_start(ptp, modes);
	}
	if (ret) {
		rtlmq_ptp_freeze_locked(ptp->tp);
		netdev_err(ptp->tp->ndev,
			   "PTP clock write failed: %d; PTP inactive until device restore\n",
			   ret);
	}
	return ret;
}

int rtlmq_ptp_program_frequency_locked(struct rtlmq_ptp *ptp, s32 ppb)
{
	u32 rate;
	int ret;

	if (ppb < 0)
		rate = -div_u64((u64)(-(s64)ppb) << 32, 1000000000ULL);
	else
		rate = div_u64((u64)ppb << 32, 1000000000ULL);

	ret = rtlmq_ptp_acquire_clock_locked(ptp);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_NS_LO, lower_16_bits(rate));
	if (!ret)
		ret = rtlmq_ptp_phy_write(ptp, RTLMQ_PHY_PTP_CFG_NS_HI,
					  (rate >> 16) & GENMASK(5, 0));
	if (!ret)
		ret = rtlmq_ptp_phy_clock_command(ptp, RTLMQ_PHY_PTP_CLOCK_RATE_WRITE);
	rtlmq_ptp_hw_release(ptp);
	return ret;
}

static int rtlmq_phc_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	struct rtlmq_ptp *ptp = rtlmq_ptp_from_info(info);
	s32 ppb = scaled_ppm_to_ppb(scaled_ppm);
	struct rtlmq_private *tp = ptp->tp;
	int ret;

	if (ppb > info->max_adj || ppb < -info->max_adj)
		return -ERANGE;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_ptp_program_frequency_locked(ptp, ppb);
	if (!ret)
		ptp->frequency_ppb = ppb;
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int rtlmq_phc_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct rtlmq_ptp *ptp = rtlmq_ptp_from_info(info);
	struct rtlmq_private *tp = ptp->tp;
	struct timespec64 adjusted;
	struct timespec64 offset;
	int ret;

	mutex_lock(&tp->control_lock);
	/*
	 * On tested RTL8125B PHY-v3 hardware, increment/decrement commands
	 * replaced absolute time: adding 1.5 s set the PHC to 1.5 s.
	 * Use read/add/write under control_lock instead. The write helper checks
	 * the 48-bit seconds range before touching hardware.
	 */
	ret = rtlmq_ptp_read_time_locked(ptp, &adjusted);
	if (!ret) {
		offset = ns_to_timespec64(delta);
		adjusted = timespec64_add(adjusted, offset);
		ret = rtlmq_ptp_write_time_locked(ptp, &adjusted);
	}
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int rtlmq_phc_gettimex64(struct ptp_clock_info *info,
				struct timespec64 *ts,
				struct ptp_system_timestamp *sts)
{
	struct rtlmq_ptp *ptp = rtlmq_ptp_from_info(info);
	struct rtlmq_private *tp = ptp->tp;
	int ret;

	mutex_lock(&tp->control_lock);
	ptp_read_system_prets(sts);
	ret = rtlmq_ptp_read_time_locked(ptp, ts);
	ptp_read_system_postts(sts);
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int rtlmq_phc_settime64(struct ptp_clock_info *info,
			       const struct timespec64 *ts)
{
	struct rtlmq_ptp *ptp = rtlmq_ptp_from_info(info);
	struct rtlmq_private *tp = ptp->tp;
	int ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_ptp_write_time_locked(ptp, ts);
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int rtlmq_phc_enable(struct ptp_clock_info *info,
			    struct ptp_clock_request *request, int on)
{
	return -EOPNOTSUPP;
}

static const struct ptp_clock_info rtlmq_ptp_info_template = {
	.owner		= THIS_MODULE,
	.name		= "rtlmq PHC",
	.max_adj	= RTLMQ_PTP_MAX_ADJ_PPB,
	.n_alarm	= 0,
	.n_ext_ts	= 0,
	.n_per_out	= 0,
	.n_pins		= 0,
	.pps		= 0,
	.adjfine	= rtlmq_phc_adjfine,
	.adjtime	= rtlmq_phc_adjtime,
	.gettimex64	= rtlmq_phc_gettimex64,
	.settime64	= rtlmq_phc_settime64,
	.enable		= rtlmq_phc_enable,
};

void rtlmq_ptp_clock_init(struct rtlmq_ptp *ptp)
{
	ptp->info = rtlmq_ptp_info_template;
}

struct ptp_clock *rtlmq_ptp_register_clock(struct rtlmq_private *tp,
					   struct rtlmq_ptp *ptp)
{
	return ptp_clock_register(&ptp->info, &tp->pdev->dev);
}
