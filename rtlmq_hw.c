// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/iopoll.h>
#include <linux/log2.h>
#include <linux/pci_ids.h>

#include "rtlmq_diag.h"
#include "rtlmq_chip.h"
#include "rtlmq_ring.h"
#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_regs.h"
#include "rtlmq_tx.h"

static int rtlmq_read_rx_desc_mode(struct rtlmq_private *tp, u8 *value);

static int rtlmq_tx_desc_addr_regs(const struct rtlmq_private *tp,
				   u16 queue_index, u32 *low_reg, u32 *high_reg)
{
	if (!low_reg || !high_reg)
		return -EINVAL;

	switch (queue_index) {
	case 0:
		*low_reg = RTLMQ_TX_DESC_START_ADDR_LOW;
		*high_reg = RTLMQ_TX_DESC_START_ADDR_HIGH;
		return 0;
	case 1:
		if (!rtlmq_hw_has_q1_tx_desc_base_regs(tp))
			return -EOPNOTSUPP;
		*low_reg = RTLMQ_TNPDS_Q1_LOW;
		*high_reg = RTLMQ_TNPDS_Q1_HIGH;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rtlmq_rx_desc_addr_regs(u16 queue_index, u32 *low_reg, u32 *high_reg)
{
	if (!low_reg || !high_reg)
		return -EINVAL;

	if (!queue_index) {
		*low_reg = RTLMQ_RX_DESC_ADDR_LOW;
		*high_reg = RTLMQ_RX_DESC_ADDR_HIGH;
		return 0;
	}

	if (queue_index >= RTLMQ_IMPL_MAX_RX_QUEUES)
		return -EINVAL;

	*low_reg = RTLMQ_RDSAR_Q1_LOW + ((queue_index - 1) * sizeof(u64));
	*high_reg = *low_reg + sizeof(u32);

	return 0;
}

struct rtlmq_desc_addr_write {
	u32 reg;
	u32 value;
};

static void rtlmq_plan_desc_addr_writes(dma_addr_t desc_dma,
					u32 high_reg, u32 low_reg,
					struct rtlmq_desc_addr_write writes[2])
{
	/*
	 * Both vendor descriptor-base writers use low dword first, then high.
	 */
	writes[0].reg = low_reg;
	writes[0].value = lower_32_bits((u64)desc_dma);
	writes[1].reg = high_reg;
	writes[1].value = upper_32_bits((u64)desc_dma);
}

static void rtlmq_program_desc_addr(struct rtlmq_private *tp, u32 high_reg,
				    u32 low_reg, dma_addr_t desc_dma)
{
	struct rtlmq_desc_addr_write writes[2];

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	rtlmq_plan_desc_addr_writes(desc_dma, high_reg, low_reg, writes);
	writel(writes[0].value, tp->mmio_addr + writes[0].reg);
	writel(writes[1].value, tp->mmio_addr + writes[1].reg);
}

static void rtlmq_withdraw_desc_addr(struct rtlmq_private *tp, u32 high_reg,
				     u32 low_reg)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writel(0, tp->mmio_addr + high_reg);
	writel(0, tp->mmio_addr + low_reg);
}

static int rtlmq_verify_desc_addr(struct rtlmq_private *tp, dma_addr_t expected,
				  u32 low_reg, u32 high_reg,
				  const char *direction, u16 queue_index)
{
	u64 programmed;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	programmed = ((u64)readl(tp->mmio_addr + high_reg) << 32) |
		     readl(tp->mmio_addr + low_reg);
	if (programmed == expected)
		return 0;

	netdev_err(tp->ndev,
		   "%sq%u desc base mismatch expected=%#llx got=%#llx low=%#x high=%#x\n",
		   direction, queue_index, (u64)expected, programmed,
		   low_reg, high_reg);
	return programmed == U64_MAX ? -ENODEV : -EIO;
}

int rtlmq_republish_active_rx_ring_bases_locked(struct rtlmq_private *tp,
						const struct rtlmq_runtime_config *active_cfg)
{
	u16 num_rx_queues;
	u16 queue_index;
	int ret;

	if (!tp || !tp->pdev || !tp->rx_rings || !active_cfg)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Rollback restores active ring bases with runtime_published false and
	 * the active generation cleared. Use the caller's active configuration;
	 * published accessors cannot return it during this step.
	 */
	num_rx_queues = active_cfg->num_rx_queues;
	if (!num_rx_queues ||
	    num_rx_queues > rtlmq_owned_rx_ring_slots(tp))
		return -EINVAL;

	/* Validate every ring before writing any descriptor base. */
	for (queue_index = 0; queue_index < num_rx_queues; queue_index++) {
		const struct rtlmq_rx_ring *ring =
			&tp->rx_rings[queue_index];
		u32 low_reg;
		u32 high_reg;

		if (!ring->desc || !ring->desc_alloc_size ||
		    !rtlmq_dma_range_fits_coherent(&tp->pdev->dev, ring->desc_dma,
						   ring->desc_alloc_size))
			return -EINVAL;
		ret = rtlmq_rx_desc_addr_regs(queue_index, &low_reg, &high_reg);
		if (ret)
			return ret;
	}

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Clearing CmdRxEnb resets all hardware RX cursors. Software rings have
	 * already been rebuilt from slot zero. Write all RX bases before
	 * re-enabling RX.
	 */
	dma_wmb();
	for (queue_index = 0; queue_index < num_rx_queues; queue_index++) {
		const struct rtlmq_rx_ring *ring =
			&tp->rx_rings[queue_index];
		u32 low_reg;
		u32 high_reg;

		ret = rtlmq_rx_desc_addr_regs(queue_index, &low_reg, &high_reg);
		if (ret)
			return ret;
		rtlmq_program_desc_addr(tp, high_reg, low_reg, ring->desc_dma);
		ret = rtlmq_verify_desc_addr(tp, ring->desc_dma, low_reg, high_reg,
					     "rx", queue_index);
		if (ret)
			return ret;
	}

	return 0;
}

int rtlmq_republish_active_tx_ring_bases_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int queue_index;
	u32 high_reg;
	u32 low_reg;
	int ret;

	if (!tp || !tp->pdev || !tp->tx_rings)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || !cfg->num_tx_queues ||
	    cfg->num_tx_queues > rtlmq_owned_tx_ring_slots(tp))
		return -EINVAL;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	for (queue_index = 0; queue_index < cfg->num_tx_queues;
	     queue_index++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[queue_index];
		size_t ring_bytes =
			rtlmq_tx_desc_ring_bytes_for_stride(ring->ring_count,
							    rtlmq_tx_ring_desc_stride(ring));

		if (!ring->desc || !ring_bytes ||
		    !rtlmq_dma_range_fits_coherent(&tp->pdev->dev, ring->desc_dma, ring_bytes))
			return -EINVAL;
		ret = rtlmq_tx_desc_addr_regs(tp, queue_index, &low_reg, &high_reg);
		if (ret)
			return ret;
	}

	for (queue_index = 0; queue_index < cfg->num_tx_queues;
	     queue_index++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[queue_index];

		ret = rtlmq_tx_desc_addr_regs(tp, queue_index,
					      &low_reg, &high_reg);
		if (ret)
			return ret;
		rtlmq_program_desc_addr(tp, high_reg, low_reg, ring->desc_dma);
		ret = rtlmq_verify_desc_addr(tp, ring->desc_dma, low_reg, high_reg,
					     "tx", ring->queue_index);
		if (ret)
			return ret;
	}

	return 0;
}

static int
rtlmq_validate_ring_bases(struct rtlmq_private *tp,
			  const struct rtlmq_runtime_config *cfg)
{
	unsigned int i;
	u32 high_reg;
	u32 low_reg;
	int ret;

	if (!cfg || !cfg->num_tx_queues || !cfg->num_rx_queues ||
	    cfg->num_tx_queues > rtlmq_owned_tx_ring_slots(tp) ||
	    cfg->num_rx_queues > rtlmq_owned_rx_ring_slots(tp))
		return -EINVAL;

	for (i = 0; i < cfg->num_tx_queues; i++) {
		struct rtlmq_tx_ring *tx_ring = &tp->tx_rings[i];
		struct rtlmq_tx_desc *tx_desc;
		dma_addr_t tx_desc_dma;

		tx_desc = rtlmq_candidate_or_active_tx_desc(tp, i);
		tx_desc_dma = rtlmq_candidate_or_active_tx_desc_dma(tp, i);
		if (!tx_desc ||
		    !rtlmq_dma_range_fits_coherent(
			    &tp->pdev->dev, tx_desc_dma,
			    rtlmq_tx_desc_ring_bytes_for_stride(
				    cfg->tx_ring_count,
				    rtlmq_hw_tx_desc_stride(tp))))
			return -EINVAL;

		ret = rtlmq_tx_desc_addr_regs(tp, tx_ring->queue_index,
					      &low_reg, &high_reg);
		if (ret)
			return ret;
	}

	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring *rx_ring =
			rtlmq_candidate_or_active_rx_ring(tp, i);

		if (!rx_ring || !rx_ring->desc ||
		    !rtlmq_dma_range_fits_coherent(&tp->pdev->dev,
						   rx_ring->desc_dma,
						   rx_ring->desc_alloc_size))
			return -EINVAL;

		ret = rtlmq_rx_desc_addr_regs(rx_ring->queue_index,
					      &low_reg, &high_reg);
		if (ret)
			return ret;
	}

	return 0;
}

int rtlmq_program_ring_registers(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg =
		rtlmq_candidate_or_published_runtime_config(tp);
	u16 num_tx_queues;
	u16 num_rx_queues;
	unsigned int i;
	u32 high_reg;
	u32 low_reg;
	int ret;

	if (!tp || !tp->pdev || !tp->tx_rings || !tp->rx_rings)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -EINVAL;

	/* Reject any invalid queue before changing the first programmed base. */
	ret = rtlmq_validate_ring_bases(tp, cfg);
	if (ret)
		return ret;
	num_tx_queues = cfg->num_tx_queues;
	num_rx_queues = cfg->num_rx_queues;

	for (i = 0; i < num_tx_queues; i++) {
		struct rtlmq_tx_ring *tx_ring = &tp->tx_rings[i];
		dma_addr_t tx_desc_dma =
			rtlmq_candidate_or_active_tx_desc_dma(tp, i);

		ret = rtlmq_tx_desc_addr_regs(tp, tx_ring->queue_index,
					      &low_reg, &high_reg);
		if (ret)
			return ret;

		rtlmq_program_desc_addr(tp, high_reg, low_reg, tx_desc_dma);
		ret = rtlmq_verify_desc_addr(tp, tx_desc_dma, low_reg, high_reg,
					     "tx", tx_ring->queue_index);
		if (ret)
			return ret;
	}

	for (i = num_tx_queues; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		ret = rtlmq_tx_desc_addr_regs(tp, i, &low_reg, &high_reg);
		if (ret == -EOPNOTSUPP)
			continue;
		if (ret)
			return ret;

		rtlmq_withdraw_desc_addr(tp, high_reg, low_reg);
	}

	for (i = 0; i < num_rx_queues; i++) {
		const struct rtlmq_rx_ring *rx_ring =
			rtlmq_candidate_or_active_rx_ring(tp, i);

		ret = rtlmq_rx_desc_addr_regs(rx_ring->queue_index,
					      &low_reg, &high_reg);
		if (ret)
			return ret;

		rtlmq_program_desc_addr(tp, high_reg, low_reg,
					rx_ring->desc_dma);
		ret = rtlmq_verify_desc_addr(tp, rx_ring->desc_dma,
					     low_reg, high_reg,
					     "rx", rx_ring->queue_index);
		if (ret)
			return ret;
	}

	for (i = num_rx_queues; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		ret = rtlmq_rx_desc_addr_regs(i, &low_reg, &high_reg);
		if (ret)
			return ret;

		rtlmq_withdraw_desc_addr(tp, high_reg, low_reg);
	}

	return 0;
}

int rtlmq_withdraw_ring_registers(struct rtlmq_private *tp)
{
	const u32 required_phases =
		RTLMQ_QUIESCE_PUBLICATION_REVOKED |
		RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED |
		RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED |
		RTLMQ_QUIESCE_NAPI_DISABLED |
		RTLMQ_QUIESCE_DMA_REVOKED;
	enum rtlmq_mmio_authority authority;
	unsigned int i;
	u32 high_reg;
	u32 low_reg;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Clear descriptor bases only after runtime access, host service, and
	 * device DMA have stopped. Skip MMIO after terminal PCI failure or when
	 * partial probe did not map the BAR.
	 */
	if (WARN_ON_ONCE(!READ_ONCE(tp->dma_quiesced) ||
			 (READ_ONCE(tp->quiesce_phase) & required_phases) !=
				 required_phases))
		return -EBUSY;
	authority = READ_ONCE(tp->mmio_authority);
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		if (READ_ONCE(tp->mmio_addr) &&
		    authority != RTLMQ_MMIO_AUTHORITY_TERMINAL)
			return -EACCES;
		rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_RING_REGISTERS_REVOKED);
		return 0;
	}

	/*
	 * Clear all descriptor-base registers before freeing ring DMA storage.
	 * DMA is already stopped; clear the high dword before the low dword.
	 */
	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		int ret = rtlmq_tx_desc_addr_regs(tp, i, &low_reg, &high_reg);

		if (ret)
			continue;

		rtlmq_withdraw_desc_addr(tp, high_reg, low_reg);
	}

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		if (rtlmq_rx_desc_addr_regs(i, &low_reg, &high_reg))
			continue;

		rtlmq_withdraw_desc_addr(tp, high_reg, low_reg);
	}
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_RING_REGISTERS_REVOKED);
	return 0;
}

int rtlmq_program_queue_state(struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *cfg)
{
	enum rtlmq_rx_desc_format current_fmt;
	enum rtlmq_rx_desc_format wanted_fmt;
	enum rtlmq_rx_desc_format got_fmt;
	bool unlocked = false;
	u16 num_rx_queues;
	u16 num_tx_queues;
	u8 rx_desc_mode;
	u16 q_num_ctrl;
	u16 rx_q_count;
	u16 tx_q_count;
	int tx_q_value;
	u16 tx_q_ctrl;
	u32 rx_config;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!cfg || !rtlmq_mmio_ordinary_authorized(tp))
		return -EINVAL;

	num_rx_queues = cfg->num_rx_queues;
	num_tx_queues = cfg->num_tx_queues;
	if (!num_rx_queues || !num_tx_queues)
		return -EINVAL;

	if (!is_power_of_2(num_rx_queues) ||
	    !is_power_of_2(num_tx_queues))
		return -EINVAL;

	current_fmt = READ_ONCE(tp->rx_desc_fmt);
	wanted_fmt = rtlmq_wanted_rx_desc_fmt(tp);

	rx_q_count = ilog2(num_rx_queues);
	tx_q_count = ilog2(num_tx_queues);

	rtlmq_unlock_config_regs(tp);
	unlocked = true;

	q_num_ctrl = readw(tp->mmio_addr + RTLMQ_Q_NUM_CTRL);
	if (q_num_ctrl == U16_MAX) {
		ret = -ENODEV;
		goto out_lock_config;
	}
	q_num_ctrl &= ~RTLMQ_Q_NUM_RX_MASK;
	q_num_ctrl |= rx_q_count << RTLMQ_Q_NUM_RX_SHIFT;
	writew(q_num_ctrl, tp->mmio_addr + RTLMQ_Q_NUM_CTRL);
	q_num_ctrl = readw(tp->mmio_addr + RTLMQ_Q_NUM_CTRL);
	if (q_num_ctrl == U16_MAX ||
	    (q_num_ctrl & RTLMQ_Q_NUM_RX_MASK) !=
	    (rx_q_count << RTLMQ_Q_NUM_RX_SHIFT)) {
		ret = -EIO;
		goto out_lock_config;
	}

	tx_q_value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_TX_QUEUE_COUNT_OCP_REG);
	if (tx_q_value < 0) {
		ret = tx_q_value;
		goto out_lock_config;
	}
	tx_q_ctrl = tx_q_value;
	tx_q_ctrl &= ~RTLMQ_TX_QUEUE_COUNT_MASK;
	tx_q_ctrl |= tx_q_count << RTLMQ_TX_QUEUE_COUNT_SHIFT;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_TX_QUEUE_COUNT_OCP_REG,
					  tx_q_ctrl);
	if (ret)
		goto out_lock_config;
	tx_q_value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_TX_QUEUE_COUNT_OCP_REG);
	if (tx_q_value < 0) {
		ret = tx_q_value;
		goto out_lock_config;
	}
	if ((tx_q_value & RTLMQ_TX_QUEUE_COUNT_MASK) !=
	    (tx_q_count << RTLMQ_TX_QUEUE_COUNT_SHIFT)) {
		ret = -EIO;
		goto out_lock_config;
	}

	/*
	 * RSS/multiqueue RX changes the descriptor layout on 8125. Program the
	 * exact bit pair for the desired format and refuse to continue if
	 * hardware latches anything else.
	 */
	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		goto out_lock_config;
	ret = rtlmq_read_rx_desc_mode(tp, &rx_desc_mode);
	if (ret)
		goto out_lock_config;

	switch (wanted_fmt) {
	case RX_DESC_FMT_V3:
		rx_desc_mode &= ~RTLMQ_RX_DESC_MODE_V4;
		writeb(rx_desc_mode, tp->mmio_addr + RTLMQ_RX_DESC_MODE);
		rx_config |= RTLMQ_RX_CONFIG_ENABLE_RX_DESC_V3;
		writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);
		break;
	case RX_DESC_FMT_V4:
		rx_config &= ~RTLMQ_RX_CONFIG_ENABLE_RX_DESC_V3;
		writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);
		rx_desc_mode |= RTLMQ_RX_DESC_MODE_V4;
		writeb(rx_desc_mode, tp->mmio_addr + RTLMQ_RX_DESC_MODE);
		break;
	case RX_DESC_FMT_V1:
	default:
		rx_config &= ~RTLMQ_RX_CONFIG_ENABLE_RX_DESC_V3;
		writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);
		rx_desc_mode &= ~RTLMQ_RX_DESC_MODE_V4;
		writeb(rx_desc_mode, tp->mmio_addr + RTLMQ_RX_DESC_MODE);
		break;
	}

	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		goto out_lock_config;
	ret = rtlmq_read_rx_desc_mode(tp, &rx_desc_mode);
	if (ret)
		goto out_lock_config;

	rtlmq_lock_config_regs(tp);
	unlocked = false;

	ret = rtlmq_decode_rx_desc_fmt(rx_config, rx_desc_mode, &got_fmt);
	if (ret) {
		netdev_err(tp->ndev,
			   "RX desc format readback invalid rxcfg=%#08x rx_desc_mode=%#02x rxq=%u txq=%u\n",
			   rx_config, rx_desc_mode, num_rx_queues,
			   num_tx_queues);
		return ret;
	}

	WRITE_ONCE(tp->rx_desc_fmt, got_fmt);
	if (got_fmt != wanted_fmt) {
		netdev_err(tp->ndev,
			   "RX desc format mismatch wanted=%s got=%s rxcfg=%#08x rx_desc_mode=%#02x rxq=%u txq=%u\n",
			   rtlmq_rx_desc_fmt_name(wanted_fmt),
			   rtlmq_rx_desc_fmt_name(got_fmt),
			   rx_config,
			   rx_desc_mode,
			   num_rx_queues,
			   num_tx_queues);
		return -EIO;
	}

	if (got_fmt != current_fmt) {
		ret = rtlmq_sync_rx_desc_format(tp);
		if (ret) {
			netdev_err(tp->ndev,
				   "failed to republish RX desc format %s: %d\n",
				   rtlmq_rx_desc_fmt_name(got_fmt), ret);
			return ret;
		}
	}

	return 0;

out_lock_config:
	if (unlocked && ret != -ENODEV)
		rtlmq_lock_config_regs(tp);
	return ret;
}

int rtlmq_hw_reset(struct rtlmq_private *tp, const char *where)
{
	unsigned int reset_polls = 0;
	u8 chipcmd;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	ret = rtlmq_chip_prepare_reset(tp, where);
	if (ret)
		return ret;

	writeb(RTLMQ_CHIP_CMD_RESET, tp->mmio_addr + RTLMQ_CHIP_CMD);

	{
		for (reset_polls = 0; reset_polls < 100; reset_polls++) {
			fsleep(100);
			chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
			if (chipcmd == U8_MAX)
				return -ENODEV;
			if (!(chipcmd & RTLMQ_CHIP_CMD_RESET))
				break;
		}
		if (reset_polls == 100) {
			netdev_err(tp->ndev,
				   "reset admission %s CmdReset timeout chipcmd=%#02x polls=%u\n",
				   where ? where : "unknown", chipcmd,
				   reset_polls);
			return -ETIMEDOUT;
		}
	}

	writel(RTLMQ_RX_CONFIG_RESET,
	       tp->mmio_addr + RTLMQ_RX_CONFIG);
	return 0;
}

int rtlmq_read_rx_config(struct rtlmq_private *tp, u32 *value)
{
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !value)
		return -ENODEV;

	*value = readl(tp->mmio_addr + RTLMQ_RX_CONFIG);
	if (*value == ~0U)
		return -ENODEV;

	return 0;
}

int rtlmq_clear_rx_acceptance(struct rtlmq_private *tp)
{
	u32 rx_config;
	int ret;

	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		return ret;
	writel(rx_config & ~RTLMQ_RX_CONFIG_ACCEPT_MASK,
	       tp->mmio_addr + RTLMQ_RX_CONFIG);
	return 0;
}

static int rtlmq_read_rx_desc_mode(struct rtlmq_private *tp, u8 *value)
{
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !value)
		return -ENODEV;

	*value = readb(tp->mmio_addr + RTLMQ_RX_DESC_MODE);
	if (*value == U8_MAX)
		return -ENODEV;

	return 0;
}

static int rtlmq_init_cplus_cmd(struct rtlmq_private *tp)
{
	u16 cp_cmd = 0;

	if (!tp)
		return -EINVAL;

	if (rtlmq_mmio_ordinary_authorized(tp)) {
		cp_cmd = readw(tp->mmio_addr + RTLMQ_CPLUS_CMD);
		if (cp_cmd == U16_MAX)
			return -ENODEV;
	}

	WRITE_ONCE(tp->cp_cmd, cp_cmd & RTLMQ_CPCMD_MASK);
	return 0;
}

int rtlmq_sync_cplus_cmd_features(struct rtlmq_private *tp,
				  netdev_features_t features)
{
	enum rtlmq_lifecycle_state state;
	u8 chipcmd;
	u16 cp_cmd;

	if (!tp)
		return -EINVAL;

	cp_cmd = READ_ONCE(tp->cp_cmd) & RTLMQ_CPCMD_MASK;
	if (features & NETIF_F_RXCSUM)
		cp_cmd |= RTLMQ_CPCMD_RX_CHKSUM;
	else
		cp_cmd &= ~RTLMQ_CPCMD_RX_CHKSUM;
	if (rtlmq_qualified_supports_vlan_ctag_rx(tp))
		cp_cmd &= ~RTLMQ_CPCMD_RX_VLAN;
	WRITE_ONCE(tp->cp_cmd, cp_cmd);

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return 0;

	state = READ_ONCE(tp->state);
	if (state != RTLMQ_STATE_OPENING && state != RTLMQ_STATE_OPEN &&
	    state != RTLMQ_STATE_RESETTING)
		return 0;

	writew(cp_cmd, tp->mmio_addr + RTLMQ_CPLUS_CMD);
	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;

	return 0;
}

int rtlmq_sync_rx_config_features(struct rtlmq_private *tp,
				  netdev_features_t features)
{
	enum rtlmq_lifecycle_state state;
	u32 rx_config;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) ||
	    (!rtlmq_qualified_supports_vlan_ctag_rx(tp) &&
	     !rtlmq_qualified_supports_rx_all(tp)))
		return 0;

	state = READ_ONCE(tp->state);
	if (state != RTLMQ_STATE_OPENING && state != RTLMQ_STATE_OPEN &&
	    state != RTLMQ_STATE_RESETTING)
		return 0;

	if (rtlmq_read_rx_config(tp, &rx_config))
		return -ENODEV;
	if (rtlmq_qualified_supports_rx_all(tp) &&
	    (features & NETIF_F_RXALL))
		rx_config |= RTLMQ_RX_CONFIG_ACCEPT_ERROR_MASK;
	else
		rx_config &= ~RTLMQ_RX_CONFIG_ACCEPT_ERROR_MASK;
	if (rtlmq_qualified_supports_vlan_ctag_rx(tp) &&
	    (features & NETIF_F_HW_VLAN_CTAG_RX))
		rx_config |= RTLMQ_RX_CONFIG_VLAN_TAG;
	else
		rx_config &= ~RTLMQ_RX_CONFIG_VLAN_TAG;
	writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);

	return 0;
}

static void rtlmq_write_irq_moderation16(struct rtlmq_private *tp, u32 reg,
					 u16 value)
{
	writew(value, tp->mmio_addr + reg);
}

static void rtlmq_write_irq_moderation8(struct rtlmq_private *tp, u32 reg,
					u8 value)
{
	writeb(value, tp->mmio_addr + reg);
}

static void rtlmq_write_irq_moderation32(struct rtlmq_private *tp, u32 reg,
					 u32 value)
{
	writel(value, tp->mmio_addr + reg);
}

static int
rtlmq_program_irq_moderation_v2(struct rtlmq_private *tp,
				const struct rtlmq_runtime_config *cfg,
				u16 rx, u16 tx)
{
	struct {
		u32 reg;
		u16 old;
		u16 value;
	} slots[RTLMQ_IMPL_MAX_RX_QUEUES + RTLMQ_IMPL_MAX_TX_QUEUES];
	unsigned int count = 0;
	bool restored = true;
	unsigned int i;

	for (i = 0; i < cfg->num_rx_queues; i++, count++) {
		slots[count].reg = RTLMQ_INT_MITI_V2_0_RX +
				   RTLMQ_INT_MITI_V2_STRIDE * i;
		slots[count].value = rx;
	}
	for (i = 0; i < cfg->num_tx_queues; i++, count++) {
		slots[count].reg = RTLMQ_INT_MITI_V2_0_TX +
				   RTLMQ_INT_MITI_V2_STRIDE * i;
		slots[count].value = tx;
	}
	for (i = 0; i < count; i++) {
		slots[i].old = readw(tp->mmio_addr + slots[i].reg);
		if (slots[i].old == U16_MAX)
			return -ENODEV;
	}

	for (i = 0; i < count; i++)
		rtlmq_write_irq_moderation16(tp, slots[i].reg, slots[i].value);
	for (i = 0; i < count; i++)
		if (readw(tp->mmio_addr + slots[i].reg) != slots[i].value)
			goto rollback;

	return 0;

rollback:
	for (i = 0; i < count; i++)
		rtlmq_write_irq_moderation16(tp, slots[i].reg, slots[i].old);
	for (i = 0; i < count; i++)
		if (readw(tp->mmio_addr + slots[i].reg) != slots[i].old)
			restored = false;

	return restored ? -EIO : -EUCLEAN;
}

static int
rtlmq_program_irq_moderation_disabled_bank(struct rtlmq_private *tp, u16 rx, u16 tx)
{
	enum {
		MAX_BANK_DWORDS = (RTLMQ_INT_MITI_V5_BANK_END -
				   RTLMQ_INT_MITI_V2_0_RX) / sizeof(u32),
	};
	u32 old_bank[MAX_BANK_DWORDS];
	const u8 bypass_mask = RTLMQ_INT_CFG0_TIMEOUT_BYPASS |
			       RTLMQ_INT_CFG0_MITIGATION_BYPASS |
			       RTLMQ_INT_CFG0_RDU_BYPASS;
	const unsigned int bank_dwords = MAX_BANK_DWORDS;
	bool restored = true;
	unsigned int i;
	u16 old_cfg1;
	u8 old_cfg0;

	/* V5 clears 0xa00..0xa7c, INT_CFG0 bypass bits 1/2/4 and INT_CFG1. */
	if (rx || tx)
		return -EOPNOTSUPP;

	for (i = 0; i < bank_dwords; i++) {
		old_bank[i] = readl(tp->mmio_addr + RTLMQ_INT_MITI_V2_0_RX +
				    i * sizeof(u32));
		if (old_bank[i] == U32_MAX)
			return -ENODEV;
	}
	{
		old_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
		if (old_cfg0 == U8_MAX)
			return -ENODEV;
		old_cfg1 = readw(tp->mmio_addr + RTLMQ_INT_CFG1);
		if (old_cfg1 == U16_MAX)
			return -ENODEV;
	}

	for (i = 0; i < bank_dwords; i++) {
		u32 reg = RTLMQ_INT_MITI_V2_0_RX + i * sizeof(u32);

		rtlmq_write_irq_moderation32(tp, reg, 0);
	}
	{
		rtlmq_write_irq_moderation8(tp, RTLMQ_INT_CFG0,
					    old_cfg0 & ~bypass_mask);
		rtlmq_write_irq_moderation16(tp, RTLMQ_INT_CFG1, 0);
	}

	for (i = 0; i < bank_dwords; i++)
		if (readl(tp->mmio_addr + RTLMQ_INT_MITI_V2_0_RX +
			  i * sizeof(u32)))
			goto rollback;
	if (
	    (readb(tp->mmio_addr + RTLMQ_INT_CFG0) !=
		   (old_cfg0 & ~bypass_mask) ||
		   readw(tp->mmio_addr + RTLMQ_INT_CFG1)))
		goto rollback;

	return 0;

rollback:
	for (i = 0; i < bank_dwords; i++) {
		u32 reg = RTLMQ_INT_MITI_V2_0_RX + i * sizeof(u32);

		rtlmq_write_irq_moderation32(tp, reg, old_bank[i]);
	}
	{
		rtlmq_write_irq_moderation8(tp, RTLMQ_INT_CFG0, old_cfg0);
		rtlmq_write_irq_moderation16(tp, RTLMQ_INT_CFG1, old_cfg1);
	}

	for (i = 0; i < bank_dwords; i++)
		if (readl(tp->mmio_addr + RTLMQ_INT_MITI_V2_0_RX +
			  i * sizeof(u32)) != old_bank[i])
			restored = false;
	if (
	    (readb(tp->mmio_addr + RTLMQ_INT_CFG0) != old_cfg0 ||
		   readw(tp->mmio_addr + RTLMQ_INT_CFG1) != old_cfg1))
		restored = false;

	return restored ? -EIO : -EUCLEAN;
}

/*
 * One bounded transaction covers the active RX/TX sources. Callers publish
 * requested/effective policy only on success; -EUCLEAN means rollback could
 * not establish the previous hardware state and live service must withdraw.
 */
int rtlmq_program_irq_moderation(struct rtlmq_private *tp, u16 rx, u16 tx)
{
	const struct rtlmq_runtime_config *cfg;
	enum rtlmq_lifecycle_state state;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	lockdep_assert_held(&tp->control_lock);

	state = READ_ONCE(tp->state);
	if (state != RTLMQ_STATE_OPENING &&
	    state != RTLMQ_STATE_RESETTING &&
	    state != RTLMQ_STATE_OPEN)
		return -EBUSY;

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || !cfg->num_rx_queues || !cfg->num_tx_queues ||
	    cfg->num_rx_queues > RTLMQ_IMPL_MAX_RX_QUEUES ||
	    cfg->num_tx_queues > RTLMQ_IMPL_MAX_TX_QUEUES)
		return -EINVAL;

	switch (rtlmq_hw_irq_moderation_version(tp)) {
	case RTLMQ_IRQ_MODERATION_V2:
		return rtlmq_program_irq_moderation_v2(tp, cfg, rx, tx);
	case RTLMQ_IRQ_MODERATION_V5:
		return rtlmq_program_irq_moderation_disabled_bank(tp, rx, tx);
	default:
		return -EOPNOTSUPP;
	}
}

int rtlmq_validate_tx_engine_queue_count_latch(struct rtlmq_private *tp,
					       const char *where)
{
	u16 staging_txq;
	int tx_q_ctrl;
	u16 expected;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	staging_txq = rtlmq_candidate_or_published_num_tx_queues(tp);
	if (!staging_txq)
		return 0;

	if (!is_power_of_2(staging_txq))
		return -EINVAL;

	tx_q_ctrl = rtlmq_mac_ocp_read_checked(tp, RTLMQ_TX_QUEUE_COUNT_OCP_REG);
	if (tx_q_ctrl < 0)
		return tx_q_ctrl;
	if (tx_q_ctrl == U16_MAX)
		return -ENODEV;
	expected = ilog2(staging_txq) << RTLMQ_TX_QUEUE_COUNT_SHIFT;
	if ((tx_q_ctrl & RTLMQ_TX_QUEUE_COUNT_MASK) == expected)
		return 0;

	netdev_err(tp->ndev,
		   "%s: tx_engine_latch lost staged tx queue count staging_txq=%u tx_q_ctrl=%#04x expected_masked=%#04x\n",
		   where ? where : "runtime_build",
		   staging_txq, tx_q_ctrl, expected);
	return -EIO;
}

int rtlmq_program_active_irq_moderation(struct rtlmq_private *tp)
{
	u16 rx, tx;

	if (!tp)
		return -ENODEV;

	rx = rtlmq_coalesce_encode_moderation(READ_ONCE(tp->requested_irq_coalesce_usecs),
					      READ_ONCE(tp->requested_irq_coalesce_frames));
	tx = rtlmq_coalesce_effective_tx_moderation(
		tp, READ_ONCE(tp->requested_tx_irq_coalesce_usecs),
		READ_ONCE(tp->requested_tx_irq_coalesce_frames));

	return rtlmq_program_irq_moderation(tp, rx, tx);
}

static int rtlmq_hw_config_bootstrap(struct rtlmq_private *tp)
{
	unsigned int i;
	u32 tx_config = RTLMQ_TX_CONFIG_BOOTSTRAP;
	u8 rx_desc_mode;
	u32 rx_config;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Initialize registers before enabling queues and interrupts.
	 * Keep AUTO_FIFO clear for close/doorbell TX mode. Queue counts,
	 * descriptor format, and RSS are programmed later.
	 */
	writeb(0x00, tp->mmio_addr + RTLMQ_INT_CFG0);
	writew(0x0000, tp->mmio_addr + RTLMQ_INT_CFG1);

	for (i = RTLMQ_INT_MITI_V2_0_RX; i < RTLMQ_INT_MITI_V5_BANK_END;
	     i += sizeof(u32))
		writel(0, tp->mmio_addr + i);

	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_EXT_TALLY_CTRL,
					   0, BIT(1) | BIT(0));
	if (ret)
		return ret;
	ret = rtlmq_chip_configure_mac(tp);
	if (ret)
		return ret;
	ret = rtlmq_init_cplus_cmd(tp);
	if (ret)
		return ret;
	ret = rtlmq_sync_cplus_cmd_features(tp,
					    READ_ONCE(tp->ndev->features));
	if (ret)
		return ret;
	writew(RTLMQ_RX_HW_FRAME_SIZE,
	       tp->mmio_addr + RTLMQ_RX_MAX_SIZE);

	writel(tx_config, tp->mmio_addr + RTLMQ_TX_CONFIG);
	writel(RTLMQ_RX_CONFIG_BOOTSTRAP, tp->mmio_addr + RTLMQ_RX_CONFIG);
	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		return ret;
	rx_config &= ~RTLMQ_RX_CONFIG_ENABLE_RX_DESC_V3;
	writel(rx_config,
	       tp->mmio_addr + RTLMQ_RX_CONFIG);
	ret = rtlmq_read_rx_desc_mode(tp, &rx_desc_mode);
	if (ret)
		return ret;
	rx_desc_mode &= ~RTLMQ_RX_DESC_MODE_V4;
	writeb(rx_desc_mode,
	       tp->mmio_addr + RTLMQ_RX_DESC_MODE);
	ret = rtlmq_sync_rx_config_features(tp,
					    READ_ONCE(tp->ndev->features));
	if (ret)
		return ret;

	ret = rtlmq_set_bootstrap_rx_mode(tp);
	if (ret)
		return ret;
	ret = rtlmq_program_mac_address(tp, tp->ndev->dev_addr);
	if (ret)
		netdev_err(tp->ndev,
			   "failed to restore station address: %d\n", ret);

	return ret;
}

int rtlmq_hw_init(struct rtlmq_private *tp)
{
	u8 chipcmd;
	int ret;

	if (!tp || !tp->ndev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Reset the MAC without starting it. PHY resume, firmware loading, and
	 * PHY configuration must complete before final MAC configuration.
	 */

	rtlmq_mask_and_ack_intrs(tp);

	ret = rtlmq_chip_prepare_mac(tp);
	if (ret) {
		netdev_err(tp->ndev, "hw_init failed at profile prepare_mac: %d\n",
			   ret);
		return ret;
	}

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	writeb(chipcmd & ~(RTLMQ_CHIP_CMD_TX_ENABLE |
			   RTLMQ_CHIP_CMD_RX_ENABLE),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);
	rtlmq_mask_and_ack_intrs(tp);

	return 0;
}

int rtlmq_hw_start(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !tp->ndev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Match upstream RTL8125 ordering: finish PHY resume, firmware loading,
	 * and PHY configuration before programming the MAC.
	 */
	rtlmq_unlock_config_regs(tp);

	ret = rtlmq_hw_config_bootstrap(tp);

	if (ret != -ENODEV)
		rtlmq_lock_config_regs(tp);

	if (ret) {
		netdev_err(tp->ndev,
			   "hw_start failed at hw_config_bootstrap: %d\n",
			   ret);
		return ret;
	}

	WRITE_ONCE(tp->rx_desc_fmt, rtlmq_wanted_rx_desc_fmt(tp));
	return rtlmq_apply_wol(tp);
}

int rtlmq_hw_enable_datapath(struct rtlmq_private *tp)
{
	u8 chipcmd;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == 0xff)
		return -ENODEV;

	/*
	 * The RXDV drain sets StopReq. Write exactly CmdTxEnb|CmdRxEnb, as
	 * r8169 does, to clear it. OR-ing the enables into the old value leaves
	 * the MAC stopped and causes TX watchdog timeouts after link-up.
	 */
	chipcmd = RTLMQ_CHIP_CMD_TX_ENABLE | RTLMQ_CHIP_CMD_RX_ENABLE;
	/*
	 * Order prepared RX descriptors before enabling MAC DMA.
	 */
	dma_wmb();
	writeb(chipcmd, tp->mmio_addr + RTLMQ_CHIP_CMD);

	if (readb_poll_timeout(tp->mmio_addr + RTLMQ_CHIP_CMD, chipcmd,
			       chipcmd != 0xff &&
			       !(chipcmd & RTLMQ_CHIP_CMD_STOP_REQ) &&
			       (chipcmd & (RTLMQ_CHIP_CMD_TX_ENABLE |
					   RTLMQ_CHIP_CMD_RX_ENABLE)) ==
			       (RTLMQ_CHIP_CMD_TX_ENABLE |
				RTLMQ_CHIP_CMD_RX_ENABLE),
			       20, 20000))
		return chipcmd == U8_MAX ? -ENODEV : -ETIMEDOUT;

	return 0;
}

int rtlmq_hw_stop(struct rtlmq_private *tp)
{
	u8 chipcmd;
	int ret;

	if (!tp)
		return -ENODEV;
	/* A new stop attempt must not inherit a previous epoch's success. */
	WRITE_ONCE(tp->dma_quiesced, false);
	WRITE_ONCE(tp->quiesce_phase,
		   READ_ONCE(tp->quiesce_phase) &
			   ~RTLMQ_QUIESCE_HARDWARE_STOPPED);
	rtlmq_serial_tx_pause(tp);
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	rtlmq_mask_and_ack_intrs(tp);
	rtlmq_update_counters(tp);

	ret = rtlmq_hw_reset(tp, "hw_stop");
	/*
	 * Preserve accumulated software totals, but discard the previous raw
	 * sample. The next activation must establish a new baseline because
	 * reset may preserve or clear the hardware counters.
	 */
	rtlmq_tally_reset_epoch(tp);
	if (ret)
		netdev_err(tp->ndev,
			   "supported MAC function reset timed out: %d\n", ret);
	if (ret == -ENODEV)
		return ret;
	rtlmq_mask_and_ack_intrs(tp);

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return ret ? ret : -ENODEV;
	writeb(chipcmd & ~(RTLMQ_CHIP_CMD_TX_ENABLE |
			   RTLMQ_CHIP_CMD_RX_ENABLE),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);

	if (!ret)
		rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_HARDWARE_STOPPED);
	return ret;
}
