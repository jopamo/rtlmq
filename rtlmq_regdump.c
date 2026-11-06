// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/pci.h>

#include "rtlmq_chip.h"
#include "rtlmq.h"
#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_tx.h"

static u32 rtlmq_reg_dump_read(const struct rtlmq_private *tp,
			       const struct rtlmq_regdump_register *desc)
{
	void __iomem *addr;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return U32_MAX;
	addr = tp->mmio_addr + desc->offset;

	switch (desc->width) {
	case sizeof(u8):
		return readb(addr);
	case sizeof(u16):
		return readw(addr);
	case sizeof(u32):
		return readl(addr);
	default:
		return U32_MAX;
	}
}

static bool
rtlmq_reg_dump_read_failed(const struct rtlmq_regdump_register *desc, u32 value)
{
	switch (desc->width) {
	case sizeof(u8):
		return value == U8_MAX;
	case sizeof(u16):
		return value == U16_MAX;
	case sizeof(u32):
		return value == U32_MAX;
	default:
		return true;
	}
}

static bool
rtlmq_reg_dump_schema_valid(const struct rtlmq_regdump_schema *schema)
{
	return schema && schema->registers && schema->register_count;
}

static enum rtlmq_reg_dump_status
rtlmq_reg_dump_status_locked(const struct rtlmq_private *tp)
{
	const struct rtlmq_regdump_schema *schema =
		rtlmq_chip_profile_regdump(tp->profile);

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_reg_dump_schema_valid(schema))
		return RTLMQ_REG_DUMP_UNSUPPORTED_PROFILE;
	if (tp->state != RTLMQ_STATE_OPEN &&
	    tp->state != RTLMQ_STATE_PROBED)
		return RTLMQ_REG_DUMP_UNAVAILABLE_STATE;
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE)
		return RTLMQ_REG_DUMP_UNAVAILABLE_STATE;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return RTLMQ_REG_DUMP_MMIO_REVOKED;
	if (!tp->pdev || pci_channel_offline(tp->pdev) ||
	    tp->pdev->current_state != PCI_D0)
		return RTLMQ_REG_DUMP_PCI_UNAVAILABLE;

	return RTLMQ_REG_DUMP_VALID;
}

static void
rtlmq_reg_dump_fill_identity_locked(const struct rtlmq_private *tp,
				    struct rtlmq_reg_dump_header *header,
				    u16 entry_count, unsigned int total_length)
{
	const struct rtlmq_hw_contract *hw =
		rtlmq_chip_profile_hw(tp->profile);
	const struct pci_dev *pdev = tp->pdev;

	lockdep_assert_held(&tp->control_lock);

	header->magic = cpu_to_le32(RTLMQ_REG_DUMP_MAGIC);
	header->format_version =
		cpu_to_le16(RTLMQ_REG_DUMP_FORMAT_VERSION);
	header->header_length = cpu_to_le16(sizeof(*header));
	header->entry_length =
		cpu_to_le16(sizeof(struct rtlmq_reg_dump_entry));
	header->entry_count = cpu_to_le16(entry_count);
	header->total_length = cpu_to_le32(total_length);
	if (pdev) {
		header->pci_vendor = cpu_to_le16(pdev->vendor);
		header->pci_device = cpu_to_le16(pdev->device);
		header->subsystem_vendor =
			cpu_to_le16(pdev->subsystem_vendor);
		header->subsystem_device =
			cpu_to_le16(pdev->subsystem_device);
	}
	header->xid = cpu_to_le16(tp->xid);
	header->raw_xid = cpu_to_le16(tp->raw_xid);
	header->mac_version = hw ? hw->mac_version : RTLMQ_MAC_NONE;
	header->config_method = tp->config_method;
	header->lifecycle_state = tp->state;
	header->mmio_authority = tp->mmio_authority;
}

static unsigned int
rtlmq_reg_dump_length(const struct rtlmq_regdump_schema *schema)
{
	return sizeof(struct rtlmq_reg_dump_header) +
	       (rtlmq_reg_dump_schema_valid(schema) ?
		schema->register_count * sizeof(struct rtlmq_reg_dump_entry) : 0) +
	       RTLMQ_IMPL_MAX_TX_QUEUES * sizeof(struct rtlmq_reg_dump_cbs) +
	       sizeof(struct rtlmq_reg_dump_tx_service) +
	       sizeof(struct rtlmq_reg_dump_identity) +
	       sizeof(struct rtlmq_reg_dump_ownership) +
	       sizeof(struct rtlmq_reg_dump_moderation) +
	       sizeof(struct rtlmq_reg_dump_tally);
}

static void
rtlmq_reg_dump_moderation_locked(const struct rtlmq_private *tp,
				 const struct rtlmq_regdump_schema *schema,
				 bool published,
				 enum rtlmq_reg_dump_status status,
				 struct rtlmq_reg_dump_moderation *record)
{
	const struct rtlmq_hw_contract *hw =
		rtlmq_chip_profile_hw(tp->profile);
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);
	record->requested_rx_usecs =
		cpu_to_le32(tp->requested_irq_coalesce_usecs);
	record->requested_rx_frames =
		cpu_to_le32(tp->requested_irq_coalesce_frames);
	record->requested_tx_usecs =
		cpu_to_le32(tp->requested_tx_irq_coalesce_usecs);
	record->requested_tx_frames =
		cpu_to_le32(tp->requested_tx_irq_coalesce_frames);
	record->version = hw ? hw->irq_moderation_version :
			     RTLMQ_IRQ_MODERATION_NONE;
	if (published && hw &&
	    (hw->irq_moderation_version == RTLMQ_IRQ_MODERATION_V2 ||
	     hw->irq_moderation_version == RTLMQ_IRQ_MODERATION_V5
	    )) {
		record->applied_valid = true;
		if (hw->irq_moderation_version == RTLMQ_IRQ_MODERATION_V2) {
			record->applied_rx_image =
				cpu_to_le16(rtlmq_coalesce_encode_moderation(
					tp->requested_irq_coalesce_usecs,
					tp->requested_irq_coalesce_frames));
			record->applied_tx_image =
				cpu_to_le16(rtlmq_coalesce_effective_tx_moderation(
					tp, tp->requested_tx_irq_coalesce_usecs,
					tp->requested_tx_irq_coalesce_frames));
		}
		/* Disabled banks have only the verified zero image. */
	}
	if (!schema || !schema->moderation ||
	    !schema->moderation_count ||
	    schema->moderation_count > RTLMQ_REG_DUMP_MAX_MODERATION_SLOTS) {
		record->readback_status = RTLMQ_REG_DUMP_UNSUPPORTED_PROFILE;
		return;
	}

	record->slot_count = schema->moderation_count;
	for (i = 0; i < schema->moderation_count; i++) {
		record->slots[i].offset =
			cpu_to_le16(schema->moderation[i].offset);
		record->slots[i].width = schema->moderation[i].width;
	}
	record->readback_status = status;
	if (status != RTLMQ_REG_DUMP_VALID)
		return;
	for (i = 0; i < schema->moderation_count; i++) {
		u32 value = rtlmq_reg_dump_read(tp, &schema->moderation[i]);

		if (rtlmq_reg_dump_read_failed(&schema->moderation[i], value)) {
			record->readback_status = RTLMQ_REG_DUMP_READ_FAILED;
			for (i = 0; i < schema->moderation_count; i++)
				record->slots[i].value = 0;
			return;
		}
		record->slots[i].value = cpu_to_le32(value);
	}
}

static void
rtlmq_reg_dump_ownership_locked(const struct rtlmq_private *tp, bool published,
				struct rtlmq_reg_dump_ownership *record)
{
	const struct rtlmq_runtime_profile_state *active = &tp->active_profile;
	const struct rtlmq_runtime_layout_state *layout = &active->layout;
	const struct rtlmq_runtime_config *cfg = &active->config;
	unsigned int i, j;

	lockdep_assert_held(&tp->control_lock);
	if (!published || !layout->valid || !cfg->num_vectors ||
	    cfg->num_vectors > RTLMQ_IMPL_MAX_NAPI)
		return;

	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *owner = &layout->vectors[i];
		struct rtlmq_reg_dump_vector *vector = &record->vectors[i];
		u16 rx_mask = 0, tx_mask = 0;

		if (owner->num_irq_sources > ARRAY_SIZE(owner->irq_sources) ||
		    owner->num_rx_rings > RTLMQ_IMPL_MAX_RX_QUEUES ||
		    owner->num_tx_rings > RTLMQ_IMPL_MAX_TX_QUEUES)
			goto invalid;
		for (j = 0; j < owner->num_rx_rings; j++) {
			if (owner->rx_ring_indices[j] >= RTLMQ_IMPL_MAX_RX_QUEUES)
				goto invalid;
			rx_mask |= BIT(owner->rx_ring_indices[j]);
		}
		for (j = 0; j < owner->num_tx_rings; j++) {
			if (owner->tx_ring_indices[j] >= RTLMQ_IMPL_MAX_TX_QUEUES)
				goto invalid;
			tx_mask |= BIT(owner->tx_ring_indices[j]);
		}
		vector->legacy_irq_mask = cpu_to_le32(owner->irq_mask);
		vector->rx_queue_mask = cpu_to_le16(rx_mask);
		vector->tx_queue_mask = cpu_to_le16(tx_mask);
		vector->source_count = owner->num_irq_sources;
		for (j = 0; j < owner->num_irq_sources; j++) {
			vector->sources[j].message_id =
				owner->irq_sources[j].message_id;
			vector->sources[j].role = owner->irq_sources[j].role;
			vector->sources[j].kind_flags =
				owner->irq_sources[j].kind_flags;
		}
	}
	record->vector_count = cfg->num_vectors;
	record->irq_mode = cfg->irq_mode;
	record->routing_mode = cfg->routing_mode;
	record->valid = true;
	return;

invalid:
	memset(record, 0, sizeof(*record));
}

static void
rtlmq_reg_dump_identity_locked(const struct rtlmq_private *tp,
			       struct rtlmq_reg_dump_identity *record)
{
	const struct rtlmq_hw_contract *hw =
		rtlmq_chip_profile_hw(tp->profile);
	const char *name = rtlmq_chip_profile_name(tp->profile);
	const char *firmware = rtlmq_firmware_name(tp);
	const char *version = rtlmq_firmware_version(tp);

	lockdep_assert_held(&tp->control_lock);
	if (name)
		strscpy(record->profile_name, name,
			sizeof(record->profile_name));
	if (firmware)
		strscpy(record->firmware_name, firmware,
			sizeof(record->firmware_name));
	if (version) {
		strscpy(record->firmware_version, version,
			sizeof(record->firmware_version));
		record->firmware_cached = true;
	}
	if (tp->state != RTLMQ_STATE_OPEN ||
	    !READ_ONCE(tp->runtime_published) || !tp->active_profile.valid ||
	    !tp->active_resource_generation || !hw)
		return;

	record->published = true;
	record->resource_generation =
		cpu_to_le32(tp->active_resource_generation);
	record->rx_desc_format = tp->rx_desc_fmt;
	record->tx_desc_format = hw->tx_desc_fmt;
}

static void
rtlmq_reg_dump_tx_service_locked(const struct rtlmq_private *tp,
				 struct rtlmq_reg_dump_tx_service *record)
{
	lockdep_assert_held(&tp->control_lock);
	record->requested_single_frame = rtlmq_tc_cbs_enabled(&tp->tc.requested);
	record->tso_constrained = tp->cbs_tso_constrained;
	record->requested_tx_usecs = cpu_to_le32(tp->requested_tx_irq_coalesce_usecs);
	record->requested_tx_frames = cpu_to_le32(tp->requested_tx_irq_coalesce_frames);
	record->gso_max_segs = cpu_to_le32(READ_ONCE(tp->ndev->gso_max_segs));
	record->tso_max_segs = cpu_to_le32(tp->ndev->tso_max_segs);
	record->wanted_features = cpu_to_le64(tp->ndev->wanted_features);
	record->effective_features = cpu_to_le64(tp->ndev->features);
	if (tp->state == RTLMQ_STATE_OPEN && READ_ONCE(tp->runtime_published) &&
	    tp->tc.registers_valid && tp->tx_rings &&
	    tp->tx_rings[0].cold.serial.initialized) {
		const struct rtlmq_serial_tx_policy *policy =
			&tp->tx_rings[0].cold.serial.policy;

		/* Policy is immutable within this published ring generation. */
		record->published = true;
		record->epoch_descs = cpu_to_le32(policy->descs);
		record->runway_descs = cpu_to_le32(policy->runway);
		record->programmed_tx_moderation =
			cpu_to_le32(rtlmq_coalesce_effective_tx_moderation(
				tp, tp->requested_tx_irq_coalesce_usecs,
				tp->requested_tx_irq_coalesce_frames));
	}
}

static void rtlmq_reg_dump_cbs_locked(const struct rtlmq_private *tp,
				      struct rtlmq_reg_dump_cbs *records)
{
	u16 queue;

	lockdep_assert_held(&tp->control_lock);
	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++) {
		const struct rtlmq_cbs_config *cbs = &tp->tc.requested.cbs[queue];
		struct rtlmq_reg_dump_cbs *record = &records[queue];

		record->queue = queue;
		record->enabled = cbs->enabled;
		record->registers_valid = tp->tc.registers_valid;
		record->active_valid = tp->tc.active_valid;
		record->link_waiting = tp->tc.link_waiting;
		record->idleslope = cpu_to_le32(cbs->idleslope);
		record->sendslope = cpu_to_le32(cbs->sendslope);
		record->hicredit = cpu_to_le32(cbs->hicredit);
		record->locredit = cpu_to_le32(cbs->locredit);
		record->encoded_idle_credit = cpu_to_le32(cbs->idle_credit);
		if (tp->tc.registers_valid)
			record->programmed_idle_credit =
				cpu_to_le32(tp->tc.active.cbs[queue].idle_credit);
	}
}

int rtlmq_get_regs_len(struct net_device *ndev)
{
	const struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_regdump_schema *schema =
		rtlmq_chip_profile_regdump(tp->profile);

	if (!rtlmq_reg_dump_schema_valid(schema))
		schema = NULL;

	return rtlmq_reg_dump_length(schema);
}

void rtlmq_get_regs(struct net_device *ndev, struct ethtool_regs *regs,
		    void *data)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_reg_dump_moderation *moderation;
	struct rtlmq_reg_dump_header *header = data;
	struct rtlmq_reg_dump_ownership *ownership;
	struct rtlmq_reg_dump_tx_service *service;
	struct rtlmq_reg_dump_identity *identity;
	struct rtlmq_reg_dump_entry *entries;
	struct rtlmq_reg_dump_tally *tally;
	struct rtlmq_reg_dump_cbs *cbs;
	const struct rtlmq_regdump_schema *schema =
		rtlmq_chip_profile_regdump(tp->profile);
	enum rtlmq_reg_dump_status status;
	unsigned int length;
	u16 index;

	if (!regs || !data)
		return;
	if (!rtlmq_reg_dump_schema_valid(schema))
		schema = NULL;
	length = rtlmq_reg_dump_length(schema);
	if (regs->len < length) {
		memset(data, 0, regs->len);
		regs->version = (RTLMQ_REG_DUMP_FORMAT_VERSION << 16) |
				RTLMQ_REG_DUMP_READ_FAILED;
		return;
	}

	memset(data, 0, length);
	entries = (void *)(header + 1);
	cbs = (void *)(entries + (schema ? schema->register_count : 0));
	service = (void *)(cbs + RTLMQ_IMPL_MAX_TX_QUEUES);
	identity = (void *)(service + 1);
	ownership = (void *)(identity + 1);
	moderation = (void *)(ownership + 1);
	tally = (void *)(moderation + 1);
	for (index = 0; schema && index < schema->register_count; index++) {
		entries[index].offset =
			cpu_to_le16(schema->registers[index].offset);
		entries[index].width = schema->registers[index].width;
	}

	mutex_lock(&tp->control_lock);
	rtlmq_reg_dump_fill_identity_locked(tp, header, schema ? schema->register_count : 0,
					    length);
	rtlmq_reg_dump_cbs_locked(tp, cbs);
	rtlmq_reg_dump_tx_service_locked(tp, service);
	rtlmq_reg_dump_identity_locked(tp, identity);
	rtlmq_reg_dump_ownership_locked(tp, identity->published,
					ownership);
	status = rtlmq_reg_dump_status_locked(tp);
	/* Only the schema used for this snapshot can authorize register reads. */
	if (!schema)
		status = RTLMQ_REG_DUMP_UNSUPPORTED_PROFILE;
	if (status == RTLMQ_REG_DUMP_VALID) {
		for (index = 0; index < schema->register_count; index++) {
			u32 value = rtlmq_reg_dump_read(tp, &schema->registers[index]);

			if (rtlmq_reg_dump_read_failed(&schema->registers[index], value)) {
				status = RTLMQ_REG_DUMP_READ_FAILED;
				break;
			}
			entries[index].value = cpu_to_le32(value);
		}
	}
	if (status != RTLMQ_REG_DUMP_VALID)
		for (index = 0; schema && index < schema->register_count;
		     index++)
			entries[index].value = 0;
	rtlmq_reg_dump_moderation_locked(tp, schema, identity->published,
					 status, moderation);
	tally->sample_state = tp->tally.sample_state;
	tally->committed_valid = tp->tally.total_valid;
	tally->raw_valid = tp->tally.raw_valid;
	tally->command_failures = cpu_to_le64(tp->tally.command_failures);
	header->status = cpu_to_le32(status);
	header->lifecycle_state = tp->state;
	header->mmio_authority = tp->mmio_authority;
	mutex_unlock(&tp->control_lock);

	regs->version = (RTLMQ_REG_DUMP_FORMAT_VERSION << 16) | status;
}
