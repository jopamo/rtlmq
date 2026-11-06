// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "rtlmq_chip.h"
#include "rtlmq_hw.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"

#define RTLMQ_FIRMWARE_8125B_2	"rtl_nic/rtl8125b-2.fw"
#define RTLMQ_FW_VERSION_PREFIX_8125B_2	"rtl8125b-2_"
#define RTLMQ_FIRMWARE_8126A_2	"rtl_nic/rtl8126a-2.fw"
#define RTLMQ_FIRMWARE_8126A_3	"rtl_nic/rtl8126a-3.fw"

MODULE_FIRMWARE(RTLMQ_FIRMWARE_8125B_2);
MODULE_FIRMWARE(RTLMQ_FIRMWARE_8126A_2);
MODULE_FIRMWARE(RTLMQ_FIRMWARE_8126A_3);

#define RTLMQ_FW_VER_SIZE		32
#define RTLMQ_FW_OPCODE_SIZE	sizeof(__le32)
#define RTLMQ_FW_MAGIC_OFFSET		0
#define RTLMQ_FW_VERSION_OFFSET		4
#define RTLMQ_FW_START_OFFSET		36
#define RTLMQ_FW_LEN_OFFSET		40
#define RTLMQ_FW_CHECKSUM_OFFSET	44
#define RTLMQ_FW_STRUCTURED_HEADER_SIZE	(RTLMQ_FW_CHECKSUM_OFFSET + 1)
#define RTLMQ_FW_REG_MAX		0x0fffU
#define RTLMQ_FW_MAX_ACTIONS		65536U
#define RTLMQ_FW_NO_TARGET		U32_MAX
#define RTLMQ_FW_MIN_STEP_BUDGET	1024
#define RTLMQ_FW_MAX_STEPS_PER_OPCODE	64
#define RTLMQ_FW_MAX_TOTAL_DELAY_MS	300000U

enum rtlmq_fw_opcode {
	RTLMQ_FW_PHY_READ		= 0x0,
	RTLMQ_FW_PHY_DATA_OR	= 0x1,
	RTLMQ_FW_PHY_DATA_AND	= 0x2,
	RTLMQ_FW_PHY_BJMPN		= 0x3,
	RTLMQ_FW_PHY_MDIO_CHG	= 0x4,
	RTLMQ_FW_PHY_CLEAR_READCOUNT	= 0x7,
	RTLMQ_FW_PHY_WRITE		= 0x8,
	RTLMQ_FW_PHY_READCOUNT_EQ_SKIP	= 0x9,
	RTLMQ_FW_PHY_COMP_EQ_SKIPN	= 0xa,
	RTLMQ_FW_PHY_COMP_NEQ_SKIPN	= 0xb,
	RTLMQ_FW_PHY_WRITE_PREVIOUS	= 0xc,
	RTLMQ_FW_PHY_SKIPN		= 0xd,
	RTLMQ_FW_PHY_DELAY_MS	= 0xe,
};

const struct rtlmq_fw_profile rtlmq_8125b_firmware = {
	.name = RTLMQ_FIRMWARE_8125B_2,
	.version_prefix = RTLMQ_FW_VERSION_PREFIX_8125B_2,
	.max_actions = RTLMQ_FW_MAX_ACTIONS,
	.max_steps = RTLMQ_FW_MAX_ACTIONS * RTLMQ_FW_MAX_STEPS_PER_OPCODE,
	.max_total_delay_ms = RTLMQ_FW_MAX_TOTAL_DELAY_MS,
};

/*
 * Driver execution policy, not silicon timing: the A2/A3 images have 7282 /
 * 3279 actions and bounded polling loops. A 65536-step ceiling admits both
 * while keeping the worst-case delay below the existing five-minute limit.
 * Do not let a larger image silently increase its execution authority.
 */
#define RTLMQ_8126_FW_PROFILE(image, prefix) {			\
	.name = (image),						\
	.version_prefix = (prefix),				\
	.max_actions = RTLMQ_FW_MAX_ACTIONS,			\
	.max_steps = 65536U,					\
	.max_total_delay_ms = RTLMQ_FW_MAX_TOTAL_DELAY_MS,		\
}

const struct rtlmq_fw_profile rtlmq_8126a2_firmware =
	RTLMQ_8126_FW_PROFILE(RTLMQ_FIRMWARE_8126A_2, "rtl8126a-2_");
const struct rtlmq_fw_profile rtlmq_8126a3_firmware =
	RTLMQ_8126_FW_PROFILE(RTLMQ_FIRMWARE_8126A_3, "rtl8126a-3_");

typedef int (*rtlmq_fw_write_t)(struct rtlmq_private *tp, int reg,
				    int value);
typedef int (*rtlmq_fw_read_t)(struct rtlmq_private *tp, int reg);

struct rtlmq_fw_action {
	u32 branch_target;
	u16 regno;
	u16 value;
	u8 opcode;
};

struct rtlmq_fw {
	const struct rtlmq_fw_profile *profile;
	struct device *dev;
	char version[RTLMQ_FW_VER_SIZE];
	struct {
		const struct rtlmq_fw_action *actions;
		u32 count;
		u32 delay_bound_ms;
	} phy_action;
};

static const struct rtlmq_fw_profile *
rtlmq_selected_firmware_profile(const struct rtlmq_private *tp)
{
	/*
	 * Use the firmware descriptor from the profile selected at probe.
	 * A missing profile or descriptor is an error; do not select another
	 * image from PCI IDs, MAC versions, or filenames.
	 */
	return tp ? rtlmq_chip_profile_firmware(tp->profile) : NULL;
}

const char *rtlmq_firmware_name(const struct rtlmq_private *tp)
{
	const struct rtlmq_fw_profile *profile =
		rtlmq_selected_firmware_profile(tp);

	return profile ? profile->name : NULL;
}

static bool rtlmq_fw_profile_valid(const struct rtlmq_fw_profile *profile)
{
	return profile && profile->name && profile->name[0] &&
	       profile->version_prefix && profile->version_prefix[0] &&
	       profile->max_actions && profile->max_steps &&
	       profile->max_total_delay_ms;
}

static int rtlmq_fw_mac_mcu_write(struct rtlmq_private *tp, int reg, int value)
{
	int base;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	if (reg == 0x1f) {
		/* MAC pages use byte offsets, unlike the PHY page-zero alias. */
		base = rtlmq_ocp_page_to_base(value);
		if (base < 0)
			return base;
		tp->ocp_base = base;
		return 0;
	}

	return rtlmq_mac_ocp_write_checked(tp, (u32)tp->ocp_base + reg,
					   value & 0xffff);
}

static int rtlmq_fw_mac_mcu_read(struct rtlmq_private *tp, int reg)
{
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	return rtlmq_mac_ocp_read_checked(tp, (u32)tp->ocp_base + reg);
}

static void rtlmq_fw_free_actions(struct rtlmq_fw *fw)
{
	if (!fw)
		return;

	kfree(fw->phy_action.actions);
	fw->phy_action.actions = NULL;
	fw->phy_action.count = 0;
	fw->phy_action.delay_bound_ms = 0;
}

static u64 rtlmq_fw_step_budget(const struct rtlmq_fw_profile *profile,
				u32 count)
{
	u64 budget = max_t(u64, (u64)count * RTLMQ_FW_MAX_STEPS_PER_OPCODE,
			   RTLMQ_FW_MIN_STEP_BUDGET);

	return min_t(u64, budget, profile->max_steps);
}

static bool rtlmq_fw_normalize_actions(struct rtlmq_fw *fw, const u8 *stream,
				       u32 count)
{
	struct rtlmq_fw_action *actions;
	u64 max_delay_ms = 0;
	u64 delay_bound_ms;
	u64 step_budget;
	u32 index;

	if (!fw || !fw->profile || (!stream && count))
		return false;
	if (count > fw->profile->max_actions) {
		dev_err(fw->dev, "Firmware action count %u exceeds limit %u\n",
			count, fw->profile->max_actions);
		return false;
	}

	if (!count)
		return true;

	actions = kcalloc(count, sizeof(*actions), GFP_KERNEL);
	if (!actions)
		return false;

	for (index = 0; index < count; index++) {
		struct rtlmq_fw_action *normalized = &actions[index];
		u32 encoded;
		u32 target;
		u16 regno;
		u16 value;
		u8 opcode;

		encoded = get_unaligned_le32(stream + (size_t)index * RTLMQ_FW_OPCODE_SIZE);
		opcode = encoded >> 28;
		regno = (encoded >> 16) & RTLMQ_FW_REG_MAX;
		value = encoded & 0xffff;

		normalized->opcode = opcode;
		normalized->regno = regno;
		normalized->value = value;
		normalized->branch_target = RTLMQ_FW_NO_TARGET;

		switch (opcode) {
		case RTLMQ_FW_PHY_READ:
			if (value)
				goto invalid_operand;
			break;
		case RTLMQ_FW_PHY_DATA_OR:
		case RTLMQ_FW_PHY_DATA_AND:
			if (regno)
				goto invalid_operand;
			break;
		case RTLMQ_FW_PHY_CLEAR_READCOUNT:
			if (regno || value)
				goto invalid_operand;
			break;
		case RTLMQ_FW_PHY_WRITE:
			break;
		case RTLMQ_FW_PHY_MDIO_CHG:
			if (regno || value > 1)
				goto invalid_operand;
			break;
		case RTLMQ_FW_PHY_BJMPN:
			if (value)
				goto invalid_operand;
			if (!regno || regno > index)
				goto invalid_branch;

			normalized->branch_target = index - regno;
			break;
		case RTLMQ_FW_PHY_READCOUNT_EQ_SKIP:
			if (regno)
				goto invalid_operand;
			if (count - index <= 2)
				goto invalid_branch;

			normalized->branch_target = index + 2;
			break;
		case RTLMQ_FW_PHY_COMP_EQ_SKIPN:
		case RTLMQ_FW_PHY_COMP_NEQ_SKIPN:
		case RTLMQ_FW_PHY_SKIPN:
			target = (u32)((u64)index + regno + 1);
			if (target >= count)
				goto invalid_branch;

			normalized->branch_target = target;
			break;
		case RTLMQ_FW_PHY_WRITE_PREVIOUS:
			if (value)
				goto invalid_operand;
			break;
		case RTLMQ_FW_PHY_DELAY_MS:
			if (regno)
				goto invalid_operand;
			max_delay_ms = max_t(u64, max_delay_ms, value);
			break;
		default:
			dev_err(fw->dev, "Invalid firmware action 0x%08x\n",
				encoded);
			goto invalid;
		}
	}

	step_budget = rtlmq_fw_step_budget(fw->profile, count);
	if (check_mul_overflow(max_delay_ms, step_budget,
			       &delay_bound_ms) ||
	    delay_bound_ms > fw->profile->max_total_delay_ms)
		goto invalid_delay;

	fw->phy_action.actions = actions;
	fw->phy_action.count = count;
	fw->phy_action.delay_bound_ms = (u32)delay_bound_ms;
	return true;

invalid_branch:
	dev_err(fw->dev, "Invalid firmware branch at instruction %u\n",
		index);
	goto invalid;
invalid_operand:
	dev_err(fw->dev, "Invalid firmware operand at instruction %u\n",
		index);
	goto invalid;
invalid_delay:
	dev_err(fw->dev, "Firmware total delay bound exceeds %u ms\n",
		fw->profile->max_total_delay_ms);
invalid:
	kfree(actions);
	return false;
}

static bool rtlmq_fw_format_ok(struct rtlmq_fw *fw, const struct firmware *blob)
{
	size_t action_bytes;
	const u8 *version;
	u8 checksum = 0;
	const u8 *data;
	u32 magic;
	u32 start;
	u32 count;
	size_t i;

	if (!fw || !blob || !blob->data ||
	    !rtlmq_fw_profile_valid(fw->profile))
		return false;

	rtlmq_fw_free_actions(fw);
	data = blob->data;
	if (blob->size < RTLMQ_FW_OPCODE_SIZE)
		return false;

	magic = get_unaligned_le32(data + RTLMQ_FW_MAGIC_OFFSET);
	if (!magic) {
		if (blob->size < RTLMQ_FW_STRUCTURED_HEADER_SIZE)
			return false;

		version = data + RTLMQ_FW_VERSION_OFFSET;
		if (!memchr(version, '\0', RTLMQ_FW_VER_SIZE)) {
			dev_err(fw->dev, "Firmware version is not terminated\n");
			return false;
		}

		for (i = 0; i < blob->size; i++)
			checksum += data[i];
		if (checksum != 0) {
			dev_err(fw->dev, "Firmware checksum is invalid\n");
			return false;
		}
		if (!str_has_prefix((const char *)version,
				    fw->profile->version_prefix)) {
			dev_err(fw->dev,
				"Firmware version %s does not match profile %s\n",
				version, fw->profile->version_prefix);
			return false;
		}

		start = get_unaligned_le32(data + RTLMQ_FW_START_OFFSET);
		count = get_unaligned_le32(data + RTLMQ_FW_LEN_OFFSET);
		if (start < RTLMQ_FW_STRUCTURED_HEADER_SIZE ||
		    start > blob->size ||
		    start % RTLMQ_FW_OPCODE_SIZE ||
		    check_mul_overflow((size_t)count,
				       (size_t)RTLMQ_FW_OPCODE_SIZE,
				       &action_bytes) ||
		    action_bytes > blob->size - start) {
			dev_err(fw->dev, "Firmware action range is invalid\n");
			return false;
		}

		strscpy(fw->version, (const char *)version,
			sizeof(fw->version));
	} else {
		dev_err(fw->dev,
			"Firmware %s lacks a profile-bound container\n",
			fw->profile->name);
		return false;
	}

	return rtlmq_fw_normalize_actions(fw, data + start, count);
}

int rtlmq_request_firmware(struct rtlmq_private *tp)
{
	const struct rtlmq_fw_profile *profile;
	const struct firmware *blob;
	struct rtlmq_fw *fw;
	int rc;

	if (!tp)
		return -EINVAL;
	profile = rtlmq_selected_firmware_profile(tp);
	if (!profile || !tp->pdev)
		return -ENODEV;
	if (!rtlmq_fw_profile_valid(profile))
		return -EINVAL;
	if (tp->fw)
		return tp->fw->profile == profile ? 0 : -ENODEV;

	fw = kzalloc_obj(*fw, GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	fw->profile = profile;
	fw->dev = &tp->pdev->dev;

	rc = firmware_request_nowarn(&blob, profile->name, fw->dev);
	if (rc < 0)
		goto out_warn;

	rc = rtlmq_fw_format_ok(fw, blob) ? 0 : -EINVAL;
	release_firmware(blob);
	if (rc) {
		rtlmq_fw_free_actions(fw);
		goto out_warn;
	}

	tp->fw = fw;
	return 0;

out_warn:
	dev_warn(fw->dev, "Unable to load firmware %s (%d)\n",
		 profile->name, rc);
	kfree(fw);
	return rc;
}

int rtlmq_apply_firmware(struct rtlmq_private *tp)
{
	rtlmq_fw_write_t fw_write = rtlmq_mdio_write;
	rtlmq_fw_read_t fw_read = rtlmq_mdio_read;
	struct rtlmq_fw *fw;
	u64 delay_budget;
	u64 step_budget;
	int predata = 0;
	int count = 0;
	int ret = 0;
	u32 index;
	int bmcr;

	if (!tp)
		return -EINVAL;
	if (!tp->fw)
		return -ENOENT;
	if (tp->fw->profile != rtlmq_selected_firmware_profile(tp))
		return -ENODEV;
	fw = tp->fw;

	step_budget = rtlmq_fw_step_budget(fw->profile, fw->phy_action.count);
	delay_budget = fw->phy_action.delay_bound_ms;

	tp->ocp_base = RTLMQ_PHY_OCP_STD_BASE;

	for (index = 0; index < fw->phy_action.count;) {
		const struct rtlmq_fw_action *action =
			&fw->phy_action.actions[index];
		u32 next_index = index + 1;

		if (!step_budget) {
			netdev_warn(tp->ndev,
				    "firmware %s version %s exceeded step budget\n",
				    fw->profile->name, fw->version);
			ret = -E2BIG;
			goto firmware_done;
		}
		step_budget--;

		switch (action->opcode) {
		case RTLMQ_FW_PHY_READ:
			predata = fw_read(tp, action->regno);
			if (predata < 0) {
				ret = predata;
				goto firmware_done;
			}
			count++;
			break;
		case RTLMQ_FW_PHY_DATA_OR:
			predata |= action->value;
			break;
		case RTLMQ_FW_PHY_DATA_AND:
			predata &= action->value;
			break;
		case RTLMQ_FW_PHY_BJMPN:
			next_index = action->branch_target;
			break;
		case RTLMQ_FW_PHY_MDIO_CHG:
			if (action->value) {
				fw_write = rtlmq_fw_mac_mcu_write;
				fw_read = rtlmq_fw_mac_mcu_read;
			} else {
				fw_write = rtlmq_mdio_write;
				fw_read = rtlmq_mdio_read;
			}
			break;
		case RTLMQ_FW_PHY_CLEAR_READCOUNT:
			count = 0;
			break;
		case RTLMQ_FW_PHY_WRITE:
			ret = fw_write(tp, action->regno, action->value);
			if (ret)
				goto firmware_done;
			break;
		case RTLMQ_FW_PHY_READCOUNT_EQ_SKIP:
			if (count == action->value)
				next_index = action->branch_target;
			break;
		case RTLMQ_FW_PHY_COMP_EQ_SKIPN:
			if (predata == action->value)
				next_index = action->branch_target;
			break;
		case RTLMQ_FW_PHY_COMP_NEQ_SKIPN:
			if (predata != action->value)
				next_index = action->branch_target;
			break;
		case RTLMQ_FW_PHY_WRITE_PREVIOUS:
			ret = fw_write(tp, action->regno, predata);
			if (ret)
				goto firmware_done;
			break;
		case RTLMQ_FW_PHY_SKIPN:
			next_index = action->branch_target;
			break;
		case RTLMQ_FW_PHY_DELAY_MS:
			if (action->value > delay_budget) {
				netdev_warn(tp->ndev,
					    "firmware %s version %s exceeded total delay budget\n",
					    fw->profile->name, fw->version);
				ret = -ETIMEDOUT;
				goto firmware_done;
			}
			delay_budget -= action->value;
			msleep(action->value);
			break;
		}

		index = next_index;
	}

firmware_done:
	tp->ocp_base = RTLMQ_PHY_OCP_STD_BASE;
	if (!ret) {
		/*
		 * Reference r8169_apply_firmware: firmware may leave PHY reset active.
		 * Restore the PHY page first, even if the last action used MAC
		 * access. Do not start family programming until reset completes.
		 */
		ret = read_poll_timeout(rtlmq_mdio_read, bmcr,
					bmcr < 0 || !(bmcr & BMCR_RESET),
					50000, 600000, true, tp, MII_BMCR);
		if (bmcr < 0)
			ret = bmcr;
	}
	if (ret)
		netdev_warn(tp->ndev, "firmware %s version %s failed: %d\n",
			    fw->profile->name, fw->version, ret);
	return ret;
}

void rtlmq_release_firmware(struct rtlmq_private *tp)
{
	if (!tp || !tp->fw)
		return;

	rtlmq_fw_free_actions(tp->fw);
	kfree(tp->fw);
	tp->fw = NULL;
}

const char *rtlmq_firmware_version(const struct rtlmq_private *tp)
{
	if (!tp || !tp->fw || !tp->fw->version[0])
		return NULL;

	return tp->fw->version;
}
