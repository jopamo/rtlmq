// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>

#include "rtlmq_hw.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"

struct rtlmq_ephy_info {
	u16 offset;
	u16 mask;
	u16 bits;
};

static int rtlmq_ephy_write(struct rtlmq_private *tp, u16 reg, u16 value)
{
	u32 ephy;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -EINVAL;

	writel(RTLMQ_EPHYAR_WRITE_CMD |
	       (((u32)reg & RTLMQ_EPHYAR_REG_MASK) <<
		RTLMQ_EPHYAR_REG_SHIFT) | value,
	       tp->mmio_addr + RTLMQ_EPHYAR);

	ret = readl_poll_timeout(tp->mmio_addr + RTLMQ_EPHYAR, ephy,
				 !(ephy & RTLMQ_EPHYAR_FLAG), 10, 100);
	if (!ret)
		udelay(10);

	if (ephy == U32_MAX)
		return -ENODEV;

	return ret ? -ETIMEDOUT : 0;
}

static int rtlmq_ephy_read(struct rtlmq_private *tp, u16 reg)
{
	u32 ephy;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -EINVAL;

	writel(((u32)reg & RTLMQ_EPHYAR_REG_MASK) <<
	       RTLMQ_EPHYAR_REG_SHIFT,
	       tp->mmio_addr + RTLMQ_EPHYAR);

	ret = readl_poll_timeout(tp->mmio_addr + RTLMQ_EPHYAR, ephy,
				 ephy & RTLMQ_EPHYAR_FLAG, 10, 100);
	if (ret)
		return -ETIMEDOUT;
	if (ephy == U32_MAX)
		return -ENODEV;

	return ephy & RTLMQ_EPHYAR_DATA_MASK;
}

static int rtlmq_ephy_init(struct rtlmq_private *tp,
			   const struct rtlmq_ephy_info *info, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		int val;
		int ret;

		val = rtlmq_ephy_read(tp, info[i].offset);
		if (val < 0)
			return val;

		val &= ~info[i].mask;
		val |= info[i].bits;
		ret = rtlmq_ephy_write(tp, info[i].offset, val);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct rtlmq_ephy_info rtlmq_ephy_8125b[] = {
	{ 0x0b, 0xffff, 0xa908 },
	{ 0x1e, 0xffff, 0x20eb },
	{ 0x4b, 0xffff, 0xa908 },
	{ 0x5e, 0xffff, 0x20eb },
	{ 0x22, 0x0030, 0x0020 },
	{ 0x62, 0x0030, 0x0020 },
};

int rtlmq_phy_init_analog(struct rtlmq_private *tp)
{
	return rtlmq_ephy_init(tp, rtlmq_ephy_8125b,
			       ARRAY_SIZE(rtlmq_ephy_8125b));
}

static int rtlmq_r8168g_phy_param(struct phy_device *phydev, u16 parm, u16 mask,
				  u16 val)
{
	int oldpage;
	int ret = 0;

	oldpage = phy_select_page(phydev, 0x0a43);
	if (oldpage < 0)
		goto out_restore;

	ret = __phy_write(phydev, 0x13, parm);
	if (ret >= 0)
		ret = __phy_modify(phydev, 0x14, mask, val);

out_restore:
	/* phy_select_page retains the bus lock even when selection fails. */
	return phy_restore_page(phydev, oldpage, ret < 0 ? ret : 0);
}

static int rtlmq_phy_param(struct phy_device *phydev, u16 parm, u16 mask,
			   u16 val)
{
	int ret;

	phy_lock_mdio_bus(phydev);
	ret = __phy_write_mmd(phydev, MDIO_MMD_VEND2, 0xb87c, parm);
	if (ret >= 0)
		ret = __phy_modify_mmd(phydev, MDIO_MMD_VEND2, 0xb87e, mask,
				       val);
	phy_unlock_mdio_bus(phydev);

	return ret < 0 ? ret : 0;
}

static int rtlmq_common_config_eee_phy(struct phy_device *phydev)
{
	int ret;

	ret = phy_modify_paged(phydev, 0xa6d, 0x14, 0x0010, 0x0000);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xa42, 0x14, 0x0080, 0x0000);
	if (ret < 0)
		return ret;

	return phy_modify_paged(phydev, 0xa4a, 0x11, 0x0200, 0x0000);
}

static int rtlmq_config_eee_phy(struct phy_device *phydev)
{
	int ret;

	ret = phy_modify_paged(phydev, 0x0a43, 0x11, 0x0000, BIT(4));
	if (ret < 0)
		return ret;

	return rtlmq_common_config_eee_phy(phydev);
}

static int rtlmq_disable_aldps(struct phy_device *phydev)
{
	return phy_modify_paged(phydev, 0x0a43, 0x10, BIT(2), 0);
}

static int rtlmq_enable_gphy_10m(struct phy_device *phydev)
{
	return phy_modify_paged(phydev, 0x0a44, 0x11, 0x0000, BIT(11));
}

static int rtlmq_legacy_force_mode(struct phy_device *phydev)
{
	return phy_modify_paged(phydev, 0xa5b, 0x12, BIT(15), 0);
}

static int rtlmq_finish_phy_config(struct phy_device *phydev,
				   int (*config_eee)(struct phy_device *))
{
	int ret;

	ret = rtlmq_legacy_force_mode(phydev);
	if (ret < 0)
		return ret;

	ret = rtlmq_disable_aldps(phydev);
	if (ret < 0)
		return ret;

	return config_eee(phydev);
}

static int rtlmq_hw_phy_config_8125b(struct phy_device *phydev)
{
	static const u16 bias_regs[] = {
		0x8044, 0x804a, 0x8050, 0x8056, 0x805c,
		0x8062, 0x8068, 0x806e, 0x8074, 0x807a,
	};
	unsigned int i;
	int ret;

	ret = rtlmq_enable_gphy_10m(phydev);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xac4, 0x13, 0x00f0, 0x0090);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xad3, 0x10, 0x0003, 0x0001);
	if (ret < 0)
		return ret;

	ret = rtlmq_phy_param(phydev, 0x80f5, 0xffff, 0x760e);
	if (ret < 0)
		return ret;

	ret = rtlmq_phy_param(phydev, 0x8107, 0xffff, 0x360e);
	if (ret < 0)
		return ret;

	ret = rtlmq_phy_param(phydev, 0x8551, 0xff00, 0x0800);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xbf0, 0x10, 0xe000, 0xa000);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xbf4, 0x13, 0x0f00, 0x0300);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(bias_regs); i++) {
		ret = rtlmq_r8168g_phy_param(phydev, bias_regs[i], 0xffff,
					     0x2417);
		if (ret < 0)
			return ret;
	}

	ret = phy_modify_paged(phydev, 0xa4c, 0x15, 0x0000, 0x0040);
	if (ret < 0)
		return ret;

	ret = phy_modify_paged(phydev, 0xbf8, 0x12, 0xe000, 0xa000);
	if (ret < 0)
		return ret;

	return rtlmq_finish_phy_config(phydev, rtlmq_config_eee_phy);
}

static int rtlmq_hw_phy_config_8126(struct phy_device *phydev)
{
	int ret;

	/* Reference rtl8126a_hw_phy_config: no RTL8125 analog/EEE additions. */
	ret = rtlmq_enable_gphy_10m(phydev);
	if (ret < 0)
		return ret;

	return rtlmq_finish_phy_config(phydev, rtlmq_common_config_eee_phy);
}

static int rtlmq_configure_phy(struct rtlmq_private *tp,
			       int (*configure)(struct phy_device *))
{
	int ret;

	if (!tp || !tp->phydev)
		return -ENODEV;

	ret = rtlmq_request_firmware(tp);
	if (ret)
		return ret;

	ret = rtlmq_apply_firmware(tp);
	if (ret) {
		rtlmq_release_firmware(tp);
		return ret;
	}

	ret = configure(tp->phydev);
	if (ret < 0)
		return ret;

	ret = phy_speed_up(tp->phydev);
	if (ret < 0)
		return ret;

	ret = genphy_soft_reset(tp->phydev);
	if (ret < 0)
		return ret;

	return rtlmq_apply_eee_policy(tp);
}

int rtlmq_configure_phy_8125b(struct rtlmq_private *tp)
{
	return rtlmq_configure_phy(tp, rtlmq_hw_phy_config_8125b);
}

int rtlmq_configure_phy_8126(struct rtlmq_private *tp)
{
	return rtlmq_configure_phy(tp, rtlmq_hw_phy_config_8126);
}
