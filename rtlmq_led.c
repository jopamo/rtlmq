// SPDX-License-Identifier: GPL-2.0-only

#include <linux/leds.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <uapi/linux/uleds.h>

#include "rtlmq.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_regs.h"

/*
 * The register addresses, 0x23f field mask, four LED instances, and
 * RTL8125 mode bits mirror r8169. Reference commit 0c77e3f uses this layout for
 * RTL8126 too; its LED trigger mapping does not define a 5-Gbit bit.
 * Select the layout in the qualified policy, not by a related MAC version.
 */
#define RTLMQ_LED_COUNT		4
#define RTLMQ_LED_CTRL_ACT	BIT(9)
#define RTLMQ_LED_CTRL_LINK_2500	BIT(5)
#define RTLMQ_LED_CTRL_LINK_1000	BIT(3)
#define RTLMQ_LED_CTRL_LINK_100	BIT(1)
#define RTLMQ_LED_CTRL_LINK_10	BIT(0)

struct rtlmq_led {
	struct led_classdev cdev;
	struct net_device *ndev;
	char name[LED_MAX_NAME_SIZE];
	u8 index;
	u16 mode;
	bool mode_valid;
};

struct rtlmq_leds {
	unsigned int registered;
	struct rtlmq_led led[RTLMQ_LED_COUNT];
};

static struct rtlmq_led *rtlmq_cdev_to_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct rtlmq_led, cdev);
}

static u16 rtlmq_led_reg(unsigned int index)
{
	static const u16 regs[RTLMQ_LED_COUNT] = {
		RTLMQ_LEDSEL0,
		RTLMQ_LEDSEL1,
		RTLMQ_LEDSEL2,
		RTLMQ_LEDSEL3,
	};

	return index < ARRAY_SIZE(regs) ? regs[index] : 0;
}

static unsigned long rtlmq_led_supported_trigger_flags(void)
{
	return BIT(TRIGGER_NETDEV_LINK_10) |
	       BIT(TRIGGER_NETDEV_LINK_100) |
	       BIT(TRIGGER_NETDEV_LINK_1000) |
	       BIT(TRIGGER_NETDEV_LINK_2500) |
	       BIT(TRIGGER_NETDEV_TX) |
	       BIT(TRIGGER_NETDEV_RX);
}

static bool rtlmq_led_trigger_mode_valid(unsigned long flags)
{
	bool rx;
	bool tx;

	if (flags & (BIT(TRIGGER_NETDEV_HALF_DUPLEX) |
		     BIT(TRIGGER_NETDEV_FULL_DUPLEX) |
		     BIT(TRIGGER_NETDEV_TX_ERR) |
		     BIT(TRIGGER_NETDEV_RX_ERR)))
		return false;
	if (flags & ~rtlmq_led_supported_trigger_flags())
		return false;

	rx = flags & BIT(TRIGGER_NETDEV_RX);
	tx = flags & BIT(TRIGGER_NETDEV_TX);

	/* RTL8125 has one combined activity bit, not independent RX/TX bits. */
	return rx == tx;
}

static u16 rtlmq_led_mode_encode(unsigned long flags)
{
	u16 mode = 0;

	if (flags & BIT(TRIGGER_NETDEV_LINK_10))
		mode |= RTLMQ_LED_CTRL_LINK_10;
	if (flags & BIT(TRIGGER_NETDEV_LINK_100))
		mode |= RTLMQ_LED_CTRL_LINK_100;
	if (flags & BIT(TRIGGER_NETDEV_LINK_1000))
		mode |= RTLMQ_LED_CTRL_LINK_1000;
	if (flags & BIT(TRIGGER_NETDEV_LINK_2500))
		mode |= RTLMQ_LED_CTRL_LINK_2500;
	if (flags & (BIT(TRIGGER_NETDEV_TX) | BIT(TRIGGER_NETDEV_RX)))
		mode |= RTLMQ_LED_CTRL_ACT;

	return mode;
}

static void rtlmq_led_mode_decode(u16 mode, unsigned long *flags)
{
	*flags = 0;

	if (mode & RTLMQ_LED_CTRL_LINK_10)
		*flags |= BIT(TRIGGER_NETDEV_LINK_10);
	if (mode & RTLMQ_LED_CTRL_LINK_100)
		*flags |= BIT(TRIGGER_NETDEV_LINK_100);
	if (mode & RTLMQ_LED_CTRL_LINK_1000)
		*flags |= BIT(TRIGGER_NETDEV_LINK_1000);
	if (mode & RTLMQ_LED_CTRL_LINK_2500)
		*flags |= BIT(TRIGGER_NETDEV_LINK_2500);
	if (mode & RTLMQ_LED_CTRL_ACT)
		*flags |= BIT(TRIGGER_NETDEV_TX) | BIT(TRIGGER_NETDEV_RX);
}

static int rtlmq_led_set_hw_mode(struct rtlmq_led *led, u16 mode, bool remember)
{
	struct rtlmq_private *tp;
	int pm_token;
	int ret = 0;
	u16 value;
	u16 reg;

	if (!led || !led->ndev)
		return -ENODEV;

	tp = netdev_priv(led->ndev);
	reg = rtlmq_led_reg(led->index);
	if (!tp || !reg)
		return -ENODEV;
	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		ret = -ENODEV;
		goto out_control;
	}

	mutex_lock(&tp->led_lock);
	value = readw(tp->mmio_addr + reg);
	if (value == U16_MAX) {
		ret = -ENODEV;
		goto out_unlock;
	}
	value &= ~RTLMQ_LEDSEL_MASK_8125;
	writew(value | (mode & RTLMQ_LEDSEL_MASK_8125),
	       tp->mmio_addr + reg);
	if (remember)
		led->mode = mode & RTLMQ_LEDSEL_MASK_8125;
	led->mode_valid = remember;
out_unlock:
	mutex_unlock(&tp->led_lock);
out_control:
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_led_get_hw_mode(struct rtlmq_led *led, u16 *mode)
{
	struct rtlmq_private *tp;
	int pm_token;
	int ret = 0;
	u16 reg;

	if (!led || !led->ndev || !mode)
		return -EINVAL;

	tp = netdev_priv(led->ndev);
	reg = rtlmq_led_reg(led->index);
	if (!tp || !reg)
		return -ENODEV;
	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		ret = -ENODEV;
		goto out_control;
	}

	mutex_lock(&tp->led_lock);
	*mode = readw(tp->mmio_addr + reg);
	if (*mode == U16_MAX) {
		ret = -ENODEV;
		goto out_unlock;
	}
	*mode &= RTLMQ_LEDSEL_MASK_8125;
out_unlock:
	mutex_unlock(&tp->led_lock);
out_control:
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static int rtlmq_led_hw_control_is_supported(struct led_classdev *cdev,
					     unsigned long flags)
{
	struct rtlmq_led *led = rtlmq_cdev_to_led(cdev);
	int ret;

	if (rtlmq_led_trigger_mode_valid(flags))
		return 0;

	/* Disable hardware control rather than leave an unsupported mode active. */
	ret = rtlmq_led_set_hw_mode(led, 0, false);
	return ret ? ret : -EOPNOTSUPP;
}

static int rtlmq_led_hw_control_set(struct led_classdev *cdev,
				    unsigned long flags)
{
	struct rtlmq_led *led = rtlmq_cdev_to_led(cdev);

	if (!rtlmq_led_trigger_mode_valid(flags))
		return -EOPNOTSUPP;

	return rtlmq_led_set_hw_mode(led, rtlmq_led_mode_encode(flags), true);
}

static int rtlmq_led_hw_control_get(struct led_classdev *cdev,
				    unsigned long *flags)
{
	struct rtlmq_led *led = rtlmq_cdev_to_led(cdev);
	u16 mode;
	int ret;

	if (!flags)
		return -EINVAL;

	ret = rtlmq_led_get_hw_mode(led, &mode);
	if (ret)
		return ret;

	rtlmq_led_mode_decode(mode, flags);
	return 0;
}

static struct device *rtlmq_led_hw_control_get_device(struct led_classdev *cdev)
{
	struct rtlmq_led *led = rtlmq_cdev_to_led(cdev);

	return led->ndev ? &led->ndev->dev : NULL;
}

static int rtlmq_led_setup(struct rtlmq_led *led, struct rtlmq_private *tp,
			   unsigned int index)
{
	struct led_classdev *cdev = &led->cdev;
	int ret;

	led->ndev = tp->ndev;
	led->index = index;
	snprintf(led->name, sizeof(led->name), "%s-led-%u",
		 tp->ndev->name, index);
	cdev->name = led->name;
	cdev->hw_control_trigger = "netdev";
	cdev->flags |= LED_RETAIN_AT_SHUTDOWN;
	cdev->hw_control_is_supported = rtlmq_led_hw_control_is_supported;
	cdev->hw_control_set = rtlmq_led_hw_control_set;
	cdev->hw_control_get = rtlmq_led_hw_control_get;
	cdev->hw_control_get_device = rtlmq_led_hw_control_get_device;

	ret = led_classdev_register(&tp->ndev->dev, cdev);
	if (ret)
		memset(led, 0, sizeof(*led));

	return ret;
}

int rtlmq_leds_register(struct rtlmq_private *tp)
{
	struct rtlmq_leds *leds;
	unsigned int i;
	int ret;

	if (!tp || !tp->ndev)
		return -EINVAL;
	if (!rtlmq_qualified_supports_leds(tp))
		return 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	leds = kzalloc_obj(*leds, GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	for (i = 0; i < RTLMQ_LED_COUNT; i++) {
		ret = rtlmq_led_setup(&leds->led[i], tp, i);
		if (ret)
			goto err_unregister;
		leds->registered++;
	}

	mutex_lock(&tp->control_lock);
	tp->leds = leds;
	mutex_unlock(&tp->control_lock);
	return 0;

err_unregister:
	while (leds->registered)
		led_classdev_unregister(&leds->led[--leds->registered].cdev);
	kfree(leds);
	return ret;
}

void rtlmq_leds_unregister(struct rtlmq_private *tp)
{
	struct rtlmq_leds *leds;

	if (!tp)
		return;

	leds = tp->leds;
	tp->leds = NULL;
	if (!leds)
		return;

	while (leds->registered)
		led_classdev_unregister(&leds->led[--leds->registered].cdev);
	kfree(leds);
}

void rtlmq_leds_restore(struct rtlmq_private *tp)
{
	struct rtlmq_leds *leds;
	unsigned int i;

	if (!tp)
		return;
	lockdep_assert_held(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	leds = tp->leds;
	if (!leds)
		return;

	mutex_lock(&tp->led_lock);
	for (i = 0; i < leds->registered; i++) {
		struct rtlmq_led *led = &leds->led[i];
		u16 value;

		if (!led->mode_valid)
			continue;

		value = readw(tp->mmio_addr + rtlmq_led_reg(led->index));
		if (value == U16_MAX)
			continue;
		value &= ~RTLMQ_LEDSEL_MASK_8125;
		writew(value | led->mode,
		       tp->mmio_addr + rtlmq_led_reg(led->index));
	}
	mutex_unlock(&tp->led_lock);
}
