// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/unaligned.h>

#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq.h"
#include "rtlmq_xdp.h"
#include "rtlmq_ptp.h"

static int rtlmq_get_eeprom_len(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int len;

	mutex_lock(&tp->control_lock);
	len = tp->state == RTLMQ_STATE_REMOVING ||
	      tp->state == RTLMQ_STATE_DEAD ? 0 : tp->eeprom_len;
	mutex_unlock(&tp->control_lock);
	return len;
}

static int rtlmq_get_eeprom(struct net_device *ndev,
			    struct ethtool_eeprom *eeprom, u8 *data)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	u8 original, bytes[sizeof(u32)];
	int pm_token, size, ret = 0;
	u32 start, end, addr;

	if (!eeprom || !data)
		return -EINVAL;
	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	size = tp->eeprom_len;
	if (!size) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	if (!eeprom->len || eeprom->offset >= size ||
	    eeprom->len > size - eeprom->offset) {
		ret = -EINVAL;
		goto out;
	}
	if (!tp->pdev->bus ||
	    tp->pdev->cfg_size < RTLMQ_8126_EEPROM_VPD_DATA + sizeof(u32) ||
	    !rtlmq_mmio_ordinary_authorized(tp) ||
	    tp->state == RTLMQ_STATE_REMOVING ||
	    tp->state == RTLMQ_STATE_DEAD) {
		ret = -ENODEV;
		goto out;
	}
	original = readb(tp->mmio_addr + RTLMQ_CFG9346);
	if (original == U8_MAX) {
		ret = -ENODEV;
		goto out;
	}
	start = eeprom->offset & ~3U;
	end = ALIGN(eeprom->offset + eeprom->len, 4);
	writeb(original | RTLMQ_CFG9346_UNLOCK,
	       tp->mmio_addr + RTLMQ_CFG9346);
	for (addr = start; addr < end; addr += 4) {
		u16 status;
		u32 value, offset = max(addr, eeprom->offset);
		u32 count = min(addr + 4, eeprom->offset + eeprom->len) - offset;
		unsigned int poll;

		if (pci_write_config_word(tp->pdev, RTLMQ_8126_EEPROM_VPD_ADDR,
					  addr) != PCIBIOS_SUCCESSFUL) {
			ret = -EIO;
			break;
		}
		ret = -ETIMEDOUT;
		for (poll = 0; poll < 10; poll++) {
			usleep_range(400, 500);
			if (pci_read_config_word(tp->pdev,
						 RTLMQ_8126_EEPROM_VPD_ADDR,
						 &status) != PCIBIOS_SUCCESSFUL) {
				ret = -EIO;
				break;
			}
			if (status == U16_MAX) {
				ret = -ENODEV;
				break;
			}
			if (status & BIT(15)) {
				ret = 0;
				break;
			}
		}
		if (ret)
			break;
		if (pci_read_config_dword(tp->pdev, RTLMQ_8126_EEPROM_VPD_DATA,
					  &value) != PCIBIOS_SUCCESSFUL) {
			ret = -EIO;
			break;
		}
		put_unaligned_le32(value, bytes);
		memcpy(data + offset - eeprom->offset, bytes + offset - addr,
		       count);
	}
	writeb(original, tp->mmio_addr + RTLMQ_CFG9346);
out:
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

struct rtlmq_phy_request {
	struct phy_device *phydev;
};

static int rtlmq_validate_phy_request_locked(struct rtlmq_private *tp,
					     struct rtlmq_phy_request *request)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!request)
		return -EINVAL;

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		return ret;
	if (!tp->ndev->phydev)
		return -EOPNOTSUPP;

	request->phydev = tp->ndev->phydev;
	return 0;
}

struct rtlmq_wol_request {
	u32 wolopts;
};

static int rtlmq_normalize_wol_request(const struct rtlmq_private *tp,
				       const struct ethtool_wolinfo *wol,
				       struct rtlmq_wol_request *request)
{
	if (!wol || !request)
		return -EINVAL;
	if (!rtlmq_get_supported_wol_opts(tp))
		return -EOPNOTSUPP;
	if (wol->wolopts & ~rtlmq_get_supported_wol_opts(tp))
		return -EINVAL;

	request->wolopts = wol->wolopts;
	return 0;
}

struct rtlmq_eee_request {
	struct rtlmq_phy_request phy;
	struct ethtool_keee settings;
};

static int
rtlmq_normalize_eee_request_locked(struct rtlmq_private *tp,
				   const struct ethtool_keee *settings,
				   struct rtlmq_eee_request *request)
{
	int ret;

	if (!settings || !request)
		return -EINVAL;
	if (!rtlmq_qualified_supports_eee(tp))
		return -EOPNOTSUPP;

	ret = rtlmq_validate_phy_request_locked(tp, &request->phy);
	if (ret)
		return ret;

	/* MAC Tx-LPI uses an MTU-derived byte timer, not a requested delay. */
	if (settings->tx_lpi_timer != rtlmq_get_eee_tx_lpi_timer_us(tp))
		return -EOPNOTSUPP;

	request->settings = *settings;
	return 0;
}

struct rtlmq_pause_request {
	struct rtlmq_phy_request phy;
	bool rx_pause;
	bool tx_pause;
};

static int
rtlmq_normalize_pause_request_locked(struct rtlmq_private *tp,
				     const struct ethtool_pauseparam *pause,
				     struct rtlmq_pause_request *request)
{
	int ret;

	if (!pause || !request)
		return -EINVAL;

	ret = rtlmq_validate_phy_request_locked(tp, &request->phy);
	if (ret)
		return ret;
	if (pause->autoneg > 1 || pause->rx_pause > 1 ||
	    pause->tx_pause > 1)
		return -EINVAL;
	if (!pause->autoneg)
		return -EOPNOTSUPP;

	request->rx_pause = pause->rx_pause;
	request->tx_pause = pause->tx_pause;
	return 0;
}

struct rtlmq_link_request {
	struct rtlmq_phy_request phy;
	struct ethtool_link_ksettings settings;
};

static int rtlmq_normalize_link_request_locked(struct rtlmq_private *tp,
					       const struct ethtool_link_ksettings *settings,
					       struct rtlmq_link_request *request)
{
	int ret;

	if (!settings || !request)
		return -EINVAL;

	ret = rtlmq_validate_phy_request_locked(tp, &request->phy);
	if (ret)
		return ret;

	request->settings = *settings;
	return 0;
}

static int rtlmq_validate_reset_request(const u32 *flags)
{
	if (!flags)
		return -EINVAL;
	if (*flags != ETH_RESET_DEDICATED && *flags != ETH_RESET_ALL)
		return -EOPNOTSUPP;

	return 0;
}

static void rtlmq_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const char *fw_version;

	strscpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strscpy(info->version, RTLMQ_DRIVER_VERSION, sizeof(info->version));
	info->fw_version[0] = '\0';
	if (tp->pdev)
		strscpy(info->bus_info, pci_name(tp->pdev),
			sizeof(info->bus_info));
	fw_version = rtlmq_firmware_version(tp);
	if (fw_version)
		strscpy(info->fw_version, fw_version,
			sizeof(info->fw_version));
}

static u32 rtlmq_get_msglevel(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	return READ_ONCE(tp->msg_enable);
}

static void rtlmq_set_msglevel(struct net_device *ndev, u32 value)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	WRITE_ONCE(tp->msg_enable, value);
}

static void rtlmq_get_wol(struct net_device *ndev, struct ethtool_wolinfo *wol)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	memset(wol, 0, sizeof(*wol));
	mutex_lock(&tp->control_lock);
	wol->supported = rtlmq_get_supported_wol_opts(tp);
	wol->wolopts = tp->wol_opts;
	mutex_unlock(&tp->control_lock);
}

static int rtlmq_set_wol(struct net_device *ndev, struct ethtool_wolinfo *wol)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_wol_request request;
	int ret, restore_ret;
	u32 saved_wol;
	int pm_token;

	ret = rtlmq_normalize_wol_request(tp, wol, &request);
	if (ret)
		return ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;
	saved_wol = tp->wol_opts;
	tp->wol_opts = request.wolopts;
	ret = rtlmq_apply_wol(tp);
	if (!ret)
		ret = rtlmq_resume_wol_phy(tp);
	if (ret) {
		tp->wol_opts = saved_wol;
		/* ENODEV may mean lost MMIO; defer restoration to lifecycle replay. */
		if (ret != -ENODEV) {
			restore_ret = rtlmq_apply_wol(tp);
			if (restore_ret)
				netdev_err(ndev, "WoL restore failed: %d (request: %d)\n",
					   restore_ret, ret);
		}
	}
out_unlock:
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);

	return ret;
}

static int rtlmq_get_eee(struct net_device *ndev, struct ethtool_keee *edata)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	if (!edata)
		return -EINVAL;
	if (!rtlmq_qualified_supports_eee(tp))
		return -EOPNOTSUPP;
	if (!ndev->phydev)
		return -EOPNOTSUPP;

	ret = phy_ethtool_get_eee(ndev->phydev, edata);
	if (ret)
		return ret;

	/* Convert the MAC's byte-count timer to microseconds at the link speed. */
	edata->tx_lpi_timer = rtlmq_get_eee_tx_lpi_timer_us(tp);
	return 0;
}

static int rtlmq_set_eee(struct net_device *ndev, struct ethtool_keee *edata)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_eee_request request;
	int pm_token;
	int ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	ret = rtlmq_normalize_eee_request_locked(tp, edata, &request);
	if (!ret)
		ret = phy_ethtool_set_eee(request.phy.phydev,
					  &request.settings);
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);
	return ret;
}

static void rtlmq_ethtool_get_pauseparam(struct net_device *ndev,
					 struct ethtool_pauseparam *pause)
{
	bool tx_pause = false;
	bool rx_pause = false;

	if (!pause)
		return;

	memset(pause, 0, sizeof(*pause));
	if (!ndev->phydev)
		return;

	phy_get_pause(ndev->phydev, &tx_pause, &rx_pause);
	pause->autoneg = AUTONEG_ENABLE;
	pause->tx_pause = tx_pause ? 1 : 0;
	pause->rx_pause = rx_pause ? 1 : 0;
}

static int rtlmq_ethtool_set_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *pause)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_pause_request request;
	int pm_token;
	int ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	ret = rtlmq_normalize_pause_request_locked(tp, pause, &request);
	if (!ret)
		phy_set_asym_pause(request.phy.phydev, request.rx_pause,
				   request.tx_pause);
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);

	return ret;
}

static int
rtlmq_set_link_ksettings(struct net_device *ndev,
			 const struct ethtool_link_ksettings *settings)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_link_request request;
	int pm_token;
	int ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	ret = rtlmq_normalize_link_request_locked(tp, settings, &request);
	if (!ret)
		ret = phy_ethtool_ksettings_set(request.phy.phydev,
						&request.settings);
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);

	return ret;
}

static int rtlmq_nway_reset(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_phy_request request;
	int pm_token;
	int ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);
	ret = rtlmq_validate_phy_request_locked(tp, &request);
	if (!ret)
		ret = phy_ethtool_nway_reset(ndev);
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);

	return ret;
}

static void rtlmq_get_channels(struct net_device *ndev,
			       struct ethtool_channels *channels)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;
	u16 max_rx;
	u16 max_tx;

	memset(channels, 0, sizeof(*channels));
	mutex_lock(&tp->control_lock);
	max_rx = rtlmq_advertised_max_rx_queues(tp);
	max_tx = rtlmq_advertised_max_tx_queues(tp);
	cfg = rtlmq_active_or_requested_runtime_config(tp);
	/* Netlink partial updates preserve fields returned by this getter. */
	channels->max_rx = max_rx;
	channels->max_tx = max_tx;
	channels->rx_count = cfg ? cfg->num_rx_queues : 0;
	channels->tx_count = cfg ? cfg->num_tx_queues : 0;
	mutex_unlock(&tp->control_lock);
}

static void rtlmq_get_ringparam(struct net_device *ndev,
				struct ethtool_ringparam *ring,
				struct kernel_ethtool_ringparam *kernel_ring,
				struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;

	(void)kernel_ring;
	(void)extack;

	mutex_lock(&tp->control_lock);
	cfg = rtlmq_active_or_requested_runtime_config(tp);
	ring->rx_max_pending = RTLMQ_MAX_RX_DESC;
	ring->tx_max_pending = RTLMQ_MAX_TX_DESC;
	ring->rx_pending = cfg ? cfg->rx_ring_count : 0;
	ring->tx_pending = cfg ? cfg->tx_ring_count : 0;
	mutex_unlock(&tp->control_lock);
}

struct rtlmq_ringparam_request {
	u16 rx_ring_count;
	u16 tx_ring_count;
};

static int
rtlmq_normalize_ringparam_request(const struct ethtool_ringparam *ring,
				  struct rtlmq_ringparam_request *request,
				  struct netlink_ext_ack *extack)
{
	int ret;

	if (!ring || !request)
		return -EINVAL;
	if (ring->rx_mini_pending || ring->rx_jumbo_pending) {
		NL_SET_ERR_MSG_MOD(extack,
				   "mini and jumbo rings are not supported");
		return -EOPNOTSUPP;
	}
	if (ring->rx_pending > U16_MAX || ring->tx_pending > U16_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "ring depth exceeds driver range");
		return -EINVAL;
	}

	ret = rtlmq_validate_ring_counts(ring->rx_pending,
					 ring->tx_pending);
	if (ret) {
		NL_SET_ERR_MSG_MOD(extack,
				   "RX and TX depths must be powers of two from 256 through 1024");
		return ret;
	}

	request->rx_ring_count = ring->rx_pending;
	request->tx_ring_count = ring->tx_pending;
	return 0;
}

static int rtlmq_set_ringparam(struct net_device *ndev,
			       struct ethtool_ringparam *ring,
			       struct kernel_ethtool_ringparam *kernel_ring,
			       struct netlink_ext_ack *extack)
{
	struct rtlmq_requested_runtime_snapshot saved_requested;
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_ringparam_request request;
	int ret;

	(void)kernel_ring;

	ret = rtlmq_normalize_ringparam_request(ring, &request, extack);
	if (ret)
		return ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	if (request.rx_ring_count == rtlmq_requested_rx_ring_count(tp) &&
	    request.tx_ring_count == rtlmq_requested_tx_ring_count(tp) &&
	    (!netif_running(ndev) ||
	     (request.rx_ring_count == rtlmq_published_rx_ring_count(tp) &&
	      request.tx_ring_count == rtlmq_published_tx_ring_count(tp)))) {
		ret = 0;
		goto out_unlock;
	}

	rtlmq_capture_requested_runtime(tp, &saved_requested);
	rtlmq_set_requested_ring_counts(tp, request.rx_ring_count,
					request.tx_ring_count);

	if (netif_running(ndev)) {
		ret = rtlmq_apply_live_ring_reconfiguration_locked(tp, &saved_requested,
								   "set_ringparam");
		goto out_unlock;
	}
	if (!rtlmq_can_apply_down_admin_change_locked(tp)) {
		rtlmq_restore_requested_runtime(tp, &saved_requested);
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = rtlmq_apply_requested_runtime_transaction(tp, &saved_requested, false);

out_unlock:
	mutex_unlock(&tp->control_lock);
	return ret;
}

static int
rtlmq_apply_requested_channels(struct rtlmq_private *tp,
			       const struct rtlmq_requested_runtime_snapshot *saved_requested,
			       bool reset_rss)
{
	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_can_apply_down_admin_change_locked(tp))
		return -EBUSY;

	return rtlmq_apply_requested_runtime_transaction(tp, saved_requested,
							 reset_rss);
}

static int rtlmq_rxfh_fields_for_flow_locked(struct rtlmq_private *tp,
					     u32 flow_type, u32 *fields)
{
	const struct rtlmq_rss *rss;
	u32 hash_fields = 0;
	u32 hash_options;

	lockdep_assert_held(&tp->control_lock);

	if (!fields)
		return -EINVAL;
	if (!rtlmq_chip_supports_rss(tp))
		return -EOPNOTSUPP;

	rss = rtlmq_active_or_requested_rss_state(tp);
	hash_options = READ_ONCE(rss->hash_options);
	switch (flow_type) {
	case TCP_V4_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV4_SUPP)
			hash_fields |= RXH_IP_SRC | RXH_IP_DST;
		if (hash_options & RTLMQ_RSS_CTRL_TCP_IPV4_SUPP)
			hash_fields |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case TCP_V6_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV6_SUPP)
			hash_fields |= RXH_IP_SRC | RXH_IP_DST;
		if (hash_options & RTLMQ_RSS_CTRL_TCP_IPV6_SUPP)
			hash_fields |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case UDP_V4_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV4_SUPP)
			hash_fields |= RXH_IP_SRC | RXH_IP_DST;
		if (hash_options & RTLMQ_RSS_CTRL_UDP_IPV4_SUPP)
			hash_fields |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case UDP_V6_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV6_SUPP)
			hash_fields |= RXH_IP_SRC | RXH_IP_DST;
		if (hash_options & RTLMQ_RSS_CTRL_UDP_IPV6_SUPP)
			hash_fields |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case SCTP_V4_FLOW:
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case IPV4_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV4_SUPP)
			hash_fields = RXH_IP_SRC | RXH_IP_DST;
		break;
	case SCTP_V6_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case IPV6_FLOW:
		if (hash_options & RTLMQ_RSS_CTRL_IPV6_SUPP)
			hash_fields = RXH_IP_SRC | RXH_IP_DST;
		break;
	default:
		return -EOPNOTSUPP;
	}

	*fields = hash_fields;
	return 0;
}

static int rtlmq_get_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *cmd,
			   u32 *rule_locs)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;
	u32 fields;
	int ret;

	(void)rule_locs;

	switch (cmd->cmd) {
	case ETHTOOL_GRXRINGS:
		mutex_lock(&tp->control_lock);
		cfg = rtlmq_active_or_requested_runtime_config(tp);
		cmd->data = cfg ? cfg->num_rx_queues : 0;
		mutex_unlock(&tp->control_lock);
		return 0;
	case ETHTOOL_GRXFH:
		if ((cmd->flow_type & FLOW_RSS) && cmd->rss_context)
			return -EOPNOTSUPP;
		mutex_lock(&tp->control_lock);
		ret = rtlmq_rxfh_fields_for_flow_locked(tp, cmd->flow_type & ~FLOW_RSS, &fields);
		mutex_unlock(&tp->control_lock);
		if (ret)
			return ret;
		cmd->data = fields;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rtlmq_get_rxfh_fields(struct net_device *ndev,
				 struct ethtool_rxfh_fields *fields)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	if (fields->rss_context)
		return -EOPNOTSUPP;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_rxfh_fields_for_flow_locked(tp, fields->flow_type,
						&fields->data);
	mutex_unlock(&tp->control_lock);

	return ret;
}

static int rtlmq_rxnfc_hash_options_for_cmd(const struct ethtool_rxnfc *cmd,
					    u32 current_hash_options,
					    u32 *new_hash_options)
{
	u32 clear_options = 0;
	u32 port_options = 0;
	u32 set_options = 0;
	u64 hash_fields;
	u32 flow_type;

	if (!cmd || !new_hash_options)
		return -EINVAL;
	if ((cmd->flow_type & FLOW_RSS) && cmd->rss_context)
		return -EOPNOTSUPP;

	flow_type = cmd->flow_type & ~FLOW_RSS;
	hash_fields = cmd->data;
	if (hash_fields & ~(u64)(RXH_IP_SRC | RXH_IP_DST |
				 RXH_L4_B_0_1 | RXH_L4_B_2_3))
		return -EINVAL;

	if ((flow_type == TCP_V4_FLOW || flow_type == TCP_V6_FLOW ||
	     flow_type == UDP_V4_FLOW || flow_type == UDP_V6_FLOW) &&
	    hash_fields != (RXH_IP_SRC | RXH_IP_DST) &&
	    hash_fields != (RXH_IP_SRC | RXH_IP_DST |
			    RXH_L4_B_0_1 | RXH_L4_B_2_3))
		return -EINVAL;

	switch (flow_type) {
	case TCP_V4_FLOW:
		set_options = RTLMQ_RSS_CTRL_IPV4_SUPP;
		port_options = RTLMQ_RSS_CTRL_TCP_IPV4_SUPP;
		break;
	case TCP_V6_FLOW:
		set_options = RTLMQ_RSS_CTRL_IPV6_SUPP |
			      RTLMQ_RSS_CTRL_IPV6_EXT_SUPP;
		port_options = RTLMQ_RSS_CTRL_TCP_IPV6_SUPP |
			       RTLMQ_RSS_CTRL_TCP_IPV6_EXT_SUPP;
		break;
	case UDP_V4_FLOW:
		set_options = RTLMQ_RSS_CTRL_IPV4_SUPP;
		port_options = RTLMQ_RSS_CTRL_UDP_IPV4_SUPP;
		break;
	case UDP_V6_FLOW:
		set_options = RTLMQ_RSS_CTRL_IPV6_SUPP |
			      RTLMQ_RSS_CTRL_IPV6_EXT_SUPP;
		port_options = RTLMQ_RSS_CTRL_UDP_IPV6_SUPP |
			       RTLMQ_RSS_CTRL_UDP_IPV6_EXT_SUPP;
		break;
	case SCTP_V4_FLOW:
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case IPV4_FLOW:
		if (hash_fields != 0 &&
		    hash_fields != (RXH_IP_SRC | RXH_IP_DST))
			return -EINVAL;

		if (!hash_fields) {
			clear_options = RTLMQ_RSS_CTRL_IPV4_SUPP |
					RTLMQ_RSS_CTRL_TCP_IPV4_SUPP |
					RTLMQ_RSS_CTRL_UDP_IPV4_SUPP;
		} else {
			set_options = RTLMQ_RSS_CTRL_IPV4_SUPP;
		}
		break;
	case SCTP_V6_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case IPV6_FLOW:
		if (hash_fields != 0 &&
		    hash_fields != (RXH_IP_SRC | RXH_IP_DST))
			return -EINVAL;

		if (!hash_fields) {
			clear_options = RTLMQ_RSS_CTRL_IPV6_SUPP |
					RTLMQ_RSS_CTRL_IPV6_EXT_SUPP |
					RTLMQ_RSS_CTRL_TCP_IPV6_SUPP |
					RTLMQ_RSS_CTRL_TCP_IPV6_EXT_SUPP |
					RTLMQ_RSS_CTRL_UDP_IPV6_SUPP |
					RTLMQ_RSS_CTRL_UDP_IPV6_EXT_SUPP;
		} else {
			set_options = RTLMQ_RSS_CTRL_IPV6_SUPP |
				      RTLMQ_RSS_CTRL_IPV6_EXT_SUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (hash_fields & (RXH_L4_B_0_1 | RXH_L4_B_2_3))
		set_options |= port_options;
	clear_options |= port_options;
	*new_hash_options = (current_hash_options & ~clear_options) | set_options;
	return 0;
}

struct rtlmq_rxnfc_request {
	u32 hash_options;
};

static int
rtlmq_normalize_rxnfc_request_locked(struct rtlmq_private *tp,
				     const struct ethtool_rxnfc *cmd,
				     struct rtlmq_rxnfc_request *request)
{
	const struct rtlmq_rss *rss;

	lockdep_assert_held(&tp->control_lock);

	if (!request || !cmd || cmd->cmd != ETHTOOL_SRXFH)
		return -EOPNOTSUPP;

	rss = rtlmq_active_or_requested_rss_state(tp);
	if (!rss)
		return -EINVAL;

	return rtlmq_rxnfc_hash_options_for_cmd(cmd, rss->hash_options, &request->hash_options);
}

static int rtlmq_set_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *cmd)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_rxnfc_request request;
	struct rtlmq_rss *rss;
	int ret = 0;

	if (!rtlmq_chip_supports_rss(tp))
		return -EOPNOTSUPP;
	if (!cmd || cmd->cmd != ETHTOOL_SRXFH)
		return -EOPNOTSUPP;

	mutex_lock(&tp->control_lock);

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	ret = rtlmq_normalize_rxnfc_request_locked(tp, cmd, &request);
	if (ret)
		goto out_unlock;

	cfg = rtlmq_active_or_requested_runtime_config(tp);
	if (READ_ONCE(tp->state) == RTLMQ_STATE_OPEN) {
		if (!cfg) {
			ret = -EINVAL;
			goto out_unlock;
		}

		rtlmq_stage_candidate_rss_for_config(tp, cfg);
	} else {
		rtlmq_stage_candidate_rss_from_requested(tp);
	}

	rss = rtlmq_candidate_rss_state_owned_mut(tp);
	if (!rss) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (request.hash_options == rss->hash_options) {
		rtlmq_clear_candidate_rss(tp);
		goto out_unlock;
	}

	rss->hash_options = request.hash_options;

	/*
	 * Hash-field and indir/key changes share the same RSS transaction.
	 * Programming errors discard staged state without changing saved settings.
	 */
	ret = rtlmq_apply_candidate_rss_transaction(tp);

out_unlock:
	mutex_unlock(&tp->control_lock);

	return ret;
}

static int rtlmq_set_rxfh_fields(struct net_device *ndev,
				 const struct ethtool_rxfh_fields *fields,
				 struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct ethtool_rxnfc cmd;
	int ret;

	if (!fields)
		return -EINVAL;
	if (!rtlmq_chip_supports_rss(tp)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "RSS hash-field configuration is unsupported by this device");
		return -EOPNOTSUPP;
	}
	cmd = (struct ethtool_rxnfc) {
		.cmd = ETHTOOL_SRXFH,
		.flow_type = fields->flow_type,
		.data = fields->data,
	};
	if (fields->rss_context) {
		NL_SET_ERR_MSG_MOD(extack, "RSS contexts are not supported");
		return -EOPNOTSUPP;
	}

	ret = rtlmq_set_rxnfc(ndev, &cmd);
	if (ret == -EOPNOTSUPP)
		NL_SET_ERR_MSG_MOD(extack, "RSS hash fields are unsupported for this flow type");
	else if (ret == -EINVAL)
		NL_SET_ERR_MSG_MOD(extack, "invalid RSS hash fields for this flow type");

	return ret;
}

static int rtlmq_set_channels(struct net_device *ndev,
			      struct ethtool_channels *channels)
{
	struct rtlmq_requested_runtime_snapshot saved_requested;
	const struct rtlmq_runtime_config *active_cfg;
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_channel_request request;
	bool same_counts;
	u16 queue_index;
	bool running;
	int ret;

	if (!channels)
		return -EINVAL;

	mutex_lock(&tp->control_lock);
	running = netif_running(ndev);

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	if (!running && !rtlmq_can_apply_down_admin_change_locked(tp)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (running && !rtlmq_can_apply_live_admin_rebuild_locked(tp)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = rtlmq_normalize_set_channels_request(tp, channels, &request);
	if (ret)
		goto out_unlock;
	if (!rtlmq_tc_tx_queue_count_allowed(tp, request.tx_count) ||
	    !rtlmq_xsk_tx_queue_count_allowed(tp, request.tx_count)) {
		ret = -EBUSY;
		goto out_unlock;
	}
	for (queue_index = 0; queue_index < RTLMQ_IMPL_MAX_RX_QUEUES;
	     queue_index++) {
		if (!rtlmq_requested_xsk_pool(tp, queue_index))
			continue;
		/*
		 * RX-only pools may survive above the TX range. The separate
		 * TX-capability check above pins previously admitted TX queues.
		 */
		if (queue_index >= request.rx_count) {
			ret = -EBUSY;
			goto out_unlock;
		}
	}

	same_counts = request.rx_count == rtlmq_requested_num_rx_queues(tp) &&
		      request.tx_count == rtlmq_requested_num_tx_queues(tp);
	if (same_counts) {
		cfg = rtlmq_requested_runtime_config(tp);
		active_cfg = rtlmq_active_runtime_config(tp);
		if (cfg &&
		    (!running ||
		     (request.rx_count == rtlmq_published_num_rx_queues(tp) &&
		      request.tx_count == rtlmq_published_num_tx_queues(tp) &&
		      active_cfg))) {
			ret = 0;
			goto out_unlock;
		}
	}

	rtlmq_capture_requested_runtime(tp, &saved_requested);

	lockdep_assert_held(&tp->control_lock);
	rtlmq_set_requested_queue_counts(tp, request.rx_count, request.tx_count);

	if (running &&
	    request.rx_count == rtlmq_published_num_rx_queues(tp) &&
	    request.tx_count == rtlmq_published_num_tx_queues(tp)) {
		/*
		 * The running queue counts already match this request. Refresh
		 * the requested profile, layout, and RSS with the same helper
		 * used for channel changes while down instead of duplicating a
		 * partial commit here.
		 */
		ret = rtlmq_apply_requested_runtime_transaction(tp, &saved_requested, true);
		if (ret)
			goto out_unlock;

		ret = rtlmq_refresh_active_runtime_from_requested_same_shape(tp);
		if (ret) {
			rtlmq_restore_requested_runtime(tp, &saved_requested);
			goto out_unlock;
		}
		ret = 0;
		goto out_unlock;
	}

	if (!running) {
		ret = rtlmq_apply_requested_channels(tp, &saved_requested,
						     true);
		if (ret)
			goto out_unlock;

		goto out_unlock;
	}

	/*
	 * Validate the requested shape before stopping the running configuration.
	 * If rebuilding fails, reopen the old request or leave the device quiesced
	 * with the saved requested state.
	 */
	ret = rtlmq_apply_live_reconfiguration_locked(tp,
						      BIT(RTLMQ_RESET_REASON_CHANNEL_RECONFIG),
						      &saved_requested
	);

	goto out_unlock;
out_unlock:
	mutex_unlock(&tp->control_lock);

	return ret;
}

static int rtlmq_ethtool_reset(struct net_device *ndev, u32 *flags)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret = 0;

	ret = rtlmq_validate_reset_request(flags);
	if (ret)
		return ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, false);
	if (ret || !netif_running(ndev)) {
		if (!ret)
			ret = -EBUSY;
		goto out_unlock_reset;
	}

	netif_device_detach(ndev);
	rtlmq_request_reset(tp,
			    BIT(RTLMQ_RESET_REASON_USER)
	);
	*flags = 0;

out_unlock_reset:
	mutex_unlock(&tp->control_lock);

	return ret;
}

#define RTLMQ_ETHTOOL_COMMON_OPS \
	.supported_ring_params = 0, \
	.get_drvinfo		= rtlmq_get_drvinfo, \
	.get_msglevel		= rtlmq_get_msglevel, \
	.set_msglevel		= rtlmq_set_msglevel, \
	.get_regs_len		= rtlmq_get_regs_len, \
	.get_regs		= rtlmq_get_regs, \
	.get_eeprom_len		= rtlmq_get_eeprom_len, \
	.get_eeprom		= rtlmq_get_eeprom, \
	.get_link		= ethtool_op_get_link, \
	.get_ts_info		= rtlmq_ptp_get_ts_info, \
	.get_ts_stats		= rtlmq_ptp_get_ts_stats, \
	.get_wol		= rtlmq_get_wol, \
	.set_wol		= rtlmq_set_wol, \
	.get_eee		= rtlmq_get_eee, \
	.set_eee		= rtlmq_set_eee, \
	.get_coalesce		= rtlmq_ethtool_get_coalesce, \
	.set_coalesce		= rtlmq_ethtool_set_coalesce, \
	.get_pause_stats	= rtlmq_ethtool_get_pause_stats, \
	.get_eth_mac_stats	= rtlmq_ethtool_get_eth_mac_stats, \
	.get_eth_ctrl_stats	= rtlmq_ethtool_get_eth_ctrl_stats, \
	.get_rmon_stats		= rtlmq_ethtool_get_rmon_stats, \
	.get_pauseparam		= rtlmq_ethtool_get_pauseparam, \
	.set_pauseparam		= rtlmq_ethtool_set_pauseparam, \
	.get_link_ksettings	= phy_ethtool_get_link_ksettings, \
	.set_link_ksettings	= rtlmq_set_link_ksettings, \
	.nway_reset		= rtlmq_nway_reset, \
	.reset			= rtlmq_ethtool_reset, \
	.get_rxnfc		= rtlmq_get_rxnfc, \
	.set_rxnfc		= rtlmq_set_rxnfc, \
	.get_rx_ring_count	= rtlmq_ethtool_get_rx_ring_count, \
	.get_sset_count		= rtlmq_ethtool_get_sset_count, \
	.get_strings		= rtlmq_ethtool_get_strings, \
	.get_ethtool_stats	= rtlmq_ethtool_get_stats, \
	.get_rxfh_key_size	= rtlmq_get_rxfh_key_size, \
	.get_rxfh_indir_size	= rtlmq_get_rxfh_indir_size, \
	.get_rxfh		= rtlmq_get_rxfh, \
	.set_rxfh		= rtlmq_set_rxfh, \
	.get_rxfh_fields	= rtlmq_get_rxfh_fields, \
	.set_rxfh_fields	= rtlmq_set_rxfh_fields, \
	.get_ringparam		= rtlmq_get_ringparam, \
	.set_ringparam		= rtlmq_set_ringparam, \
	.get_channels		= rtlmq_get_channels, \
	.set_channels		= rtlmq_set_channels,

static const struct ethtool_ops rtlmq_ethtool_v2_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_RX_USECS |
				     ETHTOOL_COALESCE_RX_MAX_FRAMES |
				     ETHTOOL_COALESCE_TX_USECS |
				     ETHTOOL_COALESCE_TX_MAX_FRAMES,
	RTLMQ_ETHTOOL_COMMON_OPS
};

static const struct ethtool_ops rtlmq_ethtool_disabled_coalesce_ops = {
	RTLMQ_ETHTOOL_COMMON_OPS
};

const struct ethtool_ops *rtlmq_get_ethtool_ops(const struct rtlmq_private *tp)
{
	if (rtlmq_hw_irq_moderation_version(tp) == RTLMQ_IRQ_MODERATION_V2)
		return &rtlmq_ethtool_v2_ops;

	return &rtlmq_ethtool_disabled_coalesce_ops;
}
