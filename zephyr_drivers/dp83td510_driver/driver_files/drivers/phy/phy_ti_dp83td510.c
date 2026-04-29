/*
 * Copyright (c) 2025 Texas Instruments
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_dp83td510

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/mdio.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME phy_ti_dp83td510
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PHY_TI_DP83TD510_PHY_STS		0x10
#define PHY_TI_DP83TD510_PHY_STS_LINK	BIT(0)

#define PHY_TI_DP83TD510_PHYSCR_REG     0x11
#define PHY_TI_DP83TD510_PHYSCR_REG_IE  BIT(1)
#define PHY_TI_DP83TD510_PHYSCR_REG_IOE BIT(0)

#define PHY_TI_DP83TD510_MISR_REG      0x12
#define PHY_TI_DP83TD510_MISR_REG_LSCE BIT(5)

#define PHY_TI_DP83TD510_RCSR_REG         0x17
#define PHY_TI_DP83TD510_RCSR_REF_CLK_SEL BIT(7)

#define PHY_TI_DP83TD510_AN_CTRL         0x200
#define PHY_TI_DP83TD510_AN_CTRL_EN  	 BIT(12)
#define PHY_TI_DP83TD510_AN_CTRL_RST  	 BIT(9)

#define PHY_TI_DP83TD510_ANAR			0x20E
#define PHY_TI_DP83TD510_ANAR_EEE		BIT(14)
#define PHY_TI_DP83TD510_ANAR_TXLVL		BIT(13)

#define PHY_TI_DP83TD510_ANLPAR			0x20F
#define PHY_TI_DP83TD510_ANLPAR_EEE		BIT(14)
#define PHY_TI_DP83TD510_ANLPAR_TXLVL	BIT(12)

#define PHY_TI_DP83TD510_POR_DELAY 50

#define PHY_TI_DP83TD510_MMD_SEL 0x4000
#define DP83TD510E_PHYID 0x20000181

/* RMII 	  ->  50M input on PHY XI */
/* RMII_25MHz ->  25M input on PHY XI, RMII output 50M ref_clk to Host */
enum DP83TD510_interface {
	DP83TD510_RMII,
	DP83TD510_RMII_25MHZ
};

struct ti_DP83TD510_config {
	uint8_t addr;
	const struct device *mdio;
	enum DP83TD510_interface phy_iface;
	enum phy_link_speed default_speeds;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct gpio_dt_spec interrupt_gpio;
#endif
};

struct ti_DP83TD510_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	struct gpio_callback gpio_callback;
#endif
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
};


static int phy_ti_DP83TD555_mmd_range(uint16_t reg_addr) {
	if (reg_addr >= 0x0300U && reg_addr <= 0x0E01U)
	{
		return 0x1F;
	} else if (reg_addr >= 0x1000U && reg_addr <= 0x18F8U) {
		return 0x1;
	} else if (reg_addr >= 0x3000U && reg_addr <= 0x38E7U) {
		return 0x3;
	} else if (reg_addr >= 0x0200U && reg_addr <= 0x020FU) {
		return 0x7;
	} else {
		LOG_ERR("PHY register address not within valid MMD range for DP83TD510\n");
		return 0;
	}
}

static int phy_ti_DP83TD555_indirect_c22(const struct device *dev, uint8_t devad, uint16_t reg_addr) 
{
	const struct ti_DP83TD510_config *cfg = dev->config;
	int ret;

	ret = mdio_write(cfg->mdio, cfg->addr, MII_MMD_ACR, devad);
	if (ret) {
		return ret;
	}
	ret = mdio_write(cfg->mdio, cfg->addr, MII_MMD_AADR, reg_addr);
	
	if (ret) {
		return ret;
	}

	return mdio_write(cfg->mdio, cfg->addr, MII_MMD_ACR, (devad | PHY_TI_DP83TD510_MMD_SEL));
}


static int phy_ti_DP83TD510_read(const struct device *dev, uint16_t reg_addr, uint32_t *data)
{
	const struct ti_DP83TD510_config *cfg = dev->config;
	int ret;

	/* Make sure excessive bits 16-31 are reset */
	*data = 0U;
	if (reg_addr <= 0x130) {
		ret = mdio_read(cfg->mdio, cfg->addr, reg_addr, (uint16_t *)data);
		if (ret) {
			return ret;
		}
	} else {
		ret = phy_ti_DP83TD555_indirect_c22(dev, phy_ti_DP83TD555_mmd_range(reg_addr), reg_addr);
		if (ret) {
			return ret;
		}
		return  mdio_read(cfg->mdio, cfg->addr, MII_MMD_AADR, (uint16_t *)data);
	}

	return 0;
}

static int phy_ti_DP83TD510_write(const struct device *dev, uint16_t reg_addr, uint32_t data)
{
	const struct ti_DP83TD510_config *cfg = dev->config;
	int ret;

	if (reg_addr <= 0x130) {
		ret = mdio_write(cfg->mdio, cfg->addr, reg_addr, (uint16_t)data);
		if (ret) {
			return ret;
		}
	} else {
		ret = phy_ti_DP83TD555_indirect_c22(dev, phy_ti_DP83TD555_mmd_range(reg_addr), reg_addr);
		if (ret) {
			return ret;
		}
		return  mdio_write(cfg->mdio, cfg->addr, MII_MMD_AADR, (uint16_t)data);
	}

	return 0;
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static int phy_ti_DP83TD510_clear_interrupt(struct ti_DP83TD510_data *data)
{
	const struct device *dev = data->dev;
	const struct ti_DP83TD510_config *config = dev->config;
	uint32_t reg_val;
	int ret;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read/clear PHY interrupt status register */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_MISR_REG, &reg_val);
	if (ret) {
		LOG_ERR("Error reading phy (%d) interrupt status register", config->addr);
	}

	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);

	return ret;
}

static void phy_ti_DP83TD510_interrupt_handler(const struct device *port, struct gpio_callback *cb,
					     gpio_port_pins_t pins)
{
	struct ti_DP83TD510_data *data = CONTAINER_OF(cb, struct ti_DP83TD510_data, gpio_callback);
	int ret;

	ret = k_work_reschedule(&data->phy_monitor_work, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to schedule phy_monitor_work from ISR");
	}
}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

static int phy_ti_DP83TD510_autonegotiate(const struct device *dev)
{
	const struct ti_DP83TD510_config *config = dev->config;
	int ret;
	uint32_t bmcr = 0;
	uint32_t an_ctrl = 0;

	/* Read control register to check isolate bit */
	ret = phy_ti_DP83TD510_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_AN_CTRL, &an_ctrl);
	if (ret) {
		LOG_ERR("Error reading phy (%d) auto-negotiation register", config->addr);
		return ret;
	}

	/* (re)start autonegotiation */
	LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
	
	/* Disable isolate mode, enable auto-neg, reset auto-neg */
	bmcr &= ~MII_BMCR_ISOLATE;
	an_ctrl &= PHY_TI_DP83TD510_AN_CTRL_EN;
	an_ctrl &= PHY_TI_DP83TD510_AN_CTRL_RST;

	ret = phy_ti_DP83TD510_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	ret = phy_ti_DP83TD510_write(dev, PHY_TI_DP83TD510_AN_CTRL, an_ctrl);
	if (ret) {
		LOG_ERR("Error writing phy (%d) auto-negotiation control register", config->addr);
		return ret;
	}
	return 0;
}

static int phy_ti_DP83TD510_get_link(const struct device *dev, struct phy_link_state *state)
{
	const struct ti_DP83TD510_config *config = dev->config;
	struct ti_DP83TD510_data *data = dev->data;
	int ret;
	uint32_t bmsr = 0;
	uint32_t anar = 0;
	uint32_t anlpar = 0;
	uint32_t mutual_capabilities;
	bool tx_swing_2p4 = false;
	bool eee_en = false;
	struct phy_link_state old_state = data->state;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read link state */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_PHY_STS, &bmsr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic status register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	state->is_up = bmsr & PHY_TI_DP83TD510_PHY_STS_LINK;

	if (!state->is_up) {
		k_mutex_unlock(&data->mutex);
		goto result;
	}

	/* Read currently configured advertising options */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Read link partner capability */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_ANLPAR, &anlpar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) link partner register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	mutual_capabilities = anar & anlpar;

	if (mutual_capabilities & PHY_TI_DP83TD510_ANAR_EEE) {
		eee_en = true;
	}
	if (mutual_capabilities & PHY_TI_DP83TD510_ANAR_TXLVL) {
		tx_swing_2p4 = true;
	}
	
	state->speed = LINK_FULL_10BASE;

result:
	if (memcmp(&old_state, state, sizeof(struct phy_link_state)) != 0) {
		LOG_DBG("PHY %d is %s", config->addr, state->is_up ? "up" : "down");
		if (state->is_up) {
			LOG_INF("PHY (%d) Link speed %s Mb, %s duplex, eee mode %s, increased tx swing %s", 
				config->addr,
				(PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
				(PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half"),
				eee_en ? "enabled" : "disabled",
				tx_swing_2p4 ? "enabled" : "disabled");
		}
	}

	return ret;
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
static int phy_ti_DP83TD510_static_cfg(const struct device *dev)
{
	const struct ti_DP83TD510_config *config = dev->config;
	uint32_t reg_val = 0;
	int ret = 0;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	struct ti_DP83TD510_data *data = dev->data;
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	/* Select correct reference clock mode depending on interface setup */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_RCSR_REG, (uint32_t *)&reg_val);
	if (ret) {
		return ret;
	}

	if (config->phy_iface == DP83TD510_RMII) {
		reg_val |= PHY_TI_DP83TD510_RCSR_REF_CLK_SEL;
	} else {
		reg_val &= ~PHY_TI_DP83TD510_RCSR_REF_CLK_SEL;
	}

	ret = phy_ti_DP83TD510_write(dev, PHY_TI_DP83TD510_RCSR_REG, (uint32_t)reg_val);
	if (ret) {
		return ret;
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	/* Read PHYSCR register to write back */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_PHYSCR_REG, &reg_val);
	if (ret) {
		return ret;
	}

	/* Config INTR/PWRDN pin as Interrupt output, enable event interrupts */
	reg_val |= PHY_TI_DP83TD510_PHYSCR_REG_IOE | PHY_TI_DP83TD510_PHYSCR_REG_IE;

	/* Write settings to physcr register */
	ret = phy_ti_DP83TD510_write(dev, PHY_TI_DP83TD510_PHYSCR_REG, reg_val);
	if (ret) {
		return ret;
	}

	/* Clear interrupt */
	ret = phy_ti_DP83TD510_clear_interrupt(data);
	if (ret) {
		return ret;
	}

	/* Read MISR register to write back */
	ret = phy_ti_DP83TD510_read(dev, PHY_TI_DP83TD510_MISR_REG, &reg_val);
	if (ret) {
		return ret;
	}

	/* Enable link state changed interrupt*/
	reg_val |= PHY_TI_DP83TD510_MISR_REG_LSCE;

	/* Write settings to misr register */
	ret = phy_ti_DP83TD510_write(dev, PHY_TI_DP83TD510_MISR_REG, reg_val);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	return ret;
}

static int phy_ti_DP83TD510_reset(const struct device *dev)
{
	printk("_reset\n");
	const struct ti_DP83TD510_config *config = dev->config;
	struct ti_DP83TD510_data *data = dev->data;
	int ret;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	if (!config->reset_gpio.port) {
		goto skip_reset_gpio;
	}

	/* Start reset (logically ACTIVE, physically LOW) */
	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret) {
		goto done;
	}

	/* Reset pulse (minimum specified width is T1=25us) */
	k_busy_wait(USEC_PER_MSEC * 1);

	/* Reset over (logically INACTIVE, physically HIGH) */
	ret = gpio_pin_set_dt(&config->reset_gpio, 0);

	/* POR release time (minimum specified is T4=50ms) */
	k_busy_wait(USEC_PER_MSEC * PHY_TI_DP83TD510_POR_DELAY);

	goto done;
skip_reset_gpio:
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */
	ret = phy_ti_DP83TD510_write(dev, MII_BMCR, MII_BMCR_RESET);
	if (ret) {
		goto done;
	}
	/* POR release time (minimum specified is T4=50ms) */
	k_busy_wait(USEC_PER_MSEC * PHY_TI_DP83TD510_POR_DELAY);

done:
	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	LOG_DBG("PHY (%d) reset completed", config->addr);

	return ret;
}

static int phy_ti_DP83TD510_cfg_link(const struct device *dev, enum phy_link_speed speeds,
				   enum phy_cfg_link_flag flags)
{
	const struct ti_DP83TD510_config *config = dev->config;
	struct ti_DP83TD510_data *data = dev->data;
	int ret;

	if (flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) {
		LOG_ERR("Disabling auto-negotiation is not supported by this driver");
		return -ENOTSUP;
	}

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		goto done;
	}

	/* We are going to reconfigure the phy, don't need to monitor until done */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_cancel_delayable(&data->phy_monitor_work);
	}
#else
	k_work_cancel_delayable(&data->phy_monitor_work);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	/* Reset PHY */
	ret = phy_ti_DP83TD510_reset(dev);
	if (ret) {
		goto done;
	}

	/* DT configurations */
	ret = phy_ti_DP83TD510_static_cfg(dev);
	if (ret) {
		goto done;
	}

	/* (re)do autonegotiation */
	ret = phy_ti_DP83TD510_autonegotiate(dev);
	if (ret && (ret != -ENETDOWN)) {
		LOG_ERR("Error in autonegotiation");
		goto done;
	}

done:
	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	/* Start monitoring */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	}
#else
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	return ret;
}

static int phy_ti_DP83TD510_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct ti_DP83TD510_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_ti_DP83TD510_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

static void phy_ti_DP83TD510_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ti_DP83TD510_data *data =
		CONTAINER_OF(dwork, struct ti_DP83TD510_data, phy_monitor_work);
	const struct device *dev = data->dev;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct ti_DP83TD510_config *config = dev->config;
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */
	struct phy_link_state state = {};
	int ret;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (config->interrupt_gpio.port) {
		ret = phy_ti_DP83TD510_clear_interrupt(data);
		if (ret) {
			return;
		}
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	ret = phy_ti_DP83TD510_get_link(dev, &state);

	if (ret == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	}
#else
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */
}

static int phy_ti_DP83TD510_init(const struct device *dev)
{
	const struct ti_DP83TD510_config *config = dev->config;
	struct ti_DP83TD510_data *data = dev->data;
	int ret;
	uint32_t phy_id;
	uint16_t val;

	data->dev = dev;

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	if (config->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

	k_work_init_delayable(&data->phy_monitor_work, phy_ti_DP83TD510_monitor_work_handler);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		phy_ti_DP83TD510_monitor_work_handler(&data->phy_monitor_work.work);
		goto skip_int_gpio;
	}

	/* Configure interrupt pin */
	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&data->gpio_callback, phy_ti_DP83TD510_interrupt_handler,
			   BIT(config->interrupt_gpio.pin));
	ret = gpio_add_callback_dt(&config->interrupt_gpio, &data->gpio_callback);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		return ret;
	}

skip_int_gpio:
#else
	phy_ti_DP83TD510_monitor_work_handler(&data->phy_monitor_work.work);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */


	//read and return 510 PHYID
	ret = phy_ti_DP83TD510_read(dev, MII_PHYID1R, (uint32_t *)&val);
	if (ret) {
		LOG_ERR("Error reading MII_PHYID1R");
		return ret;
	}

	phy_id = (val & UINT16_MAX) << 16;

	ret = phy_ti_DP83TD510_read(dev, MII_PHYID2R, (uint32_t *)&val);
	if (ret) {
		LOG_ERR("Error reading MII_PHYID2R");
		return ret;
	}

	phy_id |= (val & UINT16_MAX);

	if(phy_id == DP83TD510E_PHYID) {
		LOG_INF("Recognized DP83TD510E PHY_ID: 0x%X\n", phy_id);
	} else {
		LOG_ERR("Unexpected PHY_ID: 0x%X", phy_id);
	}

	/* Advertise default speeds */
	phy_ti_DP83TD510_cfg_link(dev, config->default_speeds, 0);
	return 0;
}

static DEVICE_API(ethphy, ti_DP83TD510_phy_api) = {
	.get_link = phy_ti_DP83TD510_get_link,
	.cfg_link = phy_ti_DP83TD510_cfg_link,
	.link_cb_set = phy_ti_DP83TD510_link_cb_set,
	.read = phy_ti_DP83TD510_read,
	.write = phy_ti_DP83TD510_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define RESET_GPIO(n) .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#define INTERRUPT_GPIO(n) .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

		//.default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n
#define TI_DP83TD510_INIT(n)                                                                         \
	static const struct ti_DP83TD510_config ti_DP83TD510_##n##_config = {                          \
		.addr = DT_INST_REG_ADDR(n),                                                       \
		.mdio = DEVICE_DT_GET(DT_INST_PARENT(n)),                                          \
		.phy_iface = DT_INST_ENUM_IDX(n, ti_interface_type),                               \
		RESET_GPIO(n) INTERRUPT_GPIO(n)};                                                  \
                                                                                                   \
	static struct ti_DP83TD510_data ti_DP83TD510_##n##_data;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &phy_ti_DP83TD510_init, NULL, &ti_DP83TD510_##n##_data,               \
			      &ti_DP83TD510_##n##_config, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,     \
			      &ti_DP83TD510_phy_api);

DT_INST_FOREACH_STATUS_OKAY(TI_DP83TD510_INIT)
