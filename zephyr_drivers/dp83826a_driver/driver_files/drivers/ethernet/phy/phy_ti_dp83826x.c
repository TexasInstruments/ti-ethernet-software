/*
* Copyright 2024 Bernhard Kraemer 
* Copyright 2026 Jae Yoon
 *
 * Inspiration from phy_ti_dp83825.c, which is:
 * Copyright 2023-2024 NXP
 * Copyright 2026 Texas Instruments
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_dp83826x

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>


#define LOG_MODULE_NAME phy_ti_dp83826x
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "phy_mii.h"
/* DP83826X PHYID */
#define DP83826C_PHY_ID		0x2000a130
#define DP83826NC_PHY_ID	0x2000a131
#define DP83826A_PHY_ID	    0x2000a134

/* DP83826X Register Definitions */
#define PHY_TI_DP83826X_MII_PHYSID1   0x02
#define PHY_TI_DP83826X_MII_PHYSID2   0x03

#define PHY_TI_DP83826X_PHYSCR_REG     0x11
#define PHY_TI_DP83826X_PHYSCR_REG_IE  BIT(1)
#define PHY_TI_DP83826X_PHYSCR_REG_IOE BIT(0)

#define PHY_TI_DP83826X_MISR_REG      0x12
#define PHY_TI_DP83826X_MISR_REG_LSCE BIT(5)

#define PHY_TI_DP83826X_RCSR_REG         0x17
#define PHY_TI_DP83826X_RCSR_RMII_MODE_EN   BIT(5)
#define PHY_TI_DP83826X_RCSR_RMII_MODE_SEL  BIT(7) // 0: RMII Leader 1: RMII Follower

/* DP83826X VOD Configuration Registers (Vendor Specific) */
#define PHY_TI_DP83826X_VOD_CFG1	0x30b
#define PHY_TI_DP83826X_VOD_CFG2	0x30c
#define PHY_TI_DP83826X_VOD_CFG3	0x30e

/* VOD Configuration Bit Masks */
#define DP83826X_VOD_CFG1_MINUS_MDIX_MASK	GENMASK(13, 12)
#define DP83826X_VOD_CFG1_MINUS_MDI_MASK	GENMASK(11, 6)
#define DP83826X_VOD_CFG2_MINUS_MDIX_MASK	GENMASK(15, 12)
#define DP83826X_VOD_CFG2_PLUS_MDIX_MASK	GENMASK(11, 6)
#define DP83826X_VOD_CFG2_PLUS_MDI_MASK		GENMASK(5, 0)

/* VOD DAC Constants */
#define DP83826X_CFG_DAC_MINUS_MDIX_5_TO_4	GENMASK(5, 4)
#define DP83826X_CFG_DAC_MINUS_MDIX_3_TO_0	GENMASK(3, 0)
#define DP83826X_CFG_DAC_PERCENT_PER_STEP	625
#define DP83826X_CFG_DAC_PERCENT_DEFAULT	10000
#define DP83826X_CFG_DAC_MINUS_DEFAULT		0x30
#define DP83826X_CFG_DAC_PLUS_DEFAULT		0x10

#define PHY_TI_DP83826X_POR_DELAY 50

enum dp83826x_interface {
    DP83826X_MII,
    DP83826X_RMII_LEADER,
    DP83826X_RMII_FOLLOWER,
};

struct ti_dp83826x_config {
    uint8_t addr;
    const struct device *mdio_dev;
    enum dp83826x_interface phy_iface;
    enum phy_link_speed default_speeds;
    /* VOD Configuration */
    int cfg_dac_minus;
    int cfg_dac_plus;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
    const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    const struct gpio_dt_spec interrupt_gpio;
#endif
};

struct ti_dp83826x_data {
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

static int phy_ti_dp83826x_read(const struct device *dev, uint16_t reg_addr, uint32_t *data)
{
    const struct ti_dp83826x_config *config = dev->config;
    int ret;

    /* Make sure excessive bits 16-31 are reset */
    *data = 0U;

    ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int phy_ti_dp83826x_write(const struct device *dev, uint16_t reg_addr, uint32_t data)
{
    const struct ti_dp83826x_config *config = dev->config;
    int ret;

    ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
    if (ret < 0) {
        return ret;
    }

    return 0;
}


static int phy_ti_dp83826x_extended_read(const struct device *dev, uint8_t devad, uint16_t regad, uint16_t *data)
{
    const struct ti_dp83826x_config *config = dev->config;
    int ret;

    ret = mdio_write(config->mdio_dev, config->addr, 0x0D, devad);
    if (ret < 0) {
        return ret;
    }
    ret = mdio_write(config->mdio_dev, config->addr, 0x0E, regad);
    if (ret < 0) {
        return ret;
    }
    ret = mdio_write(config->mdio_dev, config->addr, 0x0D, (0x4000 & devad));
    if (ret < 0) {
        return ret;
    }

    /* Make sure excessive bits 16-31 are reset */
    *data = 0U;

    ret = mdio_read(config->mdio_dev, config->addr, 0x0E, data);
    if (ret < 0) {
        return ret;
    }

    return 0;
} 

static int phy_ti_dp83826x_extended_write(const struct device *dev, uint8_t devad, uint16_t regad, uint16_t data)
{
    const struct ti_dp83826x_config *config = dev->config;
    int ret;

    ret = mdio_write(config->mdio_dev, config->addr, 0x0D, devad);
    if (ret < 0) {
        return ret;
    }
    ret = mdio_write(config->mdio_dev, config->addr, 0x0E, regad);
    if (ret < 0) {
        return ret;
    }
    ret = mdio_write(config->mdio_dev, config->addr, 0x0D, (0x4000 & devad));
    if (ret < 0) {
        return ret;
    }

    ret = mdio_write(config->mdio_dev, config->addr, 0x0E, data);
    if (ret < 0) {
        return ret;
    }


    return 0;
} 
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static int phy_ti_dp83826x_clear_interrupt(struct ti_dp83826x_data *data)
{
    const struct device *dev = data->dev;
    const struct ti_dp83826x_config *config = dev->config;
    uint32_t reg_val;
    int ret;

    /* Lock mutex */
    ret = k_mutex_lock(&data->mutex, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("PHY mutex lock error");
        return ret;
    }

    /* Read/clear PHY interrupt status register */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_MISR_REG, &reg_val);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) interrupt status register", config->addr);
    }

    /* Unlock mutex */
    (void)k_mutex_unlock(&data->mutex);

    return ret;
}

static void phy_ti_dp83826x_interrupt_handler(const struct device *port, struct gpio_callback *cb,
                         gpio_port_pins_t pins)
{
    struct ti_dp83826x_data *data = CONTAINER_OF(cb, struct ti_dp83826x_data, gpio_callback);
    int ret;

    ret = k_work_reschedule(&data->phy_monitor_work, K_NO_WAIT);
    if (ret < 0) {
        LOG_ERR("Failed to schedule phy_monitor_work from ISR");
    }
}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

static int phy_ti_dp83826x_autonegotiate(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    int ret;
    uint32_t bmcr = 0;

    /* Read control register to write back with autonegotiation bit */
    ret = phy_ti_dp83826x_read(dev, MII_BMCR, &bmcr);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) basic control register", config->addr);
        return ret;
    }

    /* (re)start autonegotiation */
    LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
    bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
    bmcr &= ~MII_BMCR_ISOLATE;

    ret = phy_ti_dp83826x_write(dev, MII_BMCR, bmcr);
    if (ret < 0) {
        LOG_ERR("Error writing phy (%d) basic control register", config->addr);
        return ret;
    }

    return 0;
}

static int phy_ti_dp83826x_get_link(const struct device *dev, struct phy_link_state *state)
{
    const struct ti_dp83826x_config *config = dev->config;
    struct ti_dp83826x_data *data = dev->data;
    int ret;
    uint32_t bmsr = 0;
    uint32_t anar = 0;
    uint32_t anlpar = 0;
    uint32_t mutual_capabilities;
    struct phy_link_state old_state = data->state;

    /* Lock mutex */
    ret = k_mutex_lock(&data->mutex, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("PHY mutex lock error");
        return ret;
    }

    /* Read link state */
    ret = phy_ti_dp83826x_read(dev, MII_BMSR, &bmsr);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) basic status register", config->addr);
        k_mutex_unlock(&data->mutex);
        return ret;
    }
    state->is_up = bmsr & MII_BMSR_LINK_STATUS;

    if (!state->is_up) {
        k_mutex_unlock(&data->mutex);
        goto result;
    }

    /* Read currently configured advertising options */
    ret = phy_ti_dp83826x_read(dev, MII_ANAR, &anar);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) advertising register", config->addr);
        k_mutex_unlock(&data->mutex);
        return ret;
    }

    /* Read link partner capability */
    ret = phy_ti_dp83826x_read(dev, MII_ANLPAR, &anlpar);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) link partner register", config->addr);
        k_mutex_unlock(&data->mutex);
        return ret;
    }

    /* Unlock mutex */
    k_mutex_unlock(&data->mutex);

    mutual_capabilities = anar & anlpar;

    if (mutual_capabilities & MII_ADVERTISE_100_FULL) {
        state->speed = LINK_FULL_100BASE;
    } else if (mutual_capabilities & MII_ADVERTISE_100_HALF) {
        state->speed = LINK_HALF_100BASE;
    } else if (mutual_capabilities & MII_ADVERTISE_10_FULL) {
        state->speed = LINK_FULL_10BASE;
    } else if (mutual_capabilities & MII_ADVERTISE_10_HALF) {
        state->speed = LINK_HALF_10BASE;
    } else {
        return -EIO;
    }

result:
    if (memcmp(&old_state, state, sizeof(struct phy_link_state)) != 0) {
        LOG_DBG("PHY %d is %s", config->addr, state->is_up ? "up" : "down");
        if (state->is_up) {
            LOG_INF("PHY (%d) Link speed %s Mb, %s duplex\n", config->addr,
                (PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
                PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
        }
    }

    return ret;
}

/*
 * Configure VOD (Voltage Output Driver) settings for DP83826X
 * These are vendor-specific registers for signal amplitude tuning
 */
 // TODO: Harder comparison on this code. It looks kinda different from dp83822.c
static int phy_ti_dp83826x_config_vod(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    uint16_t reg_val = 0;
    int ret = 0;

    /* Configure VOD CFG1 - Minus voltage for MDI/MDIX */
    if (config->cfg_dac_minus != DP83826X_CFG_DAC_MINUS_DEFAULT) {
        uint32_t val, mask;
        // TODO: Check FIELD_PREP / FIELD_GET is avail on Zephyr
        val = FIELD_PREP(DP83826X_VOD_CFG1_MINUS_MDI_MASK, config->cfg_dac_minus) |
              FIELD_PREP(DP83826X_VOD_CFG1_MINUS_MDIX_MASK,
                 FIELD_GET(DP83826X_CFG_DAC_MINUS_MDIX_5_TO_4,
                       config->cfg_dac_minus));
        mask = DP83826X_VOD_CFG1_MINUS_MDIX_MASK | DP83826X_VOD_CFG1_MINUS_MDI_MASK;
        
        ret = phy_ti_dp83826x_extended_read(dev, 0x1F ,PHY_TI_DP83826X_VOD_CFG1, &reg_val);
        if (ret < 0) {
            LOG_ERR("Error reading phy (%d) VOD_CFG1 register", config->addr);
            return ret;
        }

        reg_val = (reg_val & ~mask) | val;

        ret = phy_ti_dp83826x_extended_write(dev, 0x1F, PHY_TI_DP83826X_VOD_CFG1, reg_val);
        if (ret < 0) {
            LOG_ERR("Error writing phy (%d) VOD_CFG1 register", config->addr);
            return ret;
        }

        LOG_DBG("PHY (%d) VOD_CFG1 set to 0x%04x", config->addr, reg_val);
    }

    /* Configure VOD CFG2 - Plus voltage and remaining minus bits */
    if (config->cfg_dac_plus != DP83826X_CFG_DAC_PLUS_DEFAULT ||
        config->cfg_dac_minus != DP83826X_CFG_DAC_MINUS_DEFAULT) {
        uint32_t val, mask;

        val = FIELD_PREP(DP83826X_VOD_CFG2_PLUS_MDIX_MASK, (uint32_t) config->cfg_dac_plus) |
              FIELD_PREP(DP83826X_VOD_CFG2_PLUS_MDI_MASK, (uint32_t) config->cfg_dac_plus) |
              FIELD_PREP(DP83826X_VOD_CFG2_MINUS_MDIX_MASK,
                 FIELD_GET(DP83826X_CFG_DAC_MINUS_MDIX_3_TO_0,
                       (uint32_t) config->cfg_dac_minus));
        mask = DP83826X_VOD_CFG2_MINUS_MDIX_MASK |
               DP83826X_VOD_CFG2_PLUS_MDIX_MASK |
               DP83826X_VOD_CFG2_PLUS_MDI_MASK;

        ret = phy_ti_dp83826x_extended_read(dev, 0x1F, PHY_TI_DP83826X_VOD_CFG2, &reg_val);
        if (ret < 0) {
            LOG_ERR("Error reading phy (%d) VOD_CFG2 register", config->addr);
            return ret;
        }

        reg_val = (reg_val & ~mask) | val;

        ret = phy_ti_dp83826x_extended_write(dev, 0x1F, PHY_TI_DP83826X_VOD_CFG2, reg_val);
        if (ret < 0) {
            LOG_ERR("Error writing phy (%d) VOD_CFG2 register", config->addr);
            return ret;
        }

        LOG_DBG("PHY (%d) VOD_CFG2 set to 0x%04x", config->addr, reg_val);
    }

    return 0;
}

/*
 * Configure interface mode (MII, RMII)
 */
static int phy_ti_dp83826x_config_interface(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    uint32_t reg_val = 0;
    int ret;

    /* Read RCSR register */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_RCSR_REG, &reg_val);
    if (ret < 0) {
        LOG_ERR("Error reading phy (%d) RCSR register", config->addr);
        return ret;
    }

    /* Clear all interface-related bits first */
    reg_val &= ~(PHY_TI_DP83826X_RCSR_RMII_MODE_EN |
             PHY_TI_DP83826X_RCSR_RMII_MODE_SEL);

    switch (config->phy_iface) {
    case DP83826X_MII:
        /* MII mode: Clear RMII bits, use default MII */
        LOG_DBG("PHY (%d) configured for MII mode", config->addr);
        break;

    case DP83826X_RMII_LEADER:
        /* RMII Leader mode */
        reg_val |= PHY_TI_DP83826X_RCSR_RMII_MODE_EN;
        /* RMII_MODE_SEL = 0 for leader (internal clock) */
        LOG_DBG("PHY (%d) configured for RMII Leader mode", config->addr);
        break;

    case DP83826X_RMII_FOLLOWER:
        /* RMII Follower mode */
        reg_val |= PHY_TI_DP83826X_RCSR_RMII_MODE_EN |
               PHY_TI_DP83826X_RCSR_RMII_MODE_SEL;
        LOG_DBG("PHY (%d) configured for RMII Follower mode", config->addr);
        break;

    default:
        LOG_ERR("PHY (%d) invalid interface mode", config->addr);
        return -EINVAL;
    }

    /* Write updated RCSR register */
    ret = phy_ti_dp83826x_write(dev, PHY_TI_DP83826X_RCSR_REG, reg_val);
    if (ret < 0) {
        LOG_ERR("Error writing phy (%d) RCSR register", config->addr);
        return ret;
    }

    return 0;
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
static int phy_ti_dp83826x_static_cfg(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    struct ti_dp83826x_data *data = dev->data;
#endif

    int ret = 0;

    /* Configure interface mode */
    ret = phy_ti_dp83826x_config_interface(dev);
    if (ret < 0) {
        LOG_ERR("PHY (%d) interface configuration failed", config->addr);
        return ret;
    }

    /* Configure VOD settings for DP83826X */
    ret = phy_ti_dp83826x_config_vod(dev);
    if (ret < 0) {
        LOG_ERR("PHY (%d) VOD configuration failed", config->addr);
        return ret;
    }

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    uint32_t reg_val = 0;
    /* Read PHYSCR register to write back */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_PHYSCR_REG, &reg_val);
    if (ret < 0) {
        return ret;
    }

    /* Config INTR/PWRDN pin as Interrupt output, enable event interrupts */
    reg_val |= PHY_TI_DP83826X_PHYSCR_REG_IOE | PHY_TI_DP83826X_PHYSCR_REG_IE;

    /* Write settings to physcr register */
    ret = phy_ti_dp83826x_write(dev, PHY_TI_DP83826X_PHYSCR_REG, reg_val);
    if (ret < 0) {
        return ret;
    }

    /* Clear interrupt */
    ret = phy_ti_dp83826x_clear_interrupt(data);
    if (ret < 0) {
        return ret;
    }

    /* Read MISR register to write back */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_MISR_REG, &reg_val);
    if (ret < 0) {
        return ret;
    }

    /* Enable link state changed interrupt */
    reg_val |= PHY_TI_DP83826X_MISR_REG_LSCE;

    /* Write settings to misr register */
    ret = phy_ti_dp83826x_write(dev, PHY_TI_DP83826X_MISR_REG, reg_val);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

    return ret;
}

static int phy_ti_dp83826x_reset(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    struct ti_dp83826x_data *data = dev->data;
    int ret;

    /* Lock mutex */
    ret = k_mutex_lock(&data->mutex, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("PHY mutex lock error");
        return ret;
    }

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
    if (!config->reset_gpio.port) {
        goto skip_reset_gpio;
    }

    /* Start reset (logically ACTIVE, physically LOW) */
    ret = gpio_pin_set_dt(&config->reset_gpio, 1);
    if (ret < 0) {
        goto done;
    }

    /* Reset pulse (minimum specified width is T1=25us) */
    k_busy_wait(USEC_PER_MSEC * 1);

    /* Reset over (logically INACTIVE, physically HIGH) */
    ret = gpio_pin_set_dt(&config->reset_gpio, 0);

    /* POR release time (minimum specified is T4=50ms) */
    k_busy_wait(USEC_PER_MSEC * PHY_TI_DP83826X_POR_DELAY);

    goto done;
skip_reset_gpio:
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */
    ret = phy_ti_dp83826x_write(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret < 0) {
        goto done;
    }
    /* POR release time (minimum specified is T4=50ms) */
    k_busy_wait(USEC_PER_MSEC * PHY_TI_DP83826X_POR_DELAY);

done:
    /* Unlock mutex */
    k_mutex_unlock(&data->mutex);

    LOG_DBG("PHY (%d) reset completed", config->addr);

    return ret;
}

static int phy_ti_dp83826x_cfg_link(const struct device *dev, enum phy_link_speed speeds,
                   enum phy_cfg_link_flag flags)
{
    const struct ti_dp83826x_config *config = dev->config;
    struct ti_dp83826x_data *data = dev->data;
    int ret;

    if (flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) {
        LOG_ERR("Disabling auto-negotiation is not supported by this driver");
        return -ENOTSUP;
    }

    /* Lock mutex */
    ret = k_mutex_lock(&data->mutex, K_FOREVER);
    if (ret < 0) {
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
    ret = phy_ti_dp83826x_reset(dev);
    if (ret < 0) {
        goto done;
    }

    /* DT configurations */
    ret = phy_ti_dp83826x_static_cfg(dev);
    if (ret < 0) {
        goto done;
    }

    ret = phy_mii_set_anar_reg(dev, speeds);
    if ((ret < 0) && (ret != -EALREADY)) {
        LOG_ERR("Error setting ANAR register for phy (%d)", config->addr);
        goto done;
    }

    /* (re)do autonegotiation */
    ret = phy_ti_dp83826x_autonegotiate(dev);
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

static int phy_ti_dp83826x_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
    struct ti_dp83826x_data *data = dev->data;

    data->cb = cb;
    data->cb_data = user_data;

    phy_ti_dp83826x_get_link(dev, &data->state);

    data->cb(dev, &data->state, data->cb_data);

    return 0;
}

static void phy_ti_dp83826x_monitor_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct ti_dp83826x_data *data =
        CONTAINER_OF(dwork, struct ti_dp83826x_data, phy_monitor_work);
    const struct device *dev = data->dev;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    const struct ti_dp83826x_config *config = dev->config;
#endif
    struct phy_link_state state = {};
    int ret;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    if (config->interrupt_gpio.port) {
        ret = phy_ti_dp83826x_clear_interrupt(data);
        if (ret < 0) {
            return;
        }
    }
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

    ret = phy_ti_dp83826x_get_link(dev, &state);

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

/*
 * Read PHY ID from registers 0x02 (PHYSID1) and 0x03 (PHYSID2)
 * 
 * Register format:
 *   PHYSID1 (0x02): [15:0] = OUI[21:6]
 *   PHYSID2 (0x03): [15:10] = OUI[5:0]
 *                   [9:4] = Model Number
 *                   [3:0] = Revision
 * 
 * Combined PHY ID: [31:0] = OUI[21:0] (24-bit) | Model (6-bit) | Rev (4-bit)
 */
static int phy_ti_dp83826x_get_phyid(const struct device *dev, uint32_t *phyid)
{
    const struct ti_dp83826x_config *config = dev->config;
    uint32_t physid1 = 0;
    uint32_t physid2 = 0;
    int ret;

    if (!phyid) {
        LOG_ERR("PHY (%d) phyid pointer is NULL", config->addr);
        return -EINVAL;
    }

    /* Read PHYSID1 register (0x02) */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_MII_PHYSID1 , &physid1);
    if (ret < 0) {
        LOG_ERR("PHY (%d) failed to read PHYSID1 register", config->addr);
        return ret;
    }

    /* Read PHYSID2 register (0x03) */
    ret = phy_ti_dp83826x_read(dev, PHY_TI_DP83826X_MII_PHYSID2, &physid2);
    if (ret < 0) {
        LOG_ERR("PHY (%d) failed to read PHYSID2 register", config->addr);
        return ret;
    }

    /* Combine registers into 32-bit PHY ID */
    *phyid = (physid1 << 16) | physid2;
    LOG_DBG("PHY (%d) PHYSID1=0x%04x, PHYSID2=0x%04x, Combined ID=0x%08x",
        config->addr, physid1, physid2, *phyid);

    return 0;
}

/*
 * Verify that the PHY ID matches a supported device
 * Returns 0 if valid, negative error code otherwise
 */
static int phy_ti_dp83826x_verify_phyid(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    uint32_t phyid = 0;
    int ret;

    /* Read the PHY ID from registers */
    ret = phy_ti_dp83826x_get_phyid(dev, &phyid);
    if (ret < 0) {
        LOG_ERR("PHY (%d) failed to read PHY ID", config->addr);
        return ret;
    }

    /* Check against supported PHY IDs */
    switch (phyid) {
    case (DP83826C_PHY_ID):
        LOG_INF("PHY (%d) detected: DP83826C", config->addr);
        return 0;

    case (DP83826NC_PHY_ID):
        LOG_INF("PHY (%d) detected: DP83826NC", config->addr);
        return 0;

    case (DP83826A_PHY_ID):
        LOG_INF("PHY (%d) detected: DP83826A", config->addr);
        return 0;

    default:
        LOG_ERR("PHY (%d) unknown PHY ID: 0x%08x",
            config->addr, phyid);
        return -ENODEV;
    }
}

static int phy_ti_dp83826x_init(const struct device *dev)
{
    const struct ti_dp83826x_config *config = dev->config;
    struct ti_dp83826x_data *data = dev->data;
    int ret;

    data->dev = dev;
    mdio_bus_enable(config->mdio_dev);
    ret = k_mutex_init(&data->mutex);
    if (ret < 0) {
        return ret;
    }
    
    /* ===== NEW: Verify PHY ID ===== */
    ret = phy_ti_dp83826x_verify_phyid(dev);
    if (ret < 0) {
        LOG_ERR("PHY (%d) verification failed: 0x%d", config->addr, ret);
        return ret;
    }
    /* ===== END: PHY ID verification ===== */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
    if (config->reset_gpio.port) {
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            return ret;
        }
    }
#endif

    k_work_init_delayable(&data->phy_monitor_work, phy_ti_dp83826x_monitor_work_handler);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
    if (!config->interrupt_gpio.port) {
        phy_ti_dp83826x_monitor_work_handler(&data->phy_monitor_work.work);
        goto skip_int_gpio;
    }

    /* Configure interrupt pin */
    ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }

    gpio_init_callback(&data->gpio_callback, phy_ti_dp83826x_interrupt_handler,
               BIT(config->interrupt_gpio.pin));
    ret = gpio_add_callback_dt(&config->interrupt_gpio, &data->gpio_callback);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        return ret;
    }

skip_int_gpio:
#else
    phy_ti_dp83826x_monitor_work_handler(&data->phy_monitor_work.work);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */
    /* Check if the PHYIC matches */

    /* Advertise default speeds */
    phy_ti_dp83826x_cfg_link(dev, config->default_speeds, 0);

    return 0;
}

static DEVICE_API(ethphy, ti_dp83826x_phy_api) = {
    .get_link = phy_ti_dp83826x_get_link,
    .cfg_link = phy_ti_dp83826x_cfg_link,
    .link_cb_set = phy_ti_dp83826x_link_cb_set,
    .read = phy_ti_dp83826x_read,
    .write = phy_ti_dp83826x_write,
    .read_c45 = phy_ti_dp83826x_extended_read,
    .write_c45 = phy_ti_dp83826x_extended_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define RESET_GPIO(n) .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),
#else
#define RESET_GPIO(n)
#endif

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#define INTERRUPT_GPIO(n) .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif

/*
 * Helper macros for VOD configuration from device tree
 * These read optional ti,cfg-dac-minus-one-bp and ti,cfg-dac-plus-one-bp
 * Device tree values are percentages (0-20000 = 0%-200% of default)
 */
#define DP83826X_CFG_DAC_MINUS(n) \
    DT_INST_PROP_OR(n, ti_cfg_dac_minus_one_bp, DP83826X_CFG_DAC_MINUS_DEFAULT)

#define DP83826X_CFG_DAC_PLUS(n) \
    DT_INST_PROP_OR(n, ti_cfg_dac_plus_one_bp, DP83826X_CFG_DAC_PLUS_DEFAULT)


#define TI_DP83826X_INIT(n)                                                                        \
    static const struct ti_dp83826x_config ti_dp83826x_##n##_config = {                       \
        .addr = DT_INST_REG_ADDR(n),                                                       \
        .mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
        .phy_iface = DT_INST_ENUM_IDX(n, ti_interface_type),                               \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),                             \
        .cfg_dac_minus = DP83826X_CFG_DAC_MINUS(n),                                        \
        .cfg_dac_plus = DP83826X_CFG_DAC_PLUS(n),                                          \
        RESET_GPIO(n) INTERRUPT_GPIO(n)};                                                  \
                                                                                                   \
    static struct ti_dp83826x_data ti_dp83826x_##n##_data;                                    \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &phy_ti_dp83826x_init, NULL, &ti_dp83826x_##n##_data,            \
                  &ti_dp83826x_##n##_config, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,     \
                  &ti_dp83826x_phy_api);

DT_INST_FOREACH_STATUS_OKAY(TI_DP83826X_INIT)