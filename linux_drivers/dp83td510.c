// SPDX-License-Identifier: GPL-2.0
/* Driver for the Texas Instruments DP83TD510 PHY
 * Copyright (c) 2022 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>

#define DP83TD510E_PHY_ID			0x20000181

/* MDIO_MMD_VEND2 registers */
#define DP83TD510E_PHY_STS			0x10
#define DP83TD510E_STS_MII_INT			BIT(7)
#define DP83TD510E_LINK_STATUS			BIT(0)

#define DP83TD510E_GEN_CFG			0x11
#define DP83TD510E_GENCFG_INT_POLARITY		BIT(3)
#define DP83TD510E_GENCFG_INT_EN		BIT(1)
#define DP83TD510E_GENCFG_INT_OE		BIT(0)

#define DP83TD510E_INTERRUPT_REG_1		0x12
#define DP83TD510E_INT1_LINK			BIT(13)
#define DP83TD510E_INT1_LINK_EN			BIT(5)

#define DP83TD510E_MAC_CFG_1 	0x17
#define DP83510_MAC_CFG_1_CFG_RGMII_RX_CLK_SHIFT_SEL	BIT(12)
#define DP83510_MAC_CFG_1_CFG_RGMII_TX_CLK_SHIFT_SEL	BIT(11)

#define DP83TD510E_LEDS_CFG_1 		0x460
#define DP83TD510E_IO_MUX_GPIO_CTRL_1 	0x462
#define DP83TD510E_IO_MUX_GPIO_CTRL_2 	0x463
#define DP83TD510E_LEDS_CFG_2 		0x469


#define DP83TD510E_AN_STAT_1			0x60c
#define DP83TD510E_MASTER_SLAVE_RESOL_FAIL	BIT(15)

#define DP83TD510E_MSE_DETECT			0xa85

#define DP83TD510_SQI_MAX	7

/* Register values are converted to SNR(dB) as suggested by
 * "Application Report - DP83TD510E Cable Diagnostics Toolkit":
 * SNR(dB) = -10 * log10 (VAL/2^17) - 1.76 dB.
 * SQI ranges are implemented according to "OPEN ALLIANCE - Advanced diagnostic
 * features for 100BASE-T1 automotive Ethernet PHYs"
 */
static const u16 dp83td510_mse_sqi_map[] = {
	0x0569, /* < 18dB */
	0x044c, /* 18dB =< SNR < 19dB */
	0x0369, /* 19dB =< SNR < 20dB */
	0x02b6, /* 20dB =< SNR < 21dB */
	0x0227, /* 21dB =< SNR < 22dB */
	0x01b6, /* 22dB =< SNR < 23dB */
	0x015b, /* 23dB =< SNR < 24dB */
	0x0000  /* 24dB =< SNR */
};

struct dp83td510_private {
	u32 rx_id_delay;
	u32 tx_id_delay;
	u32 leds_cfg_1;
	u32 leds_cfg_2;
	u32 io_mux_gpio_ctrl_1;
	u32 io_mux_gpio_ctrl_2;
	bool rgmii_tx_clk_shift_sel;
	bool rgmii_rx_clk_shift_sel;
};

static int dp83td510_config_intr(struct phy_device *phydev)
{
	int ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		/* Clear any pending interrupts */
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_PHY_STS,
				    0x0);
		if (ret)
			return ret;

		ret = phy_write_mmd(phydev, MDIO_MMD_VEND2,
				    DP83TD510E_INTERRUPT_REG_1,
				    DP83TD510E_INT1_LINK_EN);
		if (ret)
			return ret;

		ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2,
				       DP83TD510E_GEN_CFG,
				       DP83TD510E_GENCFG_INT_POLARITY |
				       DP83TD510E_GENCFG_INT_EN |
				       DP83TD510E_GENCFG_INT_OE);
	} else {
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND2,
				    DP83TD510E_INTERRUPT_REG_1, 0x0);
		if (ret)
			return ret;

		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_VEND2,
					 DP83TD510E_GEN_CFG,
					 DP83TD510E_GENCFG_INT_EN);
		if (ret)
			return ret;

		/* Clear any pending interrupts */
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_PHY_STS,
				    0x0);
	}

	return ret;
}

static irqreturn_t dp83td510_handle_interrupt(struct phy_device *phydev)
{
	int  ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_PHY_STS);
	if (ret < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	} else if (!(ret & DP83TD510E_STS_MII_INT)) {
		return IRQ_NONE;
	}

	/* Read the current enabled interrupts */
	ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_INTERRUPT_REG_1);
	if (ret < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	} else if (!(ret & DP83TD510E_INT1_LINK_EN) ||
		   !(ret & DP83TD510E_INT1_LINK)) {
		return IRQ_NONE;
	}

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int dp83td510_read_status(struct phy_device *phydev)
{
	u16 phy_sts;
	int ret;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;
	linkmode_zero(phydev->lp_advertising);

	phy_sts = phy_read(phydev, DP83TD510E_PHY_STS);

	phydev->link = !!(phy_sts & DP83TD510E_LINK_STATUS);
	if (phydev->link) {
		/* This PHY supports only one link mode: 10BaseT1L_Full */
		phydev->duplex = DUPLEX_FULL;
		phydev->speed = SPEED_10;

		if (phydev->autoneg == AUTONEG_ENABLE) {
			ret = genphy_c45_read_lpa(phydev);
			if (ret)
				return ret;

			phy_resolve_aneg_linkmode(phydev);
		}
	}

	if (phydev->autoneg == AUTONEG_ENABLE) {
		ret = genphy_c45_baset1_read_status(phydev);
		if (ret < 0)
			return ret;

		ret = phy_read_mmd(phydev, MDIO_MMD_VEND2,
				   DP83TD510E_AN_STAT_1);
		if (ret < 0)
			return ret;

		if (ret & DP83TD510E_MASTER_SLAVE_RESOL_FAIL)
			phydev->master_slave_state = MASTER_SLAVE_STATE_ERR;
	} else {
		return genphy_c45_pma_baset1_read_master_slave(phydev);
	}

	return 0;
}

static int dp83td510_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	int ret;

	ret = genphy_c45_pma_baset1_setup_master_slave(phydev);
	if (ret < 0)
		return ret;

	if (phydev->autoneg == AUTONEG_DISABLE)
		return genphy_c45_an_disable_aneg(phydev);

	ret = genphy_c45_an_config_aneg(phydev);
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	return genphy_c45_check_and_restart_aneg(phydev, changed);
}

static int dp83td510_get_sqi(struct phy_device *phydev)
{
	int sqi, ret;
	u16 mse_val;

	if (!phydev->link)
		return 0;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_MSE_DETECT);
	if (ret < 0)
		return ret;

	mse_val = 0xFFFF & ret;
	for (sqi = 0; sqi < ARRAY_SIZE(dp83td510_mse_sqi_map); sqi++) {
		if (mse_val >= dp83td510_mse_sqi_map[sqi])
			return sqi;
	}

	return -EINVAL;
}

static int dp83td510_get_sqi_max(struct phy_device *phydev)
{
	return DP83TD510_SQI_MAX;
}

static int dp83td510_get_features(struct phy_device *phydev)
{
	/* This PHY can't respond on MDIO bus if no RMII clock is enabled.
	 * In case RMII mode is used (most meaningful mode for this PHY) and
	 * the PHY do not have own XTAL, and CLK providing MAC is not probed,
	 * we won't be able to read all needed ability registers.
	 * So provide it manually.
	 */

	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT1L_Full_BIT,
			 phydev->supported);

	return 0;
}

static int dp83td510_config_init(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510 = phydev->priv;
	int mac_cfg;

	phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_LEDS_CFG_1, dp83td510->leds_cfg_1); 
	phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_LEDS_CFG_2, dp83td510->leds_cfg_2); 
	phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_IO_MUX_GPIO_CTRL_1, dp83td510->io_mux_gpio_ctrl_1); 
	phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_IO_MUX_GPIO_CTRL_2, dp83td510->io_mux_gpio_ctrl_2); 
	
	mac_cfg = phy_read_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_MAC_CFG_1);

	mac_cfg &= ~DP83510_MAC_CFG_1_CFG_RGMII_RX_CLK_SHIFT_SEL;
	if(dp83td510->rgmii_rx_clk_shift_sel)
		mac_cfg |= DP83510_MAC_CFG_1_CFG_RGMII_RX_CLK_SHIFT_SEL;

	mac_cfg &= ~DP83510_MAC_CFG_1_CFG_RGMII_TX_CLK_SHIFT_SEL;
	if(dp83td510->rgmii_tx_clk_shift_sel)
		mac_cfg |= DP83510_MAC_CFG_1_CFG_RGMII_TX_CLK_SHIFT_SEL;

	phy_write_mmd(phydev, MDIO_MMD_VEND2, DP83TD510E_MAC_CFG_1, mac_cfg); 

	return 0; 
}

static int dp83td510_of_init(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510 = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	int ret;

	if (!of_node)
		return -ENODEV;
	
	ret = of_property_read_u32(of_node, "ti,leds-cfg-1",
				&dp83td510->leds_cfg_1);

	/* if not defined in DTS, keep default */
	if (ret != 0){
		dp83td510->leds_cfg_1 = 0x458;
	}

	ret = of_property_read_u32(of_node, "ti,leds-cfg-2",
				&dp83td510->leds_cfg_2);
	
	/* if not defined in DTS, keep default */
	if (ret != 0){
		dp83td510->leds_cfg_2 = 0x0;
	}
	
	ret = of_property_read_u32(of_node, "ti,io-mux-gpio-ctrl-1",
				&dp83td510->io_mux_gpio_ctrl_1);
	
	/* if not defined in DTS, keep default */
	if (ret != 0){
		dp83td510->io_mux_gpio_ctrl_1 = 0x0;
	}
	
	ret = of_property_read_u32(of_node, "ti,io-mux-gpio-ctrl-2",
			&dp83td510->io_mux_gpio_ctrl_2);
	
	/* if not defined in DTS, keep default */
	if (ret != 0){
		dp83td510->io_mux_gpio_ctrl_2 = 0x0;
	}
	
	dp83td510->rgmii_tx_clk_shift_sel = of_property_read_bool(of_node, 
			"ti,rgmii-tx-clk-shift-sel");
			
	dp83td510->rgmii_rx_clk_shift_sel = of_property_read_bool(of_node, 
			"ti,rgmii-rx-clk-shift-sel");

	return 0;
}

static int dp83td510_probe(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510;

	dp83td510 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83td510),
			       GFP_KERNEL);
	if (!dp83td510)
		return -ENOMEM;

	phydev->priv = dp83td510;

	return dp83td510_of_init(phydev);
}

static struct phy_driver dp83td510_driver[] = {
{
	PHY_ID_MATCH_MODEL(DP83TD510E_PHY_ID),
	.name		= "TI DP83TD510E",
	.probe          = dp83td510_probe,
	.config_init	= dp83td510_config_init,

	.config_aneg	= dp83td510_config_aneg,
	.read_status	= dp83td510_read_status,
	.get_features	= dp83td510_get_features,
	.config_intr	= dp83td510_config_intr,
	.handle_interrupt = dp83td510_handle_interrupt,
	.get_sqi	= dp83td510_get_sqi,
	.get_sqi_max	= dp83td510_get_sqi_max,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
} };
module_phy_driver(dp83td510_driver);

static struct mdio_device_id __maybe_unused dp83td510_tbl[] = {
	{ PHY_ID_MATCH_MODEL(DP83TD510E_PHY_ID) },
	{ }
};
MODULE_DEVICE_TABLE(mdio, dp83td510_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TD510E PHY driver");
MODULE_AUTHOR("Oleksij Rempel <kernel@pengutronix.de>");
MODULE_LICENSE("GPL v2");
