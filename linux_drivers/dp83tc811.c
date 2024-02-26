// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DP83TC811 PHY
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Changelog
 *
 * 04/13/21 Yannik Muendler (y-muendler@ti.com)	
 *
 * Add 811 script / fix PHY identification / 
 */

#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define DP83TC811_PHY_ID	0x2000a253
#define DP83811_DEVADDR		0x1f
#define DP83811_DEVADDR_MMD1	0x1

#define MII_DP83811_SGMII_CTRL	0x09
#define MII_DP83811_INT_STAT1	0x12
#define MII_DP83811_INT_STAT2	0x13
#define MII_DP83811_INT_STAT3	0x18
#define MII_DP83811_RESET_CTRL	0x1f
#define DP83811_STRAP			0x467

#define DP83811_HW_RESET	BIT(15)
#define DP83811_SW_RESET	BIT(14)

/* Strap bits */
#define DP83811_MASTER_MODE     BIT(9)
  
/* INT_STAT1 bits */
#define DP83811_RX_ERR_HF_INT_EN	BIT(0)
#define DP83811_MS_TRAINING_INT_EN	BIT(1)
#define DP83811_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83811_ESD_EVENT_INT_EN	BIT(3)
#define DP83811_WOL_INT_EN		BIT(4)
#define DP83811_LINK_STAT_INT_EN	BIT(5)
#define DP83811_ENERGY_DET_INT_EN	BIT(6)
#define DP83811_LINK_QUAL_INT_EN	BIT(7)

/* INT_STAT2 bits */
#define DP83811_JABBER_DET_INT_EN	BIT(0)
#define DP83811_POLARITY_INT_EN		BIT(1)
#define DP83811_SLEEP_MODE_INT_EN	BIT(2)
#define DP83811_OVERTEMP_INT_EN		BIT(3)
#define DP83811_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83811_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83811_LPS_INT_EN	BIT(0)
#define DP83811_NO_FRAME_INT_EN	BIT(3)
#define DP83811_POR_DONE_INT_EN	BIT(4)

#define MII_DP83811_RXSOP1	0x04a5
#define MII_DP83811_RXSOP2	0x04a6
#define MII_DP83811_RXSOP3	0x04a7

/* WoL Registers */
#define MII_DP83811_WOL_CFG	0x04a0
#define MII_DP83811_WOL_STAT	0x04a1
#define MII_DP83811_WOL_DA1	0x04a2
#define MII_DP83811_WOL_DA2	0x04a3
#define MII_DP83811_WOL_DA3	0x04a4

/* WoL bits */
#define DP83811_WOL_MAGIC_EN	BIT(0)
#define DP83811_WOL_SECURE_ON	BIT(5)
#define DP83811_WOL_EN		BIT(7)
#define DP83811_WOL_INDICATION_SEL BIT(8)
#define DP83811_WOL_CLR_INDICATION BIT(11)

/* SGMII CTRL bits */
#define DP83811_TDR_AUTO		BIT(8)
#define DP83811_SGMII_EN		BIT(12)
#define DP83811_SGMII_AUTO_NEG_EN	BIT(13)
#define DP83811_SGMII_TX_ERR_DIS	BIT(14)
#define DP83811_SGMII_SOFT_RESET	BIT(15)

enum dp83811_chip_type {
	DP83TC811,
};

struct dp83811_init_reg {
	int reg;
	int val;
};

static const struct dp83811_init_reg dp83811_master_init[] = {
	{0x0475,0x0008},
	{0x0485,0x11FF},
	{0x0462,0x0000},
	{0x0122,0x1450},
	{0x0400,0x1000},
	{0x010F,0x0100},
	{0x0410,0x6000},
	{0x0479,0x0003},
	{0x0466,0x0000},
	{0x0107,0x2605},
	{0x0106,0xB8BB},
	{0x0116,0x03CA},
	{0x0114,0xC00A},
	{0x04D5,0xFEA4},
	{0x04D6,0x0EA4},
	{0x010B,0x0700},
	{0x0132,0x01EE},
	{0x0120,0x0067},
	{0x0125,0x7A56},
	{0x04DE,0x03F0},
	{0x003E,0x000D},
	{0x0461,0x0408},
	{0x0403,0x0030},
	{0x0404,0x0018},
	{0x048A,0x0D02},
	{0x048B,0x350F},
	{0x048D,0x010D},
	{0x0400,0x1300},
	{0x040C,0x0507},
	{0x0121,0x1B00},
	{0x04D4,0x7522},
	{0x0130,0xC720},
	{0x0126,0x0515},
	{0x0119,0x00A4},
	{0x0109,0x095D},
	{0x010E,0x3219},
	{0x010C,0x1996},
	{0x0463,0x0000},
	{0x0475,0x0000},
};

static const struct dp83811_init_reg dp83811_slave_init[] = {
	{0x0475,0x0008},
	{0x0485,0x11FF},
	{0x0462,0x0000},
	{0x0122,0x1450},
	{0x0400,0x1000},
	{0x010F,0x0100},
	{0x0410,0x6000},
	{0x0479,0x0003},
	{0x0466,0x0000},
	{0x0107,0x2605},
	{0x0106,0xB8BB},
	{0x0116,0x03CA},
	{0x0114,0xC00A},
	{0x04D5,0xFEA4},
	{0x04D6,0x0EA4},
	{0x010B,0x0700},
	{0x0132,0x01EE},
	{0x0120,0x0067},
	{0x0125,0x7A56},
	{0x04DE,0x03F0},
	{0x003E,0x000D},
	{0x0461,0x0408},
	{0x0403,0x0030},
	{0x0404,0x0018},
	{0x048A,0x0D02},
	{0x048B,0x350F},
	{0x048D,0x010D},
	{0x0400,0x1300},
	{0x040C,0x0507},
	{0x0121,0x1500},
	{0x04D4,0x7322},
	{0x0130,0xC780},
	{0x0126,0x0493},
	{0x0115,0x8AC8},
	{0x0109,0x095D},
	{0x010E,0xFAFB},
	{0x010C,0x19FA},
	{0x0463,0x0000},
	{0x0475,0x0000},
};

struct dp83811_private {
        int chip;
        bool is_master;
};
 
 
static int dp83811_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, MII_DP83811_INT_STAT1);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83811_INT_STAT2);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83811_INT_STAT3);
	if (err < 0)
		return err;

	return 0;
}

static int dp83811_set_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	u16 value;

	if (wol->wolopts & (WAKE_MAGIC | WAKE_MAGICSECURE)) {
		mac = (const u8 *)ndev->dev_addr;

		if (!is_valid_ether_addr(mac))
			return -EINVAL;

		/* MAC addresses start with byte 5, but stored in mac[0].
		 * 811 PHYs store bytes 4|5, 2|3, 0|1
		 */
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA1,
			      (mac[1] << 8) | mac[0]);
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA2,
			      (mac[3] << 8) | mac[2]);
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA3,
			      (mac[5] << 8) | mac[4]);

		value = phy_read_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG);
		if (wol->wolopts & WAKE_MAGIC)
			value |= DP83811_WOL_MAGIC_EN;
		else
			value &= ~DP83811_WOL_MAGIC_EN;

		if (wol->wolopts & WAKE_MAGICSECURE) {
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP1,
				      (wol->sopass[1] << 8) | wol->sopass[0]);
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP2,
				      (wol->sopass[3] << 8) | wol->sopass[2]);
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP3,
				      (wol->sopass[5] << 8) | wol->sopass[4]);
			value |= DP83811_WOL_SECURE_ON;
		} else {
			value &= ~DP83811_WOL_SECURE_ON;
		}

		/* Clear any pending WoL interrupt */
		phy_read(phydev, MII_DP83811_INT_STAT1);

		value |= DP83811_WOL_EN | DP83811_WOL_INDICATION_SEL |
			 DP83811_WOL_CLR_INDICATION;

		return phy_write_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG, value);
	} else {
		value = phy_read_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG);
		if (value < 0)
			return value;

		value &= ~DP83811_WOL_EN;

		return phy_write_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG, value);
	}

}

static void dp83811_get_wol(struct phy_device *phydev,
			    struct ethtool_wolinfo *wol)
{
	u16 sopass_val;
	int value;

	wol->supported = (WAKE_MAGIC | WAKE_MAGICSECURE);
	wol->wolopts = 0;

	value = phy_read_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG);

	if (value & DP83811_WOL_MAGIC_EN)
		wol->wolopts |= WAKE_MAGIC;

	if (value & DP83811_WOL_SECURE_ON) {
		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP1);
		wol->sopass[0] = (sopass_val & 0xff);
		wol->sopass[1] = (sopass_val >> 8);

		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP2);
		wol->sopass[2] = (sopass_val & 0xff);
		wol->sopass[3] = (sopass_val >> 8);

		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP3);
		wol->sopass[4] = (sopass_val & 0xff);
		wol->sopass[5] = (sopass_val >> 8);

		wol->wolopts |= WAKE_MAGICSECURE;
	}

	/* WoL is not enabled so set wolopts to 0 */
	if (!(value & DP83811_WOL_EN))
		wol->wolopts = 0;
}

static int dp83811_config_intr(struct phy_device *phydev)
{
	int misr_status, err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		misr_status = phy_read(phydev, MII_DP83811_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_RX_ERR_HF_INT_EN |
				DP83811_MS_TRAINING_INT_EN |
				DP83811_ANEG_COMPLETE_INT_EN |
				DP83811_ESD_EVENT_INT_EN |
				DP83811_WOL_INT_EN |
				DP83811_LINK_STAT_INT_EN |
				DP83811_ENERGY_DET_INT_EN |
				DP83811_LINK_QUAL_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT1, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83811_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_JABBER_DET_INT_EN |
				DP83811_POLARITY_INT_EN |
				DP83811_SLEEP_MODE_INT_EN |
				DP83811_OVERTEMP_INT_EN |
				DP83811_OVERVOLTAGE_INT_EN |
				DP83811_UNDERVOLTAGE_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT2, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83811_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_LPS_INT_EN |
				DP83811_NO_FRAME_INT_EN |
				DP83811_POR_DONE_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT3, misr_status);

	} else {
		err = phy_write(phydev, MII_DP83811_INT_STAT1, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83811_INT_STAT2, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83811_INT_STAT3, 0);
	}

	return err;
}

static int dp83811_config_aneg(struct phy_device *phydev)
{
	struct dp83811_private *dp83811 = phydev->priv;
	int value, err;
	
	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		switch (dp83811->chip) {
		case DP83TC811:
			value = phy_read(phydev, MII_DP83811_SGMII_CTRL);
			if (phydev->autoneg == AUTONEG_ENABLE) {
				err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
						(DP83811_SGMII_AUTO_NEG_EN | value));
				if (err < 0)
					return err;
			} else {
				err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
						(~DP83811_SGMII_AUTO_NEG_EN & value));
				if (err < 0)
					return err;
			}
			break;     
		}
	}
	
	return genphy_config_aneg(phydev);
}

static int dp83811_read_straps(struct phy_device *phydev)
{
        struct dp83811_private *dp83811 = phydev->priv;
        int strap;

		switch (dp83811->chip) {
		case DP83TC811:
			strap = phy_read_mmd(phydev, DP83811_DEVADDR, DP83811_STRAP);
			if (strap < 0)
					return strap;

			printk("%s: Strap is 0x%X\n", __func__, strap);
			if (strap & DP83811_MASTER_MODE)
					dp83811->is_master = true;
			break;    
		}
        return 0;
};

static int dp83811_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write(phydev, MII_DP83811_RESET_CTRL,
				DP83811_HW_RESET);
	else
		ret = phy_write(phydev, MII_DP83811_RESET_CTRL,
				DP83811_SW_RESET);

	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int dp83811_phy_reset(struct phy_device *phydev)
{
	int ret;

	ret = dp83811_reset(phydev, false);
	if (ret)
		return ret;

	ret = dp83811_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int dp83811_write_seq(struct phy_device *phydev,
			     const struct dp83811_init_reg *init_data, int size)
{
	int ret;
	int i;

	printk("%s: Size %d\n", __func__, size);
	for (i = 0; i < size; i++) {
	        ret = phy_write_mmd(phydev,DP83811_DEVADDR, init_data[i].reg,
				init_data[i].val);
	        if (ret)
	                return ret;
	}

	return 0;
}

static int dp83811_chip_init(struct phy_device *phydev)
{
	struct dp83811_private *dp83811 = phydev->priv;
	int ret;
	
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		 phydev->supported);

	ret = dp83811_reset(phydev, true);
	if (ret)
		return ret;

	switch (dp83811->chip) {
	case DP83TC811:
		if (dp83811->is_master){
			ret = dp83811_write_seq(phydev, dp83811_master_init,
						ARRAY_SIZE(dp83811_master_init));
			ret = dp83811_reset(phydev, false);
			if (ret)
				return ret;
		}
		else{
			ret = dp83811_write_seq(phydev, dp83811_slave_init,
						ARRAY_SIZE(dp83811_slave_init));
			ret = dp83811_reset(phydev, false);
			if (ret)
				return ret;
		}
		break;
	default:
		return -EINVAL;
	};
	
	if (ret)
		return ret;

	mdelay(10);

	/* Do a software reset to restart the PHY with the updated values */
	return dp83811_reset(phydev, false);
}

static int dp83811_config_init(struct phy_device *phydev)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported) = { 0, };
	struct dp83811_private *dp83811 = phydev->priv;
	int value, err, ret;

	__set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, supported);
	__set_bit(ETHTOOL_LINK_MODE_TP_BIT, supported);
	__set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, supported);

	ret = dp83811_chip_init(phydev);
	if (ret)
		return ret;

	switch (dp83811->chip) {
	case DP83TC811:
		value = phy_read(phydev, MII_DP83811_SGMII_CTRL);
		if (value < 0)
			return value;

		if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
			err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
						(DP83811_SGMII_EN | value));
		} else {
			err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
					(~DP83811_SGMII_EN & value));
		}
		break;     
	}

	if (err < 0)

		return err;

	value = DP83811_WOL_MAGIC_EN | DP83811_WOL_SECURE_ON | DP83811_WOL_EN;

	return phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG,
				  value);
}

static int dp83811_suspend(struct phy_device *phydev)
{
	int value;

	value = phy_read_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG);

	if (!(value & DP83811_WOL_EN))
		genphy_suspend(phydev);

	return 0;
}

static int dp83811_resume(struct phy_device *phydev)
{
	int value;

	genphy_resume(phydev);

	value = phy_read_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG);
	if (value < 0)
		return value;

	value |= DP83811_WOL_CLR_INDICATION;

	return phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG,
			     value);
}

static int dp83811_probe(struct phy_device *phydev)
{
        struct dp83811_private *dp83811;
        int ret;

        dp83811 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83811),
                               GFP_KERNEL);
        if (!dp83811)
                return -ENOMEM;

        phydev->priv = dp83811;

        if (phydev->phy_id == DP83TC811_PHY_ID)
			dp83811->chip = DP83TC811;
        else
			return -EINVAL;
		
		ret = dp83811_read_straps(phydev);
        if (ret)
            return ret;
 
		printk("%s: Chip is %d\n", __func__, dp83811->chip);
			
		return dp83811_config_init(phydev);
}

#define DP83TC81X_PHY_DRIVER(_id, _name)			\
	{							\
		.phy_id		= (_id),				\
		.phy_id_mask	= 0xffffffff,			\
		.name		= (_name),			\
		.config_init = dp83811_config_init,		\
		.probe = dp83811_probe,			\
		.config_aneg = dp83811_config_aneg,		\
		.soft_reset = dp83811_phy_reset,		\
		.get_wol = dp83811_get_wol,			\
		.set_wol = dp83811_set_wol,			\
		.ack_interrupt = dp83811_ack_interrupt,		\
		.config_intr = dp83811_config_intr,		\
		.suspend = dp83811_suspend,			\
		.resume = dp83811_resume,			\
	 }

static struct phy_driver dp83tc811_driver[] = {
	DP83TC81X_PHY_DRIVER(DP83TC811_PHY_ID, "TI DP83TC811"),
};
module_phy_driver(dp83tc811_driver);

static struct mdio_device_id __maybe_unused dp83811_tbl[] = {
	{ DP83TC811_PHY_ID, 0xffffffff },
	{ },
};
MODULE_DEVICE_TABLE(mdio, dp83811_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TC81X PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
