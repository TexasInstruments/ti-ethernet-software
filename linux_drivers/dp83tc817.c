// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DP83TC817 PHY
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/netdevice.h>


#define DP83TC817_CS2_0_PHY_ID	0x2000a2b2

#define MMD1F					0x1f
#define MMD1					0x1

#define DP83TC817_STRAP				0x45d
#define MII_DP83TC817_SGMII_CTRL	0x0608
#define MII_DP83TC817_INT_STAT1		0x12
#define MII_DP83TC817_INT_STAT2		0x13
#define MII_DP83TC817_INT_STAT3		0x18
#define MII_DP83TC817_RESET_CTRL	0x1f

#define DP83TC817_HW_RESET	BIT(15)
#define DP83TC817_SW_RESET	BIT(14)

/* INT_STAT1 bits */
#define DP83TC817_RX_ERR_HF_INT_EN		BIT(0)
#define DP83TC817_MS_TRAINING_INT_EN	BIT(1)
#define DP83TC817_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83TC817_ESD_EVENT_INT_EN		BIT(3)
#define DP83TC817_LINK_STAT_INT_EN		BIT(5)
#define DP83TC817_ENERGY_DET_INT_EN		BIT(6)
#define DP83TC817_LINK_QUAL_INT_EN		BIT(7)

/* INT_STAT2 bits */
#define DP83TC817_JABBER_DET_INT_EN		BIT(0)
#define DP83TC817_POLARITY_INT_EN		BIT(1)
#define DP83TC817_SLEEP_MODE_INT_EN		BIT(2)
#define DP83TC817_OVERTEMP_INT_EN		BIT(3)
#define DP83TC817_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83TC817_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83TC817_LPS_INT_EN		BIT(0)
#define DP83TC817_NO_FRAME_INT_EN	BIT(3)
#define DP83TC817_POR_DONE_INT_EN	BIT(4)

#define MII_DP83TC817_RXSOP1	0x04a5
#define MII_DP83TC817_RXSOP2	0x04a6
#define MII_DP83TC817_RXSOP3	0x04a7

/* SGMII CTRL bits */
#define DP83TC817_SGMII_EN			BIT(9)
#define DP83TC817_SGMII_AUTO_NEG_EN	BIT(0)
#define DP83TC817_SGMII_TX_ERR_DIS	BIT(15)

/* Strap bits */
#define DP83TC817_MASTER_MODE	BIT(9)
#define DP83TC817_RGMII_IS_EN	BIT(7)

/* Master/Slave bits */
#define DP83TC817_MMD1_PMA_CTRL2		0x0834
#define DP83TC817_MMD1_PMA_CTRL2_MASTER	BIT(14)

const int dp83tc817_feature_array[3] = {
	//ETHTOOL_LINK_MODE_100baseT1_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT1_Full_BIT,
	ETHTOOL_LINK_MODE_TP_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
};

enum dp83tc817_chip_type{
	DP83TC817_CS2_0,
};

struct dp83tc817_init_reg {
	int mmd;
	int reg;
	int val;
};

/*
Master Open Alliance Configuration Script
Populate array with script detailed in SNLA293
Array format: {MMD, Register, Value}
*/
static const struct dp83tc817_init_reg dp83tc817_cs2_0_master_init[] = {
};

/*
Slave Open Alliance Configuration Script
Populate array with script detailed in SNLA293
Array format: {MMD, Register, Value}
*/
static const struct dp83tc817_init_reg dp83tc817_cs2_0_slave_init[] = {
};

struct dp83tc817_private{
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
};

static int dp83tc817_read_straps(struct phy_device *phydev)
{
	struct dp83tc817_private *DP83TC817 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, MMD1F, DP83TC817_STRAP);
	if (strap < 0)
		return strap;

	if (strap & DP83TC817_MASTER_MODE)
		DP83TC817->is_master = true;

	if (strap & DP83TC817_RGMII_IS_EN)
		DP83TC817->is_rgmii = true;
	return 0;
};

static int dp83tc817_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC817_RESET_CTRL,
				DP83TC817_HW_RESET);
	else
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC817_RESET_CTRL,
				DP83TC817_SW_RESET);

	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int dp83tc817_phy_reset(struct phy_device *phydev)
{
	int err;
	int ret;

	err = phy_write_mmd(phydev, MMD1F, MII_DP83TC817_RESET_CTRL, DP83TC817_HW_RESET);
	if (err < 0)
		return err;

	ret = dp83tc817_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int dp83tc817_write_sequence(struct phy_device *phydev,
			       const struct dp83tc817_init_reg *init_data,
			       int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
		ret = phy_write_mmd(phydev, init_data[i].mmd, init_data[i].reg,
				    init_data[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int dp83tc817_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, MII_DP83TC817_INT_STAT1);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83TC817_INT_STAT2);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83TC817_INT_STAT3);
	if (err < 0)
		return err;

	return 0;
}

static int dp83tc817_config_intr(struct phy_device *phydev)
{
	int misr_status, err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = dp83tc817_ack_interrupt(phydev);
		if (err)
			return err;

		misr_status = phy_read(phydev, MII_DP83TC817_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC817_RX_ERR_HF_INT_EN |
				DP83TC817_MS_TRAINING_INT_EN |
				DP83TC817_ANEG_COMPLETE_INT_EN |
				DP83TC817_ESD_EVENT_INT_EN |
				DP83TC817_LINK_STAT_INT_EN |
				DP83TC817_ENERGY_DET_INT_EN |
				DP83TC817_LINK_QUAL_INT_EN);

		err = phy_write(phydev, MII_DP83TC817_INT_STAT1, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83TC817_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC817_JABBER_DET_INT_EN |
				DP83TC817_POLARITY_INT_EN |
				DP83TC817_SLEEP_MODE_INT_EN |
				DP83TC817_OVERTEMP_INT_EN |
				DP83TC817_OVERVOLTAGE_INT_EN |
				DP83TC817_UNDERVOLTAGE_INT_EN);

		err = phy_write(phydev, MII_DP83TC817_INT_STAT2, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83TC817_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC817_LPS_INT_EN |
				DP83TC817_NO_FRAME_INT_EN |
				DP83TC817_POR_DONE_INT_EN);

		err = phy_write(phydev, MII_DP83TC817_INT_STAT3, misr_status);

	} else {
		err = phy_write(phydev, MII_DP83TC817_INT_STAT1, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83TC817_INT_STAT2, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83TC817_INT_STAT3, 0);
		if (err < 0)
			return err;

		err = dp83tc817_ack_interrupt(phydev);
	}

	return err;
}

static irqreturn_t dp83tc817_handle_interrupt(struct phy_device *phydev)
{
	bool trigger_machine = false;
	int irq_status;

	/* The INT_STAT registers 1, 2 and 3 are holding the interrupt status
	 * in the upper half (15:8), while the lower half (7:0) is used for
	 * controlling the interrupt enable state of those individual interrupt
	 * sources. To determine the possible interrupt sources, just read the
	 * INT_STAT* register and use it directly to know which interrupts have
	 * been enabled previously or not.
	 */
	irq_status = phy_read(phydev, MII_DP83TC817_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83TC817_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83TC817_INT_STAT3);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	if (!trigger_machine)
		return IRQ_NONE;

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int dp83tc81x_setup_sgmii(struct phy_device *phydev)
{
	int value, err;

	value = phy_read(phydev, MII_DP83TC817_SGMII_CTRL);
	if (phydev->autoneg == AUTONEG_ENABLE) {
		err = phy_write(phydev, MII_DP83TC817_SGMII_CTRL,
				(DP83TC817_SGMII_AUTO_NEG_EN | value));
		if (err < 0)
			return err;
	} else {
		err = phy_write(phydev, MII_DP83TC817_SGMII_CTRL,
				(~DP83TC817_SGMII_AUTO_NEG_EN & value));
		if (err < 0)
			return err;
	}

	return err;
}

static int dp83tc817_setup_master_slave(struct phy_device *phydev)
{
	switch(phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_FORCE:
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
		return phy_write_mmd(phydev, MDIO_MMD_PMAPMD,
					DP83TC817_MMD1_PMA_CTRL2,
					DP83TC817_MMD1_PMA_CTRL2_MASTER);
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
		return phy_write_mmd(phydev, MDIO_MMD_PMAPMD,
					  DP83TC817_MMD1_PMA_CTRL2,
					  0);
	case MASTER_SLAVE_CFG_UNKNOWN:
	case MASTER_SLAVE_CFG_UNSUPPORTED:
	default:
		return 0;
	}

	return 0;
}

static int dp83tc817_read_master_slave(struct phy_device *phydev)
{
	int ctrl2 = phy_read_mmd(phydev, MDIO_MMD_PMAPMD,
				 DP83TC817_MMD1_PMA_CTRL2);

	if (ctrl2 < 0)
		return ctrl2;

	if (ctrl2 & DP83TC817_MMD1_PMA_CTRL2_MASTER) {
		phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_FORCE;
		phydev->master_slave_state = MASTER_SLAVE_STATE_MASTER;
		return 1;
	} else {
		phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_FORCE;
		phydev->master_slave_state = MASTER_SLAVE_STATE_SLAVE;
		return 0;
	}

	return 0;
}

static int dp83tc817_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	int err;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		err = dp83tc81x_setup_sgmii(phydev);
		if (err)
			return err;
	}

	err = dp83tc817_setup_master_slave(phydev);
	if (err < 0)
		return err;
	else if (err)
		changed = true;

	if (AUTONEG_ENABLE != phydev->autoneg)
		return genphy_setup_forced(phydev);

	return genphy_check_and_restart_aneg(phydev, changed);
}

static int dp83tc817_chip_init(struct phy_device *phydev)
{
	struct dp83tc817_private *DP83TC817 = phydev->priv;
	int ret;

	ret = dp83tc817_reset(phydev, true);
	if (ret)
		return ret;
	
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		 phydev->supported);
	
	if (DP83TC817->is_master)
		ret = phy_write_mmd(phydev, MMD1, 0x0834, 0xc001);
	else
		ret = phy_write_mmd(phydev, MMD1, 0x0834, 0x8001);

	switch (DP83TC817->chip){
		case DP83TC817_CS2_0:
		if (DP83TC817->is_master)
			{
				ret = dp83tc817_write_sequence(phydev, dp83tc817_cs2_0_master_init, ARRAY_SIZE(dp83tc817_cs2_0_master_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6));
			}
			else
			{
				ret = dp83tc817_write_sequence(phydev, dp83tc817_cs2_0_slave_init, ARRAY_SIZE(dp83tc817_cs2_0_slave_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6));
			}
			break;
		default:
			return -EINVAL;
	};

	if (ret)
		return ret;
	
	mdelay(10);

	return dp83tc817_reset(phydev, false);
}

static int dp83tc817_config_init(struct phy_device *phydev)
{
	int value, err;

	err = dp83tc817_chip_init(phydev);
	if (err)
		return err;

	value = phy_read(phydev, MII_DP83TC817_SGMII_CTRL);
	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		err = phy_write(phydev, MII_DP83TC817_SGMII_CTRL,
					(DP83TC817_SGMII_EN | value));
	} else {
		err = phy_write(phydev, MII_DP83TC817_SGMII_CTRL,
				(~DP83TC817_SGMII_EN & value));
	}

	if (err < 0)
		return err;
	
	return 0;
}

static int dp83tc817_probe(struct phy_device *phydev)
{
	struct dp83tc817_private *DP83TC817;
	int ret;

	DP83TC817 = devm_kzalloc(&phydev->mdio.dev, sizeof(*DP83TC817), GFP_KERNEL);
	if(!DP83TC817)
		return -ENOMEM;
	
	phydev->priv = DP83TC817;
	ret = dp83tc817_read_straps(phydev);
	if (ret)
		return ret;
	
	switch (phydev->phy_id)
	{
		case DP83TC817_CS2_0_PHY_ID:
			DP83TC817->chip = DP83TC817_CS2_0;
		break;
		default:
			return -EINVAL;
	};

	return dp83tc817_config_init(phydev);
}

static int dp83tc817_get_features(struct phy_device *phydev)
{
	genphy_read_abilities(phydev);

	linkmode_set_bit_array(dp83tc817_feature_array,
			       ARRAY_SIZE(dp83tc817_feature_array),
			       phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT1_Full_BIT, phydev->advertising);

	/* Only allow advertising what this PHY supports */
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	return 0;
}

static int dp83tc817_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_status(phydev);
	if (ret)
		return ret;

	ret = dp83tc817_read_master_slave(phydev);

	return 0;
}

#define DP83TC817_PHY_DRIVER(_id, _name)			\
	{							\
		PHY_ID_MATCH_EXACT(_id),			\
		.name		= (_name),			\
		.probe		= dp83tc817_probe,		\
		.config_init = dp83tc817_config_init,		\
		.config_aneg = dp83tc817_config_aneg,		\
		.soft_reset = dp83tc817_phy_reset,		\
		.handle_interrupt = dp83tc817_handle_interrupt,	\
		.config_intr = dp83tc817_config_intr,		\
		.suspend = genphy_suspend,			\
		.resume = genphy_resume,			\
		.get_features	= dp83tc817_get_features,	\
		.read_status	= dp83tc817_read_status,		\
	 }

static struct phy_driver dp83tc817_driver[] = {
	DP83TC817_PHY_DRIVER(DP83TC817_CS2_0_PHY_ID, "TI DP83TC817CS2.0"),
};
module_phy_driver(dp83tc817_driver);

static struct mdio_device_id __maybe_unused dp83tc817_tbl[] = {
	{ PHY_ID_MATCH_EXACT(DP83TC817_CS2_0_PHY_ID) },
	{ },
};
MODULE_DEVICE_TABLE(mdio, dp83tc817_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TC817 PHY driver");
MODULE_AUTHOR("Alvaro Reyes <a-reyes1@ti.com>");
MODULE_AUTHOR("Drew Miller <drew-miller@ti.com>");
MODULE_LICENSE("GPL");