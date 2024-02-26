// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DP83TC812 PHY
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#define DP83TC812_CS1_0_PHY_ID	0x2000a270
#define DP83TC812_CS2_0_PHY_ID	0x2000a271
#define DP83TC813_CS2_0_PHY_ID	0x2000a211
#define DP83TC814_CS2_0_PHY_ID	0x2000a261

#define DP83812_DEVADDR		0x1f
#define DP83812_DEVADDR_MMD1	0x1

#define DP83812_STRAP		0x45d
#define MII_DP83812_SGMII_CTRL	0x608
#define SGMII_CONFIG_VAL	0x027B
#define MII_DP83812_RGMII_CTRL	0x600
#define MII_DP83812_INT_STAT1	0x12
#define MII_DP83812_INT_STAT2	0x13
#define MII_DP83812_INT_STAT3	0x18
#define MII_DP83812_RESET_CTRL	0x1f

#define DP83812_HW_RESET	BIT(15)
#define DP83812_SW_RESET	BIT(14)

/* INT_STAT1 bits */
#define DP83812_RX_ERR_CNT_HALF_FULL_INT_EN	BIT(0)
#define DP83812_TX_ERR_CNT_HALF_FULL_INT_EN	BIT(1)
#define DP83812_MS_TRAIN_DONE_INT_EN		BIT(2)
#define DP83812_ESD_EVENT_INT_EN		BIT(3)
#define DP83812_LINK_STAT_INT_EN		BIT(5)
#define DP83812_ENERGY_DET_INT_EN		BIT(6)
#define DP83812_LINK_QUAL_INT_EN		BIT(7)

/* INT_STAT2 bits */
#define DP83812_JABBER_INT_EN		BIT(0)
#define DP83812_POL_INT_EN		BIT(1)
#define DP83812_SLEEP_MODE_INT_EN	BIT(2)
#define DP83812_OVERTEMP_INT_EN		BIT(3)
#define DP83812_FIFO_INT_EN		BIT(4)
#define DP83812_PAGE_RXD_INT_EN		BIT(5)
#define DP83812_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83812_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83812_LPS_INT_EN		BIT(0)
#define DP83812_WUP_INT_EN		BIT(1)
#define DP83812_WAKE_REQ_INT_EN		BIT(2)
#define DP83811_NO_FRAME_INT_EN		BIT(3)
#define DP83811_POR_DONE_INT_EN		BIT(4)
#define DP83812_SLEEP_FAIL_INT_EN	BIT(5)

/* RGMII_CTRL bits */
#define DP83812_RGMII_EN		BIT(3)

/* SGMII CTRL bits */
#define DP83812_SGMII_AUTO_NEG_EN	BIT(0)
#define DP83812_SGMII_EN		BIT(9)

/* Strap bits */
#define DP83812_MASTER_MODE	BIT(9)
#define DP83812_RGMII_IS_EN	BIT(7)

/* RGMII ID CTRL */
#define DP83812_RGMII_ID_CTRL	0x602
#define DP83812_RX_CLK_SHIFT	BIT(1)
#define DP83812_TX_CLK_SHIFT	BIT(0)

enum dp83812_chip_type {
	DP83812_CS1 = 0,
	DP83812_CS2,
	DP83813_CS2,
	DP83814_CS2,
};

struct dp83812_init_reg {
	int	reg;
	int	val;
};

static const struct dp83812_init_reg dp83812_master_cs1_0_init[] = {
	{0x523, 0x0001},
	{0x800, 0xf864},
	{0x803, 0x1552},
	{0x804, 0x1a66},
	{0x805, 0x1f7b},
	{0x81f, 0x2a88},
	{0x825, 0x40e5},
	{0x82b, 0x7f3f},
	{0x830, 0x0543},
	{0x836, 0x5008},
	{0x83a, 0x08e0},
	{0x83b, 0x0845},
	{0x83e, 0x0445},
	{0x855, 0x9b9a},
	{0x85f, 0x2010},
	{0x860, 0x6040},
	{0x86c, 0x1333},
	{0x86b, 0x3e10},
	{0x872, 0x88c0},
	{0x873, 0x0003},
	{0x879, 0x000f},
	{0x87b, 0x0070},
	{0x87c, 0x003f},
	{0x89e, 0x00aa},
	{0x523, 0x0000},
};

static const struct dp83812_init_reg dp83812_master_cs2_0_init[] = {
	{0x523, 0x0001},
	{0x81C, 0x0fe2},
	{0x872, 0x0300},
	{0x879, 0x0f00},
	{0x806, 0x2952},
	{0x807, 0x3361},
	{0x808, 0x3D7B},
	{0x83E, 0x045F},
	{0x834, 0x8000},
	{0x862, 0x00E8},
	{0x896, 0x32CB},
	{0x03E, 0x0009},
	{0x01f, 0x4000},
	{0x523, 0x0000},
};

static const struct dp83812_init_reg dp83812_slave_cs1_0_init[] = {
	{0x523, 0x0001},
	{0x803, 0x1b52},
	{0x804, 0x216c},
	{0x805, 0x277b},
	{0x827, 0x3000},
	{0x830, 0x0543},
	{0x83a, 0x0020},
	{0x83c, 0x0001},
	{0x855, 0x9b9a},
	{0x85f, 0x2010},
	{0x860, 0x6040},
	{0x86c, 0x0333},
	{0x872, 0x88c0},
	{0x873, 0x0021},
	{0x879, 0x000f},
	{0x87b, 0x0070},
	{0x87c, 0x0002},
	{0x897, 0x003f},
	{0x89e, 0x00a2},
	{0x510, 0x000f},
	{0x523, 0x0000},
};

static const struct dp83812_init_reg dp83812_slave_cs2_0_init[] = {
	{0x523, 0x0001},
	{0x873, 0x0821},
	{0x896, 0x22ff},
	{0x89E, 0x0000},
	{0x01f, 0x4000},
	{0x523, 0x0000},
};

struct dp83812_private {
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
};

static int dp83812_read_straps(struct phy_device *phydev)
{
	struct dp83812_private *dp83812 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, DP83812_DEVADDR, DP83812_STRAP);
	if (strap < 0)
		return strap;

	printk("%s: Strap is 0x%X\n", __func__, strap);
	if (strap & DP83812_MASTER_MODE)
		dp83812->is_master = true;

	if (strap & DP83812_RGMII_IS_EN)
		dp83812->is_rgmii = true;
	return 0;
};

static int dp83812_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, DP83812_DEVADDR, MII_DP83812_RESET_CTRL,
				DP83812_HW_RESET);
	else
		ret = phy_write_mmd(phydev, DP83812_DEVADDR, MII_DP83812_RESET_CTRL,
				DP83812_SW_RESET);

	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int dp83812_phy_reset(struct phy_device *phydev)
{
	int err;
	int ret;

	err = phy_write_mmd(phydev, DP83812_DEVADDR, MII_DP83812_RESET_CTRL, DP83812_HW_RESET);
	if (err < 0)
		return err;

	ret = dp83812_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int dp83812_write_seq(struct phy_device *phydev, const struct
				dp83812_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
		ret = phy_write_mmd(phydev, DP83812_DEVADDR, init_data[i].reg,
					init_data[i].val);
	if (ret)
		return ret;
	}
	return 0;
}

static int dp83812_chip_init(struct phy_device *phydev)
{
	struct dp83812_private *dp83812 = phydev->priv;
	int ret;

	ret = dp83812_reset(phydev, true);
	if (ret)
		return ret;

	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		 phydev->supported);

	if (dp83812->is_master)
		ret = phy_write_mmd(phydev, DP83812_DEVADDR_MMD1, 0x0834, 0xc001);
		// ret = phy_write_mmd(phydev, DP83812_DEVADDR, 0x0834, 0xc001);
	else
		ret = phy_write_mmd(phydev, DP83812_DEVADDR_MMD1, 0x0834, 0x8001);
		// ret = phy_write_mmd(phydev, DP83812_DEVADDR, 0x0834, 0x8001);

	switch (dp83812->chip) {
	case DP83812_CS1:
		if (dp83812->is_master)
			ret = dp83812_write_seq(phydev,
						dp83812_master_cs1_0_init,
						ARRAY_SIZE(dp83812_master_cs1_0_init));
		else
			ret = dp83812_write_seq(phydev,
						dp83812_slave_cs1_0_init,
						ARRAY_SIZE(dp83812_slave_cs1_0_init));
	break;
	case DP83812_CS2:
		if (dp83812->is_master)
			ret = dp83812_write_seq(phydev,
						dp83812_master_cs2_0_init,
						ARRAY_SIZE(dp83812_master_cs2_0_init));
		else
			ret = dp83812_write_seq(phydev,
						dp83812_slave_cs2_0_init,
						ARRAY_SIZE(dp83812_slave_cs2_0_init));
	break;
	case DP83813_CS2:
		if (dp83812->is_master)
			ret = dp83812_write_seq(phydev,
						dp83812_master_cs2_0_init,
						ARRAY_SIZE(dp83812_master_cs2_0_init));
		else
			ret = dp83812_write_seq(phydev,
						dp83812_slave_cs2_0_init,
						ARRAY_SIZE(dp83812_slave_cs2_0_init));
	break;
	case DP83814_CS2:
		if (dp83812->is_master)
			ret = dp83812_write_seq(phydev,
						dp83812_master_cs2_0_init,
						ARRAY_SIZE(dp83812_master_cs2_0_init));
		else
			ret = dp83812_write_seq(phydev,
						dp83812_slave_cs2_0_init,
						ARRAY_SIZE(dp83812_slave_cs2_0_init));
	break;
	default:
		return -EINVAL;
	};

	if (ret)
		return ret;

	mdelay(10);
	// phy_write_mmd(phydev, DP83812_DEVADDR, 0x523, 0x00);
	/* Do a soft reset to restart the PHY with updated values */
	return dp83812_reset(phydev, false);
}

static int dp83812_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	s32 rx_int_delay;
	s32 tx_int_delay;
	int rgmii_delay;
	int value, ret;

	ret = dp83812_chip_init(phydev);
	if (ret)
		return ret;

	if (phy_interface_is_rgmii(phydev)) {
		rx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      true);

		if (rx_int_delay <= 0)
			rgmii_delay = 0;
		else
			rgmii_delay = DP83812_RX_CLK_SHIFT;

		tx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      false);
		if (tx_int_delay <= 0)
			rgmii_delay &= ~DP83812_TX_CLK_SHIFT;
		else
			rgmii_delay |= DP83812_TX_CLK_SHIFT;

		if (rgmii_delay) {
			ret = phy_set_bits_mmd(phydev, DP83812_DEVADDR_MMD1,
					       DP83812_RGMII_ID_CTRL,
					       rgmii_delay);
			if (ret)
				return ret;
		}
	}

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, MII_DP83812_SGMII_CTRL);
		ret = phy_write_mmd(phydev, DP83812_DEVADDR, MII_DP83812_SGMII_CTRL,
				SGMII_CONFIG_VAL);
	if (ret < 0)
		return ret;
	}

	return 0;
}

static int dp83812_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, MII_DP83812_INT_STAT1);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83812_INT_STAT2);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83812_INT_STAT3);
	if (err < 0)
		return err;

	return 0;
}

static int dp83812_config_intr(struct phy_device *phydev)
{
	int misr_status, err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		misr_status = phy_read(phydev, MII_DP83812_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83812_ESD_EVENT_INT_EN |
				DP83812_LINK_STAT_INT_EN |
				DP83812_ENERGY_DET_INT_EN |
				DP83812_LINK_QUAL_INT_EN);

		err = phy_write(phydev, MII_DP83812_INT_STAT1, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83812_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83812_SLEEP_MODE_INT_EN |
				DP83812_OVERTEMP_INT_EN |
				DP83812_OVERVOLTAGE_INT_EN |
				DP83812_UNDERVOLTAGE_INT_EN);

		err = phy_write(phydev, MII_DP83812_INT_STAT2, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83812_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83812_LPS_INT_EN |
				DP83812_WAKE_REQ_INT_EN |
				DP83811_NO_FRAME_INT_EN |
				DP83811_POR_DONE_INT_EN);

		err = phy_write(phydev, MII_DP83812_INT_STAT3, misr_status);

	} else {
		err = phy_write(phydev, MII_DP83812_INT_STAT1, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83812_INT_STAT2, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83812_INT_STAT3, 0);
	}

	return err;
}

#if 0
static irqreturn_t dp83812_handle_interrupt(struct phy_device *phydev)
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
	irq_status = phy_read(phydev, MII_DP83812_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83812_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83812_INT_STAT3);
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
#endif

static int dp83812_config_aneg(struct phy_device *phydev)
{
	int value, ret;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, MII_DP83812_SGMII_CTRL);
		ret = phy_write_mmd(phydev, DP83812_DEVADDR, MII_DP83812_SGMII_CTRL,
				SGMII_CONFIG_VAL);
		if (ret < 0)
			return ret;
	}
	
	return genphy_config_aneg(phydev);
}



static int dp83812_probe(struct phy_device *phydev)
{
	struct dp83812_private *dp83812;
	int ret;

	dp83812 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83812), GFP_KERNEL);
	if (!dp83812)
		return -ENOMEM;

	phydev->priv = dp83812;					
	ret = dp83812_read_straps(phydev);
	if (ret)
		return ret;

	switch (phydev->phy_id) {
	case DP83TC812_CS1_0_PHY_ID:
		dp83812->chip = DP83812_CS1;
	break;
	case DP83TC812_CS2_0_PHY_ID:
		dp83812->chip = DP83812_CS2;
	break;
	case DP83TC813_CS2_0_PHY_ID:
		dp83812->chip = DP83813_CS2;
		break;
	case DP83TC814_CS2_0_PHY_ID:
		dp83812->chip = DP83814_CS2;
	break;
	default:
	return -EINVAL;
	};
/* vikram : above code added to switch between different phy ids */

	return dp83812_config_init(phydev);
}

#define DP83812_PHY_DRIVER(_id, _name)				\
	{							\
		PHY_ID_MATCH_EXACT(_id),			\
		.name           = (_name),			\
		.probe          = dp83812_probe,		\
		/* PHY_BASIC_FEATURES */			\
		.soft_reset     = dp83812_phy_reset,		\
		.config_init    = dp83812_config_init,		\
		.config_aneg = dp83812_config_aneg,		\
		.ack_interrupt = dp83812_ack_interrupt,		\
/*if 0								\
		.handle_interrupt = dp83812_handle_interrupt,	\
#endif	*/							\
		.config_intr = dp83812_config_intr,		\
		.suspend = genphy_suspend,			\
		.resume = genphy_resume,			\
	}

static struct phy_driver dp83812_driver[] = {
	DP83812_PHY_DRIVER(DP83TC812_CS1_0_PHY_ID, "TI DP83TC812CS1.0"),
	DP83812_PHY_DRIVER(DP83TC812_CS2_0_PHY_ID, "TI DP83TC812CS2.0"),
	DP83812_PHY_DRIVER(DP83TC813_CS2_0_PHY_ID, "TI DP83TC813CS2.0"),
	DP83812_PHY_DRIVER(DP83TC814_CS2_0_PHY_ID, "TI DP83TC814CS2.0"),
	};

module_phy_driver(dp83812_driver);

static struct mdio_device_id __maybe_unused dp83812_tbl[] = {
	{ PHY_ID_MATCH_EXACT(DP83TC812_CS1_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC812_CS2_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC813_CS2_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC814_CS2_0_PHY_ID) },
	{ },
};
MODULE_DEVICE_TABLE(mdio, dp83812_tbl);									   

MODULE_DESCRIPTION("Texas Instruments DP83TC812 PHY driver");
MODULE_AUTHOR("Hari Nagalla <hnagalla@ti.com");
MODULE_LICENSE("GPL");
