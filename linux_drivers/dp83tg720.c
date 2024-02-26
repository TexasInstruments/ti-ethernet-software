// SPDX-License-Identifier: GPL-2.0-only
/* Driver for the Texas Instruments DP83TG720 PHY
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define DP83TG720ES1_PHY_ID	0x2000a280
#define DP83TG720ES2_PHY_ID	0x2000a281
#define DP83TG720CS_1_0_PHY_ID	0x2000a283
#define DP83TG720CS_1_1_PHY_ID	0x2000a284
#define DP83TG721CS_1_0_PHY_ID	0x2000a290
#define DP83720_DEVADDR		0x1f
#define DP83720_DEVADDR_MMD1	0x1

#define MII_DP83720_INT_STAT1	0x12
#define MII_DP83720_INT_STAT2	0x13
#define MII_DP83720_INT_STAT3	0x18
#define MII_DP83720_RESET_CTRL	0x1f

#define DP83720_HW_RESET	BIT(15)
#define DP83720_SW_RESET	BIT(14)

#define DP83720_STRAP		0x45d
#define DP83720_SGMII_CTRL	0x608
#define SGMII_CONFIG_VAL	0x027B

/* INT_STAT1 bits */
#define DP83720_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83720_ESD_EVENT_INT_EN	BIT(3)
#define DP83720_LINK_STAT_INT_EN	BIT(5)
#define DP83720_ENERGY_DET_INT_EN	BIT(6)
#define DP83720_LINK_QUAL_INT_EN	BIT(7)

/* INT_STAT2 bits */
#define DP83720_SLEEP_MODE_INT_EN	BIT(2)
#define DP83720_OVERTEMP_INT_EN		BIT(3)
#define DP83720_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83720_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83720_LPS_INT_EN	BIT(0)
#define DP83720_WAKE_REQ_EN	BIT(1)
#define DP83720_NO_FRAME_INT_EN	BIT(2)
#define DP83720_POR_DONE_INT_EN	BIT(3)

/* SGMII CTRL bits */
#define DP83720_SGMII_AUTO_NEG_EN	BIT(0)
#define DP83720_SGMII_EN		BIT(9)

/* Strap bits */
#define DP83720_MASTER_MODE	BIT(5)
#define DP83720_RGMII_IS_EN	BIT(12)
#define DP83720_SGMII_IS_EN	BIT(13)
#define DP83720_RX_SHIFT_EN	BIT(14)
#define DP83720_TX_SHIFT_EN	BIT(15)

/* RGMII ID CTRL */
#define DP83720_RGMII_ID_CTRL	0x602
#define DP83720_RX_CLK_SHIFT	BIT(1)
#define DP83720_TX_CLK_SHIFT	BIT(0)

enum dp83720_chip_type {
	DP83720_ES1,
	DP83720_ES2,
	DP83720_CS1,
	DP83720_CS1_1,
	DP83721_CS1,
};

struct dp83720_init_reg {
	int reg;
	int val;
};

static const struct dp83720_init_reg dp83720_es1_init[] = {
	{0x182, 0x3000},
	{0x56a, 0xfc5},
	{0x510, 0x2d51},
	{0x408, 0x400},
	{0x409, 0x2b},
	{0x509, 0x4c04},
	{0x8a1, 0xbff},
	{0x802, 0x422},
	{0x853, 0x632},
	{0x824, 0x15e0},
	{0x86a, 0x106},
	{0x852, 0x3261},
	{0x851, 0x5141},
	{0x852, 0x327a},
	{0x851, 0x6652},
	{0x405, 0x1a0},
	{0x423, 0x2},
	{0x422, 0x0},
	{0x420, 0x5510},
	{0x421, 0x4077},
	{0x412, 0x10},
	{0x40f, 0x10},
	{0x85d, 0x6405},
	{0x894, 0x5557},
	{0x892, 0x1b0},
	{0x877, 0x55},
	{0x80b, 0x16},
	{0x864, 0x1fd0},
	{0x865, 0xa},
};

static const struct dp83720_init_reg dp83720_es2_master_init[] = {
	{0x408, 0x580},
	{0x409, 0x2a},
	{0x8a1, 0xbff},
	{0x802, 0x422},
	{0x840, 0x4120},
	{0x841, 0x6151},
	{0x8a3, 0x24e9},
	{0x800, 0x2090},
	{0x864, 0x1fd0},
	{0x865, 0x2},
	{0x405, 0x6800},
	{0x420, 0x3310},
	{0x412, 0x10},
	{0x40f, 0xe4ce},
	{0x844, 0x3f10},
	{0x8a0, 0x1e7},
	{0x843, 0x327a},
	{0x842, 0x6652},
	{0x50b, 0x7e7c},
	{0x56a, 0x7f41},
	{0x56b, 0xffb4},
	{0x813, 0x3fa0},
	{0x88d, 0x3fa0},
	{0x899, 0x3fa0},
};

static const struct dp83720_init_reg dp83720_es2_slave_init[] = {
	{0x408, 0x580},
	{0x409, 0x2a},
	{0x8a1, 0xbff},
	{0x802, 0x422},
	{0x853, 0x632},
	{0x824, 0x15e0},
	{0x86a, 0x106},
	{0x852, 0x327a},
	{0x851, 0x6652},
	{0x405, 0x6800},
	{0x420, 0x3310},
	{0x412, 0x10},
	{0x40f, 0x10},
	{0x85d, 0x6405},
	{0x894, 0x5057},
	{0x892, 0x1b0},
	{0x877, 0x55},
	{0x80b, 0x16},
	{0x864, 0x1fd0},
	{0x865, 0x2},
	{0x50b, 0x7e7c},
	{0x56a, 0x7f41},
	{0x56c, 0xffb4},
	{0x813, 0x3fa0},
	{0x88d, 0x3fa0},
	{0x899, 0x3fa0},
};

static const struct dp83720_init_reg dp83720_cs1_master_init[] = {
	{0x408, 0x580},
	{0x409, 0x2a},
	{0x8a1, 0xbff},
	{0x802, 0x422},
	{0x864, 0x1fd0},
	{0x865, 0x2},
	{0x8a3, 0x24e9},
	{0x800, 0x2090},
	{0x840, 0x4120},
	{0x841, 0x6151},
	{0x8a0, 0x01e7},
	{0x879, 0xe4ce},
	{0x89f, 0x1},
	{0x844, 0x3f10},
	{0x843, 0x327a},
	{0x842, 0x6652},
	{0x8a8, 0xe080},
	{0x8a9, 0x3f0},
	{0x88d, 0x3fa0},
	{0x889, 0x3fa0},
	{0x50b, 0x7e7c},
	{0x56a, 0x5f41},
	{0x56b, 0xffb4},
	{0x56c, 0xffb4},
	{0x573, 0x1},
};

static const struct dp83720_init_reg dp83720_cs1_slave_init[] = {
	{0x408, 0x580},
	{0x409, 0x2a},
	{0x8a1, 0xbff},
	{0x802, 0x422},
	{0x864, 0x1fd0},
	{0x865, 0x2},
	{0x853, 0x632},
	{0x824, 0x15e0},
	{0x86a, 0x106},
	{0x894, 0x5057},
	{0x85d, 0x6405},
	{0x892, 0x1b0},
	{0x852, 0x327a},
	{0x851, 0x6652},
	{0x877, 0x55},
	{0x80b, 0x16},
	{0x8a8, 0xe080},
	{0x8a9, 0x3f0},
	{0x88d, 0x3fa0},
	{0x899, 0x3fa0},
	{0x1f, 0x4000},
	{0x56a, 0x5f41},
	{0x56b, 0xffb4},
	{0x56c, 0xffb4},
	{0x573, 0x1},
};

static const struct dp83720_init_reg dp83720_cs1_1_master_init[] = {
	{0x405, 0x5800},
	{0x8ad, 0x3c51},
	{0x894, 0x5df7},
	{0x8a0, 0x9e7},
	{0x8c0, 0x4000},
	{0x814, 0x4800},
	{0x80d, 0x2ebf},
	{0x8c1, 0xb00},
	{0x87d, 0x001},
	{0x82e, 0x000},
	{0x837, 0x0f4},
	{0x8be, 0x200},
	{0x8c5, 0x4000},
	{0x8c7, 0x2000},
	{0x8b3, 0x05a},
	{0x8b4, 0x05a},
	{0x8b0, 0x202},
	{0x8b5, 0x0ea},
	{0x8ba, 0x2828},
	{0x8bb, 0x6828},
	{0x8bc, 0x028},
	{0x8bf, 0x000},
	{0x8b1, 0x014},
	{0x8b2, 0x008},
	{0x8ec, 0x000},
	{0x8c8, 0x003},
	{0x8be, 0x201},
	{0x18c, 0x001},
};

static const struct dp83720_init_reg dp83720_cs1_1_slave_init[] = {
	{0x894, 0x5df7},
	{0x56a, 0x5f40},
	{0x405, 0x5800},
	{0x8ad, 0x3c51},
	{0x894, 0x5df7},
	{0x8a0, 0x9e7},
	{0x8c0, 0x4000},
	{0x814, 0x4800},
	{0x80d, 0x2ebf},
	{0x8c1, 0xb00},
	{0x87d, 0x001},
	{0x82e, 0x000},
	{0x837, 0x0f4},
	{0x8be, 0x200},
	{0x8c5, 0x4000},
	{0x8c7, 0x2000},
	{0x8b3, 0x05a},
	{0x8b4, 0x05a},
	{0x8b0, 0x202},
	{0x8b5, 0x0ea},
	{0x8ba, 0x2828},
	{0x8bb, 0x6828},
	{0x8bc, 0x028},
	{0x8bf, 0x000},
	{0x8b1, 0x014},
	{0x8b2, 0x008},
	{0x8ec, 0x000},
	{0x8c8, 0x003},
	{0x8be, 0x201},
	{0x56a, 0x5f40},
	{0x18c, 0x001},
};

static const struct dp83720_init_reg dp83721_cs1_master_init[] = {
	{0x001F,0x8000},
	{0x0573,0x0801},
	{0x0834,0xC001},
	{0x0405,0x6C00},
	{0x08AD,0x3C51},
	{0x0894,0x5DF7},
	{0x08A0,0x09E7},
	{0x08C0,0x4000},
	{0x0814,0x4800},
	{0x080D,0x2EBF},
	{0x08C1,0x0B00},
	{0x087D,0x0001},
	{0x082E,0x0000},
	{0x0837,0x00F8},
	{0x08BE,0x0200},
	{0x08C5,0x4000},
	{0x08C7,0x2000},
	{0x08B3,0x005A},
	{0x08B4,0x005A},
	{0x08B0,0x0202},
	{0x08B5,0x00EA},
	{0x08BA,0x2828},
	{0x08BB,0x6828},
	{0x08BC,0x0028},
	{0x08BF,0x0000},
	{0x08B1,0x0014},
	{0x08B2,0x0008},
	{0x08EC,0x0000},
	{0x08FC,0x0091},
	{0x08BE,0x0201},
	{0x0335,0x0010},
	{0x0336,0x0009},
	{0x0337,0x0208},
	{0x0338,0x0208},
	{0x0339,0x02CB},
	{0x033A,0x0208},
	{0x033B,0x0109},
	{0x0418,0x0380},
	{0x0420,0xFF10},
	{0x0421,0x4033},
	{0x0422,0x0800},
	{0x0423,0x0002},
	{0x0484,0x0003},
	{0x055D,0x0008},
	{0x042B,0x0018},
	{0x087C,0x0080},
	{0x08C1,0x0900},
	{0x08fc,0x4091},
	{0x0881,0x5146},
	{0x08be,0x02a1},
	{0x0867,0x9999},
	{0x0869,0x9666},
	{0x086a,0x0009},
	{0x0822,0x11e1},
	{0x08f9,0x1f11},
	{0x08a3,0x24e8},
	{0x018C,0x0001},
	{0x001F,0x4000},
	{0x0573,0x0001},
	{0x056A,0x5F41},
};

static const struct dp83720_init_reg dp83721_cs1_slave_init[] = {
	{0x405, 0x6C00},
	{0x8ad, 0x3c51},
	{0x894, 0x5df7},
	{0x8a0, 0x9e7},
	{0x8c0, 0x4000},
	{0x814, 0x4800},
	{0x80d, 0x2ebf},
	{0x8c1, 0xb00},
	{0x87d, 0x001},
	{0x82e, 0x000},
	{0x837, 0x0f4},
	{0x8be, 0x200},
	{0x8c5, 0x4000},
	{0x8c7, 0x2000},
	{0x8b3, 0x05a},
	{0x8b4, 0x05a},
	{0x8b0, 0x202},
	{0x8b5, 0x0ea},
	{0x8ba, 0x2828},
	{0x8bb, 0x6828},
	{0x8bc, 0x028},
	{0x8bf, 0x000},
	{0x8b1, 0x014},
	{0x8b2, 0x008},
	{0x8ec, 0x000},
	{0x8c8, 0x003},
	{0x8be, 0x201},
	{0x56a, 0x5f40},
	{0x18c, 0x001},
};

struct dp83720_private {
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
	bool rx_shift;
	bool tx_shift;
};

#if 0
static irqreturn_t dp83720_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, MII_DP83720_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83720_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83720_INT_STAT3);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	return IRQ_NONE;

trigger_machine:
	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}
#endif

static int dp83720_config_intr(struct phy_device *phydev)
{
	int misr_status, ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		misr_status = phy_read(phydev, MII_DP83720_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83720_ANEG_COMPLETE_INT_EN |
				DP83720_ESD_EVENT_INT_EN |
				DP83720_LINK_STAT_INT_EN |
				DP83720_ENERGY_DET_INT_EN |
				DP83720_LINK_QUAL_INT_EN);

		ret = phy_write(phydev, MII_DP83720_INT_STAT1, misr_status);
		if (ret < 0)
			return ret;

		misr_status = phy_read(phydev, MII_DP83720_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83720_SLEEP_MODE_INT_EN |
				DP83720_OVERTEMP_INT_EN |
				DP83720_OVERVOLTAGE_INT_EN |
				DP83720_UNDERVOLTAGE_INT_EN);

		ret = phy_write(phydev, MII_DP83720_INT_STAT2, misr_status);
		if (ret < 0)
			return ret;

		misr_status = phy_read(phydev, MII_DP83720_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83720_LPS_INT_EN |
				DP83720_WAKE_REQ_EN |
				DP83720_NO_FRAME_INT_EN |
				DP83720_POR_DONE_INT_EN);

		ret = phy_write(phydev, MII_DP83720_INT_STAT3, misr_status);

	} else {
		ret = phy_write(phydev, MII_DP83720_INT_STAT1, 0);
		if (ret < 0)
			return ret;

		ret = phy_write(phydev, MII_DP83720_INT_STAT2, 0);
		if (ret < 0)
			return ret;

		ret = phy_write(phydev, MII_DP83720_INT_STAT3, 0);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83720_INT_STAT1);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83720_INT_STAT2);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83720_INT_STAT3);
		if (ret < 0)
			return ret;

		ret = 0;

	}

	return ret;
}

static int dp83720_config_aneg(struct phy_device *phydev)
{
	int value, ret;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, DP83720_SGMII_CTRL);
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, DP83720_SGMII_CTRL,
				SGMII_CONFIG_VAL);
		if (ret < 0)
			return ret;
	}

	return genphy_config_aneg(phydev);
}

static int dp83720_read_straps(struct phy_device *phydev)
{
	struct dp83720_private *dp83720 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, DP83720_DEVADDR, DP83720_STRAP);
	if (strap < 0)
		return strap;

	if (strap & DP83720_MASTER_MODE)
		dp83720->is_master = true;

	if (strap & DP83720_RGMII_IS_EN)
		dp83720->is_rgmii = true;

	if (strap & DP83720_SGMII_IS_EN)
		dp83720->is_sgmii = true;

	if (strap & DP83720_RX_SHIFT_EN)
		dp83720->rx_shift = true;

	if (strap & DP83720_TX_SHIFT_EN)
		dp83720->tx_shift = true;

	return 0;
};

static int dp83720_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, MII_DP83720_RESET_CTRL,
				DP83720_HW_RESET);
	else
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, MII_DP83720_RESET_CTRL,
				DP83720_SW_RESET);
	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int dp83720_phy_reset(struct phy_device *phydev)
{
	int ret;

	ret = dp83720_reset(phydev, false);
	if (ret)
		return ret;

	ret = dp83720_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int dp83720_write_seq(struct phy_device *phydev,
			     const struct dp83720_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
			ret = phy_write_mmd(phydev, DP83720_DEVADDR, init_data[i].reg,
				init_data[i].val);
			if (ret)
					return ret;
	}

	return 0;
}

static int dp83720_chip_init(struct phy_device *phydev)
{
	struct dp83720_private *dp83720 = phydev->priv;
	int ret;

	ret = dp83720_reset(phydev, true);
	if (ret)
		return ret;
	
	phydev->autoneg = AUTONEG_DISABLE;
    	phydev->speed = SPEED_1000;
	phydev->duplex = DUPLEX_FULL;
    	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
                              phydev->supported);


	if (dp83720->chip == DP83720_CS1 && dp83720->is_master) {
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, MII_BMSR, 0x940);
		if (ret)
			return ret;

		ret = phy_write_mmd(phydev, DP83720_DEVADDR, MII_BMSR, 0x140);
		if (ret)
			return ret;
	}

	if (dp83720->is_master)
	        ret = phy_write_mmd(phydev, DP83720_DEVADDR_MMD1, 0x0834,
				0xc001);
	else
	        ret = phy_write_mmd(phydev, DP83720_DEVADDR_MMD1, 0x0834,
				0x8001);
	if (ret)
		return ret;

	switch (dp83720->chip) {
	case DP83720_ES1:
		ret = dp83720_write_seq(phydev, dp83720_es1_init,
					ARRAY_SIZE(dp83720_es1_init));
		break;
	case DP83720_ES2:
		if (dp83720->is_master)
			ret = dp83720_write_seq(phydev, dp83720_es2_master_init,
						ARRAY_SIZE(dp83720_es2_master_init));
		else
			ret = dp83720_write_seq(phydev, dp83720_es2_slave_init,
						ARRAY_SIZE(dp83720_es2_slave_init));
		break;
	case DP83720_CS1:
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x573, 0x101);
		if (ret)
			return ret;

		if (dp83720->is_master)
			ret = dp83720_write_seq(phydev, dp83720_cs1_master_init,
						ARRAY_SIZE(dp83720_cs1_master_init));
		else
			ret = dp83720_write_seq(phydev, dp83720_cs1_slave_init,
						ARRAY_SIZE(dp83720_cs1_slave_init));
		break;
	case DP83720_CS1_1:
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x573, 0x101);
		if (ret)
			return ret;

		if (dp83720->is_master)
			ret = dp83720_write_seq(phydev, dp83720_cs1_1_master_init,
						ARRAY_SIZE(dp83720_cs1_1_master_init));
		else
			ret = dp83720_write_seq(phydev, dp83720_cs1_1_slave_init,
						ARRAY_SIZE(dp83720_cs1_1_slave_init));

		ret = dp83720_reset(phydev, false);

		ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, DP83720_DEVADDR, 0x56a, 0x5f41);
	case DP83721_CS1:
		ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x573, 0x101);
		if (ret)
			return ret;

		if (dp83720->is_master)
			ret = dp83720_write_seq(phydev, dp83721_cs1_master_init,
						ARRAY_SIZE(dp83721_cs1_master_init));
		else
			ret = dp83720_write_seq(phydev, dp83721_cs1_slave_init,
						ARRAY_SIZE(dp83721_cs1_slave_init));

		ret = dp83720_reset(phydev, false);

		ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, DP83720_DEVADDR, 0x56a, 0x5f41);
	default:
		return -EINVAL;
	};

	if (ret)
		return ret;

	/* Enable the PHY */
	ret = phy_write_mmd(phydev, DP83720_DEVADDR, 0x18c, 0x1);
	if (ret)
		return ret;

	mdelay(10);

	/* Do a software reset to restart the PHY with the updated values */
	return dp83720_reset(phydev, false);
}

static int dp83720_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	s32 rx_int_delay;
	s32 tx_int_delay;
	int rgmii_delay;
	int value, ret;

	ret = dp83720_chip_init(phydev);
	if (ret)
		return ret;

	if (phy_interface_is_rgmii(phydev)) {
		rx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      true);

		if (rx_int_delay <= 0)
			rgmii_delay = 0;
		else
			rgmii_delay = DP83720_RX_CLK_SHIFT;

		tx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      false);
		if (tx_int_delay <= 0)
			rgmii_delay &= ~DP83720_TX_CLK_SHIFT;
		else
			rgmii_delay |= DP83720_TX_CLK_SHIFT;

		if (rgmii_delay) {
			ret = phy_set_bits_mmd(phydev, DP83720_DEVADDR_MMD1,
					       DP83720_RGMII_ID_CTRL,
					       rgmii_delay);
			if (ret)
				return ret;
		}
	}

	value = phy_read_mmd(phydev, DP83720_DEVADDR, DP83720_SGMII_CTRL);
	if (value < 0)
		return value;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII)
		value |= DP83720_SGMII_EN;
	else
		value &= ~DP83720_SGMII_EN;

	ret = phy_write_mmd(phydev, DP83720_DEVADDR, DP83720_SGMII_CTRL, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int dp83720_probe(struct phy_device *phydev)
{
	struct dp83720_private *dp83720;
	int ret;

	dp83720 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83720),
			       GFP_KERNEL);
	if (!dp83720)
		return -ENOMEM;

	phydev->priv = dp83720;

	ret = dp83720_read_straps(phydev);
	if (ret)
		return ret;

	switch (phydev->phy_id) {
	case DP83TG720ES1_PHY_ID:
		dp83720->chip = DP83720_ES1;
		break;
	case DP83TG720ES2_PHY_ID:
		dp83720->chip = DP83720_ES2;
		break;
	case DP83TG720CS_1_0_PHY_ID:
		dp83720->chip = DP83720_CS1;
		break;
	case DP83TG720CS_1_1_PHY_ID:
		dp83720->chip = DP83720_CS1_1;
		break;
	case DP83TG721CS_1_0_PHY_ID:
		dp83720->chip = DP83721_CS1;
		break;
	default:
		return -EINVAL;
	};

	return dp83720_config_init(phydev);
}

#define DP83720_PHY_DRIVER(_id, _name)				\
	{							\
		PHY_ID_MATCH_EXACT(_id),			\
		.name		= (_name),			\
		.probe          = dp83720_probe,		\
		/* PHY_GBIT_FEATURES */				\
		.soft_reset	= dp83720_phy_reset,		\
		.config_init	= dp83720_config_init,		\
		.config_aneg = dp83720_config_aneg,		\
/*if 0								\
		.handle_interrupt = dp83720_handle_interrupt,	\
#endif	*/							\
		.config_intr = dp83720_config_intr,		\
		.suspend = genphy_suspend,			\
		.resume = genphy_resume,			\
	}

static struct phy_driver dp83720_driver[] = {
	DP83720_PHY_DRIVER(DP83TG720ES1_PHY_ID, "TI DP83TG720ES1"),
	DP83720_PHY_DRIVER(DP83TG720ES2_PHY_ID, "TI DP83TG720ES2"),
	DP83720_PHY_DRIVER(DP83TG720CS_1_0_PHY_ID, "TI DP83TG720CS1.0"),
	DP83720_PHY_DRIVER(DP83TG720CS_1_1_PHY_ID, "TI DP83TG720CS1.1"),
	DP83720_PHY_DRIVER(DP83TG721CS_1_0_PHY_ID, "TI DP83TG721CS1.0"),
};
module_phy_driver(dp83720_driver);

static struct mdio_device_id __maybe_unused dp83720_tbl[] = {
	{ PHY_ID_MATCH_EXACT(DP83TG720ES1_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TG720ES2_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TG720CS_1_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TG720CS_1_1_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TG721CS_1_0_PHY_ID) },
	{ },
};
MODULE_DEVICE_TABLE(mdio, dp83720_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TG720 PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
