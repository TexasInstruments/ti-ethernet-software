// SPDX-License-Identifier: GPL-2.0-only
/* Driver for the Texas Instruments DP83TG720 PHY
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */
/* 	10/05/2023 Alvaro Reyes (a-reyes1@ti.com)
 *	Updated Open Alliance set up script for 720/721 in both Master and Slave
 *	Added SQI, TDR, and Forced-Master/Slave features
 *  
 *  02/08/2024 Alvaro Reyes (a-reyes1@ti.com)
 * 
 *  New Global variables
 *  Functions Updated
 *  #if 0 #endif added to dp83720_handle_interuppt
 *  dp83720_config_aneg
 *  dp83720_reset
 *  dp83720_chip_init
 */
 

#include <linux/ethtool.h>
#include <linux/ethtool_netlink.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define DP83TG720_CS_1_1_PHY_ID			0x2000a284
#define DP83TG721_CS_1_0_PHY_ID			0x2000a290
#define MMD1F							0x1f
#define MMD1							0x1

#define MII_DP83TG720_INT_STAT1			0x12
#define MII_DP83TG720_INT_STAT2			0x13
#define MII_DP83TG720_INT_STAT3			0x18
#define MII_DP83TG720_RESET_CTRL		0x1f

#define PMA_PMD_CONTROL					0x1834
#define DP83TG720_TDR_CFG5				0x0306
#define DP83TG720_CDCR					0x1E
#define TDR_DONE						BIT(1)
#define TDR_FAIL						BIT(0)
#define DP83TG720_TDR_TC1				0x310
#define DP83TG720_TDR_START_BIT			BIT(15)	
#define BRK_MS_CFG						BIT(14)
#define PEAK_DETECT						BIT(7)
#define PEAK_SIGN						BIT(6)

#define DP83TG720_HW_RESET				BIT(15)
#define DP83TG720_SW_RESET				BIT(14)

#define DP83TG720_STRAP					0x45d
#define DP83TG720_SGMII_CTRL			0x608
#define SGMII_CONFIG_VAL				0x027B

/* INT_STAT1 bits */
#define DP83TG720_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83TG720_ESD_EVENT_INT_EN		BIT(3)
#define DP83TG720_LINK_STAT_INT_EN		BIT(5)
#define DP83TG720_ENERGY_DET_INT_EN		BIT(6)
#define DP83TG720_LINK_QUAL_INT_EN		BIT(7)

/* INT_STAT2 bits */
#define DP83TG720_SLEEP_MODE_INT_EN		BIT(2)
#define DP83TG720_OVERTEMP_INT_EN		BIT(3)
#define DP83TG720_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83TG720_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83TG720_LPS_INT_EN			BIT(0)
#define DP83TG720_WAKE_REQ_EN			BIT(1)
#define DP83TG720_NO_FRAME_INT_EN		BIT(2)
#define DP83TG720_POR_DONE_INT_EN		BIT(3)

/* SGMII CTRL bits */
#define DP83TG720_SGMII_AUTO_NEG_EN		BIT(0)
#define DP83TG720_SGMII_EN				BIT(9)

/* Strap bits */
#define DP83TG720_MASTER_MODE			BIT(5)
#define DP83TG720_RGMII_IS_EN			BIT(12)
#define DP83TG720_SGMII_IS_EN			BIT(13)
#define DP83TG720_RX_SHIFT_EN			BIT(14)
#define DP83TG720_TX_SHIFT_EN			BIT(15)

/* RGMII ID CTRL */
#define DP83TG720_RGMII_ID_CTRL			0x602
#define DP83TG720_RX_CLK_SHIFT			BIT(1)
#define DP83TG720_TX_CLK_SHIFT			BIT(0)

/* SQI Status Bits */
#define DP83TG720_sqi_reg_1				0x871
#define MAX_SQI_VALUE					0x7

enum DP83TG720_chip_type {
	DP83TG720_CS1_1,
	DP83TG721_CS1,
};

struct DP83TG720_init_reg {
	int MMD;
	int reg;
	int val;
};

static const struct DP83TG720_init_reg DP83TG720_cs1_1_master_init[] = {
	{0x1F, 0x001F, 0X8000},
	{0x1F, 0x0573, 0x0101},
	{0x1, 0x0834, 0xC001},
	{0x1F, 0x0405, 0x5800},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F4},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08C8, 0x0003},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

static const struct DP83TG720_init_reg DP83TG720_cs1_1_slave_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0101},
	{0x1, 0x0834, 0x8001},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x056a, 0x5F40},
	{0x1F, 0x0405, 0x5800},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087d, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00f4},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08C8, 0x0003},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x056A, 0x5F40},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0X5F41},
};

static const struct DP83TG720_init_reg DP83TG721_cs1_master_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0801},
	{0x1, 0x0834, 0xC001},
	{0x1F, 0x0405, 0x6C00},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F8},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08FC, 0x0091},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x0335, 0x0010},
	{0x1F, 0x0336, 0x0009},
	{0x1F, 0x0337, 0x0208},
	{0x1F, 0x0338, 0x0208},
	{0x1F, 0x0339, 0x02CB},
	{0x1F, 0x033A, 0x0208},
	{0x1F, 0x033B, 0x0109},
	{0x1F, 0x0418, 0x0380},
	{0x1F, 0x0420, 0xFF10},
	{0x1F, 0x0421, 0x4033},
	{0x1F, 0x0422, 0x0800},
	{0x1F, 0x0423, 0x0002},
	{0x1F, 0x0484, 0x0003},
	{0x1F, 0x055D, 0x0008},
	{0x1F, 0x042B, 0x0018},
	{0x1F, 0x087C, 0x0080},
	{0x1F, 0x08C1, 0x0900},
	{0x1F, 0x08fc, 0x4091},
	{0x1F, 0x0881, 0x5146},
	{0x1F, 0x08be, 0x02a1},
	{0x1F, 0x0867, 0x9999},
	{0x1F, 0x0869, 0x9666},
	{0x1F, 0x086a, 0x0009},
	{0x1F, 0x0822, 0x11e1},
	{0x1F, 0x08f9, 0x1f11},
	{0x1F, 0x08a3, 0x24e8},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

static const struct DP83TG720_init_reg DP83TG721_cs1_slave_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0801},
	{0x1, 0x0834, 0x8001},
	{0x1F, 0x0405, 0X6C00},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F8},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08FC, 0x0091},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x0456, 0x0160},
	{0x1F, 0x0335, 0x0010},
	{0x1F, 0x0336, 0x0009},
	{0x1F, 0x0337, 0x0208},
	{0x1F, 0x0338, 0x0208},
	{0x1F, 0x0339, 0x02CB},
	{0x1F, 0x033A, 0x0208},
	{0x1F, 0x033B, 0x0109},
	{0x1F, 0x0418, 0x0380},
	{0x1F, 0x0420, 0xFF10},
	{0x1F, 0x0421, 0x4033},
	{0x1F, 0x0422, 0x0800},
	{0x1F, 0x0423, 0x0002},
	{0x1F, 0x0484, 0x0003},
	{0x1F, 0x055D, 0x0008},
	{0x1F, 0x042B, 0x0018},
	{0x1F, 0x082D, 0x120F},
	{0x1F, 0x0888, 0x0438},
	{0x1F, 0x0824, 0x09E0},
	{0x1F, 0x0883, 0x5146},
	{0x1F, 0x08BE, 0x02A1},
	{0x1F, 0x0822, 0x11E1},
	{0x1F, 0x056A, 0x5F40},
	{0x1F, 0x08C1, 0x0900},
	{0x1F, 0x08FC, 0x4091},
	{0x1F, 0x08F9, 0x1F11},
	{0x1F, 0x084F, 0x290C},
	{0x1F, 0x0850, 0x3D33},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

static const struct DP83TG720_init_reg DP83TG720_tdr_config_init[] = {
	{0x1F, 0x301, 0xA008},
	{0x1F, 0x303, 0x0928},
	{0x1F, 0x304, 0x0004},
	{0x1F, 0x405, 0x6400},
};

struct DP83TG720_private {
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
	bool rx_shift;
	bool tx_shift;
};

static int DP83TG720_read_straps(struct phy_device *phydev)
{
	struct DP83TG720_private *DP83TG720 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, MMD1F, DP83TG720_STRAP);
	if (strap < 0)
		return strap;

	if (strap & DP83TG720_MASTER_MODE)
		DP83TG720->is_master = true;

	if (strap & DP83TG720_RGMII_IS_EN)
		DP83TG720->is_rgmii = true;

	if (strap & DP83TG720_SGMII_IS_EN)
		DP83TG720->is_sgmii = true;

	if (strap & DP83TG720_RX_SHIFT_EN)
		DP83TG720->rx_shift = true;

	if (strap & DP83TG720_TX_SHIFT_EN)
		DP83TG720->tx_shift = true;

	return 0;
};

static int DP83TG720_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TG720_RESET_CTRL,
				DP83TG720_HW_RESET);
	else
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TG720_RESET_CTRL,
				DP83TG720_SW_RESET);
	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int DP83TG720_phy_reset(struct phy_device *phydev)
{
	int ret;

	ret = DP83TG720_reset(phydev, false);
	if (ret)
		return ret;

	ret = DP83TG720_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int DP83TG720_write_seq(struct phy_device *phydev,
			     const struct DP83TG720_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
			ret = phy_write_mmd(phydev, init_data[i].MMD, init_data[i].reg,
				init_data[i].val);
			if (ret)
					return ret;
	}

	return 0;
}

static int DP83TG720_setup_master_slave(struct phy_device *phydev)
{
	switch(phydev->master_slave_set)
	{
		case MASTER_SLAVE_CFG_MASTER_FORCE:
		case MASTER_SLAVE_CFG_MASTER_PREFERRED:
			return phy_write_mmd(phydev, MMD1, 0x0834, 0xC001);
		case MASTER_SLAVE_CFG_SLAVE_FORCE:
		case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
			return phy_write_mmd(phydev, MMD1, 0x0834,0x8001);
		case MASTER_SLAVE_CFG_UNKNOWN:
		case MASTER_SLAVE_CFG_UNSUPPORTED:
		default:
			return 0;	
	}
}

static int DP83TG720_read_master_slave(struct phy_device *phydev)
{
	int ctrl2 = phy_read_mmd(phydev, MMD1, PMA_PMD_CONTROL);
	if (ctrl2 < 0)
		return ctrl2;
	
	if (ctrl2 & BRK_MS_CFG){
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

static int DP83TG720_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_status(phydev);
	if (ret)
		return ret;
	
	ret = DP83TG720_read_master_slave(phydev);

	return 0;
}

static int DP83TG720_sqi(struct phy_device *phydev){
	int sqi;

	sqi = phy_read_mmd(phydev,MMD1F,DP83TG720_sqi_reg_1);

	if(sqi < 0){
		printk("Invalid Value.\n");
		return sqi;
	}
	sqi = (sqi >> 1) & 0x7;

	return sqi;
}

static int DP83TG720_sqi_max(struct phy_device *phydev){
	return MAX_SQI_VALUE;
}

static int DP83TG720_cable_test_start(struct phy_device *phydev){
	
	if(DP83TG720_read_master_slave(phydev))
	{
		phy_write_mmd(phydev, MMD1F, 0x0576, 0x0400);
		msleep(100);
		printk("PHY is set as Master.\n");
	} else
	{
		printk("PHY is set as Slave. \n)");
	}

	DP83TG720_write_seq(phydev,DP83TG720_tdr_config_init, ARRAY_SIZE(DP83TG720_tdr_config_init));

	phy_write_mmd(phydev, MMD1F, DP83TG720_CDCR, DP83TG720_TDR_START_BIT);

	msleep(100);

	return 0;
}

static int DP83TG720_cable_test_report_trans(u32 result) {
	int length_of_fault = 0, TDR_STATE = 0;
	
	length_of_fault = (result & 0x3F00) >> 8;
	TDR_STATE = (result & 0xF0) >> 4;
	switch(TDR_STATE){
		case 3:
			printk("Short detected. \n");
			break;
		case 5:
			printk("Noise detected. \n");
			break;
		case 6:
			printk("Open detected. \n");
			break;
		case 7:
			printk("No fault detected. Cable OK \n");
			break;
		case 8:
			printk("Test in progress; initial value with TDR ON \n");
			break;
		case 13:
			printk("Test not possible (for example, noise, active link) \n");
			break;
		default :
			printk("Invalid value read.\n");
			break;
	}
	printk("Length of Fault: %d Meters\n", length_of_fault);
	return ETHTOOL_A_CABLE_RESULT_CODE_OK;	
}

static int DP83TG720_cable_test_report(struct phy_device *phydev)
{
	int ret;
	ret = phy_read_mmd(phydev, MMD1F, 0x030F);

	if (ret < 0)
		return ret;
	ethnl_cable_test_result(phydev, ETHTOOL_A_CABLE_PAIR_A, DP83TG720_cable_test_report_trans(ret));

	return 0;
}

static int DP83TG720_cable_test_get_status(struct phy_device *phydev, bool *finished){
	int statusReg;
	int TDR_Done;
	int TDR_Fail;
	*finished = false;
	statusReg = phy_read_mmd(phydev, MMD1F, DP83TG720_CDCR);
	
	TDR_Done = statusReg & TDR_DONE;
	TDR_Fail = statusReg & TDR_FAIL;

	if(TDR_Done && !TDR_Fail){
		*finished = true;
		printk("TDR HAS COMPLETED AND PASSED\n");
		phy_write_mmd(phydev, MMD1F, 0x0576, 0x0000);
		return DP83TG720_cable_test_report(phydev);
	}
	else if (TDR_Fail){
		printk("TDR HAS FAILED\n");
		phy_write_mmd(phydev, MMD1F, 0x0576, 0x0000);
	}
	return -EINVAL;
	
}

static int DP83TG720_chip_init(struct phy_device *phydev)
{
	struct DP83TG720_private *DP83TG720 = phydev->priv;
	int ret;

	ret = DP83TG720_reset(phydev, true);
	if (ret)
		return ret;
	
	phydev->autoneg = AUTONEG_DISABLE;
    	phydev->speed = SPEED_1000;
	phydev->duplex = DUPLEX_FULL;
    	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
                              phydev->supported);

	if (DP83TG720->is_master)
	        ret = phy_write_mmd(phydev, MMD1, 0x0834,
				0xc001);
	else
	        ret = phy_write_mmd(phydev, MMD1, 0x0834,
				0x8001);
	if (ret)
		return ret;

	switch (DP83TG720->chip) {
	case DP83TG720_CS1_1:
		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x101);
		if (ret)
			return ret;

		if (DP83TG720->is_master)
			ret = DP83TG720_write_seq(phydev, DP83TG720_cs1_1_master_init,
						ARRAY_SIZE(DP83TG720_cs1_1_master_init));
		else
			ret = DP83TG720_write_seq(phydev, DP83TG720_cs1_1_slave_init,
						ARRAY_SIZE(DP83TG720_cs1_1_slave_init));

		ret = DP83TG720_reset(phydev, false);

		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, MMD1F, 0x56a, 0x5f41);
	case DP83TG721_CS1:
		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x101);
		if (ret)
			return ret;

		if (DP83TG720->is_master)
			ret = DP83TG720_write_seq(phydev, DP83TG721_cs1_master_init,
						ARRAY_SIZE(DP83TG721_cs1_master_init));
		else
			ret = DP83TG720_write_seq(phydev, DP83TG721_cs1_slave_init,
						ARRAY_SIZE(DP83TG721_cs1_slave_init));

		ret = DP83TG720_reset(phydev, false);

		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, MMD1F, 0x56a, 0x5f41);
	default:
		return -EINVAL;
	};

	if (ret)
		return ret;

	/* Enable the PHY */
	ret = phy_write_mmd(phydev, MMD1F, 0x18c, 0x1);
	if (ret)
		return ret;

	mdelay(10);

	/* Do a software reset to restart the PHY with the updated values */
	return DP83TG720_reset(phydev, false);
}

static int DP83TG720_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	s32 rx_int_delay;
	s32 tx_int_delay;
	int rgmii_delay;
	int value, ret;

	ret = DP83TG720_chip_init(phydev);
	if (ret)
		return ret;

	if (phy_interface_is_rgmii(phydev)) {
		rx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0, true);
		if (rx_int_delay <= 0)
			rgmii_delay = 0;
		else
			rgmii_delay = DP83TG720_RX_CLK_SHIFT;

		tx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0, false);
		if (tx_int_delay <= 0)
			rgmii_delay &= ~DP83TG720_TX_CLK_SHIFT;
		else
			rgmii_delay |= DP83TG720_TX_CLK_SHIFT;

		if (rgmii_delay) {
			ret = phy_set_bits_mmd(phydev, MMD1, DP83TG720_RGMII_ID_CTRL, rgmii_delay);
			if (ret)
				return ret;
		}
	}

	value = phy_read_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL);
	if (value < 0)
		return value;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII)
		value |= DP83TG720_SGMII_EN;
	else
		value &= ~DP83TG720_SGMII_EN;

	ret = phy_write_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int DP83TG720_config_intr(struct phy_device *phydev)
{
	int misr_status, ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		misr_status = phy_read(phydev, MII_DP83TG720_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TG720_ANEG_COMPLETE_INT_EN |
				DP83TG720_ESD_EVENT_INT_EN |
				DP83TG720_LINK_STAT_INT_EN |
				DP83TG720_ENERGY_DET_INT_EN |
				DP83TG720_LINK_QUAL_INT_EN);

		ret = phy_write(phydev, MII_DP83TG720_INT_STAT1, misr_status);
		if (ret < 0)
			return ret;

		misr_status = phy_read(phydev, MII_DP83TG720_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TG720_SLEEP_MODE_INT_EN |
				DP83TG720_OVERTEMP_INT_EN |
				DP83TG720_OVERVOLTAGE_INT_EN |
				DP83TG720_UNDERVOLTAGE_INT_EN);

		ret = phy_write(phydev, MII_DP83TG720_INT_STAT2, misr_status);
		if (ret < 0)
			return ret;

		misr_status = phy_read(phydev, MII_DP83TG720_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TG720_LPS_INT_EN |
				DP83TG720_WAKE_REQ_EN |
				DP83TG720_NO_FRAME_INT_EN |
				DP83TG720_POR_DONE_INT_EN);

		ret = phy_write(phydev, MII_DP83TG720_INT_STAT3, misr_status);

	} else {
		ret = phy_write(phydev, MII_DP83TG720_INT_STAT1, 0);
		if (ret < 0)
			return ret;

		ret = phy_write(phydev, MII_DP83TG720_INT_STAT2, 0);
		if (ret < 0)
			return ret;

		ret = phy_write(phydev, MII_DP83TG720_INT_STAT3, 0);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83TG720_INT_STAT1);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83TG720_INT_STAT2);
		if (ret < 0)
			return ret;

		ret = phy_read(phydev, MII_DP83TG720_INT_STAT3);
		if (ret < 0)
			return ret;

		ret = 0;

	}

	return ret;
}

static int DP83TG720_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	int err, value, ret;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, DP83TG720_SGMII_CTRL);
		ret = phy_write_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL,
				SGMII_CONFIG_VAL);
		if (ret < 0)
			return ret;
	}

	err = DP83TG720_setup_master_slave(phydev);
	if (err < 0)
		return err;
	else if (err)
		changed = true;
	
	if (AUTONEG_ENABLE != phydev->autoneg)
		return genphy_setup_forced(phydev);

	return genphy_config_aneg(phydev);
}

static int DP83TG720_probe(struct phy_device *phydev)
{
	struct DP83TG720_private *DP83TG720;
	int ret;

	DP83TG720 = devm_kzalloc(&phydev->mdio.dev, sizeof(*DP83TG720),
			       GFP_KERNEL);
	if (!DP83TG720)
		return -ENOMEM;

	phydev->priv = DP83TG720;

	ret = DP83TG720_read_straps(phydev);
	if (ret)
		return ret;

	switch (phydev->phy_id) {
	case DP83TG720_CS_1_1_PHY_ID:
		DP83TG720->chip = DP83TG720_CS1_1;
		break;
	case DP83TG721_CS_1_0_PHY_ID:
		DP83TG720->chip = DP83TG721_CS1;
		break;
	default:
		return -EINVAL;
	};

	return DP83TG720_config_init(phydev);
}

#if 0
static irqreturn_t DP83TG720_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, MII_DP83TG720_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83TG720_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83TG720_INT_STAT3);
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

#define DP83TG720_PHY_DRIVER(_id, _name)				\
	{							\
		PHY_ID_MATCH_EXACT(_id),			\
		.name		= (_name),			\
		.probe          = DP83TG720_probe,		\
		/* PHY_GBIT_FEATURES */				\
		.soft_reset	= DP83TG720_phy_reset,		\
		.config_init	= DP83TG720_config_init,		\
		.config_aneg = DP83TG720_config_aneg,		\
		.get_sqi = DP83TG720_sqi,		\
		.get_sqi_max = DP83TG720_sqi_max,		\
		.cable_test_start = DP83TG720_cable_test_start,		\
		.cable_test_get_status = DP83TG720_cable_test_get_status,	\
		.read_status = DP83TG720_read_status,	\
/*if 0								\
		.handle_interrupt = DP83TG720_handle_interrupt,	\
#endif	*/							\
		.config_intr = DP83TG720_config_intr,		\
		.suspend = genphy_suspend,			\
		.resume = genphy_resume,			\
	}

static struct phy_driver DP83TG720_driver[] = {
	DP83TG720_PHY_DRIVER(DP83TG720_CS_1_1_PHY_ID, "TI DP83TG720CS1.1"),
	DP83TG720_PHY_DRIVER(DP83TG721_CS_1_0_PHY_ID, "TI DP83TG721CS1.0"),
};
module_phy_driver(DP83TG720_driver);

static struct mdio_device_id __maybe_unused DP83TG720_tbl[] = {
	{ PHY_ID_MATCH_EXACT(DP83TG720_CS_1_1_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TG721_CS_1_0_PHY_ID) },
	{ },
};
MODULE_DEVICE_TABLE(mdio, DP83TG720_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TG720 PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
