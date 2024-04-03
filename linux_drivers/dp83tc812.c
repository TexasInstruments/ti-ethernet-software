// SPDX-License-Identifier: GPL-2.0
/* Driver for the Texas Instruments DP83TC812 PHY
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 */
/* 01/17/2024 Alvaro Reyes (a-reyes1@ti.com)
* Updated Open Alliance script for the 812 in both Master and Slave
* Added SQI, TDR, and Forced-Master/Slave features
 * 
 * 02/08/2024 Alvaro Reyes (a-reyes1@ti.com)
 * 
 * New Global Variables
 * 
 * Updated Functions:
 * dp83812_reset
 * dp83812_phy_reset
 * dp83812_chip_init
 * dp83812_config_init
 * dp83812_config_aned
 * dp83812_probe
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


#define DP83TC812_CS2_0_PHY_ID			0x2000a271
#define DP83TC813_CS2_0_PHY_ID			0x2000a211
#define DP83TC814_CS2_0_PHY_ID			0x2000a261

#define MMD1F							0x1f
#define MMD1							0x1

#define DP83TC812_STRAP					0x45d
#define MII_DP83TC812_SGMII_CTRL		0x608
#define SGMII_CONFIG_VAL				0x027B
#define MII_DP83TC812_RGMII_CTRL		0x600
#define MII_DP83TC812_INT_STAT1			0x12
#define MII_DP83TC812_INT_STAT2			0x13
#define MII_DP83TC812_INT_STAT3			0x18
#define MII_DP83TC812_RESET_CTRL		0x1f
#define MMD1_PMA_CTRL_2					0x1834
#define DP83TC812_TDR_CFG5				0x0306
#define DP83TC812_CDCR					0x1E
#define TDR_DONE						BIT(1)
#define TDR_FAIL						BIT(0)
#define DP83TC812_TDR_TC1				0x310
#define DP83TC812_TDR_START_BIT			BIT(15)	
#define DP83TC812_TDR_half_open_det_en 	BIT(4)
#define BRK_MS_CFG						BIT(14)
#define HALF_OPEN_DETECT				BIT(8)
#define PEAK_DETECT						BIT(7)
#define PEAK_SIGN						BIT(6)

#define DP83TC812_HW_RESET				BIT(15)
#define DP83TC812_SW_RESET				BIT(14)

/* INT_STAT1 bits */
#define DP83TC812_RX_ERR_CNT_HALF_FULL_INT_EN	BIT(0)
#define DP83TC812_TX_ERR_CNT_HALF_FULL_INT_EN	BIT(1)
#define DP83TC812_MS_TRAIN_DONE_INT_EN			BIT(2)
#define DP83TC812_ESD_EVENT_INT_EN				BIT(3)
#define DP83TC812_LINK_STAT_INT_EN				BIT(5)
#define DP83TC812_ENERGY_DET_INT_EN				BIT(6)
#define DP83TC812_LINK_QUAL_INT_EN				BIT(7)

/* INT_STAT2 bits */
#define DP83TC812_JABBER_INT_EN			BIT(0)
#define DP83TC812_POL_INT_EN			BIT(1)
#define DP83TC812_SLEEP_MODE_INT_EN		BIT(2)
#define DP83TC812_OVERTEMP_INT_EN		BIT(3)
#define DP83TC812_FIFO_INT_EN			BIT(4)
#define DP83TC812_PAGE_RXD_INT_EN		BIT(5)
#define DP83TC812_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83TC812_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83TC812_LPS_INT_EN			BIT(0)
#define DP83TC812_WUP_INT_EN			BIT(1)
#define DP83TC812_WAKE_REQ_INT_EN		BIT(2)
#define DP83TC811_NO_FRAME_INT_EN		BIT(3)
#define DP83TC811_POR_DONE_INT_EN		BIT(4)
#define DP83TC812_SLEEP_FAIL_INT_EN		BIT(5)

/* RGMII_CTRL bits */
#define DP83TC812_RGMII_EN				BIT(3)

/* SGMII CTRL bits */
#define DP83TC812_SGMII_AUTO_NEG_EN		BIT(0)
#define DP83TC812_SGMII_EN				BIT(9)

/* Strap bits */
#define DP83TC812_MASTER_MODE			BIT(9)
#define DP83TC812_RGMII_IS_EN			BIT(7)

/* RGMII ID CTRL */
#define DP83TC812_RGMII_ID_CTRL			0x602
#define DP83TC812_RX_CLK_SHIFT			BIT(1)
#define DP83TC812_TX_CLK_SHIFT			BIT(0)

/*SQI Status bits*/
#define DP83TC812_dsp_reg_71			0x871
#define MAX_SQI_VALUE					0x7

enum DP83TC812_chip_type {
	DP83TC812_CS1 = 0,
	DP83TC812_CS2,
	DP83TC813_CS2,
	DP83TC814_CS2,
};

struct DP83TC812_init_reg {
	int	reg;
	int	val;
};

static const struct DP83TC812_init_reg DP83TC812_master_cs2_0_init[] = {
	{0x001F, 0x8000},
	{0x0523, 0x0001},
	{0x0834, 0xC001},
	{0x081C, 0x0FE2},
	{0x0872, 0x0300},
	{0x0879, 0x0F00},
	{0x0806, 0x2952},
	{0x0807, 0x3361},
	{0x0808, 0x3D7B},
	{0x083E, 0x045F},
	{0x0834, 0x8000},
	{0x0862, 0x00E8},
	{0x0896, 0x32CB},
	{0x003E, 0x0009},
	{0x001F, 0x4000},
	{0x0523, 0x0000},
};

static const struct DP83TC812_init_reg DP83TC812_slave_cs2_0_init[] = {
	{0x001F, 0x8000},
	{0x0523, 0x0001},
	{0x0834, 0x8001},
	{0x0873, 0x0821},
	{0x0896, 0x22FF},
	{0x089E, 0x0000},
	{0x001F, 0x4000},
	{0x0523, 0x0000},
};

static const struct DP83TC812_init_reg DP83TC812_tdr_config_init[] = {
	{0x523, 0x0001},
	{0x827, 0x4800},
	{0x301, 0x1701},
	{0x303, 0x023D},
	{0x305, 0x0015},
	{0x306, 0x001A},
	{0x01f, 0x4000},
	{0x523, 0x0000},
	{0x01f, 0x0000},
};

struct DP83TC812_private {
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
};

static int DP83TC812_read_straps(struct phy_device *phydev)
{
	struct DP83TC812_private *DP83TC812 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, MMD1F, DP83TC812_STRAP);
	if (strap < 0)
		return strap;

	if (strap & DP83TC812_MASTER_MODE)
		DP83TC812->is_master = true;

	if (strap & DP83TC812_RGMII_IS_EN)
		DP83TC812->is_rgmii = true;
	return 0;
};

static int DP83TC812_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC812_RESET_CTRL,
				DP83TC812_HW_RESET);
	else
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC812_RESET_CTRL,
				DP83TC812_SW_RESET);

	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static int DP83TC812_phy_reset(struct phy_device *phydev)
{
	int err;
	int ret;

	err = phy_write_mmd(phydev, MMD1F, MII_DP83TC812_RESET_CTRL, DP83TC812_HW_RESET);
	if (err < 0)
		return err;

	ret = DP83TC812_read_straps(phydev);
	if (ret)
		return ret;

	return 0;
}

static int DP83TC812_write_seq(struct phy_device *phydev, const struct
				DP83TC812_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
		ret = phy_write_mmd(phydev, MMD1F, init_data[i].reg,
					init_data[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static int DP83TC812_setup_master_slave(struct phy_device *phydev)
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

static int DP83TC812_read_master_slave(struct phy_device *phydev)
{
	int ctrl2 = phy_read_mmd(phydev, MMD1, MMD1_PMA_CTRL_2);

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

static int DP83TC812_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_status(phydev);
	if (ret)
		return ret;
	
	ret = DP83TC812_read_master_slave(phydev);
	
	return 0;
}

static int DP83TC812_sqi(struct phy_device *phydev){
	int sqi;

	sqi = phy_read_mmd(phydev,MMD1F,DP83TC812_dsp_reg_71);

	if(sqi < 0){
		printk("Error: Invalid Value.\n");
		return sqi;
	}

	sqi = (sqi >> 1) & 0x7;

	return sqi;
}

static int DP83TC812_sqi_max(struct phy_device *phydev){
	return MAX_SQI_VALUE;
}

static int DP83TC812_cable_test_start(struct phy_device *phydev){
	
	if(DP83TC812_read_master_slave(phydev)) 
		printk("PHY is set as Master.\n");
	else
		printk("PHY is set as Slave.\n");
	
	DP83TC812_write_seq(phydev, DP83TC812_tdr_config_init, ARRAY_SIZE(DP83TC812_tdr_config_init));

	phy_write_mmd(phydev,MMD1F,DP83TC812_CDCR, DP83TC812_TDR_START_BIT);

	msleep(100);

	return 0;
}

static int DP83TC812_cable_test_report_trans(u32 result) {
	int length_of_fault;
	if (result == 0) {
		printk("No Fault Detected. \n");
		return ETHTOOL_A_CABLE_RESULT_CODE_OK;
	} 
	else if (result & PEAK_DETECT) {
		length_of_fault = (result & 0x3F);

		// If Cable is Open
		if(result & PEAK_SIGN){	

			/*if(result & HALF_OPEN_DETECT){
				printk("Half Open Cable Detected\n");
				printk("Length of Fault: %d Meters\n",length_of_fault);
				return ETHTOOL_A_CABLE_RESULT_CODE_OPEN;
			} */				
			printk("Open Cable Detected\n");
			printk("Length of Fault: %d Meters\n",length_of_fault);
			return ETHTOOL_A_CABLE_RESULT_CODE_OPEN;
		}
		// Else it is Short
		printk("Short Detected\n");
		printk("Length of Fault: %d Meters\n",length_of_fault);
		return ETHTOOL_A_CABLE_RESULT_CODE_SAME_SHORT;

	}

	return ETHTOOL_A_CABLE_RESULT_CODE_UNSPEC;
}


static int DP83TC812_cable_test_report(struct phy_device *phydev){

	int ret;
	ret = phy_read_mmd(phydev,MMD1F,DP83TC812_TDR_TC1);
	
	if (ret < 0)
		return ret;
	ethnl_cable_test_result(phydev,ETHTOOL_A_CABLE_PAIR_A,DP83TC812_cable_test_report_trans(ret));
	
	return 0;
}

static int DP83TC812_cable_test_get_status(struct phy_device *phydev, bool *finished){
	int statusReg;
	int TDR_Done;
	int TDR_Fail;
	*finished = false;
	statusReg = phy_read_mmd(phydev, MMD1F,DP83TC812_CDCR);
	
	TDR_Done = statusReg & TDR_DONE;
	TDR_Fail = statusReg & TDR_FAIL;

	if(TDR_Done && !TDR_Fail){
		*finished = true;
		printk("\nTDR HAS COMPLETED AND PASSED\n");
		return DP83TC812_cable_test_report(phydev);
	}
	else if (TDR_Fail){
		printk("\nTDR HAS FAILED\n");
	}
	
	return -EINVAL;
}

static int DP83TC812_chip_init(struct phy_device *phydev)
{
	struct DP83TC812_private *DP83TC812 = phydev->priv;
	int ret;

	ret = DP83TC812_reset(phydev, true);
	if (ret)
		return ret;

	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		 phydev->supported);
	
	if (DP83TC812->is_master)
		ret = phy_write_mmd(phydev, MMD1, 0x0834, 0xc001);
	else
		ret = phy_write_mmd(phydev, MMD1, 0x0834, 0x8001);

	switch (DP83TC812->chip) {
	case DP83TC812_CS2:
		if (DP83TC812->is_master)
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_master_cs2_0_init,
						ARRAY_SIZE(DP83TC812_master_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
		else
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_slave_cs2_0_init,
						ARRAY_SIZE(DP83TC812_slave_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
	break;
	case DP83TC813_CS2:
		if (DP83TC812->is_master)
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_master_cs2_0_init,
						ARRAY_SIZE(DP83TC812_master_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
		else
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_slave_cs2_0_init,
						ARRAY_SIZE(DP83TC812_slave_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
	break;
	case DP83TC814_CS2:
		if (DP83TC812->is_master)
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_master_cs2_0_init,
						ARRAY_SIZE(DP83TC812_master_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
		else
			{
				ret = DP83TC812_write_seq(phydev,
						DP83TC812_slave_cs2_0_init,
						ARRAY_SIZE(DP83TC812_slave_cs2_0_init));
				phy_set_bits_mmd(phydev, MMD1F, 0x018B, BIT(6)); //Enables Autonomous Mode
			}
	break;
	default:
		return -EINVAL;
	};

	if (ret)
		return ret;

	mdelay(10);
	// phy_write_mmd(phydev, DP83TC812_DEVADDR, 0x523, 0x00);
	/* Do a soft reset to restart the PHY with updated values */
	return DP83TC812_reset(phydev, false);
}

static int DP83TC812_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	s32 rx_int_delay;
	s32 tx_int_delay;
	int rgmii_delay;
	int value, ret;

	ret = DP83TC812_chip_init(phydev);
	if (ret)
		return ret;

	if (phy_interface_is_rgmii(phydev)) {
		rx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      true);

		if (rx_int_delay <= 0)
			rgmii_delay = 0;
		else
			rgmii_delay = DP83TC812_RX_CLK_SHIFT;

		tx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0,
						      false);
		if (tx_int_delay <= 0)
			rgmii_delay &= ~DP83TC812_TX_CLK_SHIFT;
		else
			rgmii_delay |= DP83TC812_TX_CLK_SHIFT;

		if (rgmii_delay) {
			ret = phy_set_bits_mmd(phydev, MMD1,
					       DP83TC812_RGMII_ID_CTRL,
					       rgmii_delay);
			if (ret)
				return ret;
		}
	}

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, MII_DP83TC812_SGMII_CTRL);
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC812_SGMII_CTRL,
				SGMII_CONFIG_VAL);
	if (ret < 0)
		return ret;
	}

	return 0;
}

static int DP83TC812_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, MII_DP83TC812_INT_STAT1);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83TC812_INT_STAT2);
	if (err < 0)
		return err;

	err = phy_read(phydev, MII_DP83TC812_INT_STAT3);
	if (err < 0)
		return err;

	return 0;
}

static int DP83TC812_config_intr(struct phy_device *phydev)
{
	int misr_status, err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		misr_status = phy_read(phydev, MII_DP83TC812_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC812_ESD_EVENT_INT_EN |
				DP83TC812_LINK_STAT_INT_EN |
				DP83TC812_ENERGY_DET_INT_EN |
				DP83TC812_LINK_QUAL_INT_EN);

		err = phy_write(phydev, MII_DP83TC812_INT_STAT1, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83TC812_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC812_SLEEP_MODE_INT_EN |
				DP83TC812_OVERTEMP_INT_EN |
				DP83TC812_OVERVOLTAGE_INT_EN |
				DP83TC812_UNDERVOLTAGE_INT_EN);

		err = phy_write(phydev, MII_DP83TC812_INT_STAT2, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83TC812_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83TC812_LPS_INT_EN |
				DP83TC812_WAKE_REQ_INT_EN |
				DP83TC811_NO_FRAME_INT_EN |
				DP83TC811_POR_DONE_INT_EN);

		err = phy_write(phydev, MII_DP83TC812_INT_STAT3, misr_status);

	} else {
		err = phy_write(phydev, MII_DP83TC812_INT_STAT1, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83TC812_INT_STAT2, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83TC812_INT_STAT3, 0);
	}

	return err;
}

#if 0
static irqreturn_t DP83TC812_handle_interrupt(struct phy_device *phydev)
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
	irq_status = phy_read(phydev, MII_DP83TC812_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83TC812_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		trigger_machine = true;

	irq_status = phy_read(phydev, MII_DP83TC812_INT_STAT3);
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

static int DP83TC812_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	int err, value, ret;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, MII_DP83TC812_SGMII_CTRL);
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TC812_SGMII_CTRL,
				SGMII_CONFIG_VAL);
		if (ret < 0)
			return ret;
	}

	err = DP83TC812_setup_master_slave(phydev);
	if (err < 0)
		return err;
	else if (err)
		changed = true;
	
	if (AUTONEG_ENABLE != phydev->autoneg)
		return genphy_setup_forced(phydev);
	
	return genphy_config_aneg(phydev);
}

static int DP83TC812_probe(struct phy_device *phydev)
{
	struct DP83TC812_private *DP83TC812;
	int ret;

	DP83TC812 = devm_kzalloc(&phydev->mdio.dev, sizeof(*DP83TC812), GFP_KERNEL);
	if (!DP83TC812)
		return -ENOMEM;

	phydev->priv = DP83TC812;					
	ret = DP83TC812_read_straps(phydev);
	if (ret)
		return ret;

	switch (phydev->phy_id) {
	case DP83TC812_CS2_0_PHY_ID:
		DP83TC812->chip = DP83TC812_CS2;
	break;
	case DP83TC813_CS2_0_PHY_ID:
		DP83TC812->chip = DP83TC813_CS2;
		break;
	case DP83TC814_CS2_0_PHY_ID:
		DP83TC812->chip = DP83TC814_CS2;
	break;
	default:
	return -EINVAL;
	};

	return DP83TC812_config_init(phydev);
}

#define DP83TC812_PHY_DRIVER(_id, _name)				\
	{							\
		PHY_ID_MATCH_EXACT(_id),			\
		.name           = (_name),			\
		.probe          = DP83TC812_probe,		\
		/* PHY_BASIC_FEATURES */			\
		.soft_reset     = DP83TC812_phy_reset,		\
		.config_init    = DP83TC812_config_init,		\
		.config_aneg = DP83TC812_config_aneg,		\
		.ack_interrupt = DP83TC812_ack_interrupt,		\
/*if 0								\
		.handle_interrupt = dp83812_handle_interrupt,	\
#endif	*/									\
		.config_intr = DP83TC812_config_intr,		\
		.suspend = genphy_suspend,			\
		.resume = genphy_resume,			\
		.get_sqi = DP83TC812_sqi,				\
		.get_sqi_max = DP83TC812_sqi_max,		\
		.cable_test_start = DP83TC812_cable_test_start, \
		.cable_test_get_status = DP83TC812_cable_test_get_status, \
		.read_status	= DP83TC812_read_status,		\
	}

static struct phy_driver DP83TC812_driver[] = {
	DP83TC812_PHY_DRIVER(DP83TC812_CS2_0_PHY_ID, "TI DP83TC812CS2.0"),
	DP83TC812_PHY_DRIVER(DP83TC813_CS2_0_PHY_ID, "TI DP83TC813CS2.0"),
	DP83TC812_PHY_DRIVER(DP83TC814_CS2_0_PHY_ID, "TI DP83TC814CS2.0"),
	};

module_phy_driver(DP83TC812_driver);

static struct mdio_device_id __maybe_unused DP83TC812_tbl[] = {
	// { PHY_ID_MATCH_EXACT(DP83TC812_CS1_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC812_CS2_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC813_CS2_0_PHY_ID) },
	{ PHY_ID_MATCH_EXACT(DP83TC814_CS2_0_PHY_ID) },
	{ },
};
MODULE_DEVICE_TABLE(mdio, DP83TC812_tbl);									   

MODULE_DESCRIPTION("Texas Instruments DP83TC812 PHY driver");
MODULE_AUTHOR("Hari Nagalla <hnagalla@ti.com");
MODULE_LICENSE("GPL");
