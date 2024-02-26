/*
 *  Copyright (c) Texas Instruments Incorporated 2020
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changelog:
 *
 * Yannik Mündler (y-muendler@ti.com)
 *
 * 4/14/2021
 * Remove ES1/2 Script
 * 08/04/2022
 *
 */


/*!
 * \file  dp83tg720.c
 *
 * \brief This file contains the implementation of the DP83TG720 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/drv/enet/include/core/enet_utils.h>
#include <ti/drv/enet/priv/core/enet_trace_priv.h>
#include <ti/drv/enet/include/phy/enetphy.h>
#include <ti/drv/enet/include/phy/dp83tg720.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83tg720_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TG720_OUI                         (0x080028U)
#define DP83TG720_MODEL                       (0x28U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
enum dp83720_chip_type {
         DP83720_CS1 = 3,
         DP83720_CS1_1 = 4,
};

struct dp83720_init_reg {
         uint32_t reg;
         uint16_t val;
};

struct dp83tg720_private {
         int chip;
         bool is_master;
         bool rx_shift;
         bool tx_shift;
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
         {0x8a0, 0x01E7},
         {0x879, 0xe4ce},
         {0x89f, 0x1},
         {0x844, 0x3f10},
         {0x843, 0x327a},
         {0x842, 0x6652},
         {0x8a8, 0xe080},
         {0x8a9, 0x3f0},
         {0x88d, 0x3fa0},
         {0x899, 0x3fa0},
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
	{0x0405,0x5800},
	{0x08AD,0x3C51},
	{0x0894,0x5DF7},
	{0x08A0,0x09E7},
	{0x08C0,0x4000},
	{0x0814,0x4800},
	{0x080D,0x2EBF},
	{0x08C1,0x0B00},
	{0x087D,0x0001},
	{0x082E,0x0000},
	{0x0837,0x00F4},
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
	{0x08C8,0x0003},
	{0x08BE,0x0201},         
};
 
static const struct dp83720_init_reg dp83720_cs1_1_slave_init[] = {
	{0x0894,0x5DF7},
	{0x056A,0x5F40},
	{0x0405,0x5800},
	{0x08AD,0x3C51},
	{0x0894,0x5DF7},
	{0x08A0,0x09E7},
	{0x08C0,0x4000},
	{0x0814,0x4800},
	{0x080D,0x2EBF},
	{0x08C1,0x0B00},
	{0x087D,0x0001},
	{0x082E,0x0000},
	{0x0837,0x00F4},
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
	{0x08C8,0x0003},
	{0x08BE,0x0201},
	{0x056A,0x5F40},
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83tg720_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83tg720_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83tg720_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83tg720_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83tg720_reset(EnetPhy_Handle hPhy);

static void Dp83tg720_resetHw(EnetPhy_Handle hPhy);

static bool Dp83tg720_isResetComplete(EnetPhy_Handle hPhy);
							  
static int32_t Dp83tg720_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val);

static void Dp83tg720_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tg720_setBitsMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tg720_readStraps(EnetPhy_Handle hPhy);

static void Dp83tg720_writeSeq(EnetPhy_Handle hPhy, const struct dp83720_init_reg *init_data, int size);							  

static void Dp83tg720_chipInit(EnetPhy_Handle hPhy);

static void Dp83tg720_configIntr(EnetPhy_Handle hPhy, bool intrEn);

static void Dp83tg720_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83tg720 =
{
    .name               = "dp83tg720",
    .isPhyDevSupported  = Dp83tg720_isPhyDevSupported,
    .isMacModeSupported = Dp83tg720_isMacModeSupported,
    .config             = Dp83tg720_config,
    .reset              = Dp83tg720_reset,
    .isResetComplete    = Dp83tg720_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Dp83tg720_printRegs,
};

/* PHY Device Attributes */
static struct dp83tg720_private dp83tg720 = {
	.chip = -1,
	.is_master = false,
    .rx_shift = false,
    .tx_shift = false,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static bool Dp83tg720_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;
    if ((version->oui == DP83TG720_OUI) &&
        (version->model == DP83TG720_MODEL)
    {
        supported = true;
    }
	
	//Determine phy version
    if (version->revision == DP83TG720CS1_PHY_ID)
		dp83tg720.chip = DP83720_CS1;
    else if (version->revision == DP83TG720CS2_PHY_ID)
		dp83tg720.chip = DP83720_CS1_1;

    return supported;
}

static bool Dp83tg720_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported = false;

    switch (mii)
    {
        case ENETPHY_MAC_MII_RGMII:
		case ENETPHY_MAC_MII_SGMII:
            supported = true;
            break;
		case ENETPHY_MAC_MII_MII:
        case ENETPHY_MAC_MII_GMII:
        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Dp83tg720_config(EnetPhy_Handle hPhy,
								const EnetPhy_Cfg *cfg,
								EnetPhy_Mii mii)
{
    const Dp83tg720_Cfg *extendedCfg = (const Dp83tg720_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
	
	uint16_t rx_int_delay = 1U;	//Enable RX Clock Shift
	uint16_t tx_int_delay = 1U;	//Enable TX Clock Shift
	uint16_t rgmii_delay = 0U;
	uint16_t value = 0;
	bool enableAutoNeg = true;
	
    int32_t status = ENETPHY_SOK;
	
	/* Set Parameters for Automotive PHYs - Fixed Speed */
	//cfg->nwayCaps = ENETPHY_LINK_CAP_FD1000; // 1Gbit/s - Full Duplex
	EnetPhy_State *state = &hPhy->state;
    state->phyLinkCaps = ENETPHY_LINK_CAP_FD1000; // 1000Mbit/s - Full Duplex
	
	/* Extended Config is not necessary! */ //*****************************************
    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        /*ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                         hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;*/
    }
	
	//PHY Init (Depends on Version and Interface)
	ENETTRACE_DBG("Applying configuration for Chip %d\n", dp83tg720.chip);
	
	Dp83tg720_readStraps(hPhy);
	
	if (mii == ENETPHY_MAC_MII_SGMII)
	{
		ENETTRACE_DBG("PHY %u: Enabling SGMII Mode\n", hPhy->addr);
	}
	else
	{
		ENETTRACE_DBG("PHY %u: Enabling RGMII Mode\n", hPhy->addr);
	}
		
	//Init specific chip
	Dp83tg720_chipInit(hPhy);
	
	//Enable/Disable SGMII Interface
	status = Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, DP83720_SGMII_CTRL, &value);
	if(status != ENETPHY_SOK)
		return status;
	if (mii == ENETPHY_MAC_MII_SGMII)
	{
		value |= DP83720_SGMII_EN;
		if(enableAutoNeg)
			value |= DP83720_SGMII_AUTO_NEG_EN;
		else
			value &= ~DP83720_SGMII_AUTO_NEG_EN;
	}
	else
	{
		value &= ~(DP83720_SGMII_EN | DP83720_SGMII_AUTO_NEG_EN);
	}
	Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, DP83720_SGMII_CTRL, value);
	
	//Enable RGMII Interface	
	if (mii == ENETPHY_MAC_MII_RGMII) 
	{
		Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, DP83720_RGMII_CTRL, 0x0128U);
		
		if (rx_int_delay <= 0)
			rgmii_delay &= ~DP83720_RX_CLK_SHIFT;
		else
			rgmii_delay |= DP83720_RX_CLK_SHIFT;
 
		if (tx_int_delay <= 0)
			rgmii_delay &= ~DP83720_TX_CLK_SHIFT;
		else
			rgmii_delay |= DP83720_TX_CLK_SHIFT;
 
		Dp83tg720_setBitsMmd(hPhy, DP83720_DEVADDR,
							 DP83720_RGMII_ID_CTRL,
							 rgmii_delay);
							 
		/* Debugging Clock Shift adjustment TX: Pos 3 (default: 9), RX: Pos 2 (default: 8) */
		//Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, 0x0430U, 0x0980U);
	}
	
	// Configure Interrupts
	Dp83tg720_configIntr(hPhy, false);
	
	/* Set loopback configuration: enable or disable */
    if (status == ENETPHY_SOK)
    {
        Dp83tg720_setLoopbackCfg(hPhy, cfg->loopbackEn);
    }
	
    return status;
}

static void Dp83tg720_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable)
{
    bool complete;
	int32_t status;
    uint16_t val;

    ENETTRACE_DBG("PHY %u: %s loopback\n", hPhy->addr, enable ? "enable" : "disable");
	
	status = EnetPhy_readReg(hPhy, PHY_BMCR, &val);
	if(status != ENETPHY_SOK && enable)
	{
		ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to set loopback mode: could not read reg %u\n", hPhy->addr, PHY_BMCR);
		return;
	}
    if (enable)
    {
		//xMII Loopback Mode
        val |= BMCR_LOOPBACK;
    }
    else
    {
		//Normal Mode
        val &= ~BMCR_LOOPBACK;
    }

	/* Specific predefined loopback configuration values are required for
     * normal mode or loopback mode */
    EnetPhy_writeReg(hPhy, PHY_BMCR, val);
	
    /* Software restart is required after changing LOOPCR register */
    Dp83tg720_reset(hPhy);

    do
    {
        complete = Dp83tg720_isResetComplete(hPhy);
    }
    while (!complete);
}


static void Dp83tg720_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83720_RESET_CTRL, DP83720_SW_RESET, DP83720_SW_RESET);
}

static void Dp83tg720_resetHw(EnetPhy_Handle hPhy)
{
    /* Global hardware reset */
    ENETTRACE_DBG("PHY %u: global hard-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83720_RESET_CTRL, DP83720_HW_RESET, DP83720_HW_RESET);
}

static bool Dp83tg720_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bits have self-cleared */
    status = EnetPhy_readReg(hPhy, MII_DP83720_RESET_CTRL, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & (DP83720_SW_RESET | DP83720_HW_RESET)) == 0U);
    }

    ENETTRACE_DBG("PHY %u: global reset is %s complete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

static int32_t Dp83tg720_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val)
{	
    int32_t status;

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
						 "PHY %u: read reg %u val 0x%04x\n", hPhy->addr, reg, *val);                       
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to read reg %u\n", hPhy->addr, reg);
					 
	return status;
}

static void Dp83tg720_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{	
    int32_t status;

    ENETTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n", hPhy->addr, reg, val);

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", hPhy->addr, reg, val);
}

static void Dp83tg720_setBitsMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
	uint16_t value;
	int32_t status;
	
	status = Dp83tg720_readMmd(hPhy, devad, reg, &value);
	if (status == ENETPHY_SOK)
    {
		value = value | val;	
		Dp83tg720_writeMmd(hPhy, devad, reg, value);
	}
}

static void Dp83tg720_readStraps(EnetPhy_Handle hPhy)
{
	uint16_t strap;
	int32_t status;
	
	status = Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, DP83720_STRAP, &strap);
	if (status != ENETPHY_SOK)
		return;
	
	ENETTRACE_DBG("Strap is 0x%X\n", strap);
	if (strap & DP83720_MASTER_MODE)
	{
		dp83tg720.is_master = true;
		ENETTRACE_DBG("Strap: Master Mode enabled\n");
	}
	else
	{
		dp83tg720.is_master = false;
		ENETTRACE_DBG("Strap: Slave Mode enabled\n");
	}
	if (strap & DP83720_RGMII_IS_EN)
	{
		ENETTRACE_DBG("Strap: RGMII Mode enabled\n");
		if (strap & DP83720_RX_SHIFT_EN)
		{
			dp83tg720.rx_shift = true;
			ENETTRACE_DBG("Strap: RX Clock Shift enabled\n");
		}	
		if (strap & DP83720_TX_SHIFT_EN)
		{
			dp83tg720.tx_shift = true;
			ENETTRACE_DBG("Strap: TX Clock Shift enabled\n");
		}	
	}
	if (strap & DP83720_SGMII_IS_EN)
	{
		ENETTRACE_DBG("Strap: SGMII Mode enabled\n");
	}
};

static void Dp83tg720_writeSeq(EnetPhy_Handle hPhy, const struct dp83720_init_reg *init_data, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, init_data[i].reg, init_data[i].val);
	}
}
 
static void Dp83tg720_chipInit(EnetPhy_Handle hPhy)
{
	bool complete = false;
	
	/* Hardware Reset */
	Dp83tg720_resetHw(hPhy);
 
	/* Dont let PHY start link-up procedure */
	Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, 0x0573U, 0x0101U);
	
	/* Set specific value in BMSR register for CS1 */
	if (dp83tg720.chip == DP83720_CS1 && dp83tg720.is_master) {
		EnetPhy_writeReg(hPhy, PHY_BMSR, 0x0940U);
		EnetPhy_writeReg(hPhy, PHY_BMSR, 0x0140U);
	}
		
	/* Set Master or Slave mode */
	if (dp83tg720.is_master)
		Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR_MMD1, 0x0834U, 0xC001U);
	else
		Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR_MMD1, 0x0834U, 0x8001U);
    	
	/* Apply chip-specific configuration */
	switch (dp83tg720.chip) {
		case DP83720_CS1:
			if (dp83tg720.is_master)
				Dp83tg720_writeSeq(hPhy, dp83720_cs1_master_init,
								   sizeof(dp83720_cs1_master_init)/sizeof(dp83720_cs1_master_init[0]));
			else
				Dp83tg720_writeSeq(hPhy, dp83720_cs1_slave_init,
								   sizeof(dp83720_cs1_slave_init)/sizeof(dp83720_cs1_slave_init[0]));
			break;
		case DP83720_CS1_1:
			if (dp83tg720.is_master)
				Dp83tg720_writeSeq(hPhy, dp83720_cs1_1_master_init,
								   sizeof(dp83720_cs1_1_master_init)/sizeof(dp83720_cs1_1_master_init[0]));
			else
				Dp83tg720_writeSeq(hPhy, dp83720_cs1_1_slave_init,
								   sizeof(dp83720_cs1_1_slave_init)/sizeof(dp83720_cs1_1_slave_init[0]));
			break;
		default:
			ENETTRACE_DBG("No supported DP83TG720 Chip. Cannot apply configuration!\n");
			break;
	};
  
	/* Enable the PHY */
	EnetPhy_writeReg(hPhy, 0x018CU, 0x0001U);
		
	/* Do a software reset to restart the PHY with the updated values */
	Dp83tg720_reset(hPhy);	
	do
    {
        complete = Dp83tg720_isResetComplete(hPhy);
    }
    while (!complete);
	
	/* Let the PHY start link-up procedure */
	Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, 0x0573U, 0x0001U);
	
	/* Start send-s detection during link-up sequence */
	Dp83tg720_writeMmd(hPhy, DP83720_DEVADDR, 0x056AU, 0x5F41U);
}

static void Dp83tg720_configIntr(EnetPhy_Handle hPhy, bool intrEn)
 {
	uint16_t reg_val;
	int32_t status;
 
	if (intrEn) {
		ENETTRACE_DBG("PHY %u: Enable interrupts\n", hPhy->addr);
		status = EnetPhy_readReg(hPhy, MII_DP83720_INT_STAT1, &reg_val);
        if (status != ENETPHY_SOK)
			return;
 
        reg_val |= (DP83720_ANEG_COMPLETE_INT_EN |
					DP83720_ESD_EVENT_INT_EN |
					DP83720_LINK_STAT_INT_EN |
                    DP83720_ENERGY_DET_INT_EN |
                    DP83720_LINK_QUAL_INT_EN);
 
		EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT1, reg_val);
 
		status = EnetPhy_readReg(hPhy, MII_DP83720_INT_STAT2, &reg_val);
        if (status != ENETPHY_SOK)
			return;
 
        reg_val |= (DP83720_SLEEP_MODE_INT_EN |
					DP83720_OVERTEMP_INT_EN |
					DP83720_OVERVOLTAGE_INT_EN |
					DP83720_UNDERVOLTAGE_INT_EN);
 
        EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT2, reg_val);
 
        status = EnetPhy_readReg(hPhy, MII_DP83720_INT_STAT3, &reg_val);
        if (status != ENETPHY_SOK)
			return;

        reg_val |= (DP83720_LPS_INT_EN |
					DP83720_WAKE_REQ_EN |
					DP83720_NO_FRAME_INT_EN |
					DP83720_POR_DONE_INT_EN);
 
		EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT3, reg_val);
 
    } 
	else {
		ENETTRACE_DBG("PHY %u: Disable interrupts\n", hPhy->addr);
		EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT1, 0U);
 
        EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT2, 0U);
 
        EnetPhy_writeReg(hPhy, MII_DP83720_INT_STAT3, 0U);
    }
}
 
static void Dp83tg720_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2 = 0x%04x\n", phyAddr, val);
	Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, DP83720_SGMII_CTRL, &val);
	EnetUtils_printf("PHY %u: SGMII_CTRL = 0x%04x\n", hPhy->addr, val);
	Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, DP83720_RGMII_CTRL, &val);
	EnetUtils_printf("PHY %u: RGMII_CTRL = 0x%04x\n", hPhy->addr, val);
	Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, DP83720_RGMII_ID_CTRL, &val);
	EnetUtils_printf("PHY %u: RGMII_DELAY_CTRL = 0x%04x\n", hPhy->addr, val);
	Dp83tg720_readMmd(hPhy, DP83720_DEVADDR, 0x0430U, &val);
	EnetUtils_printf("PHY %u: RGMII_DELAY_TX_RX = 0x%04x\n", hPhy->addr, val);
	Dp83tg720_readMmd(hPhy, DP83720_DEVADDR_MMD1, 0x0834U, &val);
	EnetUtils_printf("PHY %u: REG_MasterSlave = 0x%04x\n", hPhy->addr, val);
}
