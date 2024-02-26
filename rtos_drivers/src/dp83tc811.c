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
 */

/*!
 * \file  dp83tc811.c
 *
 * \brief This file contains the implementation of the DP83TC811 PHYs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/drv/enet/include/core/enet_utils.h>
#include <ti/drv/enet/priv/core/enet_trace_priv.h>
#include <ti/drv/enet/include/phy/enetphy.h>
#include <ti/drv/enet/include/phy/dp83tc811.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83tc811_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TC811_OUI                         (0x080028U)
#define DP83TC811_MODEL                       (0x25U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

struct dp83tc811_init_reg {
         uint32_t reg;
         uint16_t val;
};

struct dp83tc811_private {
         int chip;
		 int model;
         bool is_master;
};

static const struct dp83tc811_init_reg dp83tc811_master_init[] = {
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
 
static const struct dp83tc811_init_reg dp83tc811_slave_init[] = {
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

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83tc811_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83tc811_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83tc811_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83tc811_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83tc811_reset(EnetPhy_Handle hPhy);

static void Dp83tc811_resetHw(EnetPhy_Handle hPhy);

static bool Dp83tc811_isResetComplete(EnetPhy_Handle hPhy);

static int32_t Dp83tc811_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val);

static void Dp83tc811_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc811_readStraps(EnetPhy_Handle hPhy);

static void Dp83tc811_writeSeq(EnetPhy_Handle hPhy, const struct dp83tc811_init_reg *init_data, int size);							  

static void Dp83tc811_chipInit(EnetPhy_Handle hPhy);

static void Dp83tc811_configIntr(EnetPhy_Handle hPhy, bool intrEn);

static void Dp83tc811_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83tc811 =
{
    .name               = "dp83tc811",
    .isPhyDevSupported  = Dp83tc811_isPhyDevSupported,
    .isMacModeSupported = Dp83tc811_isMacModeSupported,
    .config             = Dp83tc811_config,
    .reset              = Dp83tc811_reset,
    .isResetComplete    = Dp83tc811_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Dp83tc811_printRegs,
};

/* PHY Device Attributes */
static struct dp83tc811_private dp83tc811 = {
	.is_master = false,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static bool Dp83tc811_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

	if ((version->oui == DP83TC811_OUI) &&
        (version->model == (DP83TC811_MODEL)))
    {
        supported = true;
    }

	if (version->model == DP83TC811_MODEL)
		dp83tc811.model = DP83TC811_MODEL;
		
    return supported;
}

static bool Dp83tc811_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported = false;

    switch (mii)
    {
		case ENETPHY_MAC_MII_RMII:
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

static int32_t Dp83tc811_config(EnetPhy_Handle hPhy,
								const EnetPhy_Cfg *cfg,
								EnetPhy_Mii mii)
{
    const Dp83tc811_Cfg *extendedCfg = (const Dp83tc811_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    
	uint16_t value = 0;
	bool enableAutoNeg = true;
	
	int32_t status = ENETPHY_SOK;
	
	/* Set Parameters for Automotive PHYs - Fixed Speed */
	EnetPhy_State *state = &hPhy->state;
    	state->phyLinkCaps = ENETPHY_LINK_CAP_FD100; // 100Mbit/s - Full Duplex
	
	/* Extended Config is not necessary! */ //*****************************************
    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        /*ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                         hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;*/
    }
	
	//PHY Init (Depends on Interface and Mode)
	ENETTRACE_DBG("Applying configuration\n");
	
	Dp83tc811_readStraps(hPhy);
	
	if (mii == ENETPHY_MAC_MII_SGMII)
	{
		ENETTRACE_DBG("PHY %u: Enabling SGMII Mode\n", hPhy->addr);
	}
	else
	{
		ENETTRACE_DBG("PHY %u: Enabling RGMII Mode\n", hPhy->addr);
	}
		
	//Init specific chip
	Dp83tc811_chipInit(hPhy);
	
	//Enable RGMII/SGMII Interface
	status = Dp83tc811_readMmd(hPhy, DP83TC811_DEVADDR, MII_DP83TC811_SGMII_CTRL, &value);
	if(status != ENETPHY_SOK)
		return status;
	if (mii == ENETPHY_MAC_MII_SGMII)
	{
		value |= DP83TC811_SGMII_EN;
		if(enableAutoNeg)
			value |= DP83TC811_SGMII_AUTO_NEG_EN;
		else		
			value &= ~DP83TC811_SGMII_AUTO_NEG_EN;
	}
	else
	{
		value &= ~(DP83TC811_SGMII_EN | DP83TC811_SGMII_AUTO_NEG_EN);
	}
	Dp83tc811_writeMmd(hPhy, DP83TC811_DEVADDR, MII_DP83TC811_SGMII_CTRL, value);
	
	// Configure Interrupts
	Dp83tc811_configIntr(hPhy, false);
	
	/* Set loopback configuration: enable or disable */
    if (status == ENETPHY_SOK)
    {
        Dp83tc811_setLoopbackCfg(hPhy, cfg->loopbackEn);
    }
		
    return status;
}

static void Dp83tc811_setLoopbackCfg(EnetPhy_Handle hPhy,
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
    Dp83tc811_reset(hPhy);

    do
    {
        complete = Dp83tc811_isResetComplete(hPhy);
    }
    while (!complete);
}


static void Dp83tc811_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83TC811_RESET_CTRL, DP83TC811_SW_RESET, DP83TC811_SW_RESET);
}

static void Dp83tc811_resetHw(EnetPhy_Handle hPhy)
{
    /* Global hardware reset */
    ENETTRACE_DBG("PHY %u: global hard-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83TC811_RESET_CTRL, DP83TC811_HW_RESET, DP83TC811_HW_RESET);
}

static bool Dp83tc811_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bits have self-cleared */
    status = EnetPhy_readReg(hPhy, MII_DP83TC811_RESET_CTRL, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & (DP83TC811_SW_RESET | DP83TC811_HW_RESET)) == 0U);
    }

    ENETTRACE_DBG("PHY %u: global reset is %scomplete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

static int32_t Dp83tc811_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val)
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

static void Dp83tc811_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
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

static void Dp83tc811_readStraps(EnetPhy_Handle hPhy)
{
	uint16_t strap;
	int32_t status;
	
	status = Dp83tc811_readMmd(hPhy, DP83TC811_DEVADDR, DP83TC811_STRAP, &strap);
	if (status != ENETPHY_SOK)
		return;
	
	ENETTRACE_DBG("Strap is 0x%X\n", strap);
	if (strap & DP83TC811_MASTER_MODE)
	{
		dp83tc811.is_master = true;
		ENETTRACE_DBG("Strap: Master Mode enabled\n");
	}
	else
	{
		dp83tc811.is_master = false;
		ENETTRACE_DBG("Strap: Slave Mode enabled\n");
	}
	if (strap & DP83TC811_RGMII_IS_EN)
	{
		ENETTRACE_DBG("Strap: RGMII Mode enabled\n");
	}
	else
	{
		ENETTRACE_DBG("Strap: SGMII Mode enabled\n");
	}	
};

static void Dp83tc811_writeSeq(EnetPhy_Handle hPhy, const struct dp83tc811_init_reg *init_data, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		Dp83tc811_writeMmd(hPhy, DP83TC811_DEVADDR, init_data[i].reg, init_data[i].val);
	}
}
 
static void Dp83tc811_chipInit(EnetPhy_Handle hPhy)
{
	bool complete = false;
	
	/* Hardware Reset */
	Dp83tc811_resetHw(hPhy);
	
	
	/* Apply configuration */			   
	switch (dp83tc811.model) {
		case DP83TC811_MODEL:
			if (dp83tc811.is_master){
				Dp83tc811_writeSeq(hPhy, dp83tc811_master_init, 
								   sizeof(dp83tc811_master_init)/sizeof(dp83tc811_master_init[0]));
				ENETTRACE_DBG("Applying configuration for 811 Master\n");}
            else{
            	Dp83tc811_writeSeq(hPhy, dp83tc811_slave_init,
								   sizeof(dp83tc811_slave_init)/sizeof(dp83tc811_slave_init[0]));
			    ENETTRACE_DBG("Applying configuration for 811 Slave\n");}
			break;
		default:
			ENETTRACE_DBG("No supported DP83TC811 Chip. Cannot apply configuration!\n");
			break;
	};
			
	/* Do a software reset to restart the PHY with the updated values */
	Dp83tc811_reset(hPhy);	
	do
    {
        complete = Dp83tc811_isResetComplete(hPhy);
    }
    while (!complete);
	
	/* optional - Disable CLKOUT after SW reset, if not required for EMC testing */
	//Dp83tc811_writeMmd(hPhy, DP83TC811_DEVADDR, 0x0463U, 0x0000U);
	/* Enable TX */
	Dp83tc811_writeMmd(hPhy, DP83TC811_DEVADDR, 0x0475U, 0x0000U);
}

static void Dp83tc811_configIntr(EnetPhy_Handle hPhy, bool intrEn)
{
	uint16_t reg_val;
	int32_t status;
 
	if (intrEn) {
		ENETTRACE_DBG("PHY %u: Enable interrupts\n", hPhy->addr);
		status = EnetPhy_readReg(hPhy, MII_DP83TC811_INT_STAT1, &reg_val);
        if (status != ENETPHY_SOK)
			return;
 
        reg_val |= (DP83TC811_RX_ERR_HF_INT_EN |
					DP83TC811_MS_TRAINING_INT_EN |
					DP83TC811_ANEG_COMPLETE_INT_EN |
					DP83TC811_ESD_EVENT_INT_EN |
					DP83TC811_WOL_INT_EN |
					DP83TC811_LINK_STAT_INT_EN |
					DP83TC811_ENERGY_DET_INT_EN |
					DP83TC811_LINK_QUAL_INT_EN);
 
		EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT1, reg_val);
 
		status = EnetPhy_readReg(hPhy, MII_DP83TC811_INT_STAT2, &reg_val);
        if (status != ENETPHY_SOK)
			return;
 
        reg_val |= (DP83TC811_JABBER_DET_INT_EN |
					DP83TC811_POLARITY_INT_EN |
					DP83TC811_SLEEP_MODE_INT_EN |
					DP83TC811_OVERTEMP_INT_EN |
					DP83TC811_OVERVOLTAGE_INT_EN |
					DP83TC811_UNDERVOLTAGE_INT_EN);
 
        EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT2, reg_val);
 
        status = EnetPhy_readReg(hPhy, MII_DP83TC811_INT_STAT3, &reg_val);
        if (status != ENETPHY_SOK)
			return;

        reg_val |= (DP83TC811_LPS_INT_EN |
					DP83TC811_NO_FRAME_INT_EN |
					DP83TC811_POR_DONE_INT_EN);
 
		EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT3, reg_val);
 
    } 
	else {
		ENETTRACE_DBG("PHY %u: Disable interrupts\n", hPhy->addr);
		EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT1, 0U);
 
        EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT2, 0U);
 
        EnetPhy_writeReg(hPhy, MII_DP83TC811_INT_STAT3, 0U);
    }
}

static void Dp83tc811_printRegs(EnetPhy_Handle hPhy)
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
	Dp83tc811_readMmd(hPhy, DP83TC811_DEVADDR, MII_DP83TC811_SGMII_CTRL, &val);
	EnetUtils_printf("PHY %u: SGMII_CFG = 0x%04x\n", hPhy->addr, val);
	Dp83tc811_readMmd(hPhy, DP83TC811_DEVADDR, MII_DP83TC811_MII_CTRL, &val);
	EnetUtils_printf("PHY %u: xMII_CTRL = 0x%04x\n", hPhy->addr, val);
}
