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
 * \file  dp83tg720.c
 *
 * \brief This file contains the implementation of the DP83TG720 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

#include "../include/dp83tg720.h"
#include "dp83tg720_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TG720_OUI                         (0x080028U)
#define DP83TG720_MODEL                       (0x28U)
#define DP83TG721_MODEL                       (0x29U)
#define DP83TG721_REV_CS1    				  (0U)
#define DP83TG720_REV_CS1    				  (3U)
#define DP83TG720_REV_CS1_1   				  (4U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

enum dp83tg720_chip_type {
         DP83TG720_CS1 = 3,
         DP83TG720_CS1_1 = 4,
		 DP83TG721_CS1 = 5,
};

struct dp83tg720_privParams {
         int chip;
         bool is_master;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Dp83tg720_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable);

static void Dp83tg720_resetHw(EthPhyDrv_Handle hPhy);

static int32_t Dp83tg720_readMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val);

static void Dp83tg720_writeMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tg720_setBitsMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tg720_readStraps(EthPhyDrv_Handle hPhy, Dp83tg720_MasterSlaveMode msMode);

static void Dp83tg720_writeSeq(EthPhyDrv_Handle hPhy, const struct dp83tg720_init_reg *init_data, int size);

static void Dp83tg720_chipInit(EthPhyDrv_Handle hPhy);

static void Dp83tg720_setMiiMode(EthPhyDrv_Handle hPhy, Phy_Mii mii);

static void Dp83tg720_configAutoNeg(EthPhyDrv_Handle hPhy, bool sgmiiAutoNegEn);

static void Dp83tg720_configClkShift(EthPhyDrv_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn);

static void Dp83tg720_configIntr(EthPhyDrv_Handle hPhy, bool intrEn);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Phy_DrvObj_t gEnetPhyDrvDp83tg720 =
{
    .fxn =
    {
        .name               = "Dp83tg720",
        .bind               = Dp83tg720_bind,
    	.isPhyDevSupported  = Dp83tg720_isPhyDevSupported,
    	.isMacModeSupported = Dp83tg720_isMacModeSupported,
    	.config             = Dp83tg720_config,
    	.reset              = Dp83tg720_reset,
    	.isResetComplete    = Dp83tg720_isResetComplete,
        .readExtReg         = GenericPhy_readExtReg,
        .writeExtReg        = GenericPhy_writeExtReg,
        .printRegs          = Dp83tg720_printRegs,
    	.adjPtpFreq              = NULL,
    	.adjPtpPhase             = NULL,
    	.getPtpTime              = NULL,
    	.setPtpTime              = NULL,
    	.getPtpTxTime            = NULL,
    	.getPtpRxTime            = NULL,
    	.waitPtpTxTime           = NULL,
    	.procStatusFrame         = NULL,
    	.getStatusFrameEthHeader = NULL,
    	.enablePtp               = NULL,
    	.tickDriver              = NULL,
    	.enableEventCapture      = NULL,
    	.enableTriggerOutput     = NULL,
    	.getEventTs              = NULL,		
    }
};

/* PHY Device Attributes */
static struct dp83tg720_privParams dp83tg720_params = {
	.chip = -1,
	.is_master = false,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83tg720_initCfg(Dp83tg720_Cfg *cfg)
{
	cfg->txClkShiftEn = true;
	cfg->rxClkShiftEn = true;
	cfg->interruptEn = false;
	cfg->sgmiiAutoNegEn = true;
	cfg->MasterSlaveMode = DP83TG720_MASTER_SLAVE_STRAP;
}

void Dp83tg720_bind(EthPhyDrv_Handle* hPhy, 
					uint8_t phyAddr, 
					Phy_RegAccessCb_t* pRegAccessCb)
{
    Phy_Obj_t* pObj = (Phy_Obj_t*) hPhy;
    pObj->phyAddr = phyAddr;
    pObj->regAccessApi = *pRegAccessCb;
}

bool Dp83tg720_isPhyDevSupported(EthPhyDrv_Handle hPhy,
                                const void *pVersion)
{
	const Phy_Version *version = (Phy_Version *)pVersion;

    bool supported = false;

    if ((version->oui == DP83TG720_OUI) && (
        (version->model == DP83TG720_MODEL) || (version->model == DP83TG721_MODEL) ))
    {
        supported = true;
    }


	//Determine phy version
    if (version->revision == DP83TG720_REV_CS1)
		dp83tg720_params.chip = DP83TG720_CS1;
    else if (version->revision == DP83TG720_REV_CS1_1)
		dp83tg720_params.chip = DP83TG720_CS1_1;
	else if (version->revision == DP83TG721_REV_CS1)
		dp83tg720_params.chip = DP83TG721_CS1;
    return supported;
}

bool Dp83tg720_isMacModeSupported(EthPhyDrv_Handle hPhy,
                                Phy_Mii mii)
{
    bool supported = false;

    switch (mii)
    {
        case PHY_MAC_MII_MII:
			break;
        case PHY_MAC_MII_GMII:
			break;
        case PHY_MAC_MII_RGMII:
			supported = true;
            break;
		case PHY_MAC_MII_SGMII:
            supported = true;
            break;
        default:
            supported = false;
            break;
    }

    return supported;
}

int32_t Dp83tg720_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
						Phy_Mii mii, 
						bool loopbackEn)
{
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
	
    const Dp83tg720_Cfg *extendedCfg = (const Dp83tg720_Cfg *)pExtCfg;
    uint32_t extendedCfgSize = extCfgSize;

    int32_t status = PHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        PHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                        phyAddr, extendedCfg, extendedCfgSize);
        status = PHY_EINVALIDPARAMS;
    }

	/* Read strap register */
	if (status == PHY_SOK)
    {
		Dp83tg720_readStraps(hPhy, extendedCfg->MasterSlaveMode);
	}

	/* Set Master/Slave mode - through PHY config */
	if(extendedCfg->MasterSlaveMode == DP83TG720_MASTER_MODE)
	{
		dp83tg720_params.is_master = true;
		PHYTRACE_DBG("PHY %u: Master Mode enabled\n",phyAddr);
	}
	else if(extendedCfg->MasterSlaveMode == DP83TG720_SLAVE_MODE)
	{
		dp83tg720_params.is_master = false;
		PHYTRACE_DBG("PHY %u: Slave Mode enabled\n",phyAddr);
	}

	/* Init specific chip */
	if (status == PHY_SOK)
    {
		Dp83tg720_chipInit(hPhy);
	}

	/* Configure MII interface */
	if (status == PHY_SOK)
    {
		Dp83tg720_setMiiMode(hPhy, mii);
	}

	/* Configure SGMII auto negotiation */
	if (status == PHY_SOK &&
		mii == PHY_MAC_MII_SGMII)
    {
		Dp83tg720_configAutoNeg(hPhy, extendedCfg->sgmiiAutoNegEn);
	}

	/* Configure RGMII clock shift */
	if (status == PHY_SOK &&
		mii == PHY_MAC_MII_RGMII)
	{
		Dp83tg720_configClkShift(hPhy,
								 extendedCfg->txClkShiftEn,
							     extendedCfg->rxClkShiftEn);
	}

	/* Configure interrupts */
	if (status == PHY_SOK)
    {
		Dp83tg720_configIntr(hPhy, extendedCfg->interruptEn);
	}

	/* Set loopback configuration: enable or disable */
    if (status == PHY_SOK)
    {
        Dp83tg720_setLoopbackCfg(hPhy, loopbackEn);
    }

    return status;
}

static void Dp83tg720_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable)
{
    bool complete;
	int32_t status;
    uint16_t val;
    const uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    PHYTRACE_DBG("PHY %u: %s loopback\n", phyAddr, enable ? "enable" : "disable");

	status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
	if(status != PHY_SOK && enable)
	{
		PHYTRACE_ERR_IF(status != PHY_SOK,
                     "PHY %u: failed to set loopback mode: could not read reg %u\n",phyAddr, PHY_BMCR);
		return;
	}
    if (enable)
    {
		//xMII Loopback Mode
        val |= PHY_BMCR_LOOPBACK;
    }
    else
    {
		//Normal Mode
        val &= ~PHY_BMCR_LOOPBACK;
    }

	/* Specific predefined loopback configuration values are required for
     * normal mode or loopback mode */
    pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_BMCR, val);

    /* Software restart is required after changing LOOPCR register */
    Dp83tg720_reset(hPhy);

    do
    {
        complete = Dp83tg720_isResetComplete(hPhy);
    }
    while (!complete);
}


void Dp83tg720_reset(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    /* Global software reset */
    PHYTRACE_DBG("PHY %u: global soft-reset\n", PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, MII_DP83TG720_RESET_CTRL, DP83TG720_SW_RESET, DP83TG720_SW_RESET);
}

static void Dp83tg720_resetHw(EthPhyDrv_Handle hPhy)
{
    /* Global hardware reset */
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    PHYTRACE_DBG("PHY %u: global hard-reset\n", PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, MII_DP83TG720_RESET_CTRL, DP83TG720_HW_RESET, DP83TG720_HW_RESET);
}

bool Dp83tg720_isResetComplete(EthPhyDrv_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* Reset is complete when RESET bits have self-cleared */
    status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TG720_RESET_CTRL, &val);
    if (status == PHY_SOK)
    {
        complete = ((val & (DP83TG720_SW_RESET | DP83TG720_HW_RESET)) == 0U);
    }

    PHYTRACE_DBG("PHY %u: global reset is %s complete\n", PhyPriv_getPhyAddr(hPhy), complete ? "" : "not");

    return complete;
}

static int32_t Dp83tg720_readMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val)
{
    int32_t status;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    status = pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_ADDR);

    if (status == PHY_SOK)
    {
        status = pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_DR, reg);
    }

    if (status == PHY_SOK)
    {
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == PHY_SOK)
    {
        status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_MMD_DR, val);
    }

    PHYTRACE_VERBOSE_IF(status == PHY_SOK,
						 "PHY %u: read reg %u val 0x%04x\n", PhyPriv_getPhyAddr(hPhy), reg, *val);
    PHYTRACE_ERR_IF(status != PHY_SOK,
                     "PHY %u: failed to read reg %u\n", PhyPriv_getPhyAddr(hPhy), reg);

	return status;
}

static void Dp83tg720_writeMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
    int32_t status;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    PHYTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n", PhyPriv_getPhyAddr(hPhy), reg, val);

    status = pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == PHY_SOK)
    {
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_DR, reg);
    }

    if (status == PHY_SOK)
    {
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == PHY_SOK)
    {
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_DR, val);
    }

    PHYTRACE_ERR_IF(status != PHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", PhyPriv_getPhyAddr(hPhy), reg, val);
}

static void Dp83tg720_setBitsMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
	uint16_t value;
	int32_t status;

	status = Dp83tg720_readMmd(hPhy, devad, reg, &value);
	if (status == PHY_SOK)
    {
		value = value | val;
		Dp83tg720_writeMmd(hPhy, devad, reg, value);
	}
}

static void Dp83tg720_readStraps(EthPhyDrv_Handle hPhy, Dp83tg720_MasterSlaveMode msMode)
{
	uint16_t strap;
	int32_t status;

	status = Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_STRAP, &strap);
	if (status != PHY_SOK)
		return;

	PHYTRACE_DBG("PHY %u: Strap register is 0x%X\n",PhyPriv_getPhyAddr(hPhy), strap);

	if(msMode == DP83TG720_MASTER_SLAVE_STRAP)
	{
		if (strap & DP83TG720_MASTER_MODE_EN)
		{
			dp83tg720_params.is_master = true;
			PHYTRACE_DBG("PHY %u: Strap: Master Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
		}
		else
		{
			dp83tg720_params.is_master = false;
			PHYTRACE_DBG("PHY %u: Strap: Slave Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
		}
	}

	if (strap & DP83TG720_RGMII_IS_EN)
	{
		PHYTRACE_DBG("PHY %u: Strap: RGMII Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
		if (strap & DP83TG720_TX_SHIFT_EN)
			PHYTRACE_DBG("PHY %u: Strap: TX Clock Shift enabled\n",PhyPriv_getPhyAddr(hPhy));
		if (strap & DP83TG720_RX_SHIFT_EN)
			PHYTRACE_DBG("PHY %u: Strap: RX Clock Shift enabled\n",PhyPriv_getPhyAddr(hPhy));
	}
	else if (strap & DP83TG720_SGMII_IS_EN)
	{
		PHYTRACE_DBG("PHY %u: Strap: SGMII Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
	}

};

static void Dp83tg720_writeSeq(EthPhyDrv_Handle hPhy, const struct dp83tg720_init_reg *init_data, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		Dp83tg720_writeMmd(hPhy, init_data[i].mmd, init_data[i].reg, init_data[i].val);
	}
}

static void Dp83tg720_chipInit(EthPhyDrv_Handle hPhy)
{
	bool complete = false;

	/* Perform a hardware reset prior to configuration */
	Dp83tg720_resetHw(hPhy);
 	do
    {
        complete = Dp83tg720_isResetComplete(hPhy);
    }
    while (!complete);

	/* Apply chip-specific configuration */
	switch (dp83tg720_params.chip) {
		case DP83TG720_CS1:
			if (dp83tg720_params.is_master){
				Dp83tg720_writeSeq(hPhy, dp83tg720_cs1_master_init,
								   sizeof(dp83tg720_cs1_master_init)/sizeof(dp83tg720_cs1_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG720 CS1.0 Master\n",PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tg720_writeSeq(hPhy, dp83tg720_cs1_slave_init,
								   sizeof(dp83tg720_cs1_slave_init)/sizeof(dp83tg720_cs1_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG720 CS1.0 Slave\n",PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TG720_CS1_1:
			if (dp83tg720_params.is_master){
				Dp83tg720_writeSeq(hPhy, dp83tg720_cs1_1_master_init,
								   sizeof(dp83tg720_cs1_1_master_init)/sizeof(dp83tg720_cs1_1_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG720 CS1.1 Master\n",PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tg720_writeSeq(hPhy, dp83tg720_cs1_1_slave_init,
								   sizeof(dp83tg720_cs1_1_slave_init)/sizeof(dp83tg720_cs1_1_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG720 CS1.1 Slave\n",PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TG721_CS1:
			if (dp83tg720_params.is_master){
				Dp83tg720_writeSeq(hPhy, dp83tg721_cs1_master_init,
								   sizeof(dp83tg721_cs1_master_init)/sizeof(dp83tg721_cs1_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG721 CS1 Master\n",PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tg720_writeSeq(hPhy, dp83tg721_cs1_slave_init,
								   sizeof(dp83tg721_cs1_slave_init)/sizeof(dp83tg721_cs1_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TG721 CS1 Slave\n",PhyPriv_getPhyAddr(hPhy));}
			break;
		default:
			PHYTRACE_DBG("PHY %u: No supported DP83TG720 Chip. Skipping chip-specific configuration!\n",PhyPriv_getPhyAddr(hPhy));
			break;
	};

	/* Perform a software reset to restart the PHY with the updated configuration */
	Dp83tg720_reset(hPhy);
	do
    {
        complete = Dp83tg720_isResetComplete(hPhy);
    }
    while (!complete);

	/* Let the PHY start link-up procedure */
	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, 0x0573U, 0x0001U);

	/* Start send-s detection during link-up sequence */
	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, 0x056AU, 0x5F41U);
}

static void Dp83tg720_setMiiMode(EthPhyDrv_Handle hPhy, Phy_Mii mii)
{
    uint16_t rgmii_val = 0U;
	uint16_t sgmii_val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_CTRL, &rgmii_val);
	if(status != PHY_SOK)
		return;
	status = Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_SGMII_CTRL, &sgmii_val);
	if(status != PHY_SOK)
		return;

    if (mii == PHY_MAC_MII_RGMII)
    {
		rgmii_val |= DP83TG720_RGMII_EN;
		sgmii_val &= ~DP83TG720_SGMII_EN;
		PHYTRACE_DBG("PHY %u: RGMII Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
    }
	else if (mii == PHY_MAC_MII_SGMII)
	{
		rgmii_val &= ~DP83TG720_RGMII_EN;
		sgmii_val |= DP83TG720_SGMII_EN;
		PHYTRACE_DBG("PHY %u: SGMII Mode enabled\n",PhyPriv_getPhyAddr(hPhy));
	}
	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_CTRL, rgmii_val);
	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_SGMII_CTRL, sgmii_val);
}

static void Dp83tg720_configAutoNeg(EthPhyDrv_Handle hPhy, bool sgmiiAutoNegEn)
{
	uint16_t val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_SGMII_CTRL, &val);

	if(status != PHY_SOK)
		return;

	if(sgmiiAutoNegEn)
	{
		val |= DP83TG720_SGMII_AUTO_NEG_EN;
		PHYTRACE_DBG("PHY %u: SGMII Auto Negotiation enabled\n",PhyPriv_getPhyAddr(hPhy));
	}
	else
	{
		val &= ~DP83TG720_SGMII_AUTO_NEG_EN;
		PHYTRACE_DBG("PHY %u: SGMII Auto Negotiation disabled\n",PhyPriv_getPhyAddr(hPhy));
	}

	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_SGMII_CTRL, val);
}

static void Dp83tg720_configClkShift(EthPhyDrv_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn)
{
	uint16_t val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_ID_CTRL, &val);

	if(status != PHY_SOK)
		return;

	if (!txClkShiftEn)
		val &= ~DP83TG720_TX_CLK_SHIFT;
	else
		val |= DP83TG720_TX_CLK_SHIFT;
	if (!rxClkShiftEn)
		val &= ~DP83TG720_RX_CLK_SHIFT;
	else
		val |= DP83TG720_RX_CLK_SHIFT;

	Dp83tg720_writeMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_ID_CTRL, val);

	PHYTRACE_DBG("PHY %u: RGMII TX Clock Shift %s\n",PhyPriv_getPhyAddr(hPhy), txClkShiftEn ? "enabled" : "disabled");
	PHYTRACE_DBG("PHY %u: RGMII RX Clock Shift %s\n",PhyPriv_getPhyAddr(hPhy), rxClkShiftEn ? "enabled" : "disabled");
}

static void Dp83tg720_configIntr(EthPhyDrv_Handle hPhy, bool intrEn)
 {
	uint16_t reg_val;
	int32_t status;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
	
	if (intrEn) {
		PHYTRACE_DBG("PHY %u: Enable interrupts\n", PhyPriv_getPhyAddr(hPhy));
		status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT1, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TG720_ANEG_COMPLETE_INT_EN |
					DP83TG720_ESD_EVENT_INT_EN |
					DP83TG720_LINK_STAT_INT_EN |
                    DP83TG720_ENERGY_DET_INT_EN |
                    DP83TG720_LINK_QUAL_INT_EN);

		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT1, reg_val);

		status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT2, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TG720_SLEEP_MODE_INT_EN |
					DP83TG720_OVERTEMP_INT_EN |
					DP83TG720_OVERVOLTAGE_INT_EN |
					DP83TG720_UNDERVOLTAGE_INT_EN);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT2, reg_val);

        status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT3, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TG720_LPS_INT_EN |
					DP83TG720_WAKE_REQ_EN |
					DP83TG720_NO_FRAME_INT_EN |
					DP83TG720_POR_DONE_INT_EN);

		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT3, reg_val);

    }
	else {
		PHYTRACE_DBG("PHY %u: Disable interrupts\n",PhyPriv_getPhyAddr(hPhy));
		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT1, 0U);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT2, 0U);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TG720_INT_STAT3, 0U);
    }
}

void Dp83tg720_printRegs(EthPhyDrv_Handle hPhy)
{
    uint16_t val;
    const uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
    printf("PHY %u: BMCR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMSR, &val);
    printf("PHY %u: BMSR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR1, &val);
    printf("PHY %u: PHYIDR1 = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR2, &val);
    printf("PHY %u: PHYIDR2 = 0x%04x\r\n",phyAddr, val);

	pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_PHYSTS, &val);
    printf("PHY %u: PHYSTS  = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_PHYCR, &val);
    printf("PHY %u: PHYCR   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_MISR1, &val);
    printf("PHY %u: MISR1   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_MISR2, &val);
    printf("PHY %u: MISR2   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_RECR, &val);
    printf("PHY %u: RECR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_BISCR, &val);
    printf("PHY %u: BISCR   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_MISR3, &val);
    printf("PHY %u: MISR3   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_REG19, &val);
    printf("PHY %u: REG19   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_CDCR, &val);
    printf("PHY %u: CDCR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TG720_PHYRCR, &val);
    printf("PHY %u: PHYRCR  = 0x%04x\r\n",phyAddr, val);

	Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_SGMII_CTRL, &val);
	printf("PHY %u: SGMII_CTRL = 0x%04x\r\n",phyAddr, val);
	Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_CTRL, &val);
	printf("PHY %u: RGMII_CTRL = 0x%04x\r\n",phyAddr, val);
	Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR, DP83TG720_RGMII_ID_CTRL, &val);
	printf("PHY %u: RGMII_DELAY_CTRL = 0x%04x\r\n",phyAddr, val);
	Dp83tg720_readMmd(hPhy, DP83TG720_DEVADDR_MMD1, DP83TG720_MMD1_PMA_PMD_CONTROL, &val);
	printf("PHY %u: MMD1_PMA_PMD_CONTROL = 0x%04x\r\n",phyAddr, val);
}
