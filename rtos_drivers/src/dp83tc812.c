/*
 *  Copyright (c) Texas Instruments Incorporated 2021-2024
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
 * \file  dp83tc812.c
 *
 * \brief This file contains the implementation of the DP83TC812 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

#include "../include/dp83tc812.h"
#include "dp83tc812_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TC812_OUI                         (0x080028U)
#define DP83TC812_MODEL                       (0x27U)
#define DP83TC812_REV_CS1                     (0U)
#define DP83TC812_REV_CS2                     (1U)
#define DP83TC813_MODEL                       (0x21U)
#define DP83TC814_MODEL                       (0x26U)
#define DP83TC815_MODEL                       (0x20U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

enum dp83tc812_chip_type {
         DP83TC812_CS1 = 0,
         DP83TC812_CS2 = 1,
         DP83TC813 = 2,
         DP83TC814 = 3,
         DP83TC815 = 4,
};

struct dp83tc812_privParams {
         int chip;
         bool is_master;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Dp83tc812_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable);

static void Dp83tc812_resetHw(EthPhyDrv_Handle hPhy);

static int32_t Dp83tc812_readMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val);

static void Dp83tc812_writeMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_setBitsMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_readStraps(EthPhyDrv_Handle hPhy, Dp83tc812_MasterSlaveMode msMode);

static void Dp83tc812_writeSeq(EthPhyDrv_Handle hPhy, const struct dp83tc812_init_reg *init_data, int size);

static void Dp83tc812_chipInit(EthPhyDrv_Handle hPhy);

static void Dp83tc812_setMiiMode(EthPhyDrv_Handle hPhy, Phy_Mii mii);

static void Dp83tc812_configAutoNeg(EthPhyDrv_Handle hPhy, bool sgmiiAutoNegEn);

static void Dp83tc812_configClkShift(EthPhyDrv_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn);

static void Dp83tc812_configIntr(EthPhyDrv_Handle hPhy, bool intrEn);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Phy_DrvObj_t gEnetPhyDrvDp83tc812 =
{
    .fxn =
    {
        .name               = "Dp83tc812",
        .bind               = Dp83tc812_bind,
    	.isPhyDevSupported  = Dp83tc812_isPhyDevSupported,
    	.isMacModeSupported = Dp83tc812_isMacModeSupported,
    	.config             = Dp83tc812_config,
    	.reset              = Dp83tc812_reset,
    	.isResetComplete    = Dp83tc812_isResetComplete,
    	.readExtReg         = GenericPhy_readExtReg,
    	.writeExtReg        = GenericPhy_writeExtReg,
    	.printRegs          = Dp83tc812_printRegs,
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
static struct dp83tc812_privParams dp83tc812_params = {
	.chip = -1,
	.is_master = false,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83tc812_initCfg(Dp83tc812_Cfg *cfg)
{
	cfg->txClkShiftEn = true;
	cfg->rxClkShiftEn = true;
	cfg->interruptEn = false;
	cfg->sgmiiAutoNegEn = true;
	cfg->MasterSlaveMode = DP83TC812_MASTER_SLAVE_STRAP;
}

void Dp83tc812_bind(EthPhyDrv_Handle* hPhy, 
					uint8_t phyAddr, 
					Phy_RegAccessCb_t* pRegAccessCb)
{
    Phy_Obj_t* pObj = (Phy_Obj_t*) hPhy;
    pObj->phyAddr = phyAddr;
    pObj->regAccessApi = *pRegAccessCb;
}

bool Dp83tc812_isPhyDevSupported(EthPhyDrv_Handle hPhy,
                                const void *pVersion)
{
	const Phy_Version *version = (Phy_Version *)pVersion;

    bool supported = false;

    if ((version->oui == DP83TC812_OUI) &&
        ((version->model == (DP83TC812_MODEL)) || (version->model == (DP83TC813_MODEL)) || (version->model == (DP83TC814_MODEL)) || (version->model == (DP83TC815_MODEL))))
    {
        supported = true;
    }

	if (version->model == DP83TC812_MODEL)
	{
			//Determine PHY version
		if (version->revision == DP83TC812_REV_CS1)
			dp83tc812_params.chip = DP83TC812_CS1;
		else if (version->revision == DP83TC812_REV_CS2)
			dp83tc812_params.chip = DP83TC812_CS2;
	}
	else if (version->model == DP83TC813_MODEL)
		dp83tc812_params.chip = DP83TC813;
	else if (version->model == DP83TC814_MODEL)
		dp83tc812_params.chip = DP83TC814;
	else if (version->model == DP83TC815_MODEL)
		dp83tc812_params.chip = DP83TC815;

    return supported;
}

bool Dp83tc812_isMacModeSupported(EthPhyDrv_Handle hPhy,
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

int32_t Dp83tc812_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
						Phy_Mii mii, 
						bool loopbackEn)
{
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);

    const Dp83tc812_Cfg *extendedCfg = (const Dp83tc812_Cfg *)pExtCfg;
    uint32_t extendedCfgSize = extCfgSize;

    int32_t status = PHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        PHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                        phyAddr, extendedCfg, extendedCfgSize);
        status = PHY_EINVALIDPARAMS;
		return status;
    }

	/* Read strap register */
	if (status == PHY_SOK)
    {
		Dp83tc812_readStraps(hPhy, extendedCfg->MasterSlaveMode);
	}

	/* Set Master/Slave mode - through PHY config */
	if(extendedCfg->MasterSlaveMode == DP83TC812_MASTER_MODE)
	{
		dp83tc812_params.is_master = true;
		PHYTRACE_DBG("PHY %u: Master Mode enabled\n",phyAddr);
	}
	else if(extendedCfg->MasterSlaveMode == DP83TC812_SLAVE_MODE)
	{
		dp83tc812_params.is_master = false;
		PHYTRACE_DBG("PHY %u: Slave Mode enabled\n",phyAddr);
	}

	/* Init specific chip */
	if (status == PHY_SOK)
    {
		Dp83tc812_chipInit(hPhy);
	}

	/* Configure MII interface */
	if (status == PHY_SOK)
    {
		Dp83tc812_setMiiMode(hPhy, mii);
	}

	/* Configure SGMII auto negotiation */
	if (status == PHY_SOK &&
		mii == PHY_MAC_MII_SGMII)
    {
		Dp83tc812_configAutoNeg(hPhy, extendedCfg->sgmiiAutoNegEn);
	}

	/* Configure RGMII clock shift */
	if (status == PHY_SOK &&
		mii == PHY_MAC_MII_RGMII)
	{
		Dp83tc812_configClkShift(hPhy,
								 extendedCfg->txClkShiftEn,
							     extendedCfg->rxClkShiftEn);
	}

	/* Configure interrupts */
	if (status == PHY_SOK)
    {
		Dp83tc812_configIntr(hPhy, extendedCfg->interruptEn);
	}

	/* Set loopback configuration: enable or disable */
    if (status == PHY_SOK)
    {
        Dp83tc812_setLoopbackCfg(hPhy, loopbackEn);
    }

    return status;
}

static void Dp83tc812_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable)
{
    bool complete;
    int32_t status;
    uint16_t val;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    PHYTRACE_DBG("PHY %u: %s loopback\n", PhyPriv_getPhyAddr(hPhy), enable ? "enable" : "disable");

	status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
	if(status != PHY_SOK && enable)
	{
		PHYTRACE_ERR_IF(status != PHY_SOK,
                     "PHY %u: failed to set loopback mode: could not read reg %u\n", PhyPriv_getPhyAddr(hPhy), PHY_BMCR);
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
    Dp83tc812_reset(hPhy);

    do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);
}


void Dp83tc812_reset(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    /* Global software reset */
    PHYTRACE_DBG("PHY %u: global soft-reset\n", PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, MII_DP83TC812_RESET_CTRL, DP83TC812_SW_RESET, DP83TC812_SW_RESET);
}

static void Dp83tc812_resetHw(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* Global hardware reset */
    PHYTRACE_DBG("PHY %u: global hard-reset\n", PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, MII_DP83TC812_RESET_CTRL, DP83TC812_HW_RESET, DP83TC812_HW_RESET);
}

bool Dp83tc812_isResetComplete(EthPhyDrv_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* Reset is complete when RESET bits have self-cleared */
    status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TC812_RESET_CTRL, &val);
    if (status == PHY_SOK)
    {
        complete = ((val & (DP83TC812_SW_RESET | DP83TC812_HW_RESET)) == 0U);
    }

    PHYTRACE_DBG("PHY %u: global reset is %s complete\n", PhyPriv_getPhyAddr(hPhy), complete ? "" : "not");

    return complete;
}

static int32_t Dp83tc812_readMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val)
{
    int32_t status;
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
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
						 "PHY %u: read reg %u val 0x%04x\n",phyAddr, reg, *val);
    PHYTRACE_ERR_IF(status != PHY_SOK,
                     "PHY %u: failed to read reg %u\n",phyAddr, reg);

	return status;
}

static void Dp83tc812_writeMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
    int32_t status;
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    PHYTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n",phyAddr, reg, val);

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
                     "PHY %u: failed to write reg %u val 0x%04x\n",phyAddr, reg, val);
}

static void Dp83tc812_setBitsMmd(EthPhyDrv_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
	uint16_t value;
	int32_t status;

	status = Dp83tc812_readMmd(hPhy, devad, reg, &value);
	if (status == PHY_SOK)
    {
		value = value | val;
		Dp83tc812_writeMmd(hPhy, devad, reg, value);
	}
}

static void Dp83tc812_readStraps(EthPhyDrv_Handle hPhy, Dp83tc812_MasterSlaveMode msMode)
{
	uint16_t strap;
	int32_t status;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_STRAP, &strap);
	if (status != PHY_SOK)
		return;

	PHYTRACE_DBG("PHY %u: Strap register is 0x%X\n", PhyPriv_getPhyAddr(hPhy), strap);

	if(msMode == DP83TC812_MASTER_SLAVE_STRAP)
	{
		if (strap & DP83TC812_MASTER_MODE_EN)
		{
			dp83tc812_params.is_master = true;
			PHYTRACE_DBG("PHY %u: Strap: Master Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
		}
		else
		{
			dp83tc812_params.is_master = false;
			PHYTRACE_DBG("PHY %u: Strap: Slave Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
		}
	}

	if (strap & DP83TC812_RGMII_IS_EN)
	{
		PHYTRACE_DBG("PHY %u: Strap: RGMII Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
		if (((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_RX_SHIFT_EN) ||
			((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_SHIFT_EN))
			PHYTRACE_DBG("PHY %u: Strap: TX Clock Shift enabled\n", PhyPriv_getPhyAddr(hPhy));
		if (((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_RX_SHIFT_EN) ||
			((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_RX_SHIFT_EN))
			PHYTRACE_DBG("PHY %u: Strap: RX Clock Shift enabled\n", PhyPriv_getPhyAddr(hPhy));
	}
	else
	{
		PHYTRACE_DBG("PHY %u: Strap: SGMII Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
	}

};

static void Dp83tc812_writeSeq(EthPhyDrv_Handle hPhy, const struct dp83tc812_init_reg *init_data, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		Dp83tc812_writeMmd(hPhy, init_data[i].mmd, init_data[i].reg, init_data[i].val);
	}
}

static void Dp83tc812_chipInit(EthPhyDrv_Handle hPhy)
{
	bool complete = false;

	/* Perform a hardware reset prior to configuration */
	Dp83tc812_resetHw(hPhy);
	do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);

/* Apply chip-specific configuration */
	switch (dp83tc812_params.chip) {
		case DP83TC812_CS1:
			if (dp83tc812_params.is_master){
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs1_master_init,
								   sizeof(dp83tc812_cs1_master_init)/sizeof(dp83tc812_cs1_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS1.0 Master\n", PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs1_slave_init,
								   sizeof(dp83tc812_cs1_slave_init)/sizeof(dp83tc812_cs1_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS1.0 Slave\n", PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TC812_CS2:
			if (dp83tc812_params.is_master){
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_master_init,
								   sizeof(dp83tc812_cs2_master_init)/sizeof(dp83tc812_cs2_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS2.0 Master\n", PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_slave_init,
								   sizeof(dp83tc812_cs2_slave_init)/sizeof(dp83tc812_cs2_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS2.0 Slave\n", PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TC813:
			if (dp83tc812_params.is_master){
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_master_init,
								   sizeof(dp83tc812_cs2_master_init)/sizeof(dp83tc812_cs2_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC813 CS2.0 Master\n", PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_slave_init,
								   sizeof(dp83tc812_cs2_slave_init)/sizeof(dp83tc812_cs2_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC813 CS2.0 Slave\n", PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TC814:
			if (dp83tc812_params.is_master){
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_master_init,
								   sizeof(dp83tc812_cs2_master_init)/sizeof(dp83tc812_cs2_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC814 CS2.0 Master\n", PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_slave_init,
								   sizeof(dp83tc812_cs2_slave_init)/sizeof(dp83tc812_cs2_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC814 CS2.0 Slave\n", PhyPriv_getPhyAddr(hPhy));}
			break;
		case DP83TC815:
			if (dp83tc812_params.is_master){
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_master_init,
								   sizeof(dp83tc812_cs2_master_init)/sizeof(dp83tc812_cs2_master_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC815 CS2.0 Master\n", PhyPriv_getPhyAddr(hPhy));}
			else{
				Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_slave_init,
								   sizeof(dp83tc812_cs2_slave_init)/sizeof(dp83tc812_cs2_slave_init[0]));
				PHYTRACE_DBG("PHY %u: Applying configuration for DP83TC815 CS2.0 Slave\n", PhyPriv_getPhyAddr(hPhy));}
			break;
		default:
			PHYTRACE_DBG("PHY %u: No supported DP83TC81x Chip. Skipping chip-specific configuration!\n", PhyPriv_getPhyAddr(hPhy));
			break;
	};

	/* Perform a software reset to restart the PHY with the updated configuration */
	Dp83tc812_reset(hPhy);
	do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);

	/* Enable transmitter */
	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, 0x0523U, 0x0000U);

}

static void Dp83tc812_setMiiMode(EthPhyDrv_Handle hPhy, Phy_Mii mii)
{
    uint16_t rgmii_val = 0U;
	uint16_t sgmii_val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, &rgmii_val);
	if(status != PHY_SOK)
		return;
	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &sgmii_val);
	if(status != PHY_SOK)
		return;

    if (mii == PHY_MAC_MII_RGMII)
    {
		rgmii_val |= DP83TC812_RGMII_EN;
		sgmii_val &= ~DP83TC812_SGMII_EN;
		PHYTRACE_DBG("PHY %u: RGMII Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
    }
	else if (mii == PHY_MAC_MII_SGMII)
	{
		rgmii_val &= ~DP83TC812_RGMII_EN;
		sgmii_val |= DP83TC812_SGMII_EN;
		PHYTRACE_DBG("PHY %u: SGMII Mode enabled\n", PhyPriv_getPhyAddr(hPhy));
	}

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, rgmii_val);
	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, sgmii_val);
}

static void Dp83tc812_configAutoNeg(EthPhyDrv_Handle hPhy, bool sgmiiAutoNegEn)
{
	uint16_t val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &val);

	if(status != PHY_SOK)
		return;

	if(sgmiiAutoNegEn)
	{
		val |= DP83TC812_SGMII_AUTO_NEG_EN;
		PHYTRACE_DBG("PHY %u: SGMII Auto Negotiation enabled\n", PhyPriv_getPhyAddr(hPhy));
	}
	else
	{
		val &= ~DP83TC812_SGMII_AUTO_NEG_EN;
		PHYTRACE_DBG("PHY %u: SGMII Auto Negotiation disabled\n", PhyPriv_getPhyAddr(hPhy));
	}

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, val);
}

static void Dp83tc812_configClkShift(EthPhyDrv_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn)
{
	uint16_t val = 0U;
	int32_t status = PHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, &val);

	if(status != PHY_SOK)
		return;

	if (!txClkShiftEn)
		val &= ~DP83TC812_TX_CLK_SHIFT;
	else
		val |= DP83TC812_TX_CLK_SHIFT;
	if (!rxClkShiftEn)
		val &= ~DP83TC812_RX_CLK_SHIFT;
	else
		val |= DP83TC812_RX_CLK_SHIFT;

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, val);

	PHYTRACE_DBG("PHY %u: RGMII TX Clock Shift %s\n", PhyPriv_getPhyAddr(hPhy), txClkShiftEn ? "enabled" : "disabled");
	PHYTRACE_DBG("PHY %u: RGMII RX Clock Shift %s\n", PhyPriv_getPhyAddr(hPhy), rxClkShiftEn ? "enabled" : "disabled");
}

static void Dp83tc812_configIntr(EthPhyDrv_Handle hPhy, bool intrEn)
 {
	uint16_t reg_val;
	int32_t status = PHY_SOK;
	Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

	if (intrEn) {
		PHYTRACE_DBG("PHY %u: Enable interrupts\n", PhyPriv_getPhyAddr(hPhy));
		status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT1, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TC812_ANEG_COMPLETE_INT_EN |
					DP83TC812_ESD_EVENT_INT_EN |
					DP83TC812_LINK_STAT_INT_EN |
                    DP83TC812_ENERGY_DET_INT_EN |
                    DP83TC812_LINK_QUAL_INT_EN);

		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT1, reg_val);

		status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT2, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TC812_SLEEP_MODE_INT_EN |
					DP83TC812_OVERTEMP_INT_EN |
					DP83TC812_OVERVOLTAGE_INT_EN |
					DP83TC812_UNDERVOLTAGE_INT_EN);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT2, reg_val);

        status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT3, &reg_val);
        if (status != PHY_SOK)
			return;

        reg_val |= (DP83TC812_LPS_INT_EN |
					DP83TC812_WAKE_REQ_EN |
					DP83TC812_NO_FRAME_INT_EN |
					DP83TC812_POR_DONE_INT_EN);

		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT3, reg_val);

    }
	else {
		PHYTRACE_DBG("PHY %u: Disable interrupts\n", PhyPriv_getPhyAddr(hPhy));
		pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT1, 0U);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT2, 0U);

        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, MII_DP83TC812_INT_STAT3, 0U);
    }
}

void Dp83tc812_printRegs(EthPhyDrv_Handle hPhy)
{
    uint16_t val;
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
    printf("PHY %u: BMCR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMSR, &val);
    printf("PHY %u: BMSR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR1, &val);
    printf("PHY %u: PHYIDR1 = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR2, &val);
    printf("PHY %u: PHYIDR2 = 0x%04x\r\n",phyAddr, val);

    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_PHYSTS, &val);
    printf("PHY %u: PHYSTS  = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_PHYCR, &val);
    printf("PHY %u: PHYCR   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_MISR1, &val);
    printf("PHY %u: MISR1   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_MISR2, &val);
    printf("PHY %u: MISR2   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_RECR, &val);
    printf("PHY %u: RECR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_BISCR, &val);
    printf("PHY %u: BISCR   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_MISR3, &val);
    printf("PHY %u: MISR3   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_REG19, &val);
    printf("PHY %u: REG19   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_CDCR, &val);
    printf("PHY %u: CDCR    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83TC812_PHYRCR, &val);
    printf("PHY %u: PHYRCR  = 0x%04x\r\n",phyAddr, val);

	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &val);
	printf("PHY %u: SGMII_CTRL      = 0x%04x\r\n",phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, &val);
	printf("PHY %u: RGMII_CTRL      = 0x%04x\r\n",phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, &val);
	printf("PHY %u: RGMII_ID_CTRL   = 0x%04x\r\n",phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR_MMD1, DP83TC812_MMD1_PMA_CTRL_2, &val);
	printf("PHY %u: MMD1_PMA_CTRL_2 = 0x%04x\r\n",phyAddr, val);
}
