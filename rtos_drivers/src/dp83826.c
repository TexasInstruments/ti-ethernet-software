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
 * \file  dp83826.c
 *
 * \brief This file contains the implementation of the dp83826 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

#include "../include/dp83826.h"
#include "dp83826_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define Dp83826_OUI                           (0x080028U)
#define Dp83826_MODEL                         (0x11U)
#define Dp83826_REV                           (0x01U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

static void Dp83826_enableAutoMdix(EthPhyDrv_Handle hPhy,
                                   bool enable);

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Phy_DrvObj_t gEnetPhyDrvDp83826 =
{
    .fxn =
    {
        .name               = "DP83826",
        .bind               = Dp83826_bind,
        .isPhyDevSupported  = Dp83826_isPhyDevSupported,
        .isMacModeSupported = Dp83826_isMacModeSupported,
        .config             = Dp83826_config,
        .reset              = GenericPhy_reset,
        .isResetComplete    = GenericPhy_isResetComplete,
        .readExtReg         = GenericPhy_readExtReg,
        .writeExtReg        = GenericPhy_writeExtReg,
        .printRegs          = Dp83826_printRegs,
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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83826_initCfg(Dp83826_Cfg *cfg)
{
    /* No extended config parameters at the moment */
}

void Dp83826_bind(EthPhyDrv_Handle* hPhy, 
					uint8_t phyAddr, 
					Phy_RegAccessCb_t* pRegAccessCb)
{
    Phy_Obj_t* pObj = (Phy_Obj_t*) hPhy;
    pObj->phyAddr = phyAddr;
    pObj->regAccessApi = *pRegAccessCb;
}

bool Dp83826_isPhyDevSupported(EthPhyDrv_Handle hPhy,
                                const void *pVersion)
{
	const Phy_Version *version = (Phy_Version *)pVersion;
    bool supported = false;

    if ((version->oui == Dp83826_OUI) &&
        (version->model == Dp83826_MODEL) &&
        (version->revision == Dp83826_REV))
    {
        supported = true;
    }

    return supported;
}

bool Dp83826_isMacModeSupported(EthPhyDrv_Handle hPhy,
								Phy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case PHY_MAC_MII_RMII:
            supported = true;
            break;

        /* This driver doesn't support MII and RGMII interfaces,
         * but the dp83826 PHY does support them */
        case PHY_MAC_MII_MII:
        case PHY_MAC_MII_RGMII:
        default:
            supported = false;
            break;
    }

    return supported;
}

int32_t Dp83826_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
                        Phy_Mii mii, 
                        bool loopbackEn)
{
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
	
    const Dp83826_Cfg *extendedCfg = (const Dp83826_Cfg *)pExtCfg;
    uint32_t extendedCfgSize = extCfgSize;
    bool enableAutoMdix;
    int32_t status = PHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        PHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\r\n",
                     phyAddr, extendedCfg, extendedCfgSize);
        status = PHY_EINVALIDPARAMS;
    }

    /* Auto-MDIX should be disabled in near-end loopback modes */
        enableAutoMdix = !loopbackEn;

    /* Enable Auto-MDIX and Robust Auto-MDIX */
    if (status == PHY_SOK)
    {
        Dp83826_enableAutoMdix(hPhy, enableAutoMdix);
    }

    return status;
}

static void Dp83826_enableAutoMdix(EthPhyDrv_Handle hPhy,
                                   bool enable)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
	
    PHYTRACE_DBG("PHY %u: %s automatic cross-over\n",
                 PhyPriv_getPhyAddr(hPhy), enable ? "enable" : "disable");
                  pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, Dp83826_PHYCR,
                  PHYCR_AUTOMDIX_ENABLE,
                  enable ? PHYCR_AUTOMDIX_ENABLE : 0);

    if (enable)
    {
        PHYTRACE_DBG("PHY %u: enable Robust Auto-MDIX\n", PhyPriv_getPhyAddr(hPhy));
        pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, Dp83826_CR1,
                       CR1_ROBUSTAUTOMDIX,
                       CR1_ROBUSTAUTOMDIX);
    }
    else
    {
        pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, Dp83826_PHYCR,
                       PHYCR_FORCEMDIX_MASK,
                       PHYCR_FORCEMDIX_MDI);
    }
}

void Dp83826_printRegs(EthPhyDrv_Handle hPhy)
{

    uint16_t val;
    const uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMCR, &val);
    printf("PHY %u: BMCR        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_BMSR, &val);
    printf("PHY %u: BMSR        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR1, &val);
    printf("PHY %u: PHYIDR1     = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_PHYIDR2, &val);
    printf("PHY %u: PHYIDR2     = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_ANAR, &val);
    printf("PHY %u: ANAR        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_ANLPAR, &val);
    printf("PHY %u: ANLPAR      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_ANER, &val);
    printf("PHY %u: ANER        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_ANNPTR, &val);
    printf("PHY %u: ANNPTR      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_ANNPRR, &val);
    printf("PHY %u: ANNPRR      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, Dp83826_CR1, &val);
    printf("PHY %u: CR1     = 0x%04x\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_GIGSR, &val);
    printf("PHY %u: STS1        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_GIGESR, &val);
    printf("PHY %u: 1KSCR       = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, Dp83826_PHYCR, &val);
    printf("PHY %u: PHYCR   = 0x%04x\n",phyAddr, val);
}