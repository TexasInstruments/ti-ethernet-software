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
 * \file  dp83822.c
 *
 * \brief This file contains the implementation of the DP83822 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/drv/enet/include/core/enet_utils.h>
#include <ti/drv/enet/priv/core/enet_trace_priv.h>
#include <ti/drv/enet/include/phy/enetphy.h>
#include <ti/drv/enet/include/phy/dp83822.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83822_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83822_OUI                           (0x080028U)
#define DP83822_MODEL                         (0x24U)
#define DP83822_REV                           (0x00U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

static bool Dp83822_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83822_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83822_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83822_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83822_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83822 =
{
    .name               = "dp83822",
    .isPhyDevSupported  = Dp83822_isPhyDevSupported,
    .isMacModeSupported = Dp83822_isMacModeSupported,
    .config             = Dp83822_config,
    .reset              = GenericPhy_reset,
    .isResetComplete    = GenericPhy_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Dp83822_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83822_initCfg(Dp83822_Cfg *cfg)
{
    /* No extended config parameters at the moment */
}

static bool Dp83822_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

    if ((version->oui == DP83822_OUI) &&
        (version->model == DP83822_MODEL) &&
        (version->revision == DP83822_REV))
    {
        supported = true;
    }

    return supported;
}

static bool Dp83822_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case ENETPHY_MAC_MII_RMII:
            supported = true;
            break;

        /* This driver doesn't support MII and RGMII interfaces,
         * but the DP83822 PHY does support them */
        case ENETPHY_MAC_MII_MII:
        case ENETPHY_MAC_MII_RGMII:
        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Dp83822_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii)
{
    const Dp83822_Cfg *extendedCfg = (const Dp83822_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    bool enableAutoMdix;
    int32_t status = ENETPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                      hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;
    }

    /* Auto-MDIX should be disabled in near-end loopback modes */
    enableAutoMdix = !cfg->loopbackEn;

    /* Enable Auto-MDIX and Robust Auto-MDIX */
    if (status == ENETPHY_SOK)
    {
        Dp83822_enableAutoMdix(hPhy, enableAutoMdix);
    }

    return status;
}

static void Dp83822_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable)
{
    ENETTRACE_DBG("PHY %u: %s automatic cross-over %s\n",
                  hPhy->addr, enable ? "enable" : "disable");
    EnetPhy_rmwReg(hPhy, DP83822_PHYCR,
                   PHYCR_AUTOMDIX_ENABLE,
                   enable ? PHYCR_AUTOMDIX_ENABLE : 0);

    if (enable)
    {
        ENETTRACE_DBG("PHY %u: enable Robust Auto-MDIX\n", hPhy->addr);
        EnetPhy_rmwReg(hPhy, DP83822_CR1,
                       CR1_ROBUSTAUTOMDIX,
                       CR1_ROBUSTAUTOMDIX);
    }
    else
    {
        EnetPhy_rmwReg(hPhy, DP83822_PHYCR,
                       PHYCR_FORCEMDIX_MASK,
                       PHYCR_FORCEMDIX_MDI);
    }
}

static void Dp83822_printRegs(EnetPhy_Handle hPhy)
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
    EnetPhy_readReg(hPhy, PHY_ANAR, &val);
    EnetUtils_printf("PHY %u: ANAR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANLPAR, &val);
    EnetUtils_printf("PHY %u: ANLPAR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANER, &val);
    EnetUtils_printf("PHY %u: ANER    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPTR, &val);
    EnetUtils_printf("PHY %u: ANNPTR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPRR, &val);
    EnetUtils_printf("PHY %u: ANNPRR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83822_CR1, &val);
    EnetUtils_printf("PHY %u: CR1     = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGSR, &val);
    EnetUtils_printf("PHY %u: STS1    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGESR, &val);
    EnetUtils_printf("PHY %u: 1KSCR   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83822_PHYCR, &val);
    EnetUtils_printf("PHY %u: PHYCR   = 0x%04x\n", phyAddr, val);
}
