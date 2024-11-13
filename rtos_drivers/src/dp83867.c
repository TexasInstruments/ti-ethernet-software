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
 * \file  dp83867.c
 *
 * \brief This file contains the implementation of the DP3867 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

#include "../include/dp83867.h"
#include "dp83867_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83867_OUI                           (0x080028U)
#define DP83867_MODEL                         (0x23U)
#define DP83867_REV                           (0x01U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Dp83867_setMiiMode(EthPhyDrv_Handle hPhy,
                               Phy_Mii mii);

static void Dp83867_setVtmIdleThresh(EthPhyDrv_Handle hPhy,
                                    uint32_t idleThresh);

static void Dp83867_setDspFFE(EthPhyDrv_Handle hPhy);

static void Dp83867_fixFldStrap(EthPhyDrv_Handle hPhy);

static void Dp83867_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable);

static void Dp83867_enableAutoMdix(EthPhyDrv_Handle hPhy,
                                   bool enable);

static void Dp83867_setClkShift(EthPhyDrv_Handle hPhy,
                                bool txShiftEn,
                                bool rxShiftEn);

static int32_t Dp83867_setTxFifoDepth(EthPhyDrv_Handle hPhy,
                                      uint8_t depth);

static int32_t Dp83867_setClkDelay(EthPhyDrv_Handle hPhy,
                                   uint32_t txDelay,
                                   uint32_t rxDelay);

static int32_t Dp83867_setOutputImpedance(EthPhyDrv_Handle hPhy,
                                          uint32_t impedance);

static void Dp83867_setGpioMux(EthPhyDrv_Handle hPhy,
                               Dp83867_Gpio0Mode gpio0Mode,
                               Dp83867_Gpio1Mode gpio1Mode);

static void Dp83867_setLedMode(EthPhyDrv_Handle hPhy,
                               const Dp83867_LedMode *ledMode);

static void Dp83867_restart(EthPhyDrv_Handle hPhy);

static void Dp83867_rmwExtReg(EthPhyDrv_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Phy_DrvObj_t gEnetPhyDrvDp83867 =
{
    .fxn =
    {
        .name               = "DP83867",
        .bind               = Dp83867_bind,
        .isPhyDevSupported  = Dp83867_isPhyDevSupported,
        .isMacModeSupported = Dp83867_isMacModeSupported,
        .config             = Dp83867_config,
        .reset              = Dp83867_reset,
        .isResetComplete    = Dp83867_isResetComplete,
        .readExtReg         = GenericPhy_readExtReg,
        .writeExtReg        = GenericPhy_writeExtReg,
        .printRegs          = Dp83867_printRegs,
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

void Dp83867_initCfg(Dp83867_Cfg *cfg)
{
    cfg->txClkShiftEn         = false;
    cfg->rxClkShiftEn         = false;
    cfg->txDelayInPs          = 2000U;  /* 2.00 ns */
    cfg->rxDelayInPs          = 2000U;  /* 2.00 ns */
    cfg->txFifoDepth          = 4U;     /* 4 bytes/nibbles */
    cfg->impedanceInMilliOhms = 50000U; /* 50 ohms */
    cfg->idleCntThresh        = 5U;
    cfg->gpio0Mode            = DP83867_GPIO0_RXERR;
    cfg->gpio1Mode            = DP83867_GPIO1_COL;
    cfg->ledMode[0]           = DP83867_LED_LINKED;
    cfg->ledMode[1]           = DP83867_LED_LINKED_1000BT;
    cfg->ledMode[2]           = DP83867_LED_RXTXACT;
    cfg->ledMode[3]           = DP83867_LED_LINKED_100BTX;
}

void Dp83867_bind(EthPhyDrv_Handle* hPhy, uint8_t phyAddr, Phy_RegAccessCb_t* pRegAccessCb)
{
    Phy_Obj_t* pObj = (Phy_Obj_t*) hPhy;
    pObj->phyAddr = phyAddr;
    pObj->regAccessApi = *pRegAccessCb;
}

bool Dp83867_isPhyDevSupported(EthPhyDrv_Handle hPhy,
                                const void *pVersion)
{
	const Phy_Version *version = (Phy_Version *)pVersion;

    bool supported = false;

    if ((version->oui == DP83867_OUI) &&
        (version->model == DP83867_MODEL) &&
        (version->revision == DP83867_REV))
    {
        supported = true;
    }
    else
    {
        supported = false;
    }

    return supported;
}

bool Dp83867_isMacModeSupported(EthPhyDrv_Handle hPhy,
                                Phy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case PHY_MAC_MII_MII:
        case PHY_MAC_MII_GMII:
        case PHY_MAC_MII_RGMII:
            supported = true;
            break;

        default:
            supported = false;
            break;
    }

    return supported;
}

int32_t Dp83867_config(EthPhyDrv_Handle hPhy,
                        const void *pExtCfg,
                        const uint32_t extCfgSize,
                        Phy_Mii mii, 
                        bool loopbackEn)
{
    uint8_t phyAddr = PhyPriv_getPhyAddr(hPhy);

    const Dp83867_Cfg *extendedCfg = (const Dp83867_Cfg *)pExtCfg;
    uint32_t extendedCfgSize = extCfgSize;
    bool enableAutoMdix = true;
    int32_t status = PHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        PHYTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\r\n",
                     phyAddr, extendedCfg, extendedCfgSize);
        status = PHY_EINVALIDPARAMS;
    }

    /* Set Viterbi detector idle count threshold and DSP FFE Equalizer */
    if (status == PHY_SOK)
    {
        Dp83867_setVtmIdleThresh(hPhy, extendedCfg->idleCntThresh);
        Dp83867_setDspFFE(hPhy);
    }

    /* Apply workaround for FLD threshold when using bootstrap */
    if (status == PHY_SOK)
    {
        Dp83867_fixFldStrap(hPhy);
    }

    /* Set loopback configuration: enable or disable */
    if (status == PHY_SOK)
    {
        /* To maintain the desired operating mode, Auto-Negotiation should be
         * disabled before enabling the near-end loopback mode (MII loopback is
         * the only mode supported by the driver).  That's taken care by the
         * PHY framework when setting PHY's manual mode parameters */
        Dp83867_setLoopbackCfg(hPhy, loopbackEn);

        /* Auto-MDIX should be disabled before selecting the Near-End Loopback
         * mode. MDI or MDIX configuration should be manually configured */
        enableAutoMdix = !loopbackEn;
    }

    /* Software restart is required after Viterbi idle detector and loopback
     * configuration change */
    if (status == PHY_SOK)
    {
        Dp83867_restart(hPhy);
    }

    /* Enable Auto-MDIX and Robust Auto-MDIX */
    if (status == PHY_SOK)
    {
        Dp83867_enableAutoMdix(hPhy, enableAutoMdix);
    }

    /* Set MII mode: MII or RGMII */
    if (status == PHY_SOK)
    {
        Dp83867_setMiiMode(hPhy, mii);
    }

    /* Config RGMII TX and RX clock shift and clock delay */
    if ((status == PHY_SOK) &&
        (mii == PHY_MAC_MII_RGMII))
    {
        Dp83867_setClkShift(hPhy,
                            extendedCfg->txClkShiftEn,
                            extendedCfg->rxClkShiftEn);

        status = Dp83867_setClkDelay(hPhy,
                                     extendedCfg->txDelayInPs,
                                     extendedCfg->rxDelayInPs);
    }

    /* Config TX FIFO depth */
    if (status == PHY_SOK)
    {
        status = Dp83867_setTxFifoDepth(hPhy, extendedCfg->txFifoDepth);
    }

    /* Set output impedance */
    if (status == PHY_SOK)
    {
        status = Dp83867_setOutputImpedance(hPhy, extendedCfg->impedanceInMilliOhms);
    }

    /* Set GPIO mux control (RGZ devices only) */
    if (status == PHY_SOK)
    {
        Dp83867_setGpioMux(hPhy,
                           extendedCfg->gpio0Mode,
                           extendedCfg->gpio1Mode);
    }

    /* Set LED configuration */
    if (status == PHY_SOK)
    {
        Dp83867_setLedMode(hPhy, extendedCfg->ledMode);
    }

    return status;
}

static void Dp83867_setMiiMode(EthPhyDrv_Handle hPhy,
                               Phy_Mii mii)
{
    uint16_t val = 0U;

    PHYTRACE_DBG("PHY %u: MII mode: %u\n", PhyPriv_getPhyAddr(hPhy), mii);

    if (mii == PHY_MAC_MII_RGMII)
    {
        val = RGMIICTL_RGMIIEN;
    }

    Dp83867_rmwExtReg(hPhy, DP83867_RGMIICTL, val, RGMIICTL_RGMIIEN);
}

static void Dp83867_setVtmIdleThresh(EthPhyDrv_Handle hPhy,
                                     uint32_t idleThresh)
{
    PHYTRACE_DBG("PHY %u: Viterbi detector idle count thresh: %u\n", PhyPriv_getPhyAddr(hPhy), idleThresh);

    Dp83867_rmwExtReg(hPhy, DP83867_VTMCFG, VTMCFG_IDLETHR_MASK, idleThresh);
}

static void Dp83867_setDspFFE(EthPhyDrv_Handle hPhy)
{
    PHYTRACE_DBG("PHY %u: DSP FFE Equalizer: %u\n", PhyPriv_getPhyAddr(hPhy), DSPFFECFG_FFEEQ_SHORTCABLE);

    /* As per datasheet, it improves Short Cable performance and will not effect
     * Long Cable performance */
    Dp83867_rmwExtReg(hPhy, DP83867_DSPFFECFG,
                      DSPFFECFG_FFEEQ_MASK,
                      DSPFFECFG_FFEEQ_SHORTCABLE);
}

static void Dp83867_fixFldStrap(EthPhyDrv_Handle hPhy)
{
    uint16_t val;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* As per datasheet, when using strap to enable FLD feature, this bit defaults to 0x2.
     * Register write is needed to change it to 0x1 */
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_STRAPSTS2, &val);
    if ((val & STRAPSTS2_FLD_MASK) != 0U)
    {
        PHYTRACE_DBG("PHY %u: Apply FLD threshold workaround\n", PhyPriv_getPhyAddr(hPhy));

        Dp83867_rmwExtReg(hPhy, DP83867_FLDTHRCFG, FLDTHRCFG_FLDTHR_MASK, 1U);
    }
}

static void Dp83867_setLoopbackCfg(EthPhyDrv_Handle hPhy,
                                   bool enable)
{
    uint16_t val;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    PHYTRACE_DBG("PHY %u: %s loopback\n", PhyPriv_getPhyAddr(hPhy), enable ? "enable" : "disable");

    if (enable)
    {
        val = LOOPCR_CFG_LOOPBACK;
    }
    else
    {
        val = LOOPCR_CFG_NORMAL;
    }

    /* Specific predefined loopback configuration values are required for
     * normal mode or loopback mode */
    pRegAccessApi->EnetPhy_writeExtReg(pRegAccessApi->pArgs, DP83867_LOOPCR, val);
}

static void Dp83867_enableAutoMdix(EthPhyDrv_Handle hPhy,
                                   bool enable)
{
    uint16_t val;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    if (enable)
    {
        val = PHYCR_MDICROSSOVER_AUTO;
    }
    else
    {
        val = PHYCR_MDICROSSOVER_MDI;
    }

    PHYTRACE_DBG("PHY %u: %s automatic cross-over\n",
                   PhyPriv_getPhyAddr(hPhy), enable ? "enable" : "disable");
                   pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, DP83867_PHYCR,
                   PHYCR_MDICROSSOVER_MASK,
                   val);

    if (enable)
    {
        PHYTRACE_DBG("PHY %u: enable Robust Auto-MDIX\n",PhyPriv_getPhyAddr(hPhy));
        pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, DP83867_CFG3,
                       CFG3_ROBUSTAUTOMDIX,
                       CFG3_ROBUSTAUTOMDIX);
    }
}

static void Dp83867_setClkShift(EthPhyDrv_Handle hPhy,
                                bool txShiftEn,
                                bool rxShiftEn)
{
    uint16_t val;

    PHYTRACE_DBG("PHY %u: clock shift TX:%s RX:%s\n",
                  PhyPriv_getPhyAddr(hPhy),
                  txShiftEn ? "enable" : "disable",
                  rxShiftEn ? "enable" : "disable");

    val  = (txShiftEn == true) ? RGMIICTL_TXCLKDLY : 0U;
    val |= (rxShiftEn == true) ? RGMIICTL_RXCLKDLY : 0U;
    Dp83867_rmwExtReg(hPhy, DP83867_RGMIICTL,
                      RGMIICTL_TXCLKDLY | RGMIICTL_RXCLKDLY,
                      val);
}

static int32_t Dp83867_setTxFifoDepth(EthPhyDrv_Handle hPhy,
                                      uint8_t depth)
{
    uint16_t val = 0U;
    int32_t status = PHY_SOK;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    switch (depth)
    {
        case 3U:
            val = PHYCR_TXFIFODEPTH_3B;
            break;

        case 4U:
            val = PHYCR_TXFIFODEPTH_4B;
            break;

        case 6U:
            val = PHYCR_TXFIFODEPTH_6B;
            break;

        case 8U:
            val = PHYCR_TXFIFODEPTH_8B;
            break;

        default:
            status = PHY_EINVALIDPARAMS;
            break;
    }

    if (status == PHY_SOK)
    {
        PHYTRACE_DBG("PHY %u: set FIFO depth %u\n", PhyPriv_getPhyAddr(hPhy), depth);
        pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, DP83867_PHYCR, PHYCR_TXFIFODEPTH_MASK, val);
    }
    else
    {
        PHYTRACE_ERR("PHY %u: invalid FIFO depth: %u\n",PhyPriv_getPhyAddr(hPhy), depth);
    }

    return status;
}

static int32_t Dp83867_setClkDelay(EthPhyDrv_Handle hPhy,
                                   uint32_t txDelay,
                                   uint32_t rxDelay)
{
    uint16_t val;
    uint32_t delay;
    uint32_t delayCtrl;
    int32_t status = PHY_SOK;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    if ((txDelay <= RGMIIDCTL_DELAY_MAX) &&
        (rxDelay <= RGMIIDCTL_DELAY_MAX))
    {
        PHYTRACE_DBG("PHY %u: set delay %u ps TX, %u ps RX\n", PhyPriv_getPhyAddr(hPhy), txDelay, rxDelay);

        /* Avoids wrong value of delayCtrl if txDelay is 0 */
        delay     = (txDelay > 0U) ? txDelay : 1U;
        delayCtrl = PHY_DIV_ROUNDUP(delay, RGMIIDCTL_DELAY_STEP) - 1U;
        val       = (uint16_t)((delayCtrl << RGMIIDCTL_TXDLYCTRL_OFFSET) & RGMIIDCTL_TXDLYCTRL_MASK);

        /* Avoids wrong value of delayCtrl if rxDelay is 0 */
        delay     = (rxDelay > 0U) ? rxDelay : 1U;
        delayCtrl = PHY_DIV_ROUNDUP(delay, RGMIIDCTL_DELAY_STEP) - 1U;
        val      |= (uint16_t)((delayCtrl << RGMIIDCTL_RXDLYCTRL_OFFSET) & RGMIIDCTL_RXDLYCTRL_MASK);

        pRegAccessApi->EnetPhy_writeExtReg(pRegAccessApi->pArgs, DP83867_RGMIIDCTL, val);
    }
    else
    {
        PHYTRACE_ERR("PHY %u: invalid delay (TX=%u RX=%u)\n", PhyPriv_getPhyAddr(hPhy), txDelay, rxDelay);
        status = PHY_EINVALIDPARAMS;
    }

    return status;
}

static int32_t Dp83867_setOutputImpedance(EthPhyDrv_Handle hPhy,
                                          uint32_t impedance)
{
    int32_t status = PHY_SOK;
    uint32_t val;

    if ((impedance >= IOMUXCFG_IOIMPEDANCE_MIN) &&
        (impedance <= IOMUXCFG_IOIMPEDANCE_MAX))
    {
        PHYTRACE_DBG("PHY %u: set output impedance to %u milli-ohms\n", PhyPriv_getPhyAddr(hPhy), impedance);

        val = (IOMUXCFG_IOIMPEDANCE_MAX - impedance) * IOMUXCFG_IOIMPEDANCE_MASK;
        val = (val + IOMUXCFG_IOIMPEDANCE_RANGE / 2) / IOMUXCFG_IOIMPEDANCE_RANGE;

        Dp83867_rmwExtReg(hPhy, DP83867_IOMUXCFG,
                          IOMUXCFG_IOIMPEDANCE_MASK,
                          val);
    }
    else
    {
        PHYTRACE_ERR("PHY %u: out-of-range impedance: %u\n", PhyPriv_getPhyAddr(hPhy), impedance);
        status = PHY_EINVALIDPARAMS;
    }

    return status;
}

static void Dp83867_setGpioMux(EthPhyDrv_Handle hPhy,
                               Dp83867_Gpio0Mode gpio0Mode,
                               Dp83867_Gpio1Mode gpio1Mode)
{
    int16_t gpio0;
    int16_t gpio1;

    PHYTRACE_DBG("PHY %u: set gpio0 = mode%u, gpio1 = mode%u\n", PhyPriv_getPhyAddr(hPhy), gpio0Mode, gpio1Mode);

    gpio0  = (uint16_t)gpio0Mode << GPIOMUXCTRL_GPIO0_OFFSET;
    gpio0 &= GPIOMUXCTRL_GPIO0_MASK;

    gpio1  = (uint16_t)gpio1Mode << GPIOMUXCTRL_GPIO1_OFFSET;
    gpio1 &= GPIOMUXCTRL_GPIO1_MASK;

    Dp83867_rmwExtReg(hPhy, DP83867_GPIOMUXCTRL,
                      GPIOMUXCTRL_GPIO1_MASK | GPIOMUXCTRL_GPIO0_MASK,
                      gpio1 | gpio0);
}

static void Dp83867_setLedMode(EthPhyDrv_Handle hPhy,
                               const Dp83867_LedMode *ledMode)
{
    uint16_t val;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    PHYTRACE_DBG("PHY %u: set LED0 = mode%u, LED1 = mode%u, LED2 = mode%u, LED3 = mode%u\n",
                PhyPriv_getPhyAddr(hPhy), ledMode[0], ledMode[1], ledMode[2], ledMode[3]);

    val  = ((uint16_t)ledMode[0] << LEDCR1_LED0SEL_OFFSET) & LEDCR1_LED0SEL_MASK;
    val |= ((uint16_t)ledMode[1] << LEDCR1_LED1SEL_OFFSET) & LEDCR1_LED1SEL_MASK;
    val |= ((uint16_t)ledMode[2] << LEDCR1_LED2SEL_OFFSET) & LEDCR1_LED2SEL_MASK;
    val |= ((uint16_t)ledMode[3] << LEDCR1_LED3SEL_OFFSET) & LEDCR1_LED3SEL_MASK;

    pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, DP83867_LEDCR1, val);
}

static void Dp83867_restart(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    /* Software restart: full reset, not including registers */
    PHYTRACE_DBG("PHY %u: soft-restart\n",PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, DP83867_CTRL, CTRL_SWRESTART, CTRL_SWRESTART);
}

void Dp83867_reset(EthPhyDrv_Handle hPhy)
{
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);
    /* Global software reset: all PHY internal circuits including IEEE-defined
     * registers and all extended registers are reset */
    PHYTRACE_DBG("PHY %u: global soft-reset\n", PhyPriv_getPhyAddr(hPhy));
    pRegAccessApi->EnetPhy_rmwReg(pRegAccessApi->pArgs, DP83867_CTRL, CTRL_SWRESET, CTRL_SWRESET);
}

bool Dp83867_isResetComplete(EthPhyDrv_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    /* Reset is complete when RESET bit has self-cleared */
    status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_CTRL, &val);
    if (status == PHY_SOK)
    {
        complete = ((val & CTRL_SWRESET) == 0U);
    }

    PHYTRACE_DBG("PHY %u: global soft-reset is %scomplete\n", PhyPriv_getPhyAddr(hPhy), complete ? "" : "not");

    return complete;
}

static void Dp83867_rmwExtReg(EthPhyDrv_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    uint16_t data;
    int32_t status;
    Phy_RegAccessCb_t* pRegAccessApi = PhyPriv_getRegAccessApi(hPhy);

    PHYTRACE_VERBOSE("PHY %u: write reg %u mask 0x%04x val 0x%04x\n",
                     PhyPriv_getPhyAddr(hPhy), reg, mask, val);

    pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_ADDR);
    pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_DR, reg);
    pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    status = pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_MMD_DR, &data);

    if (status == PHY_SOK)
    {
        data = (data & ~mask) | (val & mask);
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
        pRegAccessApi->EnetPhy_writeReg(pRegAccessApi->pArgs, PHY_MMD_DR, data);
    }
}

void Dp83867_printRegs(EthPhyDrv_Handle hPhy)
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
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_GIGCR, &val);
    printf("PHY %u: CFG1        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_GIGSR, &val);
    printf("PHY %u: STS1        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, PHY_GIGESR, &val);
    printf("PHY %u: 1KSCR       = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_PHYCR, &val);
    printf("PHY %u: PHYCR       = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_PHYSTS, &val);
    printf("PHY %u: PHYSTS      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_MICR, &val);
    printf("PHY %u: MICR        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_ISR, &val);
    printf("PHY %u: ISR         = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_CFG2, &val);
    printf("PHY %u: CFG2        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_RECR, &val);
    printf("PHY %u: RECR        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_BISCR, &val);
    printf("PHY %u: BISCR       = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_STS2, &val);
    printf("PHY %u: STS2        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_LEDCR1, &val);
    printf("PHY %u: LEDCR1      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_LEDCR2, &val);
    printf("PHY %u: LEDCR2      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_LEDCR3, &val);
    printf("PHY %u: LEDCR3      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_CFG3, &val);
    printf("PHY %u: CFG3        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readReg(pRegAccessApi->pArgs, DP83867_CTRL, &val);
    printf("PHY %u: CTRL        = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_RGMIICTL, &val);
    printf("PHY %u: RGMIICTL    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_FLDTHRCFG, &val);
    printf("PHY %u: FLDTHRCFG   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_VTMCFG, &val);
    printf("PHY %u: VTMCFG      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_STRAPSTS2, &val);
    printf("PHY %u: STRAPSTS2   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_RGMIIDCTL, &val);
    printf("PHY %u: RGMIIDCTL   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_LOOPCR, &val);
    printf("PHY %u: LOOPCR      = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_DSPFFECFG, &val);
    printf("PHY %u: DSPFFECFG   = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_IOMUXCFG, &val);
    printf("PHY %u: IOMUXCFG    = 0x%04x\r\n",phyAddr, val);
    pRegAccessApi->EnetPhy_readExtReg(pRegAccessApi->pArgs, DP83867_GPIOMUXCTRL, &val);
    printf("PHY %u: GPIOMUXCTRL = 0x%04x\r\n",phyAddr, val);
}
