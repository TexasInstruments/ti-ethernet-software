/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  dp83869.c
 *
 * \brief This file contains the implementation of the DP3867 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <include/phy/dp83869.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83869_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83869_OUI                           (0x080028U)
#define DP83869_MODEL                         (0x0FU)
#define DP83869_REV                           (0x01U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83869_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83869_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83869_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83869_setMiiMode(EnetPhy_Handle hPhy,
                               EnetPhy_Mii mii);

static void Dp83869_setVtmIdleThresh(EnetPhy_Handle hPhy,
                                    uint32_t idleThresh);

static void Dp83869_setDspFFE(EnetPhy_Handle hPhy);

static void Dp83869_fixFldStrap(EnetPhy_Handle hPhy);

static void Dp83869_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83869_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83869_setClkShift(EnetPhy_Handle hPhy,
                                bool txShiftEn,
                                bool rxShiftEn);

static int32_t Dp83869_setTxFifoDepth(EnetPhy_Handle hPhy,
                                      uint8_t depth);

static int32_t Dp83869_setClkDelay(EnetPhy_Handle hPhy,
                                   uint32_t txDelay,
                                   uint32_t rxDelay);

static int32_t Dp83869_setOutputImpedance(EnetPhy_Handle hPhy,
                                          uint32_t impedance);

static void Dp83869_setGpioMux(EnetPhy_Handle hPhy,
                               Dp83869_Gpio0Mode gpio0Mode,
                               Dp83869_Gpio1Mode gpio1Mode);

static void Dp83869_setLedMode(EnetPhy_Handle hPhy,
                               const Dp83869_LedMode *ledMode);

static void Dp83869_restart(EnetPhy_Handle hPhy);

static void Dp83869_reset(EnetPhy_Handle hPhy);

static bool Dp83869_isResetComplete(EnetPhy_Handle hPhy);

static void Dp83869_rmwExtReg(EnetPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val);

static void Dp83869_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83869 =
{
    .name               = "dp83869",
    .isPhyDevSupported  = Dp83869_isPhyDevSupported,
    .isMacModeSupported = Dp83869_isMacModeSupported,
    .config             = Dp83869_config,
    .reset              = Dp83869_reset,
    .isResetComplete    = Dp83869_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Dp83869_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83869_initCfg(Dp83869_Cfg *cfg)
{
    cfg->txClkShiftEn         = false;
    cfg->rxClkShiftEn         = false;
    cfg->txDelayInPs          = 2000U;  /* 2.00 ns */
    cfg->rxDelayInPs          = 2000U;  /* 2.00 ns */
    cfg->txFifoDepth          = 4U;     /* 4 bytes/nibbles */
    cfg->impedanceInMilliOhms = 50000U; /* 50 ohms */
    cfg->idleCntThresh        = 5U;
    cfg->gpio0Mode            = DP83869_GPIO0_RXERR;
    cfg->gpio1Mode            = DP83869_GPIO1_COL;
    cfg->ledMode[0]           = DP83869_LED_LINKED;
    cfg->ledMode[1]           = DP83869_LED_LINKED_1000BT;
    cfg->ledMode[2]           = DP83869_LED_RXTXACT;
    cfg->ledMode[3]           = DP83869_LED_LINKED_100BTX;
}

static bool Dp83869_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

    if ((version->oui == DP83869_OUI) &&
        (version->model == DP83869_MODEL) &&
        (version->revision == DP83869_REV))
    {
        supported = true;
    }
    else
    {
        supported = false;
    }

    return supported;
}

static bool Dp83869_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported;

    switch (mii)
    {
        case ENETPHY_MAC_MII_MII:
        case ENETPHY_MAC_MII_GMII:
        case ENETPHY_MAC_MII_RGMII:
            supported = true;
            break;

        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Dp83869_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii)
{

    const Dp83869_Cfg *extendedCfg = (const Dp83869_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    bool enableAutoMdix = true;
    int32_t status = ENETPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                      hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;
    }

    /* Set Viterbi detector idle count threshold and DSP FFE Equalizer */
    if (status == ENETPHY_SOK)
    {
        Dp83869_setVtmIdleThresh(hPhy, extendedCfg->idleCntThresh);
        Dp83869_setDspFFE(hPhy);
    }

    /* Apply workaround for FLD threshold when using bootstrap */
    if (status == ENETPHY_SOK)
    {
        Dp83869_fixFldStrap(hPhy);
    }

    /* Set loopback configuration: enable or disable */
    if (status == ENETPHY_SOK)
    {
        /* To maintain the desired operating mode, Auto-Negotiation should be
         * disabled before enabling the near-end loopback mode (MII loopback is
         * the only mode supported by the driver).  That's taken care by the
         * PHY framework when setting PHY's manual mode parameters */
        Dp83869_setLoopbackCfg(hPhy, cfg->loopbackEn);

        /* Auto-MDIX should be disabled before selecting the Near-End Loopback
         * mode. MDI or MDIX configuration should be manually configured */
        enableAutoMdix = !cfg->loopbackEn;
    }

    /* Software restart is required after Viterbi idle detector and loopback
     * configuration change */
    if (status == ENETPHY_SOK)
    {
        Dp83869_restart(hPhy);
    }

    /* Enable Auto-MDIX and Robust Auto-MDIX */
    if (status == ENETPHY_SOK)
    {
        Dp83869_enableAutoMdix(hPhy, enableAutoMdix);
    }

    /* Set MII mode: MII or RGMII */
    if (status == ENETPHY_SOK)
    {
        Dp83869_setMiiMode(hPhy, mii);
    }

    /* Config RGMII TX and RX clock shift and clock delay */
    if ((status == ENETPHY_SOK) &&
        (mii == ENETPHY_MAC_MII_RGMII))
    {
        Dp83869_setClkShift(hPhy,
                            extendedCfg->txClkShiftEn,
                            extendedCfg->rxClkShiftEn);

        status = Dp83869_setClkDelay(hPhy,
                                     extendedCfg->txDelayInPs,
                                     extendedCfg->rxDelayInPs);
    }

    /* Config TX FIFO depth */
    if (status == ENETPHY_SOK)
    {
        status = Dp83869_setTxFifoDepth(hPhy, extendedCfg->txFifoDepth);
    }

    /* Set output impedance */
    if (status == ENETPHY_SOK)
    {
        status = Dp83869_setOutputImpedance(hPhy, extendedCfg->impedanceInMilliOhms);
    }

    /* Set GPIO mux control (RGZ devices only) */
    if (status == ENETPHY_SOK)
    {
        Dp83869_setGpioMux(hPhy,
                           extendedCfg->gpio0Mode,
                           extendedCfg->gpio1Mode);
    }

    /* Set LED configuration */
    if (status == ENETPHY_SOK)
    {
        Dp83869_setLedMode(hPhy, extendedCfg->ledMode);
    }

    return status;

}

static void Dp83869_setMiiMode(EnetPhy_Handle hPhy,
                               EnetPhy_Mii mii)
{
    uint16_t val = 0U;

    ENETTRACE_DBG("PHY %u: MII mode: %u\n", hPhy->addr, mii);

    if (mii == ENETPHY_MAC_MII_MII)
    {
        val = OP_MODE_DECODE_RGMII_MII_SEL;
    }

    Dp83869_rmwExtReg(hPhy, DP83869_OP_MODE_DECODE, val, OP_MODE_DECODE_RGMII_MII_SEL);
}

static void Dp83869_setVtmIdleThresh(EnetPhy_Handle hPhy,
                                     uint32_t idleThresh)
{
    ENETTRACE_DBG("PHY %u: Viterbi detector idle count thresh: %u\n", hPhy->addr, idleThresh);

    Dp83869_rmwExtReg(hPhy, DP83869_VTMCFG, VTMCFG_IDLETHR_MASK, idleThresh);
}

static void Dp83869_setDspFFE(EnetPhy_Handle hPhy)
{
    ENETTRACE_DBG("PHY %u: DSP FFE Equalizer: %u\n", hPhy->addr, DSPFFECFG_FFEEQ_SHORTCABLE);

    /* As per datasheet, it improves Short Cable performance and will not effect
     * Long Cable performance */
    Dp83869_rmwExtReg(hPhy, DP83869_DSPFFECFG,
                      DSPFFECFG_FFEEQ_MASK,
                      DSPFFECFG_FFEEQ_SHORTCABLE);
}

static void Dp83869_fixFldStrap(EnetPhy_Handle hPhy)
{
    uint16_t val;

    /* As per datasheet, when using strap to enable FLD feature, this bit defaults to 0x2.
     * Register write is needed to change it to 0x1 */
    EnetPhy_readExtReg(hPhy, DP83869_STRAPSTS2, &val);
    if ((val & STRAPSTS2_FLD_MASK) != 0U)
    {
        ENETTRACE_DBG("PHY %u: Apply FLD threshold workaround\n", hPhy->addr);

        Dp83869_rmwExtReg(hPhy, DP83869_FLDTHRCFG, FLDTHRCFG_FLDTHR_MASK, 1U);
    }
}

static void Dp83869_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable)
{
    uint16_t val;

    ENETTRACE_DBG("PHY %u: %s loopback\n", hPhy->addr, enable ? "enable" : "disable");

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
    GenericPhy_writeExtReg(hPhy, DP83869_LOOPCR, val);
}

static void Dp83869_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable)
{
    uint16_t val;

    if (enable)
    {
        val = PHYCR_MDICROSSOVER_AUTO;
    }
    else
    {
        val = PHYCR_MDICROSSOVER_MDI;
    }

    ENETTRACE_DBG("PHY %u: %s automatic cross-over\n",
                  hPhy->addr, enable ? "enable" : "disable");
    EnetPhy_rmwReg(hPhy, DP83869_PHYCR,
                   PHYCR_MDICROSSOVER_MASK,
                   val);

    if (enable)
    {
        ENETTRACE_DBG("PHY %u: enable Robust Auto-MDIX\n", hPhy->addr);
        EnetPhy_rmwReg(hPhy, DP83869_CFG3,
                       CFG3_ROBUSTAUTOMDIX,
                       CFG3_ROBUSTAUTOMDIX);
    }
}

static void Dp83869_setClkShift(EnetPhy_Handle hPhy,
                                bool txShiftEn,
                                bool rxShiftEn)
{
    uint16_t val;

    ENETTRACE_DBG("PHY %u: clock shift TX:%s RX:%s\n",
                  hPhy->addr,
                  txShiftEn ? "enable" : "disable",
                  rxShiftEn ? "enable" : "disable");

    val  = (txShiftEn == true) ? 0U : RGMIICTL_TXCLKDLY;
    val |= (rxShiftEn == true) ? 0U : RGMIICTL_RXCLKDLY;
    Dp83869_rmwExtReg(hPhy, DP83869_RGMIICTL,
                      RGMIICTL_TXCLKDLY | RGMIICTL_RXCLKDLY,
                      val);
}

static int32_t Dp83869_setTxFifoDepth(EnetPhy_Handle hPhy,
                                      uint8_t depth)
{
    uint16_t val = 0U;
    int32_t status = ENETPHY_SOK;

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
            status = ENETPHY_EINVALIDPARAMS;
            break;
    }

    if (status == ENETPHY_SOK)
    {
        ENETTRACE_DBG("PHY %u: set FIFO depth %u\n", hPhy->addr, depth);
        EnetPhy_rmwReg(hPhy, DP83869_PHYCR, PHYCR_TXFIFODEPTH_MASK, val);
    }
    else
    {
        ENETTRACE_ERR("PHY %u: invalid FIFO depth: %u\n", hPhy->addr, depth);
    }

    return status;
}

static int32_t Dp83869_setClkDelay(EnetPhy_Handle hPhy,
                                   uint32_t txDelay,
                                   uint32_t rxDelay)
{
    uint16_t val;
    uint32_t delay;
    uint32_t delayCtrl;
    int32_t status = ENETPHY_SOK;

    if ((txDelay <= RGMIIDCTL_DELAY_MAX) &&
        (rxDelay <= RGMIIDCTL_DELAY_MAX))
    {
        ENETTRACE_DBG("PHY %u: set delay %u ps TX, %u ps RX\n", hPhy->addr, txDelay, rxDelay);

        /* Avoids wrong value of delayCtrl if txDelay is 0 */
        delay     = (txDelay > 0U) ? txDelay : 1U;
        delayCtrl = ENETPHY_DIV_ROUNDUP(delay, RGMIIDCTL_DELAY_STEP) - 1U;
        val       = (uint16_t)((delayCtrl << RGMIIDCTL_TXDLYCTRL_OFFSET) & RGMIIDCTL_TXDLYCTRL_MASK);

        /* Avoids wrong value of delayCtrl if rxDelay is 0 */
        delay     = (rxDelay > 0U) ? rxDelay : 1U;
        delayCtrl = ENETPHY_DIV_ROUNDUP(delay, RGMIIDCTL_DELAY_STEP) - 1U;
        val      |= (uint16_t)((delayCtrl << RGMIIDCTL_RXDLYCTRL_OFFSET) & RGMIIDCTL_RXDLYCTRL_MASK);
        val      |= (RGMIIDCTL_ENFORCE_VAL_MASK | RGMIIDCTL_ENFORCE_CTRL_MASK);

        GenericPhy_writeExtReg(hPhy, DP83869_RGMIIDCTL, val);
    }
    else
    {
        ENETTRACE_ERR("PHY %u: invalid delay (TX=%u RX=%u)\n", hPhy->addr, txDelay, rxDelay);
        status = ENETPHY_EINVALIDPARAMS;
    }

    return status;
}

static int32_t Dp83869_setOutputImpedance(EnetPhy_Handle hPhy,
                                          uint32_t impedance)
{
    int32_t status = ENETPHY_SOK;
    uint32_t val;

    if ((impedance >= IOMUXCFG_IOIMPEDANCE_MIN) &&
        (impedance <= IOMUXCFG_IOIMPEDANCE_MAX))
    {
        ENETTRACE_DBG("PHY %u: set output impedance to %u milli-ohms\n", hPhy->addr, impedance);

        val = (IOMUXCFG_IOIMPEDANCE_MAX - impedance) * IOMUXCFG_IOIMPEDANCE_MASK;
        val = (val + IOMUXCFG_IOIMPEDANCE_RANGE / 2) / IOMUXCFG_IOIMPEDANCE_RANGE;

        Dp83869_rmwExtReg(hPhy, DP83869_IOMUXCFG,
                          IOMUXCFG_IOIMPEDANCE_MASK,
                          val);
    }
    else
    {
        ENETTRACE_ERR("PHY %u: out-of-range impedance: %u\n", hPhy->addr, impedance);
        status = ENETPHY_EINVALIDPARAMS;
    }

    return status;
}

static void Dp83869_setGpioMux(EnetPhy_Handle hPhy,
                               Dp83869_Gpio0Mode gpio0Mode,
                               Dp83869_Gpio1Mode gpio1Mode)
{
    int16_t gpio0;
    int16_t gpio1;

    ENETTRACE_DBG("PHY %u: set gpio0 = mode%u, gpio1 = mode%u\n", hPhy->addr, gpio0Mode, gpio1Mode);

    gpio0  = (uint16_t)gpio0Mode << GPIOMUXCTRL_GPIO0_OFFSET;
    gpio0 &= GPIOMUXCTRL_GPIO0_MASK;

    gpio1  = (uint16_t)gpio1Mode << GPIOMUXCTRL_GPIO1_OFFSET;
    gpio1 &= GPIOMUXCTRL_GPIO1_MASK;

    Dp83869_rmwExtReg(hPhy, DP83869_GPIOMUXCTRL,
                      GPIOMUXCTRL_GPIO1_MASK | GPIOMUXCTRL_GPIO0_MASK,
                      gpio1 | gpio0);
}

static void Dp83869_setLedMode(EnetPhy_Handle hPhy,
                               const Dp83869_LedMode *ledMode)
{
    uint16_t val;

    ENETTRACE_DBG("PHY %u: set LED0 = mode%u, LED1 = mode%u, LED2 = mode%u, LED3 = mode%u\n",
                  hPhy->addr, ledMode[0], ledMode[1], ledMode[2], ledMode[3]);

    val  = ((uint16_t)ledMode[0] << LEDCR1_LED0SEL_OFFSET) & LEDCR1_LED0SEL_MASK;
    val |= ((uint16_t)ledMode[1] << LEDCR1_LED1SEL_OFFSET) & LEDCR1_LED1SEL_MASK;
    val |= ((uint16_t)ledMode[2] << LEDCR1_LED2SEL_OFFSET) & LEDCR1_LED2SEL_MASK;
    val |= ((uint16_t)ledMode[3] << LEDCR1_LED3SEL_OFFSET) & LEDCR1_LED3SEL_MASK;

    EnetPhy_writeReg(hPhy, DP83869_LEDCR1, val);
}

static void Dp83869_restart(EnetPhy_Handle hPhy)
{
    /* Software restart: full reset, not including registers */
    ENETTRACE_DBG("PHY %u: soft-restart\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, DP83869_CTRL, CTRL_SWRESTART, CTRL_SWRESTART);
}

static void Dp83869_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset: all PHY internal circuits including IEEE-defined
     * registers and all extended registers are reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, DP83869_CTRL, CTRL_SWRESET, CTRL_SWRESET);
}

static bool Dp83869_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bit has self-cleared */
    status = EnetPhy_readReg(hPhy, DP83869_CTRL, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & CTRL_SWRESET) == 0U);
    }

    ENETTRACE_DBG("PHY %u: global soft-reset is %scomplete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

static void Dp83869_rmwExtReg(EnetPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t mask,
                              uint16_t val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    uint16_t data;
    int32_t status;

    ENETTRACE_VERBOSE("PHY %u: write reg %u mask 0x%04x val 0x%04x\n",
                      hPhy->addr, reg, mask, val);

    EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    status = EnetPhy_readReg(hPhy, PHY_MMD_DR, &data);

    if (status == ENETPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, data);
    }
}

static void Dp83869_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1     = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2     = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANAR, &val);
    EnetUtils_printf("PHY %u: ANAR        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANLPAR, &val);
    EnetUtils_printf("PHY %u: ANLPAR      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANER, &val);
    EnetUtils_printf("PHY %u: ANER        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPTR, &val);
    EnetUtils_printf("PHY %u: ANNPTR      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPRR, &val);
    EnetUtils_printf("PHY %u: ANNPRR      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGCR, &val);
    EnetUtils_printf("PHY %u: CFG1        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGSR, &val);
    EnetUtils_printf("PHY %u: STS1        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGESR, &val);
    EnetUtils_printf("PHY %u: 1KSCR       = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_PHYCR, &val);
    EnetUtils_printf("PHY %u: PHYCR       = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_PHYSTS, &val);
    EnetUtils_printf("PHY %u: PHYSTS      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_MICR, &val);
    EnetUtils_printf("PHY %u: MICR        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_ISR, &val);
    EnetUtils_printf("PHY %u: ISR         = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_CFG2, &val);
    EnetUtils_printf("PHY %u: CFG2        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_RECR, &val);
    EnetUtils_printf("PHY %u: RECR        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_BISCR, &val);
    EnetUtils_printf("PHY %u: BISCR       = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_STS2, &val);
    EnetUtils_printf("PHY %u: STS2        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_LEDCR1, &val);
    EnetUtils_printf("PHY %u: LEDCR1      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_LEDCR2, &val);
    EnetUtils_printf("PHY %u: LEDCR2      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_LEDCR3, &val);
    EnetUtils_printf("PHY %u: LEDCR3      = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_CFG3, &val);
    EnetUtils_printf("PHY %u: CFG3        = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83869_CTRL, &val);
    EnetUtils_printf("PHY %u: CTRL        = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_RGMIICTL, &val);
    EnetUtils_printf("PHY %u: RGMIICTL    = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_FLDTHRCFG, &val);
    EnetUtils_printf("PHY %u: FLDTHRCFG   = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_VTMCFG, &val);
    EnetUtils_printf("PHY %u: VTMCFG      = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_STRAPSTS2, &val);
    EnetUtils_printf("PHY %u: STRAPSTS2   = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_RGMIIDCTL, &val);
    EnetUtils_printf("PHY %u: RGMIIDCTL   = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_LOOPCR, &val);
    EnetUtils_printf("PHY %u: LOOPCR      = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_DSPFFECFG, &val);
    EnetUtils_printf("PHY %u: DSPFFECFG   = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_IOMUXCFG, &val);
    EnetUtils_printf("PHY %u: IOMUXCFG    = 0x%04x\n", phyAddr, val);
    EnetPhy_readExtReg(hPhy, DP83869_GPIOMUXCTRL, &val);
    EnetUtils_printf("PHY %u: GPIOMUXCTRL = 0x%04x\n", phyAddr, val);
}
