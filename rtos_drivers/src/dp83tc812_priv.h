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
 * \file  dp83tc812_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83TC812 Ethernet PHY.
 */

#ifndef DP83TC812_PRIV_H_
#define DP83TC812_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "phy_common_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define DP83TC812_DEVADDR      		(0x1FU)
#define DP83TC812_DEVADDR_MMD1 		(0x1U)

/*! \brief PHY Status Register (PHYSTS) */
#define DP83TC812_PHYSTS                        (0x10U)
/*! \brief PHY Control Register (PHYCR) */
#define DP83TC812_PHYCR                         (0x11U)
/*! \brief MII Interrupt [] Register 1 (MISR1) */
#define DP83TC812_MISR1                         (0x12U)
/*! \brief MII Interrupt [] Register 2 (MISR2) */
#define DP83TC812_MISR2                         (0x13U)
/*! \brief Receive Error Counter Register (RECR) */
#define DP83TC812_RECR                          (0x15U)
/*! \brief BIST Control Register (BISCR) */
#define DP83TC812_BISCR                         (0x16U)
/*! \brief MII Interrupt [] Register 3 (MISR3) */
#define DP83TC812_MISR3                         (0x18U)
/*! \brief REG_19 (REG_19) */
#define DP83TC812_REG19                         (0x19U)
/*! \brief CDCR (CDCR) */
#define DP83TC812_CDCR                          (0x1EU)
/*! \brief PHYRCR (PHYRCR) */
#define DP83TC812_PHYRCR                        (0x1FU)

#define MII_DP83TC812_INT_STAT1   		DP83TC812_MISR1
#define MII_DP83TC812_INT_STAT2   		DP83TC812_MISR2
#define MII_DP83TC812_INT_STAT3   		DP83TC812_MISR3
#define MII_DP83TC812_RESET_CTRL  		DP83TC812_PHYRCR

#define DP83TC812_HW_RESET        PHY_BIT(15)
#define DP83TC812_SW_RESET        PHY_BIT(14)

#define DP83TC812_STRAP           	  (0x45DU)
#define DP83TC812_RGMII_CTRL      	  (0x600U)
#define DP83TC812_RGMII_ID_CTRL   	  (0x602U)
#define DP83TC812_SGMII_CTRL      	  (0x608U)
#define DP83TC812_MMD1_PMA_CTRL_2	  (0x834U)

/*! \brief INT_STAT1 bits */
#define DP83TC812_ANEG_COMPLETE_INT_EN    PHY_BIT(2)
#define DP83TC812_ESD_EVENT_INT_EN        PHY_BIT(3)
#define DP83TC812_LINK_STAT_INT_EN        PHY_BIT(5)
#define DP83TC812_ENERGY_DET_INT_EN       PHY_BIT(6)
#define DP83TC812_LINK_QUAL_INT_EN        PHY_BIT(7)

/*! \brief INT_STAT2 bits */
#define DP83TC812_SLEEP_MODE_INT_EN       PHY_BIT(2)
#define DP83TC812_OVERTEMP_INT_EN         PHY_BIT(3)
#define DP83TC812_OVERVOLTAGE_INT_EN      PHY_BIT(6)
#define DP83TC812_UNDERVOLTAGE_INT_EN     PHY_BIT(7)

/*! \brief INT_STAT3 bits */
#define DP83TC812_LPS_INT_EN      		PHY_BIT(0)
#define DP83TC812_WAKE_REQ_EN     		PHY_BIT(2)
#define DP83TC812_NO_FRAME_INT_EN 		PHY_BIT(3)
#define DP83TC812_POR_DONE_INT_EN 		PHY_BIT(4)

/*! \brief RGMII CTRL bits */
#define DP83TC812_RGMII_EN                PHY_BIT(3)

/*! \brief SGMII CTRL bits */
#define DP83TC812_SGMII_AUTO_NEG_EN       PHY_BIT(0)
#define DP83TC812_SGMII_EN                PHY_BIT(9)

/*! \brief Strap bits */
#define DP83TC812_MASTER_MODE_EN    PHY_BIT(9)
#define DP83TC812_RGMII_IS_EN     	PHY_BIT(7)
#define DP83TC812_TX_RX_SHIFT 		(PHY_BIT(5) | PHY_BIT(6) | PHY_BIT(7))
#define DP83TC812_TX_SHIFT_EN 		(PHY_BIT(5) | PHY_BIT(7))
#define DP83TC812_TX_RX_SHIFT_EN 	(PHY_BIT(6) | PHY_BIT(7))
#define DP83TC812_RX_SHIFT_EN 		(PHY_BIT(5) | PHY_BIT(6) | PHY_BIT(7))

/*! \brief RGMII ID CTRL */
#define DP83TC812_RX_CLK_SHIFT    PHY_BIT(1)
#define DP83TC812_TX_CLK_SHIFT    PHY_BIT(0)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

struct dp83tc812_init_reg {
		 uint8_t mmd;
         uint32_t reg;
         uint16_t val;
};

/*! \brief Chip specific init scripts */
static const struct dp83tc812_init_reg dp83tc812_cs1_master_init[] = {
		{0x01,0x0834,0xC001},
		{0x1F,0x0523,0x0001},
		{0x1F,0x0800,0xF864},
		{0x1F,0x0803,0x1552},
		{0x1F,0x0804,0x1A66},
		{0x1F,0x0805,0x1F7B},
		{0x1F,0x081F,0x2A88},
		{0x1F,0x0825,0x40E5},
		{0x1F,0x082B,0x7F3F},
		{0x1F,0x0830,0x0543},
		{0x1F,0x0836,0x5008},
		{0x1F,0x083A,0x08E0},
		{0x1F,0x083B,0x0845},
		{0x1F,0x083E,0x0445},
		{0x1F,0x0855,0x9B9A},
		{0x1F,0x085F,0x2010},
		{0x1F,0x0860,0x6040},
		{0x1F,0x086C,0x1333},
		{0x1F,0x086B,0x3E10},
		{0x1F,0x0872,0x88C0},
		{0x1F,0x0873,0x0003},
		{0x1F,0x0879,0x000F},
		{0x1F,0x087B,0x0070},
		{0x1F,0x087C,0x0002},
		{0x1F,0x0897,0x003F},
		{0x1F,0x089E,0x00AA},
		{0x1F,0x0510,0x000F},
};

static const struct dp83tc812_init_reg dp83tc812_cs1_slave_init[] = {
		{0x1F,0x0523,0x0001},
		{0x01,0x0834,0x8001},
		{0x1F,0x0803,0x1B52},
		{0x1F,0x0804,0x2166},
		{0x1F,0x0805,0x277B},
		{0x1F,0x0827,0x3000},
		{0x1F,0x0830,0x0543},
		{0x1F,0x083A,0x0020},
		{0x1F,0x083C,0x0001},
		{0x1F,0x0855,0x9B9A},
		{0x1F,0x085F,0x2010},
		{0x1F,0x0860,0x6040},
		{0x1F,0x086C,0x0333},
		{0x1F,0x0872,0x88C0},
		{0x1F,0x0873,0x0021},
		{0x1F,0x0879,0x000F},
		{0x1F,0x087B,0x0070},
		{0x1F,0x087C,0x0002},
		{0x1F,0x0897,0x003F},
		{0x1F,0x089E,0x00A2},
		{0x1F,0x0510,0x000F},
};

static const struct dp83tc812_init_reg dp83tc812_cs2_master_init[] = {
		{0x1F,0x0523,0x0001},
		{0x01,0x0834,0xC001},
		{0x1F,0x081C,0x0FE2},
		{0x1F,0x0872,0x0300},
		{0x1F,0x0879,0x0F00},
		{0x1F,0x0806,0x2952},
		{0x1F,0x0807,0x3361},
		{0x1F,0x0808,0x3D7B},
		{0x1F,0x083E,0x045F},
		{0x1F,0x0834,0x8000},
		{0x1F,0x0862,0x00E8},
		{0x1F,0x0896,0x32CB},
		{0x1F,0x003E,0x0009},
};

static const struct dp83tc812_init_reg dp83tc812_cs2_slave_init[] = {
		{0x1F,0x0523,0x0001},
		{0x01,0x0834,0x8001},
		{0x1F,0x0873,0x0821},
		{0x1F,0x0896,0x22FF},
		{0x1F,0x089E,0x0000},
};

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* DP83TC812_PRIV_H_ */
