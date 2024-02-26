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
 * \file  dp83tg720_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83TG720 Ethernet PHY.
 */

#ifndef DP83TG720_PRIV_H_
#define DP83TG720_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "enetphy_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define DP83TG720ES1_PHY_ID     (0U)
#define DP83TG720ES2_PHY_ID     (1U)
#define DP83TG720CS1_PHY_ID     (3U)
#define DP83TG720CS2_PHY_ID     (4U)
#define DP83TG721CS1_PHY_ID	    (0U)
#define DP83720_DEVADDR         (0x1FU)
#define DP83720_DEVADDR_MMD1    (0x1U)

#define MII_DP83720_INT_STAT1   (0x12U)
#define MII_DP83720_INT_STAT2   (0x13U)
#define MII_DP83720_INT_STAT3   (0x18U)
#define MII_DP83720_RESET_CTRL  (0x1FU)
 
#define DP83720_HW_RESET        ENETPHY_BIT(15)
#define DP83720_SW_RESET        ENETPHY_BIT(14)
  
#define DP83720_STRAP           	(0x45DU)
#define DP83720_RGMII_CTRL      	(0x600U)
#define DP83720_RGMII_ID_CTRL   	(0x602U)
#define DP83720_SGMII_CTRL      	(0x608U)

/*! \brief INT_STAT1 bits */
#define DP83720_ANEG_COMPLETE_INT_EN    ENETPHY_BIT(2)
#define DP83720_ESD_EVENT_INT_EN        ENETPHY_BIT(3)
#define DP83720_LINK_STAT_INT_EN        ENETPHY_BIT(5)
#define DP83720_ENERGY_DET_INT_EN       ENETPHY_BIT(6)
#define DP83720_LINK_QUAL_INT_EN        ENETPHY_BIT(7)
 
/*! \brief INT_STAT2 bits */
#define DP83720_SLEEP_MODE_INT_EN       ENETPHY_BIT(2)
#define DP83720_OVERTEMP_INT_EN         ENETPHY_BIT(3)
#define DP83720_OVERVOLTAGE_INT_EN      ENETPHY_BIT(6)
#define DP83720_UNDERVOLTAGE_INT_EN     ENETPHY_BIT(7)

/*! \brief INT_STAT3 bits */
#define DP83720_LPS_INT_EN      ENETPHY_BIT(0)
#define DP83720_WAKE_REQ_EN     ENETPHY_BIT(1)
#define DP83720_NO_FRAME_INT_EN ENETPHY_BIT(2)
#define DP83720_POR_DONE_INT_EN ENETPHY_BIT(3)
 
/*! \brief SGMII CTRL bits */
#define DP83720_SGMII_AUTO_NEG_EN       ENETPHY_BIT(0)
#define DP83720_SGMII_EN                ENETPHY_BIT(9)
 
/*! \brief Strap bits */
#define DP83720_MASTER_MODE     ENETPHY_BIT(5)
#define DP83720_RGMII_IS_EN     ENETPHY_BIT(12)
#define DP83720_SGMII_IS_EN     ENETPHY_BIT(13)
#define DP83720_RX_SHIFT_EN     ENETPHY_BIT(14)
#define DP83720_TX_SHIFT_EN     ENETPHY_BIT(15)
 
/*! \brief RGMII ID CTRL */
#define DP83720_RX_CLK_SHIFT    ENETPHY_BIT(1)
#define DP83720_TX_CLK_SHIFT    ENETPHY_BIT(0)

/* CTRL register definitions */
#define CTRL_SWRESET     		ENETPHY_BIT(15)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

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

#endif /* DP83TG720_PRIV_H_ */
