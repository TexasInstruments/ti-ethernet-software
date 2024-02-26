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
 * \file  dp83tc811_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83TC811 Ethernet PHYs.
 */

#ifndef DP83TC811_PRIV_H_
#define DP83TC811_PRIV_H_

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
#define DP83TC811_PHY_ID	(3U)
#define DP83TC811_DEVADDR	(0x1FU)

#define MII_DP83TC811_SGMII_CTRL	(0x09U)
#define MII_DP83TC811_INT_STAT1		(0x12U)
#define MII_DP83TC811_INT_STAT2		(0x13U)
#define MII_DP83TC811_INT_STAT3		(0x18U)
#define MII_DP83TC811_MII_CTRL		(0x17U)
#define MII_DP83TC811_RESET_CTRL	(0x1FU)
#define DP83TC811_STRAP           	(0x467U)

#define DP83TC811_HW_RESET		ENETPHY_BIT(15)
#define DP83TC811_SW_RESET		ENETPHY_BIT(14)

/*! \brief INT_STAT1 bits */
#define DP83TC811_RX_ERR_HF_INT_EN		ENETPHY_BIT(0)
#define DP83TC811_MS_TRAINING_INT_EN	ENETPHY_BIT(1)
#define DP83TC811_ANEG_COMPLETE_INT_EN	ENETPHY_BIT(2)
#define DP83TC811_ESD_EVENT_INT_EN		ENETPHY_BIT(3)
#define DP83TC811_WOL_INT_EN			ENETPHY_BIT(4)
#define DP83TC811_LINK_STAT_INT_EN		ENETPHY_BIT(5)
#define DP83TC811_ENERGY_DET_INT_EN		ENETPHY_BIT(6)
#define DP83TC811_LINK_QUAL_INT_EN		ENETPHY_BIT(7)

/*! \brief INT_STAT2 bits */
#define DP83TC811_JABBER_DET_INT_EN		ENETPHY_BIT(0)
#define DP83TC811_POLARITY_INT_EN		ENETPHY_BIT(1)
#define DP83TC811_SLEEP_MODE_INT_EN		ENETPHY_BIT(2)
#define DP83TC811_OVERTEMP_INT_EN		ENETPHY_BIT(3)
#define DP83TC811_OVERVOLTAGE_INT_EN	ENETPHY_BIT(6)
#define DP83TC811_UNDERVOLTAGE_INT_EN	ENETPHY_BIT(7)

/*! \brief INT_STAT3 bits */
#define DP83TC811_LPS_INT_EN		ENETPHY_BIT(0)
#define DP83TC811_NO_FRAME_INT_EN	ENETPHY_BIT(3)
#define DP83TC811_POR_DONE_INT_EN	ENETPHY_BIT(4)

#define MII_DP83TC811_RXSOP1	(0x04A5)
#define MII_DP83TC811_RXSOP2	(0x04A6)
#define MII_DP83TC811_RXSOP3	(0x04A7)

/*! \brief Registers */
#define MII_DP83TC811_WOL_CFG	(0x04A0U)
#define MII_DP83TC811_WOL_STAT	(0x04A1U)
#define MII_DP83TC811_WOL_DA1	(0x04A2U)
#define MII_DP83TC811_WOL_DA2	(0x04A3U)
#define MII_DP83TC811_WOL_DA3	(0x04A4U)

/*! \brief bits */
#define DP83TC811_WOL_MAGIC_EN			ENETPHY_BIT(0)
#define DP83TC811_WOL_SECURE_ON			ENETPHY_BIT(5)
#define DP83TC811_WOL_EN				ENETPHY_BIT(7)
#define DP83TC811_WOL_INDICATION_SEL 	ENETPHY_BIT(8)
#define DP83TC811_WOL_CLR_INDICATION 	ENETPHY_BIT(11)

/*! \brief SGMII CTRL bits */
#define DP83TC811_TDR_AUTO				ENETPHY_BIT(8)
#define DP83TC811_SGMII_EN				ENETPHY_BIT(12)
#define DP83TC811_SGMII_AUTO_NEG_EN		ENETPHY_BIT(13)
#define DP83TC811_SGMII_TX_ERR_DIS		ENETPHY_BIT(14)
#define DP83TC811_SGMII_SOFT_RESET		ENETPHY_BIT(15)

/*! \brief Strap bits */
#define DP83TC811_MASTER_MODE     ENETPHY_BIT(8)
#define DP83TC811_RGMII_IS_EN     ENETPHY_BIT(3)

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

#endif /* DP83TC811_PRIV_H_ */
