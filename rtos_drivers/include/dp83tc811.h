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
 * \file  dp83tc811.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        DP83TC811 Ethernet PHY.
 */

/*!
 * \ingroup  DRV_ENETPHY
 * \defgroup ENETPHY_DP83TC811 TI DP83TC811 PHY
 *
 * TI DP83TC811 RGMII Ethernet PHY.
 *
 * @{
 */

#ifndef DP83TC811_H_
#define DP83TC811_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Number of LEDs available in the PHY. */
#define DP83TC811_LED_NUM					  (2U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief DP83TC811 Master/Slave modes.
 */
typedef enum Dp83tc811_MasterSlaveMode_e
{
    /*! Use Strap Setting */
    DP83TC811_MASTER_SLAVE_STRAP   = 0x0U,

    /*! Use Master Mode */
    DP83TC811_MASTER_MODE  		   = 0x1U,

    /*! Use Slave Mode */
    DP83TC811_SLAVE_MODE       	   = 0x2U,
} Dp83tc811_MasterSlaveMode;

/*!
 * \brief DP83TC811 PHY configuration parameters.
 */
typedef struct Dp83tc811_Cfg_s
{
	/*! Enable TX clock shift */
	bool txClkShiftEn;
	
	/*! Enable RX clock shift */
	bool rxClkShiftEn;
	
	/*! Enable PHY interrupts */
	bool interruptEn;
	
	/*! Enable SGMII auto negotiation */
	bool sgmiiAutoNegEn;
	
    /*! Master/Slave configuration */
    Dp83tc811_MasterSlaveMode MasterSlaveMode;
} Dp83tc811_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize DP83TC811 PHY specific config params.
 *
 * Initializes the DP83TC811 PHY specific configuration parameters.
 *
 * \param cfg       DP83TC811 PHY config structure pointer
 */
void Dp83tc811_initCfg(Dp83tc811_Cfg *cfg);

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

#endif /* DP83TC811_H_ */

/* @} */
