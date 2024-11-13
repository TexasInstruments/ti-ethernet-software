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
 * \file  phy_common_priv.h
 *
 * \brief This file contains the definitions of common APIs for Ethernet PHY.
 */

#ifndef PHY_COMMON_PRIV_H_
#define PHY_COMMON_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include "../include/phy_common.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


Phy_RegAccessCb_t* PhyPriv_getRegAccessApi(EthPhyDrv_Handle hPhy);

uint8_t PhyPriv_getPhyAddr(EthPhyDrv_Handle hPhy);

#ifdef __cplusplus
}
#endif

#endif // PHY_COMMON_PRIV_H_
