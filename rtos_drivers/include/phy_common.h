/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \ingroup DRV_ENETPHY
 * \defgroup PHY_COMMON_H TI ENET PHY
 *
 * TI PHY COMMON for Ethernet PHY.
 *
 * @{
 */
#ifndef PHY_COMMON_H_
#define PHY_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define ETHPHYDRV_MAX_OBJ_SIZE            (64) /* to meet the size of Phy_Obj_t */

/*! \brief Macro to perform round-up division. */
#define PHY_DIV_ROUNDUP(val, div)         (((val) + (div) - 1) / (div))

/*! \brief Macro to set bit at given bit position. */
#define PHY_BIT(n)                        (1U << (n))

/*! \brief Build-time config option is enabled. */
#define PHY_ON                                     (1U)
/*! \brief Build-time config option is disabled. */
#define PHY_OFF                                    (0U)
/*! \brief Preprocessor check if config option is enabled. */
#define PHY_CFG_IS_ON(name)                        ((PHY_CFG_ ## name) == PHY_ON)
/*! \brief Preprocessor check if config option is disabled. */
#define PHY_CFG_IS_OFF(name)                       ((PHY_CFG_ ## name) == PHY_OFF)

/*!
 * \anchor Phy_ErrorCodes
 * \name   Ethernet PHY driver error codes
 *
 * Error codes returned by the Ethernet PHY driver APIs.
 *
 * @{
 */

/*! \brief Success. */
#define PHY_SOK                           ( (int32_t) (0))
/*! \brief Generic failure error condition (typically caused by hardware). */
#define PHY_EFAIL                         (-(int32_t) (1))
/*! \brief Bad arguments (i.e. NULL pointer). */
#define PHY_EBADARGS                      (-(int32_t) (2))  (-(int32_t) (2))
/*! \brief Invalid parameters (i.e. value out-of-range). */
#define PHY_EINVALIDPARAMS                (-(int32_t) (3))
/*! \brief Time out while waiting for a given condition to happen. */
#define PHY_ETIMEOUT                      (-(int32_t) (4))
/*! \brief Allocation failure. */
#define PHY_EALLOC                        (-(int32_t) (8))
/*! \brief Operation not permitted. */
#define PHY_EPERM                         (PHY_EALLOC - 4)
/*! \brief Operation not supported. */
#define PHY_ENOTSUPPORTED                 (PHY_EALLOC - 5)

/*! @} */

/* PHY Register Definitions */

/*! \brief Basic Mode Control Register (BMCR) */
#define PHY_BMCR                              (0x00U)
/*! \brief Basic Mode Status Register (BMSR) */
#define PHY_BMSR                              (0x01U)
/*! \brief PHY Identifier Register #1 (PHYIDR1) */
#define PHY_PHYIDR1                           (0x02U)
/*! \brief PHY Identifier Register #2 (PHYIDR2) */
#define PHY_PHYIDR2                           (0x03U)
/*! \brief Auto-Negotiation Advertisement Register (ANAR) */
#define PHY_ANAR                              (0x04U)
/*! \brief Auto-Negotiation Link Partner Abilitiy Register (ANLPAR) */
#define PHY_ANLPAR                            (0x05U)
/*! \brief Auto-Negotiation Expansion Register (ANER) */
#define PHY_ANER                              (0x06U)
/*! \brief Auto-Negotiation NP TX Register (ANNPTR) */
#define PHY_ANNPTR                            (0x07U)
/*! \brief Auto-Neg NP RX Register (ANNPRR) */
#define PHY_ANNPRR                            (0x08U)
/*! \brief 1000BASE-T Control Register (GIGCR) */
#define PHY_GIGCR                             (0x09U)
/*! \brief 1000BASE-T Status Register (GIGSR) */
#define PHY_GIGSR                             (0x0AU)
/*! \brief MMD Access Control Register */
#define PHY_MMD_CR                            (0x0DU)
/*! \brief MMD Access Data Register */
#define PHY_MMD_DR                            (0x0EU)
/*! \brief 1000BASE-T Extended Status Register (GIGESR) */
#define PHY_GIGESR                            (0x0FU)

/* MMD_CR register definitions */
#define MMD_CR_ADDR                           (0x0000U)
#define MMD_CR_DATA_NOPOSTINC                 (0x4000U)
#define MMD_CR_DATA_POSTINC_RW                (0x8000U)
#define MMD_CR_DATA_POSTINC_W                 (0xC000U)
#define MMD_CR_DEVADDR                        (0x001FU)

/* BMCR register definitions */
#define PHY_BMCR_RESET                            PHY_BIT(15)
#define PHY_BMCR_LOOPBACK                         PHY_BIT(14)
#define PHY_BMCR_SPEED100                         PHY_BIT(13)
#define PHY_BMCR_ANEN                             PHY_BIT(12)
#define PHY_BMCR_PWRDOWN                          PHY_BIT(11)
#define PHY_BMCR_ISOLATE                          PHY_BIT(10)
#define PHY_BMCR_ANRESTART                        PHY_BIT(9)
#define PHY_BMCR_FD                               PHY_BIT(8)
#define PHY_BMCR_SPEED1000                        PHY_BIT(6)

/*! \brief Max extended configuration size, arbitrarily chosen. */
#define PHY_EXTENDED_CFG_SIZE_MAX         (128U)


/*! \brief 10-Mbps, half-duplex capability mask. */
#define PHY_LINK_CAP_HD10                 PHY_BIT(1)
/*! \brief 10-Mbps, full-duplex capability mask. */
#define PHY_LINK_CAP_FD10                 PHY_BIT(2)
/*! \brief 100-Mbps, half-duplex capability mask. */
#define PHY_LINK_CAP_HD100                PHY_BIT(3)
/*! \brief 100-Mbps, full-duplex capability mask. */
#define PHY_LINK_CAP_FD100                PHY_BIT(4)
/*! \brief 1-Gbps, half-duplex capability mask. */
#define PHY_LINK_CAP_HD1000               PHY_BIT(5)
/*! \brief 1-Gbps, full-duplex capability mask. */
#define PHY_LINK_CAP_FD1000               PHY_BIT(6)
/*! \brief 10-Mbps, full and half-duplex capability mask. */
#define PHY_LINK_CAP_10                   (PHY_LINK_CAP_HD10 | PHY_LINK_CAP_FD10)
/*! \brief 100-Mbps, full and half-duplex capability mask. */
#define PHY_LINK_CAP_100                  (PHY_LINK_CAP_HD100 | PHY_LINK_CAP_FD100)
/*! \brief 1-Gbps, full and half-duplex capability mask. */
#define PHY_LINK_CAP_1000                 (PHY_LINK_CAP_HD1000 | PHY_LINK_CAP_FD1000)
/*! \brief Auto-negotiation mask with all duplexity and speed values set. */
#define PHY_LINK_CAP_ALL                  (PHY_LINK_CAP_HD10 | PHY_LINK_CAP_FD10 |   \
                                           PHY_LINK_CAP_HD100 | PHY_LINK_CAP_FD100 |  \
                                           PHY_LINK_CAP_HD1000 | PHY_LINK_CAP_FD1000)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


typedef struct Phy_Version_s
{
    /*! Organizationally Unique Identifier (OUI) */
    uint32_t oui;
    /*! Manufacturer's model number */
    uint32_t model;
    /*! Revision number */
    uint32_t revision;
} Phy_Version;

typedef enum Phy_Mii_e
{
    /*! \brief MII interface */
    PHY_MAC_MII_MII = 0U,

    /*! \brief RMII interface */
    PHY_MAC_MII_RMII,

    /*! \brief GMII interface */
    PHY_MAC_MII_GMII,

    /*! \brief RGMII interface */
    PHY_MAC_MII_RGMII,

    /*! \brief SGMII interface */
    PHY_MAC_MII_SGMII,

    /*! \brief QSGMII interface */
    PHY_MAC_MII_QSGMII,
} Phy_Mii;

typedef struct
{
    int32_t (*EnetPhy_readReg)(void* pArgs, uint32_t reg, uint16_t *val);

    int32_t (*EnetPhy_writeReg)(void*  pArgs, uint32_t reg, uint16_t val);

    int32_t (*EnetPhy_rmwReg)(void*  pArgs, uint32_t reg, uint16_t mask,
            uint16_t val);

    int32_t (*EnetPhy_readExtReg)(void*  pArgs, uint32_t reg,
            uint16_t *val);

    int32_t (*EnetPhy_writeExtReg)(void*  pArgs, uint32_t reg,
            uint16_t val);

    /*!Argument that needs to be passed to the above callbacks */
    void* pArgs;

} Phy_RegAccessCb_t;

typedef uint8_t EthPhyDrv_Handle[ETHPHYDRV_MAX_OBJ_SIZE];

typedef struct
{

    struct
    {

        /*!
         * \brief Driver name.
         *
         * Name of the PHY-specific driver.
         */
        const char *name;

        /*!
         * \brief Check if driver supports a PHY model identified by its version.
         *
         * PHY-specific function that drivers must implement for upper check if the
         * PHY driver supports a PHY model identified by its version from ID1 and
         * ID2 registers.
         *
         * Note that a given PHY driver can support multiple PHY models.
         *
         * \param hPhy     PHY device handle
         * \param version  PHY version from ID registers
         *
         * \return Whether PHY model is supported or not
         */

        bool (*isPhyDevSupported)(EthPhyDrv_Handle hPhy,
                                    const void *pVersion);

        /*!
         * \brief Check if driver supports a MII interface type.
         *
         * PHY-specific function that drivers must implement for upper layer to check
         * whether a MAC mode is supported by the PHY driver or not.
         *
         * \param hPhy     PHY device handle
         * \param mii      MII interface
         *
         * \return Whether MII interface type is supported or not
         */
        bool (*isMacModeSupported)(EthPhyDrv_Handle hPhy, 
                                    Phy_Mii mii);

        /*!
         * \brief PHY bind.
         *
         * PHY-specific function that binds the driver handle adnd register 
         * access functions to specific PHY device.
         *
         * \param hPhy              PHY device handle
         * \param phyAddr           PHY address
         * \param pRegAccessCb      PHY register access function pointers
         */

        void (*bind)(EthPhyDrv_Handle* hPhy,
                        uint8_t phyAddr,
                        Phy_RegAccessCb_t* pRegAccessCb);

        /*!
         * \brief PHY specific configuration.
         *
         * PHY-specific function that drivers must implement to configure the PHY
         * device.  The configuration can be composed of generic and PHY-specific
         * parameter (via extended config).
         *
         * \param hPhy     PHY device handle
         * \param cfg      PHY configuration parameter
         * \param mii      MII interface
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*config)(EthPhyDrv_Handle hPhy,
                            const void *pExtCfg,
                            const uint32_t extCfgSize,
                            Phy_Mii mii, 
                            bool loopbackEn);

        /*!
         * \brief PHY specific soft reset.
         *
         * PHY-specific function that drivers must implement to start a soft-reset
         * operation.
         *
         * \param hPhy     PHY device handle
         */
        void (*reset)(EthPhyDrv_Handle hPhy);

        /*!
         * \brief PHY specific soft reset status.
         *
         * PHY-specific function that drivers must implement to check if soft-reset
         * operation is complete.
         *
         * \param hPhy     PHY device handle
         *
         * \return Whether soft-reset is complete or not.
         */
        bool (*isResetComplete)(EthPhyDrv_Handle hPhy);

        /*!
         * \brief Read PHY extended register.
         *
         * PHY-specific function that drivers must implement to read extended
         * registers.
         *
         * \param hPhy     PHY device handle
         * \param reg      Register number
         * \param val      Pointer to the read value
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*readExtReg)(EthPhyDrv_Handle hPhy,
                                uint32_t reg,
                                uint16_t* val);

        /*!
         * \brief Write PHY register.
         *
         * PHY-specific function that drivers must implement to write extended
         * registers.
         *
         * \param hPhy     PHY device handle
         * \param reg      Register number
         * \param val      Value to be written
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*writeExtReg)(EthPhyDrv_Handle hPhy,
                                uint32_t reg,
                                uint16_t val);

        /*!
         * \brief Read-modify-write PHY extended register.
         *
         * PHY-specific function that drivers must implement to read-write-modify
         * extended registers.
         *
         * \param group      User group (use 0 if single group is supported)
         * \param phyAddr    PHY device address
         * \param reg        Register address
         * \param val        Value read from register
         */
        int32_t (*rmwExtReg)(EthPhyDrv_Handle hPhy,
                            uint32_t reg,
                            uint16_t mask,
                            uint16_t* val);

        /*! Print PHY registers */
        void (*printRegs)(EthPhyDrv_Handle hPhy);

        /*!
         * \brief Adjust PHY PTP clock frequency.
         *
         * Optional PHY function to adjust PTP clock frequency.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param ppb      Part per billion
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*adjPtpFreq)(EthPhyDrv_Handle hPhy,
                              int64_t ppb);

        /*!
         * \brief Adjust PHY PTP clock phase.
         *
         * Optional PHY function to adjust PTP clock phase.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param offset   Offset to current clock time in nanosec unit.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*adjPtpPhase)(EthPhyDrv_Handle hPhy,
                              int64_t offset);

        /*!
         * \brief Get current PHY PTP clock time.
         *
         * Optional PHY function to get current PHY PTP clock time.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param ts64     Output current PTP clock time in nanosec unit.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*getPtpTime)(EthPhyDrv_Handle hPhy,
                              uint64_t *ts64);

        /*!
         * \brief Set PHY PTP clock time.
         *
         * Optional PHY function to set PHY PTP clock time.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param ts64     PTP time in nanosec unit will be set.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*setPtpTime)(EthPhyDrv_Handle hPhy,
                              uint64_t ts64);

        /*!
         * \brief Get PHY PTP TX packet timestamp.
         *
         * Optional PHY function to get PHY PTP TX packet timestamp.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param domain   PTP domain (in the packet header)
         * \param msgType  PTP message type (in the packet header)
         * \param seqId    PTP packet sequence ID (in the packet header)
         * \param ts64     Output PTP TX packet timestamp in nanosec unit.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*getPtpTxTime)(EthPhyDrv_Handle hPhy,
                                uint32_t domain,
                                uint32_t msgType,
                                uint32_t seqId,
                                uint64_t *ts64);

        /*!
         * \brief Get PHY PTP RX packet timestamp.
         *
         * Optional PHY function to get PHY PTP RX packet timestamp.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param domain   PTP domain (in the packet header)
         * \param msgType  PTP message type (in the packet header)
         * \param seqId    PTP packet sequence ID (in the packet header)
         * \param ts64     Output PTP RX packet timestamp in nanosec unit.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*getPtpRxTime)(EthPhyDrv_Handle hPhy,
                                uint32_t domain,
                                uint32_t msgType,
                                uint32_t seqId,
                                uint64_t *ts64);

        /*!
         * \brief Add PHY PTP TX packet info to a waiting TX timestamp list.
         *
         * Optional PHY function to get PHY PTP TX packet timestamp.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param domain   PTP domain (in the packet header)
         * \param msgType  PTP message type (in the packet header)
         * \param seqId    PTP packet sequence ID (in the packet header)
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*waitPtpTxTime)(EthPhyDrv_Handle hPhy,
                                 uint32_t domain,
                                 uint32_t msgType,
                                 uint32_t seqId);

        /*!
         * \brief Process PHY status frame.
         *
         * Optional PHY function to process PHY status frame.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param frame    Ethernet PHY status frame
         * \param size     Frame size
         * \param types    Types of processed frame
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*procStatusFrame)(EthPhyDrv_Handle hPhy,
                                   uint8_t *frame,
                                   uint32_t size,
                                   uint32_t *types);

        /*!
         * \brief Get PHY status frame header.
         *
         * Optional PHY function to get the Ethernet header of the PHY status frame.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param ethhdr   Buffer to get the ethernet header of the PHY status frame.
         * \param size     Buffer size (at least 14 bytes)
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*getStatusFrameEthHeader)(EthPhyDrv_Handle hPhy,
                                           uint8_t *ethhdr,
                                           uint32_t size);

        /*!
         * \brief Enable or disable the PHY PTP module
         *
         * Optional PHY function to enable or disable the PHY PTP module.
         * This function can only be supported when the PHY has a built-in PTP clock.
         *
         * \param hPhy     PHY device handle
         * \param on       Flag indicate enable (on=true) or disable(on=false) PTP module
         * \param srcMacStatusFrameType      The PHY-specific src MAC of the status frame.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*enablePtp)(EthPhyDrv_Handle hPhy,
                             bool on,
                             uint32_t srcMacStatusFrameType);

        /*!
         * \brief Provide timer tick to the driver.
         *
         * Provide timer tick to the driver.
         *
         * \param hPhy     PHY device handle
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*tickDriver)(EthPhyDrv_Handle hPhy);

        /*!
         * \brief Enable/Disable an event capture on a PHY GPIO pin.
         *
         * Optional PHY function to enable/disable an event capture on a PHY GPIO pin.
         * This function can only be supported when the PHY has a built-in PTP clock
         * that support event capture.
         *
         * \param hPhy     PHY device handle
         * \param eventIdx Event index
         * \param falling  Capture event on falling edge or rising edge if falling is false.
         * \param on       Enable when on is true, otherwise disable the event.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*enableEventCapture)(EthPhyDrv_Handle hPhy, uint32_t eventIdx,
                    bool falling, bool on);

        /*!
         * \brief Enable/Disable trigger output on a GPIO pin.
         *
         * Optional PHY function to enable/disable trigger output on a GPIO pin.
         * This function can only be supported when the PHY has a built-in PTP clock
         * that support trigger output.
         *
         * \param hPhy       PHY device handle
         * \param triggerIdx Trigger index
         * \param start      Start trigger time in nanosec unit.
         * \param period     Period of the pulse in nanosec unit.
         *                   Disable the trigger if the period is equal to 0.
         * \param repeat     Repeated pulse or one shot pulse if repeat is false.
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*enableTriggerOutput)(EthPhyDrv_Handle hPhy, uint32_t triggerIdx,
                    uint64_t start, uint64_t period, bool repeat);

        /*!
         * \brief Get event timestamp
         *
         * Get event timestamp
         *
         * \param hPhy       PHY device handle
         * \param eventIdx   Output event index
         * \param seqId      Output event sequence identifier
         * \param ts64       Output event timestamp
         *
         * \return \ref EnetPhy_ErrorCodes
         */
        int32_t (*getEventTs)(EthPhyDrv_Handle hPhy, uint32_t *eventIdx,
                    uint32_t *seqId, uint64_t *ts64);
    } fxn;

    EthPhyDrv_Handle hDrv;
} Phy_DrvObj_t;

typedef Phy_DrvObj_t* EthPhyDrv_If;


/* TODO: Move this to private files */
typedef struct
{
    uint8_t phyAddr;
    Phy_RegAccessCb_t regAccessApi;
} Phy_Obj_t;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t GenericPhy_readExtReg(EthPhyDrv_Handle hPhy,
                                uint32_t reg,
                                uint16_t* val);

int32_t GenericPhy_writeExtReg(EthPhyDrv_Handle hPhy,
                                uint32_t reg,
                                uint16_t val);  

void GenericPhy_reset(EthPhyDrv_Handle hPhy);

bool GenericPhy_isResetComplete(EthPhyDrv_Handle hPhy);

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

#endif /* PHY_COMMON_H_ */

/*! @} */
