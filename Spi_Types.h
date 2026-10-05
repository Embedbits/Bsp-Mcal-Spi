/**
 * \defgroup Spi Spi
 * \brief Spi module
 */

/**
 * \author Mr.Nobody
 * \file Spi_Types.h
 * \ingroup Spi
 * \brief Serial Peripheral Interface (SPI) MCAL module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 * \note  Types are identical with the other families of the module. Features of the type not
 *        available on STM32F4 (data size other than 8 / 16 bits, NSS polarity / pulse, master
 *        idle timing, CRC initialization pattern) are refused by the functions of the module.
 *
 */

#ifndef SPI_SPI_TYPES_H
#define SPI_SPI_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Dma_Types.h"                      /* DMA types definitions          */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define SPI_NULL_PTR                        ( ( void* ) 0u )

/** Maximum count of idle clock cycles (STM32F4 has no master idle timing - only 0 is allowed) */
#define SPI_IDLE_CYCLES_MAX                 ( 0u )

/** Peripheral identification bit offset in encoded pin value */
#define SPI_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Port identification bit offset in encoded pin value */
#define SPI_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in encoded pin value */
#define SPI_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternate function identification bit offset in encoded pin value */
#define SPI_BIT_MASK_AF_BIT_OFFSET          ( 0u )

/** Mask of one field (5 bits) in encoded pin value */
#define SPI_BIT_MASK_FIELD                  ( 0x1Fu )

/* ========================== EXPORTED MACROS =============================== */

/**
 * \brief Encodes pin configuration (peripheral, port, pin, alternate function) into single value
 *        of \ref spi_PinCode_t
 *
 * Alternate function of the pin has to be taken from the device datasheet (alternate function
 * mapping table), e.g. SPI1 SCK on PA5 with AF5:
 * SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_ALT_FUNC_5 )
 */
#define SPI_PIN_ENCODE( PERIPH_ID, PORT_ID, PIN_ID, AF_ID )     ( (spi_PinCode_t)( ( (uint32_t)(PERIPH_ID) << SPI_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(PORT_ID)   << SPI_BIT_MASK_PORT_BIT_OFFSET   ) | \
                                                                                   ( (uint32_t)(PIN_ID)    << SPI_BIT_MASK_PIN_BIT_OFFSET    ) | \
                                                                                   ( (uint32_t)(AF_ID)     << SPI_BIT_MASK_AF_BIT_OFFSET     )   ) )

/** Pin is not configured by the module */
#define SPI_PIN_UNUSED                      SPI_PIN_ENCODE( SPI_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )

/** Extract peripheral ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PERIPH( CODED_VAL )     ( ( (CODED_VAL) >> SPI_BIT_MASK_PERIPH_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract port ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PORT( CODED_VAL )       ( ( (CODED_VAL) >> SPI_BIT_MASK_PORT_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract pin ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PIN( CODED_VAL )        ( ( (CODED_VAL) >> SPI_BIT_MASK_PIN_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract alternate function ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_AF( CODED_VAL )         ( ( (CODED_VAL) >> SPI_BIT_MASK_AF_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t spi_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t spi_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t spi_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    spi_MajorVersion_t Major; /**< Major version */
    spi_MinorVersion_t Minor; /**< Minor version */
    spi_PatchVersion_t Patch; /**< Patch version */
}   spi_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    SPI_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    SPI_FUNCTION_ACTIVE         /**< Function status is active   */
}   spi_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    SPI_REQUEST_ERROR = 0u, /**< Processing request failed  */
    SPI_REQUEST_OK          /**< Processing request succeed */
}   spi_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    SPI_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    SPI_FLAG_ACTIVE         /**< Active flag state   */
}   spi_FlagState_t;


/** \brief Frequency value represented in Hz */
typedef uint32_t spi_FreqHz_t;

/** \brief Type representing one byte of data buffer (frame occupies 1 or 2 bytes) */
typedef uint8_t spi_Data_t;

/** \brief Type representing count of transferred data frames */
typedef uint16_t spi_DataCnt_t;

/** \brief Interrupt priority type definition */
typedef uint32_t spi_IrqPrio_t;

/** \brief Count of idle SPI clock cycles (0 - \ref SPI_IDLE_CYCLES_MAX) */
typedef uint8_t spi_IdleCycles_t;

/** \brief CRC polynomial (without the highest bit, e.g. 0x07 for CRC-8 x^8 + x^2 + x + 1), has to be odd */
typedef uint32_t spi_CrcPoly_t;

/** \brief CRC value (TXCRCR / RXCRCR register) */
typedef uint32_t spi_CrcValue_t;

/** \brief Encoded pin (\ref SPI_PIN_ENCODE / \ref SPI_PIN_UNUSED) */
typedef uint32_t spi_PinCode_t;


/** \brief SPI peripheral identification */
typedef enum
{
#ifdef SPI1
    SPI_PERIPH_1 = 0u, /**< SPI peripheral 1 ID */
#endif
#ifdef SPI2
    SPI_PERIPH_2,      /**< SPI peripheral 2 ID */
#endif
#ifdef SPI3
    SPI_PERIPH_3,      /**< SPI peripheral 3 ID */
#endif
#ifdef SPI4
    SPI_PERIPH_4,      /**< SPI peripheral 4 ID */
#endif
#ifdef SPI5
    SPI_PERIPH_5,      /**< SPI peripheral 5 ID */
#endif
#ifdef SPI6
    SPI_PERIPH_6,      /**< SPI peripheral 6 ID */
#endif
    SPI_PERIPH_CNT     /**< Count of SPI peripherals */
}   spi_PeriphId_t;


/**
 * \brief SPI kernel clock source
 *
 * STM32F4 SPI is clocked by the APB clock of the peripheral (SPI2, SPI3: PCLK1, SPI1, SPI4 -
 * SPI6: PCLK2), no kernel clock multiplexer is available.
 */
typedef enum
{
    SPI_CLK_SRC_PCLK = 0u, /**< APB clock of the peripheral */
    SPI_CLK_SRC_CNT        /**< Count of clock sources      */
}   spi_ClkSrc_t;


/** \brief SPI role on the bus */
typedef enum
{
    SPI_MODE_MASTER = 0u, /**< Master - generates SCK (and NSS in hardware NSS mode) */
    SPI_MODE_SLAVE,       /**< Slave - SCK (and NSS) driven by external master        */
    SPI_MODE_CNT          /**< Count of options                                       */
}   spi_Mode_t;


/** \brief SPI clock mode (clock polarity CPOL and phase CPHA, Motorola frame format) */
typedef enum
{
    SPI_CLOCK_MODE_0 = 0u, /**< CPOL = 0, CPHA = 0 (SCK idle low, data sampled on rising edge)   */
    SPI_CLOCK_MODE_1,      /**< CPOL = 0, CPHA = 1 (SCK idle low, data sampled on falling edge)  */
    SPI_CLOCK_MODE_2,      /**< CPOL = 1, CPHA = 0 (SCK idle high, data sampled on falling edge) */
    SPI_CLOCK_MODE_3,      /**< CPOL = 1, CPHA = 1 (SCK idle high, data sampled on rising edge)  */
    SPI_CLOCK_MODE_CNT     /**< Count of options                                                 */
}   spi_ClockMode_t;


/**
 * \brief Size of data frame (also used as CRC length)
 *
 * STM32F4 supports 8-bit and 16-bit frames only (other sizes are refused). Frame is stored in
 * the data buffer in 1 byte (8 bits) or 2 bytes (16 bits), little endian.
 */
typedef enum
{
    SPI_DATA_SIZE_4BIT = 0u, /**< 4-bit frame  */
    SPI_DATA_SIZE_5BIT,      /**< 5-bit frame  */
    SPI_DATA_SIZE_6BIT,      /**< 6-bit frame  */
    SPI_DATA_SIZE_7BIT,      /**< 7-bit frame  */
    SPI_DATA_SIZE_8BIT,      /**< 8-bit frame  */
    SPI_DATA_SIZE_9BIT,      /**< 9-bit frame  */
    SPI_DATA_SIZE_10BIT,     /**< 10-bit frame */
    SPI_DATA_SIZE_11BIT,     /**< 11-bit frame */
    SPI_DATA_SIZE_12BIT,     /**< 12-bit frame */
    SPI_DATA_SIZE_13BIT,     /**< 13-bit frame */
    SPI_DATA_SIZE_14BIT,     /**< 14-bit frame */
    SPI_DATA_SIZE_15BIT,     /**< 15-bit frame */
    SPI_DATA_SIZE_16BIT,     /**< 16-bit frame */
    SPI_DATA_SIZE_17BIT,     /**< 17-bit frame */
    SPI_DATA_SIZE_18BIT,     /**< 18-bit frame */
    SPI_DATA_SIZE_19BIT,     /**< 19-bit frame */
    SPI_DATA_SIZE_20BIT,     /**< 20-bit frame */
    SPI_DATA_SIZE_21BIT,     /**< 21-bit frame */
    SPI_DATA_SIZE_22BIT,     /**< 22-bit frame */
    SPI_DATA_SIZE_23BIT,     /**< 23-bit frame */
    SPI_DATA_SIZE_24BIT,     /**< 24-bit frame */
    SPI_DATA_SIZE_25BIT,     /**< 25-bit frame */
    SPI_DATA_SIZE_26BIT,     /**< 26-bit frame */
    SPI_DATA_SIZE_27BIT,     /**< 27-bit frame */
    SPI_DATA_SIZE_28BIT,     /**< 28-bit frame */
    SPI_DATA_SIZE_29BIT,     /**< 29-bit frame */
    SPI_DATA_SIZE_30BIT,     /**< 30-bit frame */
    SPI_DATA_SIZE_31BIT,     /**< 31-bit frame */
    SPI_DATA_SIZE_32BIT,     /**< 32-bit frame */
    SPI_DATA_SIZE_CNT        /**< Count of options */
}   spi_DataSize_t;


/** \brief Bit order of data frame */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0u, /**< Most significant bit is transferred first  */
    SPI_BIT_ORDER_LSB_FIRST,      /**< Least significant bit is transferred first */
    SPI_BIT_ORDER_CNT             /**< Count of options                           */
}   spi_BitOrder_t;


/**
 * \brief Communication direction
 *
 * Half-duplex uses one bidirectional data line (MOSI in master, MISO in slave), direction of
 * every transfer is given by the request (TxData or RxData).
 */
typedef enum
{
    SPI_DIRECTION_FULL_DUPLEX = 0u, /**< Transmit and receive at the same time (MOSI + MISO) */
    SPI_DIRECTION_SIMPLEX_TX,       /**< Transmit only                                       */
    SPI_DIRECTION_SIMPLEX_RX,       /**< Receive only                                        */
    SPI_DIRECTION_HALF_DUPLEX,      /**< Transmit or receive on one data line                */
    SPI_DIRECTION_CNT               /**< Count of options                                    */
}   spi_Direction_t;


/** \brief Frame format */
typedef enum
{
    SPI_FRAME_FORMAT_MOTOROLA = 0u, /**< Motorola frame format (clock mode, bit order and NSS are configurable) */
    SPI_FRAME_FORMAT_TI,            /**< TI frame format (clock mode, bit order and NSS are given by protocol) */
    SPI_FRAME_FORMAT_CNT            /**< Count of options                                                      */
}   spi_FrameFormat_t;


/** \brief NSS (slave select) management */
typedef enum
{
    SPI_NSS_MODE_SOFT = 0u, /**< NSS pin is not used by the peripheral (master: slave select driven by
                                 application GPIO, slave: peripheral is always selected)                */
    SPI_NSS_MODE_HARD,      /**< Master: NSS output driven by HW during transfer, slave: NSS input      */
    SPI_NSS_MODE_CNT        /**< Count of options                                                       */
}   spi_NssMode_t;


/** \brief Active level of NSS signal (STM32F4: active low only) */
typedef enum
{
    SPI_NSS_POLARITY_LOW = 0u, /**< NSS is active low  */
    SPI_NSS_POLARITY_HIGH,     /**< NSS is active high */
    SPI_NSS_POLARITY_CNT       /**< Count of options   */
}   spi_NssPolarity_t;


/** \brief NSS configuration */
typedef struct
{
    spi_NssMode_t       Mode;     /**< NSS management                                                     */
    spi_NssPolarity_t   Polarity; /**< Active level of NSS (STM32F4: SPI_NSS_POLARITY_LOW only)           */
    spi_FunctionState_t Pulse;    /**< NSS pulse between data frames (STM32F4: SPI_FUNCTION_INACTIVE only) */
}   spi_NssConfig_t;


/** \brief CRC initialization pattern of transmitter and receiver (STM32F4: all zero only) */
typedef enum
{
    SPI_CRC_INIT_ALL_ZERO = 0u, /**< CRC is initialized with all bits zero */
    SPI_CRC_INIT_ALL_ONES,      /**< CRC is initialized with all bits one  */
    SPI_CRC_INIT_CNT            /**< Count of options                      */
}   spi_CrcInit_t;


/**
 * \brief Hardware CRC configuration
 *
 * CRC frame is transmitted after the last data frame and the received CRC is checked by HW
 * (\ref SPI_XFER_ERROR_CRC). On STM32F4 the CRC length is given by the data size (Size must be
 * equal to the configured data size), the polynomial must be odd (device errata: the CRC is wrong
 * with an even polynomial) and fit into Size bits and the CRC is always initialized with zeros.
 */
typedef struct
{
    spi_FunctionState_t State;      /**< CRC calculation state            */
    spi_DataSize_t      Size;       /**< CRC length                       */
    spi_CrcPoly_t       Polynomial; /**< CRC polynomial                   */
    spi_CrcInit_t       InitValue;  /**< CRC initialization pattern       */
}   spi_CrcConfig_t;


/**
 * \brief Output speed of SPI pins (has to correspond to the bus frequency)
 *
 * Device errata "Corrupted last bit of data and/or CRC received in Master mode with delayed SCK
 * feedback": SCK pin of master limits the SPI kernel (APB) clock - low speed up to 25 MHz, medium
 * up to 75 MHz (master configuration is refused above). The errata sheet allows 84 MHz for high /
 * very high speed at 30 pF SCK load - reduce the APB clock or the SCK load above it.
 */
typedef enum
{
    SPI_PIN_SPEED_LOW = 0u,  /**< Low output speed       */
    SPI_PIN_SPEED_MEDIUM,    /**< Medium output speed    */
    SPI_PIN_SPEED_HIGH,      /**< High output speed      */
    SPI_PIN_SPEED_VERY_HIGH, /**< Very high output speed */
    SPI_PIN_SPEED_CNT        /**< Count of options       */
}   spi_PinSpeed_t;


/** DMA peripherals enumeration list */
typedef enum
{
    SPI_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    SPI_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    SPI_DMA_PERIPH_CNT
}   spi_DmaPeriphId_t;


/**
 * \brief Enumeration of available channels (DMA streams) for all DMA peripherals
 *
 * STM32F4 DMA has no request multiplexer - the stream has to be one of the streams connected to
 * the SPI request (request mapping table of the reference manual, e.g. SPI1_TX: DMA2 stream 3 or
 * 5), the channel selection of the stream is set by the module. Other streams are refused.
 */
typedef enum
{
    SPI_DMA_CHANNEL_0  = DMA_STREAM_0,  /**< DMA stream 0                    */
    SPI_DMA_CHANNEL_1  = DMA_STREAM_1,  /**< DMA stream 1                    */
    SPI_DMA_CHANNEL_2  = DMA_STREAM_2,  /**< DMA stream 2                    */
    SPI_DMA_CHANNEL_3  = DMA_STREAM_3,  /**< DMA stream 3                    */
    SPI_DMA_CHANNEL_4  = DMA_STREAM_4,  /**< DMA stream 4                    */
    SPI_DMA_CHANNEL_5  = DMA_STREAM_5,  /**< DMA stream 5                    */
    SPI_DMA_CHANNEL_6  = DMA_STREAM_6,  /**< DMA stream 6                    */
    SPI_DMA_CHANNEL_7  = DMA_STREAM_7,  /**< DMA stream 7                    */
    SPI_DMA_CHANNEL_CNT                 /**< Count of available DMA streams  */
}   spi_DmaChannelId_t;


/** Channel priority options enumeration */
typedef enum
{
    SPI_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    SPI_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    SPI_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    SPI_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very_High */
}   spi_DmaPriority_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief List of data transfer modes
 *
 * All modes use the same transfer request and report the same events through the same
 * callbacks - they differ only in the context moving the data:
 * - DMA:  DMA streams (end of transfer from DMA interrupt, errors from SPI interrupt)
 * - ISR:  SPI interrupt service routine
 * - POLL: Spi_Task() polling SPI flags (callbacks from Spi_Task() context)
 */
typedef enum
{
    SPI_XFER_MODE_NONE = 0u, /**< Data transfers are not used                                 */
    SPI_XFER_MODE_DMA,       /**< Data are transferred by DMA                                 */
    SPI_XFER_MODE_ISR,       /**< Data are transferred by SPI interrupt service routine       */
    SPI_XFER_MODE_POLL,      /**< Data are transferred by Spi_Task() (polling of SPI flags)   */
    SPI_XFER_MODE_CNT        /**< Count of data transfer modes                                */
}   spi_XferMode_t;


/**
 * \brief List of data transfer errors reported through \ref spi_XferErrCallback_t
 *
 * \note  STM32F4 does not report SPI_XFER_ERROR_UNDERRUN (no underrun detection in SPI mode) and
 *        DMA errors other than SPI_XFER_ERROR_DMA_TRANSFER.
 */
typedef enum
{
    SPI_XFER_ERROR_NONE = 0u,           /**< No error (last transfer finished successfully)                */
    SPI_XFER_ERROR_OVERRUN,             /**< Received frame was lost - receive buffer was full (OVR)       */
    SPI_XFER_ERROR_UNDERRUN,            /**< Slave had no frame to transmit when clocked by master (UDR)   */
    SPI_XFER_ERROR_CRC,                 /**< Received CRC does not match the calculated one (CRCERR)       */
    SPI_XFER_ERROR_MODE_FAULT,          /**< Master was deselected by NSS input (MODF)                     */
    SPI_XFER_ERROR_FRAME,               /**< TI frame format error - misplaced NSS pulse (FRE)             */
    SPI_XFER_ERROR_INCOMPLETE,          /**< End of transfer reached, but not all frames were moved        */
    SPI_XFER_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)                */
    SPI_XFER_ERROR_DMA_CONFIG,          /**< DMA configuration error                                       */
    SPI_XFER_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration (linked list) update error                  */
    SPI_XFER_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun                                           */
    SPI_XFER_ERROR_CNT                  /**< Count of data transfer errors                                 */
}   spi_XferErrorId_t;


/** \brief Transfer complete callback (all frames of the request were transferred) */
typedef void ( spi_XferCallback_t )( void );

/** \brief Data transfer error callback, error identification is given as parameter */
typedef void ( spi_XferErrCallback_t )( spi_XferErrorId_t errorId );


/**
 * \brief Data handling configuration (common for DMA, ISR and POLL mode)
 *
 * Callback events (equal in all modes):
 * - XferCompleteCallback: all frames of the request were transferred
 * - ErrorCallback:        transfer was terminated by an error (\ref spi_XferErrorId_t)
 *
 * Unused callback shall be set to SPI_NULL_PTR. DMA identifications / priorities are used only
 * in SPI_XFER_MODE_DMA (streams connected to the SPI requests, transmit and receive stream must
 * differ), IrqPriority is used in DMA and ISR mode.
 */
typedef struct
{
    spi_XferMode_t          XferMode;             /**< Data transfer mode (NONE / DMA / ISR / POLL)     */
    spi_DmaPeriphId_t       TxDmaPeriphId;        /**< DMA peripheral (transmission in DMA mode)        */
    spi_DmaChannelId_t      TxDmaChannelId;       /**< DMA stream (transmission in DMA mode)            */
    spi_DmaPriority_t       TxDmaPriority;        /**< DMA stream priority (transmission in DMA mode)   */
    spi_DmaPeriphId_t       RxDmaPeriphId;        /**< DMA peripheral (reception in DMA mode)           */
    spi_DmaChannelId_t      RxDmaChannelId;       /**< DMA stream (reception in DMA mode)               */
    spi_DmaPriority_t       RxDmaPriority;        /**< DMA stream priority (reception in DMA mode)      */
    spi_IrqPrio_t           IrqPriority;          /**< SPI interrupt priority (DMA / ISR mode)          */
    spi_XferCallback_t     *XferCompleteCallback; /**< Transfer complete. SPI_NULL_PTR if not used.     */
    spi_XferErrCallback_t  *ErrorCallback;        /**< Transfer error. SPI_NULL_PTR if not used.        */
}   spi_DataConfig_t;


/**
 * \brief Transfer request
 *
 * XferSize frames are transferred (1 - 65535). Every frame occupies 1 / 2 bytes of the buffer
 * (data size 8 / 16 bits).
 *
 * Use of buffers depends on the communication direction:
 * - Full-duplex: at least one buffer. TxData = SPI_NULL_PTR - zero frames are transmitted,
 *                RxData = SPI_NULL_PTR - received frames are discarded.
 * - Simplex TX:  TxData is required, RxData is ignored.
 * - Simplex RX:  RxData is required, TxData is ignored.
 * - Half-duplex: exactly one buffer - TxData (transmission) or RxData (reception).
 *
 * \note  Buffers are not copied - they must stay valid until the end of the transfer. In DMA mode
 *        the buffers must be aligned to the frame size (2 bytes for 16-bit frames).
 */
typedef struct
{
    const spi_Data_t *TxData;   /**< Data to be transmitted                   */
    spi_Data_t       *RxData;   /**< Buffer for received data                 */
    spi_DataCnt_t     XferSize; /**< Count of frames to be transferred        */
}   spi_XferRequest_t;


/** \brief SPI peripheral configuration structure */
typedef struct
{
    spi_PeriphId_t           PeriphId;    /**< SPI peripheral identification                                  */
    spi_ClkSrc_t             ClkSrc;      /**< SPI kernel clock source                                        */
    spi_Mode_t               Mode;        /**< Master / slave                                                 */
    spi_FreqHz_t             BusFreq;     /**< Required SCK frequency in Hz (master only). The closest lower
                                               frequency reachable by the prescaler is used                   */
    spi_ClockMode_t          ClockMode;   /**< Clock polarity and phase                                       */
    spi_DataSize_t           DataSize;    /**< Size of data frame                                             */
    spi_BitOrder_t           BitOrder;    /**< Bit order of data frame                                        */
    spi_Direction_t          Direction;   /**< Communication direction                                        */
    spi_FrameFormat_t        FrameFormat; /**< Motorola / TI frame format                                     */
    spi_NssConfig_t          NssConfig;   /**< NSS management                                                 */

    const spi_DataConfig_t  *DataConfig;  /**< Data handling configuration (copied). SPI_NULL_PTR - data
                                               handling is not initialized                                   */

    spi_PinCode_t            SckPin;      /**< SCK pin of PeriphId (SPI_PIN_UNUSED - not configured)          */
    spi_PinCode_t            MisoPin;     /**< MISO pin of PeriphId (SPI_PIN_UNUSED - not configured)         */
    spi_PinCode_t            MosiPin;     /**< MOSI pin of PeriphId (SPI_PIN_UNUSED - not configured)         */
    spi_PinCode_t            NssPin;      /**< NSS pin of PeriphId (SPI_PIN_UNUSED - not configured)          */
    spi_PinSpeed_t           PinSpeed;    /**< Output speed of SPI pins                                       */
}   spi_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_TYPES_H */
