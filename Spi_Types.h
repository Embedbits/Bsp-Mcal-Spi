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

/** Mask of one field (5 bits) in encoded pin / DMA stream value */
#define SPI_BIT_MASK_FIELD                  ( 0x1Fu )

/** SPI peripheral identification bit offset in encoded DMA stream value */
#define SPI_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ( 15u )

/** DMA peripheral identification bit offset in encoded DMA stream value */
#define SPI_DMA_BIT_MASK_DMA_BIT_OFFSET     ( 10u )

/** Stream identification bit offset in encoded DMA stream value */
#define SPI_DMA_BIT_MASK_STREAM_BIT_OFFSET  ( 5u )

/** Channel selection (CHSEL) bit offset in encoded DMA stream value */
#define SPI_DMA_BIT_MASK_CHSEL_BIT_OFFSET   ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/**
 * \brief Encodes pin configuration (peripheral, port, pin, alternate function) into single value
 *        of \ref spi_PinCode_t
 *
 * The macro defines the values of the pin tables \ref spi_SckPin_t, \ref spi_MisoPin_t,
 * \ref spi_MosiPin_t and \ref spi_NssPin_t, e.g. SPI1 SCK on PA5 with AF5 is \ref SPI_SCK_PIN_SPI1_PA5:
 * SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_ALT_FUNC_5 )
 */
#define SPI_PIN_ENCODE( PERIPH_ID, PORT_ID, PIN_ID, AF_ID )     ( (spi_PinCode_t)( ( (uint32_t)(PERIPH_ID) << SPI_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(PORT_ID)   << SPI_BIT_MASK_PORT_BIT_OFFSET   ) | \
                                                                                   ( (uint32_t)(PIN_ID)    << SPI_BIT_MASK_PIN_BIT_OFFSET    ) | \
                                                                                   ( (uint32_t)(AF_ID)     << SPI_BIT_MASK_AF_BIT_OFFSET     )   ) )

/** Pin is not configured by the module (value of the *_PIN_UNUSED items of the pin tables) */
#define SPI_PIN_UNUSED                      SPI_PIN_ENCODE( SPI_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )

/** Extract peripheral ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PERIPH( CODED_VAL )     ( ( (CODED_VAL) >> SPI_BIT_MASK_PERIPH_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract port ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PORT( CODED_VAL )       ( ( (CODED_VAL) >> SPI_BIT_MASK_PORT_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract pin ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_PIN( CODED_VAL )        ( ( (CODED_VAL) >> SPI_BIT_MASK_PIN_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract alternate function ID from encoded pin value */
#define SPI_BIT_MASK_DECODE_AF( CODED_VAL )         ( ( (CODED_VAL) >> SPI_BIT_MASK_AF_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/**
 * \brief Encodes DMA stream (SPI peripheral, DMA peripheral, stream, channel selection) into single
 *        value of \ref spi_DmaCode_t
 *
 * The macro defines the values of the DMA stream lists \ref spi_TxDma_t and \ref spi_RxDma_t, e.g. the
 * SPI1 transmit request on DMA2 stream 3 (channel selection 3) is \ref SPI_TX_DMA_SPI1_DMA2_STREAM3:
 * SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_3, 3u )
 */
#define SPI_DMA_ENCODE( PERIPH_ID, DMA_ID, STREAM_ID, CHSEL )   ( (spi_DmaCode_t)( ( (uint32_t)(PERIPH_ID) << SPI_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(DMA_ID)    << SPI_DMA_BIT_MASK_DMA_BIT_OFFSET    ) | \
                                                                                   ( (uint32_t)(STREAM_ID) << SPI_DMA_BIT_MASK_STREAM_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(CHSEL)     << SPI_DMA_BIT_MASK_CHSEL_BIT_OFFSET  )   ) )

/** Stream is not configured by the module (value of the *_DMA_UNUSED items of the DMA stream lists) */
#define SPI_DMA_CODE_UNUSED                 SPI_DMA_ENCODE( SPI_PERIPH_CNT, SPI_DMA_PERIPH_CNT, SPI_DMA_CHANNEL_CNT, 0u )

/** Extract SPI peripheral ID from encoded DMA stream value */
#define SPI_DMA_BIT_MASK_DECODE_PERIPH( CODED_VAL ) ( ( (CODED_VAL) >> SPI_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract DMA peripheral ID from encoded DMA stream value */
#define SPI_DMA_BIT_MASK_DECODE_DMA( CODED_VAL )    ( ( (CODED_VAL) >> SPI_DMA_BIT_MASK_DMA_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract stream ID from encoded DMA stream value */
#define SPI_DMA_BIT_MASK_DECODE_STREAM( CODED_VAL ) ( ( (CODED_VAL) >> SPI_DMA_BIT_MASK_STREAM_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

/** Extract channel selection (CHSEL) from encoded DMA stream value */
#define SPI_DMA_BIT_MASK_DECODE_CHSEL( CODED_VAL )  ( ( (CODED_VAL) >> SPI_DMA_BIT_MASK_CHSEL_BIT_OFFSET ) & SPI_BIT_MASK_FIELD )

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

/** \brief Encoded pin (value of \ref spi_SckPin_t, \ref spi_MisoPin_t, \ref spi_MosiPin_t or \ref spi_NssPin_t) */
typedef uint32_t spi_PinCode_t;

/** \brief Encoded DMA stream (value of \ref spi_TxDma_t or \ref spi_RxDma_t) */
typedef uint32_t spi_DmaCode_t;


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


/**
 * \brief List of SCK pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    SPI_SCK_PIN_SPI1_PA5           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PA5 */
    SPI_SCK_PIN_SPI1_PB3           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PB3 */
#if defined(SPI2)
#if defined(STM32F413xx) || \
    defined(STM32F423xx) || \
    defined(STM32F446xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_SCK_PIN_SPI2_PA9           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PA9 */
#endif
    SPI_SCK_PIN_SPI2_PB10          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PB10 */
    SPI_SCK_PIN_SPI2_PB13          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PB13 */
#if defined(STM32F410Rx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx) || \
    defined(STM32F446xx)
    SPI_SCK_PIN_SPI2_PC7           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PC7 */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F405xx) && \
    !defined(STM32F407xx) && \
    !defined(STM32F415xx) && \
    !defined(STM32F417xx)
    SPI_SCK_PIN_SPI2_PD3           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_D   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PD3 */
#endif
#if defined(STM32F405xx) || \
    defined(STM32F407xx) || \
    defined(STM32F415xx) || \
    defined(STM32F417xx) || \
    defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_SCK_PIN_SPI2_PI1           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PI1 */
#endif
#endif /* SPI2 */
#if defined(SPI3)
    SPI_SCK_PIN_SPI3_PB3           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PB3 */
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_SCK_PIN_SPI3_PB12          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_7  ), /**< SPI3 SCK pin connected to PB12 */
#endif
#if !defined(STM32F412Cx)
    SPI_SCK_PIN_SPI3_PC10          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PC10 */
#endif
#endif /* SPI3 */
#if defined(SPI4)
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_SCK_PIN_SPI4_PB13          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_B   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_6  ), /**< SPI4 SCK pin connected to PB13 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_SCK_PIN_SPI4_PE2           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PE2 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_SCK_PIN_SPI4_PE12          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PE12 */
#endif
#if defined(STM32F446xx)
    SPI_SCK_PIN_SPI4_PG11          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_G   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_6  ), /**< SPI4 SCK pin connected to PG11 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
#if !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_SCK_PIN_SPI5_PB0           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_B   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_6  ), /**< SPI5 SCK pin connected to PB0 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_SCK_PIN_SPI5_PE2           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_6  ), /**< SPI5 SCK pin connected to PE2 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_SCK_PIN_SPI5_PE12          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI5 SCK pin connected to PE12 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_SCK_PIN_SPI5_PF7           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI5 SCK pin connected to PF7 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_SCK_PIN_SPI5_PH6           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI5 SCK pin connected to PH6 */
#endif
#endif /* SPI5 */
#if defined(SPI6)
    SPI_SCK_PIN_SPI6_PG13          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI6 SCK pin connected to PG13 */
#endif /* SPI6 */
    SPI_SCK_PIN_UNUSED             = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_SckPin_t;


/**
 * \brief List of MISO pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if !defined(STM32F410Tx)
    SPI_MISO_PIN_SPI1_PA6          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PA6 */
#endif
    SPI_MISO_PIN_SPI1_PB4          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PB4 */
#if defined(SPI2)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MISO_PIN_SPI2_PA12         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PA12 */
#endif
    SPI_MISO_PIN_SPI2_PB14         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PB14 */
#if !defined(STM32F410Cx) && \
    !defined(STM32F412Cx)
    SPI_MISO_PIN_SPI2_PC2          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PC2 */
#endif
#if defined(STM32F407xx) || \
    defined(STM32F417xx) || \
    defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MISO_PIN_SPI2_PI2          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PI2 */
#endif
#endif /* SPI2 */
#if defined(SPI3)
    SPI_MISO_PIN_SPI3_PB4          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PB4 */
#if !defined(STM32F412Cx)
    SPI_MISO_PIN_SPI3_PC11         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PC11 */
#endif
#endif /* SPI3 */
#if defined(SPI4)
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MISO_PIN_SPI4_PA11         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_A   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_6  ), /**< SPI4 MISO pin connected to PA11 */
#endif
#if defined(STM32F446xx)
    SPI_MISO_PIN_SPI4_PD0          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_D   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PD0 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_MISO_PIN_SPI4_PE5          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PE5 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_MISO_PIN_SPI4_PE13         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PE13 */
#endif
#if defined(STM32F446xx)
    SPI_MISO_PIN_SPI4_PG12         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_G   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI4 MISO pin connected to PG12 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
#if !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_MISO_PIN_SPI5_PA12         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_A   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI5 MISO pin connected to PA12 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MISO_PIN_SPI5_PE5          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_6  ), /**< SPI5 MISO pin connected to PE5 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MISO_PIN_SPI5_PE13         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_6  ), /**< SPI5 MISO pin connected to PE13 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MISO_PIN_SPI5_PF8          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI5 MISO pin connected to PF8 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MISO_PIN_SPI5_PH7          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI5 MISO pin connected to PH7 */
#endif
#endif /* SPI5 */
#if defined(SPI6)
    SPI_MISO_PIN_SPI6_PG12         = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI6 MISO pin connected to PG12 */
#endif /* SPI6 */
    SPI_MISO_PIN_UNUSED            = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_MisoPin_t;


/**
 * \brief List of MOSI pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if !defined(STM32F410Tx)
    SPI_MOSI_PIN_SPI1_PA7          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PA7 */
#endif
    SPI_MOSI_PIN_SPI1_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PB5 */
#if defined(SPI2)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MOSI_PIN_SPI2_PA10         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PA10 */
#endif
    SPI_MOSI_PIN_SPI2_PB15         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PB15 */
#if defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MOSI_PIN_SPI2_PC1_AF5      = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PC1 (AF5) */
#endif
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI2_PC1_AF7      = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_7  ), /**< SPI2 MOSI pin connected to PC1 (AF7) */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F412Cx)
    SPI_MOSI_PIN_SPI2_PC3          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PC3 */
#endif
#if defined(STM32F407xx) || \
    defined(STM32F417xx) || \
    defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MOSI_PIN_SPI2_PI3          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PI3 */
#endif
#endif /* SPI2 */
#if defined(SPI3)
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI3_PB0          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_7  ), /**< SPI3 MOSI pin connected to PB0 */
#endif
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI3_PB2          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_7  ), /**< SPI3 MOSI pin connected to PB2 */
#endif
    SPI_MOSI_PIN_SPI3_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_6  ), /**< SPI3 MOSI pin connected to PB5 */
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI3_PC1          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI3 MOSI pin connected to PC1 */
#endif
#if !defined(STM32F412Cx)
    SPI_MOSI_PIN_SPI3_PC12         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI3 MOSI pin connected to PC12 */
#endif
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI3_PD0          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_D   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_6  ), /**< SPI3 MOSI pin connected to PD0 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F405xx) && \
    !defined(STM32F407xx) && \
    !defined(STM32F415xx) && \
    !defined(STM32F417xx)
    SPI_MOSI_PIN_SPI3_PD6          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_D   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI3 MOSI pin connected to PD6 */
#endif
#endif /* SPI3 */
#if defined(SPI4)
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MOSI_PIN_SPI4_PA1          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_A   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PA1 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_MOSI_PIN_SPI4_PE6          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PE6 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_MOSI_PIN_SPI4_PE14         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PE14 */
#endif
#if defined(STM32F446xx)
    SPI_MOSI_PIN_SPI4_PG13         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_G   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_6  ), /**< SPI4 MOSI pin connected to PG13 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
#if !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_MOSI_PIN_SPI5_PA10         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_A   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_6  ), /**< SPI5 MOSI pin connected to PA10 */
#endif
#if !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_MOSI_PIN_SPI5_PB8          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_6  ), /**< SPI5 MOSI pin connected to PB8 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MOSI_PIN_SPI5_PE6          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_6  ), /**< SPI5 MOSI pin connected to PE6 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_MOSI_PIN_SPI5_PE14         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_6  ), /**< SPI5 MOSI pin connected to PE14 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MOSI_PIN_SPI5_PF9          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI5 MOSI pin connected to PF9 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_MOSI_PIN_SPI5_PF11         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI5 MOSI pin connected to PF11 */
#endif
#endif /* SPI5 */
#if defined(SPI6)
    SPI_MOSI_PIN_SPI6_PG14         = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI6 MOSI pin connected to PG14 */
#endif /* SPI6 */
    SPI_MOSI_PIN_UNUSED            = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_MosiPin_t;


/**
 * \brief List of NSS pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if !defined(STM32F410Tx)
    SPI_NSS_PIN_SPI1_PA4           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PA4 */
#endif
    SPI_NSS_PIN_SPI1_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PA15 */
#if defined(SPI2)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_NSS_PIN_SPI2_PA11          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PA11 */
#endif
#if defined(STM32F446xx)
    SPI_NSS_PIN_SPI2_PB4           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_7  ), /**< SPI2 NSS pin connected to PB4 */
#endif
    SPI_NSS_PIN_SPI2_PB9           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PB9 */
    SPI_NSS_PIN_SPI2_PB12          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PB12 */
#if defined(STM32F446xx)
    SPI_NSS_PIN_SPI2_PD1           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_D   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_7  ), /**< SPI2 NSS pin connected to PD1 */
#endif
#if defined(STM32F405xx) || \
    defined(STM32F407xx) || \
    defined(STM32F415xx) || \
    defined(STM32F417xx) || \
    defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_NSS_PIN_SPI2_PI0           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PI0 */
#endif
#endif /* SPI2 */
#if defined(SPI3)
    SPI_NSS_PIN_SPI3_PA4           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PA4 */
    SPI_NSS_PIN_SPI3_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PA15 */
#endif /* SPI3 */
#if defined(SPI4)
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_NSS_PIN_SPI4_PB12          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_B   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI4 NSS pin connected to PB12 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_NSS_PIN_SPI4_PE4           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI4 NSS pin connected to PE4 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    SPI_NSS_PIN_SPI4_PE11          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI4 NSS pin connected to PE11 */
#endif
#if defined(STM32F446xx)
    SPI_NSS_PIN_SPI4_PG14          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_G   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_6  ), /**< SPI4 NSS pin connected to PG14 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
#if !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_NSS_PIN_SPI5_PB1           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_B   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_6  ), /**< SPI5 NSS pin connected to PB1 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_NSS_PIN_SPI5_PE4           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_6  ), /**< SPI5 NSS pin connected to PE4 */
#endif
#if defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_NSS_PIN_SPI5_PE11          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_E   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_6  ), /**< SPI5 NSS pin connected to PE11 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_NSS_PIN_SPI5_PF6           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI5 NSS pin connected to PF6 */
#endif
#if defined(STM32F427xx) || \
    defined(STM32F429xx) || \
    defined(STM32F437xx) || \
    defined(STM32F439xx) || \
    defined(STM32F469xx) || \
    defined(STM32F479xx)
    SPI_NSS_PIN_SPI5_PH5           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI5 NSS pin connected to PH5 */
#endif
#endif /* SPI5 */
#if defined(SPI6)
    SPI_NSS_PIN_SPI6_PG8           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI6 NSS pin connected to PG8 */
#endif /* SPI6 */
    SPI_NSS_PIN_UNUSED             = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_NssPin_t;


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
 * the SPI request. The streams usable by the SPI peripherals are given by the lists
 * \ref spi_TxDma_t and \ref spi_RxDma_t.
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


/**
 * \brief List of DMA streams able to serve the SPI TX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F410Tx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_TX_DMA_SPI1_DMA2_STREAM2       = SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_2, 2u ), /**< SPI1 TX request on DMA2 stream 2 (channel selection 2) */
#endif
    SPI_TX_DMA_SPI1_DMA2_STREAM3       = SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_3, 3u ), /**< SPI1 TX request on DMA2 stream 3 (channel selection 3) */
    SPI_TX_DMA_SPI1_DMA2_STREAM5       = SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_5, 3u ), /**< SPI1 TX request on DMA2 stream 5 (channel selection 3) */
#if defined(SPI2)
    SPI_TX_DMA_SPI2_DMA1_STREAM4       = SPI_DMA_ENCODE( SPI_PERIPH_2, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_4, 0u ), /**< SPI2 TX request on DMA1 stream 4 (channel selection 0) */
#endif
#if defined(SPI3)
    SPI_TX_DMA_SPI3_DMA1_STREAM5       = SPI_DMA_ENCODE( SPI_PERIPH_3, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_5, 0u ), /**< SPI3 TX request on DMA1 stream 5 (channel selection 0) */
    SPI_TX_DMA_SPI3_DMA1_STREAM7       = SPI_DMA_ENCODE( SPI_PERIPH_3, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_7, 0u ), /**< SPI3 TX request on DMA1 stream 7 (channel selection 0) */
#endif
#if defined(SPI4)
    SPI_TX_DMA_SPI4_DMA2_STREAM1       = SPI_DMA_ENCODE( SPI_PERIPH_4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_1, 4u ), /**< SPI4 TX request on DMA2 stream 1 (channel selection 4) */
    SPI_TX_DMA_SPI4_DMA2_STREAM4       = SPI_DMA_ENCODE( SPI_PERIPH_4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_4, 5u ), /**< SPI4 TX request on DMA2 stream 4 (channel selection 5) */
#endif
#if defined(SPI5)
    SPI_TX_DMA_SPI5_DMA2_STREAM4       = SPI_DMA_ENCODE( SPI_PERIPH_5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_4, 2u ), /**< SPI5 TX request on DMA2 stream 4 (channel selection 2) */
#endif
#if defined(SPI5) && \
    !defined(STM32F427xx) && \
    !defined(STM32F429xx) && \
    !defined(STM32F437xx) && \
    !defined(STM32F439xx) && \
    !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    SPI_TX_DMA_SPI5_DMA2_STREAM5       = SPI_DMA_ENCODE( SPI_PERIPH_5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_5, 5u ), /**< SPI5 TX request on DMA2 stream 5 (channel selection 5) */
#endif
#if defined(SPI5)
    SPI_TX_DMA_SPI5_DMA2_STREAM6       = SPI_DMA_ENCODE( SPI_PERIPH_5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_6, 7u ), /**< SPI5 TX request on DMA2 stream 6 (channel selection 7) */
#endif
#if defined(SPI6)
    SPI_TX_DMA_SPI6_DMA2_STREAM5       = SPI_DMA_ENCODE( SPI_PERIPH_6, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_5, 1u ), /**< SPI6 TX request on DMA2 stream 5 (channel selection 1) */
#endif
    SPI_TX_DMA_UNUSED                  = SPI_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   spi_TxDma_t;


/**
 * \brief List of DMA streams able to serve the SPI RX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    SPI_RX_DMA_SPI1_DMA2_STREAM0       = SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_0, 3u ), /**< SPI1 RX request on DMA2 stream 0 (channel selection 3) */
    SPI_RX_DMA_SPI1_DMA2_STREAM2       = SPI_DMA_ENCODE( SPI_PERIPH_1, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_2, 3u ), /**< SPI1 RX request on DMA2 stream 2 (channel selection 3) */
#if defined(SPI2)
    SPI_RX_DMA_SPI2_DMA1_STREAM3       = SPI_DMA_ENCODE( SPI_PERIPH_2, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_3, 0u ), /**< SPI2 RX request on DMA1 stream 3 (channel selection 0) */
#endif
#if defined(SPI3)
    SPI_RX_DMA_SPI3_DMA1_STREAM0       = SPI_DMA_ENCODE( SPI_PERIPH_3, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_0, 0u ), /**< SPI3 RX request on DMA1 stream 0 (channel selection 0) */
    SPI_RX_DMA_SPI3_DMA1_STREAM2       = SPI_DMA_ENCODE( SPI_PERIPH_3, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_2, 0u ), /**< SPI3 RX request on DMA1 stream 2 (channel selection 0) */
#endif
#if defined(SPI4)
    SPI_RX_DMA_SPI4_DMA2_STREAM0       = SPI_DMA_ENCODE( SPI_PERIPH_4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_0, 4u ), /**< SPI4 RX request on DMA2 stream 0 (channel selection 4) */
    SPI_RX_DMA_SPI4_DMA2_STREAM3       = SPI_DMA_ENCODE( SPI_PERIPH_4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_3, 5u ), /**< SPI4 RX request on DMA2 stream 3 (channel selection 5) */
#endif
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    SPI_RX_DMA_SPI4_DMA2_STREAM4       = SPI_DMA_ENCODE( SPI_PERIPH_4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_4, 4u ), /**< SPI4 RX request on DMA2 stream 4 (channel selection 4) */
#endif
#if defined(SPI5)
    SPI_RX_DMA_SPI5_DMA2_STREAM3       = SPI_DMA_ENCODE( SPI_PERIPH_5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_3, 2u ), /**< SPI5 RX request on DMA2 stream 3 (channel selection 2) */
    SPI_RX_DMA_SPI5_DMA2_STREAM5       = SPI_DMA_ENCODE( SPI_PERIPH_5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_5, 7u ), /**< SPI5 RX request on DMA2 stream 5 (channel selection 7) */
#endif
#if defined(SPI6)
    SPI_RX_DMA_SPI6_DMA2_STREAM6       = SPI_DMA_ENCODE( SPI_PERIPH_6, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_6, 1u ), /**< SPI6 RX request on DMA2 stream 6 (channel selection 1) */
#endif
    SPI_RX_DMA_UNUSED                  = SPI_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   spi_RxDma_t;


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
 * Unused callback shall be set to SPI_NULL_PTR. DMA streams / priorities are used only in
 * SPI_XFER_MODE_DMA (TxDma / RxDma are items of the lists \ref spi_TxDma_t / \ref spi_RxDma_t of the
 * configured SPI peripheral, SPI_TX_DMA_UNUSED / SPI_RX_DMA_UNUSED in other modes), IrqPriority is
 * used in DMA and ISR mode.
 */
typedef struct
{
    spi_XferMode_t          XferMode;             /**< Data transfer mode (NONE / DMA / ISR / POLL)     */
    spi_TxDma_t             TxDma;                /**< DMA stream (transmission in DMA mode)            */
    spi_DmaPriority_t       TxDmaPriority;        /**< DMA stream priority (transmission in DMA mode)   */
    spi_RxDma_t             RxDma;                /**< DMA stream (reception in DMA mode)               */
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

    spi_SckPin_t             SckPin;      /**< SCK pin of PeriphId (SPI_SCK_PIN_UNUSED - not configured)      */
    spi_MisoPin_t            MisoPin;     /**< MISO pin of PeriphId (SPI_MISO_PIN_UNUSED - not configured)    */
    spi_MosiPin_t            MosiPin;     /**< MOSI pin of PeriphId (SPI_MOSI_PIN_UNUSED - not configured)    */
    spi_NssPin_t             NssPin;      /**< NSS pin of PeriphId (SPI_NSS_PIN_UNUSED - not configured)      */
    spi_PinSpeed_t           PinSpeed;    /**< Output speed of SPI pins                                       */
}   spi_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_TYPES_H */
