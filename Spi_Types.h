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
 */

#ifndef SPI_SPI_TYPES_H
#define SPI_SPI_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Gpdma_Types.h"                    /* DMA types definitions          */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define SPI_NULL_PTR                        ( ( void* ) 0u )

/** Maximum count of idle clock cycles (MIDI / MSSI fields) */
#define SPI_IDLE_CYCLES_MAX                 ( 15u )

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

/** Peripheral identification bit offset in encoded kernel clock source value */
#define SPI_CLK_SRC_BIT_MASK_PERIPH_BIT_OFFSET  ( 8u )

/** Source identification bit offset in encoded kernel clock source value */
#define SPI_CLK_SRC_BIT_MASK_SOURCE_BIT_OFFSET  ( 0u )

/** Mask of one field (8 bits) in encoded kernel clock source value */
#define SPI_CLK_SRC_BIT_MASK_FIELD              ( 0xFFu )

/**
 * \brief Encodes kernel clock source (peripheral, source) into single value of \ref spi_ClkSrc_t
 *
 * The macro defines the items of the kernel clock source list \ref spi_ClkSrc_t, e.g. the PLL2 output P as kernel
 * clock of SPI1 is \ref SPI_CLK_SRC_SPI1_PLL2P: SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_PLL2 )
 */
#define SPI_CLK_SRC_ENCODE( PERIPH_ID, SOURCE_ID )  ( (uint32_t)( ( (uint32_t)(PERIPH_ID) << SPI_CLK_SRC_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                  ( (uint32_t)(SOURCE_ID) << SPI_CLK_SRC_BIT_MASK_SOURCE_BIT_OFFSET )   ) )

/** Extract SPI peripheral ID from encoded kernel clock source value */
#define SPI_CLK_SRC_BIT_MASK_DECODE_PERIPH( CODED_VAL )  ( ( (uint32_t)(CODED_VAL) >> SPI_CLK_SRC_BIT_MASK_PERIPH_BIT_OFFSET ) & SPI_CLK_SRC_BIT_MASK_FIELD )

/** Extract source ID (\ref spi_ClkSrcId_t) from encoded kernel clock source value */
#define SPI_CLK_SRC_BIT_MASK_DECODE_SOURCE( CODED_VAL )  ( ( (uint32_t)(CODED_VAL) >> SPI_CLK_SRC_BIT_MASK_SOURCE_BIT_OFFSET ) & SPI_CLK_SRC_BIT_MASK_FIELD )

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

/** \brief Type representing one byte of data buffer (frame occupies 1, 2 or 4 bytes) */
typedef uint8_t spi_Data_t;

/** \brief Type representing count of transferred data frames */
typedef uint16_t spi_DataCnt_t;

/** \brief Interrupt priority type definition */
typedef uint32_t spi_IrqPrio_t;

/** \brief Count of idle SPI clock cycles (0 - \ref SPI_IDLE_CYCLES_MAX) */
typedef uint8_t spi_IdleCycles_t;

/** \brief CRC polynomial (without the highest bit, e.g. 0x07 for CRC-8 x^8 + x^2 + x + 1) */
typedef uint32_t spi_CrcPoly_t;

/** \brief CRC value (TXCRC / RXCRC register) */
typedef uint32_t spi_CrcValue_t;

/** \brief Encoded pin (value of \ref spi_SckPin_t, \ref spi_MisoPin_t, \ref spi_MosiPin_t or \ref spi_NssPin_t) */
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


/** \brief Kernel clock source identification (source part of the items of \ref spi_ClkSrc_t) */
typedef enum
{
    SPI_CLK_SRC_ID_PLL1Q = 0u, /**< PLL1 output Q (SPI1 - SPI3)                                  */
    SPI_CLK_SRC_ID_PLL2,       /**< PLL2 output P (SPI1 - SPI3) / output Q (SPI4 - SPI6)         */
    SPI_CLK_SRC_ID_PLL3,       /**< PLL3 output P (SPI1 - SPI3) / output Q (SPI4 - SPI6)         */
    SPI_CLK_SRC_ID_PCLK,       /**< APB clock of the peripheral (SPI4 - SPI6)                    */
    SPI_CLK_SRC_ID_HSI,        /**< High Speed Internal oscillator (HSI) output                  */
    SPI_CLK_SRC_ID_CSI,        /**< 4MHz Low Power Internal oscillator (CSI)                     */
    SPI_CLK_SRC_ID_HSE,        /**< High Speed External oscillator (HSE)                         */
    SPI_CLK_SRC_ID_CNT         /**< Count of clock sources                                       */
}   spi_ClkSrcId_t;


/**
 * \brief SPI kernel clock source - list of the sources of every SPI peripheral
 *
 * Available sources differ per peripheral, the list contains one item per peripheral and source:
 * - SPI1 - SPI3: PLL1Q, PLL2 (output P), PLL3 (output P, devices with PLL3)
 * - SPI4 - SPI6: PCLK, PLL2 (output Q), PLL3 (output Q, devices with PLL3), HSI, CSI, HSE
 *
 * The item shall belong to the peripheral of the configuration (\ref spi_Config_t::PeriphId), otherwise
 * \ref Spi_Init refuses the configuration.
 */
typedef enum
{
#ifdef SPI1
    SPI_CLK_SRC_SPI1_PLL1Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_PLL1Q ), /**< SPI1 kernel clock: PLL1 output Q (default kernel clock) */
    SPI_CLK_SRC_SPI1_PLL2P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_PLL2  ), /**< SPI1 kernel clock: PLL2 output P */
#if defined(RCC_CR_PLL3ON)
    SPI_CLK_SRC_SPI1_PLL3P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_PLL3  ), /**< SPI1 kernel clock: PLL3 output P */
#endif /* RCC_CR_PLL3ON */
#endif /* SPI1 */
#ifdef SPI2
    SPI_CLK_SRC_SPI2_PLL1Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_2, SPI_CLK_SRC_ID_PLL1Q ), /**< SPI2 kernel clock: PLL1 output Q (default kernel clock) */
    SPI_CLK_SRC_SPI2_PLL2P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_2, SPI_CLK_SRC_ID_PLL2  ), /**< SPI2 kernel clock: PLL2 output P */
#if defined(RCC_CR_PLL3ON)
    SPI_CLK_SRC_SPI2_PLL3P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_2, SPI_CLK_SRC_ID_PLL3  ), /**< SPI2 kernel clock: PLL3 output P */
#endif /* RCC_CR_PLL3ON */
#endif /* SPI2 */
#ifdef SPI3
    SPI_CLK_SRC_SPI3_PLL1Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_3, SPI_CLK_SRC_ID_PLL1Q ), /**< SPI3 kernel clock: PLL1 output Q (default kernel clock) */
    SPI_CLK_SRC_SPI3_PLL2P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_3, SPI_CLK_SRC_ID_PLL2  ), /**< SPI3 kernel clock: PLL2 output P */
#if defined(RCC_CR_PLL3ON)
    SPI_CLK_SRC_SPI3_PLL3P     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_3, SPI_CLK_SRC_ID_PLL3  ), /**< SPI3 kernel clock: PLL3 output P */
#endif /* RCC_CR_PLL3ON */
#endif /* SPI3 */
#ifdef SPI4
    SPI_CLK_SRC_SPI4_PLL2Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_PLL2  ), /**< SPI4 kernel clock: PLL2 output Q */
#if defined(RCC_CR_PLL3ON)
    SPI_CLK_SRC_SPI4_PLL3Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_PLL3  ), /**< SPI4 kernel clock: PLL3 output Q */
#endif /* RCC_CR_PLL3ON */
    SPI_CLK_SRC_SPI4_PCLK2     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_PCLK  ), /**< SPI4 kernel clock: APB2 clock (PCLK2, default kernel clock) */
    SPI_CLK_SRC_SPI4_HSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_HSI   ), /**< SPI4 kernel clock: HSI oscillator output */
    SPI_CLK_SRC_SPI4_CSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_CSI   ), /**< SPI4 kernel clock: CSI oscillator output (4 MHz) */
    SPI_CLK_SRC_SPI4_HSE       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_4, SPI_CLK_SRC_ID_HSE   ), /**< SPI4 kernel clock: HSE oscillator output */
#endif /* SPI4 */
#ifdef SPI5
    SPI_CLK_SRC_SPI5_PLL2Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_PLL2  ), /**< SPI5 kernel clock: PLL2 output Q */
    SPI_CLK_SRC_SPI5_PLL3Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_PLL3  ), /**< SPI5 kernel clock: PLL3 output Q */
    SPI_CLK_SRC_SPI5_PCLK3     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_PCLK  ), /**< SPI5 kernel clock: APB3 clock (PCLK3, default kernel clock) */
    SPI_CLK_SRC_SPI5_HSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_HSI   ), /**< SPI5 kernel clock: HSI oscillator output */
    SPI_CLK_SRC_SPI5_CSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_CSI   ), /**< SPI5 kernel clock: CSI oscillator output (4 MHz) */
    SPI_CLK_SRC_SPI5_HSE       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_5, SPI_CLK_SRC_ID_HSE   ), /**< SPI5 kernel clock: HSE oscillator output */
#endif /* SPI5 */
#ifdef SPI6
    SPI_CLK_SRC_SPI6_PLL2Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_PLL2  ), /**< SPI6 kernel clock: PLL2 output Q */
    SPI_CLK_SRC_SPI6_PLL3Q     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_PLL3  ), /**< SPI6 kernel clock: PLL3 output Q */
    SPI_CLK_SRC_SPI6_PCLK2     = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_PCLK  ), /**< SPI6 kernel clock: APB2 clock (PCLK2, default kernel clock) */
    SPI_CLK_SRC_SPI6_HSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_HSI   ), /**< SPI6 kernel clock: HSI oscillator output */
    SPI_CLK_SRC_SPI6_CSI       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_CSI   ), /**< SPI6 kernel clock: CSI oscillator output (4 MHz) */
    SPI_CLK_SRC_SPI6_HSE       = SPI_CLK_SRC_ENCODE( SPI_PERIPH_6, SPI_CLK_SRC_ID_HSE   ), /**< SPI6 kernel clock: HSE oscillator output */
#endif /* SPI6 */
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
 * Frame is stored in the data buffer in 1 byte (4 - 8 bits), 2 bytes (9 - 16 bits) or 4 bytes
 * (17 - 32 bits), little endian. SPI4 - SPI6 support frames up to 16 bits only.
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


/** \brief Active level of NSS signal */
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
    spi_NssPolarity_t   Polarity; /**< Active level of NSS                                                */
    spi_FunctionState_t Pulse;    /**< NSS pulse between data frames (master, hardware NSS, Motorola
                                       format with CPHA = 0 only)                                         */
}   spi_NssConfig_t;


/** \brief CRC initialization pattern of transmitter and receiver */
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
 * (\ref SPI_XFER_ERROR_CRC). Size must not be smaller than the data size, SPI4 - SPI6 support
 * 8-bit and 16-bit CRC only. Polynomial must be non-zero and fit into Size bits.
 */
typedef struct
{
    spi_FunctionState_t State;      /**< CRC calculation state            */
    spi_DataSize_t      Size;       /**< CRC length                       */
    spi_CrcPoly_t       Polynomial; /**< CRC polynomial                   */
    spi_CrcInit_t       InitValue;  /**< CRC initialization pattern       */
}   spi_CrcConfig_t;


/** \brief Output speed of SPI pins (has to correspond to the bus frequency) */
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
 *        existing only on some STM32H5 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI1_PA2           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_4  ), /**< SPI1 SCK pin connected to PA2 */
#endif
    SPI_SCK_PIN_SPI1_PA5           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PA5 */
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI1_PA8           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_12 ), /**< SPI1 SCK pin connected to PA8 */
#endif
    SPI_SCK_PIN_SPI1_PB3           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PB3 */
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI1_PC0           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PC0 */
#endif
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI1_PC5           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PC5 */
#endif
#if !defined(STM32H503xx)
    SPI_SCK_PIN_SPI1_PG11          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_G   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI1 SCK pin connected to PG11 */
#endif
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI2_PA5           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_7  ), /**< SPI2 SCK pin connected to PA5 */
#endif
    SPI_SCK_PIN_SPI2_PA9           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PA9 */
    SPI_SCK_PIN_SPI2_PA12          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PA12 */
#if defined(STM32H503xx) || \
    defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_SCK_PIN_SPI2_PB2           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_6  ), /**< SPI2 SCK pin connected to PB2 */
#endif
    SPI_SCK_PIN_SPI2_PB10          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PB10 */
    SPI_SCK_PIN_SPI2_PB13          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PB13 */
#if !defined(STM32H503xx)
    SPI_SCK_PIN_SPI2_PD3           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_D   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PD3 */
#endif
#if !defined(STM32H503xx) && \
    !defined(STM32H523xx) && \
    !defined(STM32H533xx) && \
    !defined(STM32H543xx) && \
    !defined(STM32H553xx)
    SPI_SCK_PIN_SPI2_PI1           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI2 SCK pin connected to PI1 */
#endif
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI3_PA1           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PA1 */
#endif
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI3_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_10 ), /**< SPI3 SCK pin connected to PA15 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_SCK_PIN_SPI3_PB1           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_4  ), /**< SPI3 SCK pin connected to PB1 */
#endif
    SPI_SCK_PIN_SPI3_PB3           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PB3 */
#if defined(STM32H503xx)
    SPI_SCK_PIN_SPI3_PB7           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PB7 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_SCK_PIN_SPI3_PB9           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PB9 */
#endif
    SPI_SCK_PIN_SPI3_PC10          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_6  ), /**< SPI3 SCK pin connected to PC10 */
#if defined(SPI4)
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_SCK_PIN_SPI4_PA0           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_A   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PA0 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_SCK_PIN_SPI4_PC5           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_C   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_6  ), /**< SPI4 SCK pin connected to PC5 */
#endif
    SPI_SCK_PIN_SPI4_PE2           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PE2 */
    SPI_SCK_PIN_SPI4_PE12          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PE12 */
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    SPI_SCK_PIN_SPI4_PK5           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_K   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI4 SCK pin connected to PK5 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
    SPI_SCK_PIN_SPI5_PF7           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI5 SCK pin connected to PF7 */
    SPI_SCK_PIN_SPI5_PH6           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI5 SCK pin connected to PH6 */
#endif /* SPI5 */
#if defined(SPI6)
    SPI_SCK_PIN_SPI6_PA5           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_8  ), /**< SPI6 SCK pin connected to PA5 */
    SPI_SCK_PIN_SPI6_PB3           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_8  ), /**< SPI6 SCK pin connected to PB3 */
    SPI_SCK_PIN_SPI6_PC12          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_C   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI6 SCK pin connected to PC12 */
    SPI_SCK_PIN_SPI6_PG13          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI6 SCK pin connected to PG13 */
#endif /* SPI6 */
    SPI_SCK_PIN_UNUSED             = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_SckPin_t;


/**
 * \brief List of MISO pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32H5 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PA0          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_12 ), /**< SPI1 MISO pin connected to PA0 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PA3          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_4  ), /**< SPI1 MISO pin connected to PA3 */
#endif
    SPI_MISO_PIN_SPI1_PA6          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PA6 */
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PA9          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_4  ), /**< SPI1 MISO pin connected to PA9 */
#endif
    SPI_MISO_PIN_SPI1_PB4          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PB4 */
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PC2          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_4  ), /**< SPI1 MISO pin connected to PC2 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PC10         = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PC10 */
#endif
#if !defined(STM32H503xx)
    SPI_MISO_PIN_SPI1_PG9          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_G   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI1 MISO pin connected to PG9 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI2_PA7          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_11 ), /**< SPI2 MISO pin connected to PA7 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI2_PA15         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_7  ), /**< SPI2 MISO pin connected to PA15 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI2_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_6  ), /**< SPI2 MISO pin connected to PB5 */
#endif
    SPI_MISO_PIN_SPI2_PB14         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PB14 */
    SPI_MISO_PIN_SPI2_PC2          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PC2 */
#if !defined(STM32H503xx) && \
    !defined(STM32H523xx) && \
    !defined(STM32H533xx) && \
    !defined(STM32H543xx) && \
    !defined(STM32H553xx)
    SPI_MISO_PIN_SPI2_PI2          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_5  ), /**< SPI2 MISO pin connected to PI2 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI3_PA2          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PA2 */
#endif
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI3_PA4          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_10 ), /**< SPI3 MISO pin connected to PA4 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MISO_PIN_SPI3_PB0          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI3 MISO pin connected to PB0 */
#endif
    SPI_MISO_PIN_SPI3_PB4          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PB4 */
#if defined(STM32H503xx)
    SPI_MISO_PIN_SPI3_PB15         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PB15 */
#endif
    SPI_MISO_PIN_SPI3_PC11         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PC11 */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MISO_PIN_SPI3_PD7          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_D   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PD7 */
#endif
#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    SPI_MISO_PIN_SPI3_PF14         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_F   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_6  ), /**< SPI3 MISO pin connected to PF14 */
#endif
#if defined(SPI4)
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MISO_PIN_SPI4_PB7          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PB7 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MISO_PIN_SPI4_PC0          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_C   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_6  ), /**< SPI4 MISO pin connected to PC0 */
#endif
    SPI_MISO_PIN_SPI4_PE5          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PE5 */
    SPI_MISO_PIN_SPI4_PE13         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PE13 */
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    SPI_MISO_PIN_SPI4_PK7          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_K   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI4 MISO pin connected to PK7 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
    SPI_MISO_PIN_SPI5_PF8          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI5 MISO pin connected to PF8 */
    SPI_MISO_PIN_SPI5_PH7          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI5 MISO pin connected to PH7 */
#endif /* SPI5 */
#if defined(SPI6)
    SPI_MISO_PIN_SPI6_PA6          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_8  ), /**< SPI6 MISO pin connected to PA6 */
    SPI_MISO_PIN_SPI6_PB4          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_8  ), /**< SPI6 MISO pin connected to PB4 */
    SPI_MISO_PIN_SPI6_PG12         = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI6 MISO pin connected to PG12 */
#endif /* SPI6 */
    SPI_MISO_PIN_UNUSED            = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_MisoPin_t;


/**
 * \brief List of MOSI pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32H5 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI1_PA4          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< SPI1 MOSI pin connected to PA4 */
#endif
    SPI_MOSI_PIN_SPI1_PA7          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PA7 */
    SPI_MOSI_PIN_SPI1_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PB5 */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI1_PB15         = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_6  ), /**< SPI1 MOSI pin connected to PB15 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI1_PC3          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_4  ), /**< SPI1 MOSI pin connected to PC3 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI1_PC7          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PC7 */
#endif
#if !defined(STM32H503xx)
    SPI_MOSI_PIN_SPI1_PD7          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_D   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_5  ), /**< SPI1 MOSI pin connected to PD7 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI2_PA8          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_6  ), /**< SPI2 MOSI pin connected to PA8 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI2_PB1          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_6  ), /**< SPI2 MOSI pin connected to PB1 */
#endif
    SPI_MOSI_PIN_SPI2_PB15         = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PB15 */
    SPI_MOSI_PIN_SPI2_PC1          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PC1 */
    SPI_MOSI_PIN_SPI2_PC3          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_C   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PC3 */
#if !defined(STM32H503xx)
    SPI_MOSI_PIN_SPI2_PG1          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_G   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_7  ), /**< SPI2 MOSI pin connected to PG1 */
#endif
#if !defined(STM32H503xx) && \
    !defined(STM32H523xx) && \
    !defined(STM32H533xx) && \
    !defined(STM32H543xx) && \
    !defined(STM32H553xx)
    SPI_MOSI_PIN_SPI2_PI3          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 MOSI pin connected to PI3 */
#endif
#if defined(STM32H503xx) || \
    defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI3_PA3          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_6  ), /**< SPI3 MOSI pin connected to PA3 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI3_PA4          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< SPI3 MOSI pin connected to PA4 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI3_PA5          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_10 ), /**< SPI3 MOSI pin connected to PA5 */
#endif
#if defined(STM32H503xx)
    SPI_MOSI_PIN_SPI3_PA9          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_10 ), /**< SPI3 MOSI pin connected to PA9 */
#endif
    SPI_MOSI_PIN_SPI3_PB2          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_7  ), /**< SPI3 MOSI pin connected to PB2 */
    SPI_MOSI_PIN_SPI3_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_7  ), /**< SPI3 MOSI pin connected to PB5 */
    SPI_MOSI_PIN_SPI3_PC12         = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_C   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_6  ), /**< SPI3 MOSI pin connected to PC12 */
#if !defined(STM32H503xx)
    SPI_MOSI_PIN_SPI3_PD6          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_D   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI3 MOSI pin connected to PD6 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI3_PG8          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_G   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI3 MOSI pin connected to PG8 */
#endif
#if defined(SPI4)
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI4_PA8          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_6  ), /**< SPI4 MOSI pin connected to PA8 */
#endif
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_MOSI_PIN_SPI4_PC1          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_6  ), /**< SPI4 MOSI pin connected to PC1 */
#endif
    SPI_MOSI_PIN_SPI4_PE6          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PE6 */
    SPI_MOSI_PIN_SPI4_PE14         = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PE14 */
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    SPI_MOSI_PIN_SPI4_PK8          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_K   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI4 MOSI pin connected to PK8 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
    SPI_MOSI_PIN_SPI5_PF9          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI5 MOSI pin connected to PF9 */
    SPI_MOSI_PIN_SPI5_PF11         = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI5 MOSI pin connected to PF11 */
    SPI_MOSI_PIN_SPI5_PH8          = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI5 MOSI pin connected to PH8 */
#endif /* SPI5 */
#if defined(SPI6)
    SPI_MOSI_PIN_SPI6_PA7          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_8  ), /**< SPI6 MOSI pin connected to PA7 */
    SPI_MOSI_PIN_SPI6_PB5          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_8  ), /**< SPI6 MOSI pin connected to PB5 */
    SPI_MOSI_PIN_SPI6_PG14         = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_5  ), /**< SPI6 MOSI pin connected to PG14 */
#endif /* SPI6 */
    SPI_MOSI_PIN_UNUSED            = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_MosiPin_t;


/**
 * \brief List of NSS pins available for SPI peripherals (STM32CubeMX database, pins
 *        existing only on some STM32H5 lines are guarded by the CMSIS device line)
 */
typedef enum
{
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI1_PA1           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_4  ), /**< SPI1 NSS pin connected to PA1 */
#endif
    SPI_NSS_PIN_SPI1_PA4           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PA4 */
    SPI_NSS_PIN_SPI1_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PA15 */
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI1_PB8           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_12 ), /**< SPI1 NSS pin connected to PB8 */
#endif
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI1_PC1           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_4  ), /**< SPI1 NSS pin connected to PC1 */
#endif
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI1_PC8           = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_C   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PC8 */
#endif
#if !defined(STM32H503xx)
    SPI_NSS_PIN_SPI1_PG10          = SPI_PIN_ENCODE( SPI_PERIPH_1 , GPIO_PORT_G   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_5  ), /**< SPI1 NSS pin connected to PG10 */
#endif
    SPI_NSS_PIN_SPI2_PA3           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PA3 */
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI2_PA8           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_11 ), /**< SPI2 NSS pin connected to PA8 */
#endif
    SPI_NSS_PIN_SPI2_PA11          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_A   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PA11 */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_NSS_PIN_SPI2_PB1           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PB1 */
#endif
    SPI_NSS_PIN_SPI2_PB4           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_7  ), /**< SPI2 NSS pin connected to PB4 */
#if !defined(STM32H503xx)
    SPI_NSS_PIN_SPI2_PB9           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PB9 */
#endif
    SPI_NSS_PIN_SPI2_PB12          = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_B   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PB12 */
#if !defined(STM32H503xx) && \
    !defined(STM32H523xx) && \
    !defined(STM32H533xx) && \
    !defined(STM32H543xx) && \
    !defined(STM32H553xx)
    SPI_NSS_PIN_SPI2_PI0           = SPI_PIN_ENCODE( SPI_PERIPH_2 , GPIO_PORT_I   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI2 NSS pin connected to PI0 */
#endif
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI3_PA0           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_10 ), /**< SPI3 NSS pin connected to PA0 */
#endif
    SPI_NSS_PIN_SPI3_PA4           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PA4 */
    SPI_NSS_PIN_SPI3_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PA15 */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    SPI_NSS_PIN_SPI3_PB8           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PB8 */
#endif
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI3_PB10          = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_B   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PB10 */
#endif
#if defined(STM32H503xx)
    SPI_NSS_PIN_SPI3_PD2           = SPI_PIN_ENCODE( SPI_PERIPH_3 , GPIO_PORT_D   , GPIO_PIN_ID_2  , GPIO_ALT_FUNC_6  ), /**< SPI3 NSS pin connected to PD2 */
#endif
#if defined(SPI4)
    SPI_NSS_PIN_SPI4_PE4           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_5  ), /**< SPI4 NSS pin connected to PE4 */
    SPI_NSS_PIN_SPI4_PE11          = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_E   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_5  ), /**< SPI4 NSS pin connected to PE11 */
#if defined(STM32H5E5xx) || \
    defined(STM32H5F5xx)
    SPI_NSS_PIN_SPI4_PK6           = SPI_PIN_ENCODE( SPI_PERIPH_4 , GPIO_PORT_K   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI4 NSS pin connected to PK6 */
#endif
#endif /* SPI4 */
#if defined(SPI5)
    SPI_NSS_PIN_SPI5_PF6           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_F   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_5  ), /**< SPI5 NSS pin connected to PF6 */
    SPI_NSS_PIN_SPI5_PH5           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_5  ), /**< SPI5 NSS pin connected to PH5 */
    SPI_NSS_PIN_SPI5_PH9           = SPI_PIN_ENCODE( SPI_PERIPH_5 , GPIO_PORT_H   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_5  ), /**< SPI5 NSS pin connected to PH9 */
#endif /* SPI5 */
#if defined(SPI6)
    SPI_NSS_PIN_SPI6_PA0           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_5  ), /**< SPI6 NSS pin connected to PA0 */
    SPI_NSS_PIN_SPI6_PA4           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_8  ), /**< SPI6 NSS pin connected to PA4 */
    SPI_NSS_PIN_SPI6_PA15          = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_7  ), /**< SPI6 NSS pin connected to PA15 */
    SPI_NSS_PIN_SPI6_PG8           = SPI_PIN_ENCODE( SPI_PERIPH_6 , GPIO_PORT_G   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_5  ), /**< SPI6 NSS pin connected to PG8 */
#endif /* SPI6 */
    SPI_NSS_PIN_UNUSED             = SPI_PIN_UNUSED  /**< Pin is not configured by the module */
}   spi_NssPin_t;


/** DMA peripherals enumeration list */
typedef enum
{
#if defined(GPDMA1)
    SPI_DMA_PERIPH_1 = GPDMA_PERIPH_1, /**< DMA peripheral 1 identification */
#endif
#if defined(GPDMA2)
    SPI_DMA_PERIPH_2 = GPDMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    SPI_DMA_PERIPH_CNT
}   spi_DmaPeriphId_t;


/** Enumeration of available channels for all DMA peripherals */
typedef enum
{
    SPI_DMA_CHANNEL_0  = GPDMA_CHANNEL_0,  /**< DMA transfer channel 0 (FIFO size 8 bytes) linear addressing mode        */
    SPI_DMA_CHANNEL_1  = GPDMA_CHANNEL_1,  /**< DMA transfer channel 1 (FIFO size 8 bytes) linear addressing mode        */
    SPI_DMA_CHANNEL_2  = GPDMA_CHANNEL_2,  /**< DMA transfer channel 2 (FIFO size 8 bytes) linear addressing mode        */
    SPI_DMA_CHANNEL_3  = GPDMA_CHANNEL_3,  /**< DMA transfer channel 3 (FIFO size 8 bytes) linear addressing mode        */
    SPI_DMA_CHANNEL_4  = GPDMA_CHANNEL_4,  /**< DMA transfer channel 4 (FIFO size 32 bytes) linear addressing mode       */
    SPI_DMA_CHANNEL_5  = GPDMA_CHANNEL_5,  /**< DMA transfer channel 5 (FIFO size 32 bytes) linear addressing mode       */
    SPI_DMA_CHANNEL_6  = GPDMA_CHANNEL_6,  /**< DMA transfer channel 6 (FIFO size 32 bytes) linear or 2D addressing mode */
    SPI_DMA_CHANNEL_7  = GPDMA_CHANNEL_7,  /**< DMA transfer channel 7 (FIFO size 32 bytes) linear or 2D addressing mode */
    SPI_DMA_CHANNEL_CNT                    /**< Count of available DMA channels                                          */
}   spi_DmaChannelId_t;


/** Channel priority options enumeration */
typedef enum
{
    SPI_DMA_PRIORITY_LOW      = GPDMA_PRIORITY_LOW     , /**< Priority level : Low       */
    SPI_DMA_PRIORITY_MEDIUM   = GPDMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    SPI_DMA_PRIORITY_HIGH     = GPDMA_PRIORITY_HIGH    , /**< Priority level : High      */
    SPI_DMA_PRIORITY_VERYHIGH = GPDMA_PRIORITY_VERYHIGH, /**< Priority level : Very_High */
}   spi_DmaPriority_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief List of data transfer modes
 *
 * All modes use the same transfer request and report the same events through the same
 * callbacks - they differ only in the context moving the data:
 * - DMA:  GPDMA channels (end of transfer and errors from SPI interrupt)
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


/** \brief List of data transfer errors reported through \ref spi_XferErrCallback_t */
typedef enum
{
    SPI_XFER_ERROR_NONE = 0u,           /**< No error (last transfer finished successfully)                */
    SPI_XFER_ERROR_OVERRUN,             /**< Received frame was lost - receive FIFO was full (OVR)         */
    SPI_XFER_ERROR_UNDERRUN,            /**< Slave had no frame to transmit when clocked by master (UDR)   */
    SPI_XFER_ERROR_CRC,                 /**< Received CRC does not match the calculated one (CRCE)         */
    SPI_XFER_ERROR_MODE_FAULT,          /**< Master was deselected by NSS input (MODF)                     */
    SPI_XFER_ERROR_FRAME,               /**< TI frame format error - misplaced NSS pulse (TIFRE)           */
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
 * in SPI_XFER_MODE_DMA (transmit and receive channel must differ), IrqPriority is used in DMA
 * and ISR mode.
 */
typedef struct
{
    spi_XferMode_t          XferMode;             /**< Data transfer mode (NONE / DMA / ISR / POLL)     */
    spi_DmaPeriphId_t       TxDmaPeriphId;        /**< DMA peripheral (transmission in DMA mode)        */
    spi_DmaChannelId_t      TxDmaChannelId;       /**< DMA channel (transmission in DMA mode)           */
    spi_DmaPriority_t       TxDmaPriority;        /**< DMA channel priority (transmission in DMA mode)  */
    spi_DmaPeriphId_t       RxDmaPeriphId;        /**< DMA peripheral (reception in DMA mode)           */
    spi_DmaChannelId_t      RxDmaChannelId;       /**< DMA channel (reception in DMA mode)              */
    spi_DmaPriority_t       RxDmaPriority;        /**< DMA channel priority (reception in DMA mode)     */
    spi_IrqPrio_t           IrqPriority;          /**< SPI interrupt priority (DMA / ISR mode)          */
    spi_XferCallback_t     *XferCompleteCallback; /**< Transfer complete. SPI_NULL_PTR if not used.     */
    spi_XferErrCallback_t  *ErrorCallback;        /**< Transfer error. SPI_NULL_PTR if not used.        */
}   spi_DataConfig_t;


/**
 * \brief Transfer request
 *
 * XferSize frames are transferred (1 - 65534 for SPI1 - SPI3, 1 - 1022 for SPI4 - SPI6). Every
 * frame occupies 1 / 2 / 4 bytes of the buffer (data size up to 8 / 16 / 32 bits).
 *
 * Use of buffers depends on the communication direction:
 * - Full-duplex: at least one buffer. TxData = SPI_NULL_PTR - zero frames are transmitted,
 *                RxData = SPI_NULL_PTR - received frames are discarded.
 * - Simplex TX:  TxData is required, RxData is ignored.
 * - Simplex RX:  RxData is required, TxData is ignored.
 * - Half-duplex: exactly one buffer - TxData (transmission) or RxData (reception).
 *
 * \note  Buffers are not copied - they must stay valid until the end of the transfer. In DMA mode
 *        the buffers must be aligned to the frame size (2 / 4 bytes) and the transfer must not
 *        exceed 65535 bytes.
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
    spi_ClkSrc_t             ClkSrc;      /**< SPI kernel clock source (item of the peripheral PeriphId)      */
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
