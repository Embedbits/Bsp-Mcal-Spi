/**
 * \author Mr.Nobody
 * \file Spi.c
 * \ingroup Spi
 * \brief Serial Peripheral Interface (SPI) MCAL module common functionality
 *
 * The module drives SPI peripherals (SPI v2 IP of STM32H5) in master or slave mode:
 * - configuration: kernel clock source, SCK frequency (baudrate prescaler is calculated from the
 *   kernel clock), clock mode, data size, bit order, communication direction, Motorola / TI
 *   frame format, NSS management, master idle timing, hardware CRC, SCK / MISO / MOSI / NSS pins
 * - transfers: full-duplex, simplex and half-duplex transfers of \ref spi_XferRequest_t
 *
 * Configuration registers (CFG1, CFG2, CR2.TSIZE) are writable only while the peripheral is
 * disabled - the peripheral is therefore enabled (SPE) for the duration of one transfer only:
 * - start:  TSIZE is programmed, mode resources are armed, SPE is set, master starts by CSTART
 * - EOT:    all frames were transferred (CRC is checked), the peripheral is disabled
 * - errors: OVR / UDR / MODF / TIFRE terminate the transfer immediately
 *
 */
/* ============================== INCLUDES ================================== */
#include "Spi.h"                            /* Module private interface       */
#include "Spi_Port.h"                       /* Own port file include          */
#include "Spi_Types.h"                      /* Module types definitions       */
#include "Spi_Dma.h"                        /* DMA data transfer handler      */
#include "Spi_Isr.h"                        /* ISR data transfer handler      */
#include "Spi_Poll.h"                       /* Polling data transfer handler  */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
#include "Rcc_Port.h"                       /* RCC Mcal layer include         */
#include "Gpio_Port.h"                      /* GPIO Mcal layer include        */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
/* ============================== TYPEDEFS ================================== */

/** \brief Type representing size of SPI FIFO in bytes */
typedef uint8_t spi_FifoBytes_t;


/** \brief Configuration of one SPI peripheral */
typedef struct
{
    SPI_TypeDef          *PeriphReg;                    /**< Peripheral registers                                    */
    rcc_PeriphId_t        PeriphRcc[ SPI_CLK_SRC_ID_CNT ]; /**< RCC identification per kernel clock source
                                                             (RCC_PERIPH_ID_CNT - source not available)               */
    rcc_PeriphId_t        PeriphRccBase;                /**< RCC identification used for reset / clock disable       */
    nvic_PeriphIrqList_t  PeriphNvic;                   /**< Interrupt NVIC identification                           */
    nvic_IsrCallback_t    PeriphIsr;                    /**< Interrupt service routine                               */
    gpdma_PeriphReqId_t   PeriphDmaTxReq;               /**< DMA request ID for transmission                         */
    gpdma_PeriphReqId_t   PeriphDmaRxReq;               /**< DMA request ID for reception                            */
    spi_FunctionState_t   Limited;                      /**< Limited feature set (16-bit data / CRC 8 or 16 bits)    */
    spi_DataSize_t        DataSizeMax;                  /**< Maximum data size                                       */
    spi_DataCnt_t         XferSizeMax;                  /**< Maximum count of frames of one transfer (TSIZE)         */
    spi_FifoBytes_t       FifoBytes;                    /**< Size of transmit / receive FIFO in bytes                */
}   spi_PeriphConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

#ifdef SPI1
static void Spi_Spi1_IsrHandler( void );
#endif /* SPI1 */
#ifdef SPI2
static void Spi_Spi2_IsrHandler( void );
#endif /* SPI2 */
#ifdef SPI3
static void Spi_Spi3_IsrHandler( void );
#endif /* SPI3 */
#ifdef SPI4
static void Spi_Spi4_IsrHandler( void );
#endif /* SPI4 */
#ifdef SPI5
static void Spi_Spi5_IsrHandler( void );
#endif /* SPI5 */
#ifdef SPI6
static void Spi_Spi6_IsrHandler( void );
#endif /* SPI6 */

static spi_RequestState_t  Spi_Check_Config         ( const spi_Config_t * const spiConfig );
static spi_RequestState_t  Spi_Check_Pin            ( spi_PeriphId_t periphId, spi_PinCode_t pinCode );
static spi_RequestState_t  Spi_Check_NssConfig      ( const spi_NssConfig_t * const nssConfig );
static spi_RequestState_t  Spi_Check_ConfigAllowed  ( spi_PeriphId_t periphId );
static spi_RequestState_t  Spi_Check_XferRequest    ( spi_PeriphId_t periphId, const spi_XferRequest_t * const xferRequest );
static spi_RequestState_t  Spi_Check_DataConfig     ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );

static spi_RequestState_t  Spi_Set_Enable           ( spi_PeriphId_t periphId, spi_FunctionState_t enableState );
static spi_RequestState_t  Spi_Set_SsLevel          ( spi_PeriphId_t periphId, spi_RegValue_t cfgValue );
static spi_RequestState_t  Spi_Set_Pin              ( spi_PinCode_t pinCode, spi_PinSpeed_t pinSpeed );
static spi_RequestState_t  Spi_Get_KernelClk        ( spi_PeriphId_t periphId, spi_FreqHz_t * const clkFreq );
static spi_FrameBytes_t    Spi_Get_FrameBytes       ( spi_DataSize_t dataSize );

static spi_RequestState_t  Spi_Set_XferInit         ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
static spi_RequestState_t  Spi_Set_XferDeinit       ( spi_PeriphId_t periphId );
static spi_FunctionState_t Spi_Get_IrqUsed          ( const spi_DataConfig_t * const dataConfig );
static spi_RequestState_t  Spi_Set_IrqInit          ( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio );
static spi_RequestState_t  Spi_Set_IrqDeinit        ( spi_PeriphId_t periphId );

static spi_RequestState_t  Spi_Set_XferEnd          ( spi_PeriphId_t periphId, spi_XferErrorId_t errorId );
static spi_RequestState_t  Spi_Set_XferAbort        ( spi_PeriphId_t periphId );
static void                Spi_Set_FrameWrite       ( spi_PeriphId_t periphId );
static void                Spi_Get_FrameRead        ( spi_PeriphId_t periphId );

static spi_RequestState_t  Spi_None_Check_Config    ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
static spi_RequestState_t  Spi_None_Xfer            ( spi_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define SPI_MAJOR_VERSION               ( 1u )

/** Value of minor version of SW module */
#define SPI_MINOR_VERSION               ( 0u )

/** Value of patch version of SW module */
#define SPI_PATCH_VERSION               ( 0u )


/** Default SCK frequency used by \ref Spi_Get_DefaultConfig */
#define SPI_DEFAULT_BUS_FREQ_HZ         ( 1000000u )

/** Offset between \ref spi_DataSize_t value and DSIZE / CRCSIZE field value (4 bits -> 3) */
#define SPI_DATA_SIZE_FIELD_OFFSET      ( 3u )

/** Offset between \ref spi_DataSize_t value and count of bits (SPI_DATA_SIZE_4BIT -> 4) */
#define SPI_DATA_SIZE_BITS_OFFSET       ( 4u )

/** Count of bits of the widest CRC (polynomial range check) */
#define SPI_CRC_BITS_MAX                ( 32u )

/** Count of baudrate prescaler values (MBR field, division by 2 - 256) */
#define SPI_PRESC_CNT                   ( 8u )

/** Offset between MBR field value and division shift (MBR = 0 -> division by 2) */
#define SPI_PRESC_SHIFT_OFFSET          ( 1u )

/** Maximum count of frames of one transfer - full feature set instance (SPI1 - SPI3) */
#define SPI_XFER_SIZE_MAX_FULL          ( 0xFFFEu )

/** Maximum count of frames of one transfer - limited feature set instance (SPI4 - SPI6) */
#define SPI_XFER_SIZE_MAX_LIMITED       ( 0x3FEu )

/** FIFO size in bytes - full feature set instance (SPI1 - SPI3) */
#define SPI_FIFO_BYTES_FULL             ( 16u )

/** FIFO size in bytes - limited feature set instance (SPI4 - SPI6) */
#define SPI_FIFO_BYTES_LIMITED          ( 8u )

/** Buffer bytes of frame up to 8 bits */
#define SPI_FRAME_BYTES_8BIT            ( 1u )

/** Buffer bytes of frame up to 16 bits */
#define SPI_FRAME_BYTES_16BIT           ( 2u )

/** Buffer bytes of frame up to 32 bits */
#define SPI_FRAME_BYTES_32BIT           ( 4u )

/** Count of bits in one byte (frame assembly) */
#define SPI_BITS_PER_BYTE               ( 8u )

/* =============================== MACROS =================================== */

/** All status flags cleared through IFCR */
#define SPI_IFCR_ALL                    ( SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC  | SPI_IFCR_OVRC | \
                                          SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC )

/** CFG2 fields of NSS management */
#define SPI_CFG2_NSS_MASK               ( SPI_CFG2_SSM | SPI_CFG2_SSOE | SPI_CFG2_SSIOP | SPI_CFG2_SSOM )

/** CFG2 fields of master / slave role (alternate function control keeps pins driven while SPE = 0) */
#define SPI_CFG2_MODE_MASK              ( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR )

/** CFG2 fields of master idle timing */
#define SPI_CFG2_TIMING_MASK            ( SPI_CFG2_MIDI | SPI_CFG2_MSSI )

/** CFG1 fields of baudrate prescaler */
#define SPI_CFG1_PRESC_MASK             ( SPI_CFG1_MBR | SPI_CFG1_BPASS )

/** CR1 CRC initialization pattern fields (transmitter and receiver) */
#define SPI_CR1_CRCINI_MASK             ( SPI_CR1_TCRCINI | SPI_CR1_RCRCINI )

/** PLL3 kernel clock of SPI1 - SPI4 (devices without PLL3 do not offer it) */
#if defined(RCC_CR_PLL3ON)
#define SPI_RCC_SPI1_PLL3               ( RCC_PERIPH_SPI1_PLL3P )
#define SPI_RCC_SPI2_PLL3               ( RCC_PERIPH_SPI2_PLL3P )
#define SPI_RCC_SPI3_PLL3               ( RCC_PERIPH_SPI3_PLL3P )
#define SPI_RCC_SPI4_PLL3               ( RCC_PERIPH_SPI4_PLL3Q )
#else
#define SPI_RCC_SPI1_PLL3               ( RCC_PERIPH_ID_CNT )
#define SPI_RCC_SPI2_PLL3               ( RCC_PERIPH_ID_CNT )
#define SPI_RCC_SPI3_PLL3               ( RCC_PERIPH_ID_CNT )
#define SPI_RCC_SPI4_PLL3               ( RCC_PERIPH_ID_CNT )
#endif

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief SPI peripherals configuration array */
static const spi_PeriphConfigStruct_t spi_PeriphConf[ ] =
{
#ifdef SPI1
    { .PeriphReg      = SPI1,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_SPI1_PLL1Q, [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI1_PLL2P, [SPI_CLK_SRC_ID_PLL3] = SPI_RCC_SPI1_PLL3,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_ID_CNT,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_ID_CNT },
      .PeriphRccBase  = RCC_PERIPH_SPI1_PLL1Q,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI1,    .PeriphIsr      = Spi_Spi1_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI1_TX,       .PeriphDmaRxReq = GPDMA_REQ_SPI1_RX,
      .Limited        = SPI_FUNCTION_INACTIVE,   .DataSizeMax    = SPI_DATA_SIZE_32BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_FULL,  .FifoBytes      = SPI_FIFO_BYTES_FULL },
#endif
#ifdef SPI2
    { .PeriphReg      = SPI2,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_SPI2_PLL1Q, [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI2_PLL2P, [SPI_CLK_SRC_ID_PLL3] = SPI_RCC_SPI2_PLL3,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_ID_CNT,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_ID_CNT },
      .PeriphRccBase  = RCC_PERIPH_SPI2_PLL1Q,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI2,    .PeriphIsr      = Spi_Spi2_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI2_TX,       .PeriphDmaRxReq = GPDMA_REQ_SPI2_RX,
      .Limited        = SPI_FUNCTION_INACTIVE,   .DataSizeMax    = SPI_DATA_SIZE_32BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_FULL,  .FifoBytes      = SPI_FIFO_BYTES_FULL },
#endif
#ifdef SPI3
    { .PeriphReg      = SPI3,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_SPI3_PLL1Q, [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI3_PLL2P, [SPI_CLK_SRC_ID_PLL3] = SPI_RCC_SPI3_PLL3,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_ID_CNT,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_ID_CNT },
      .PeriphRccBase  = RCC_PERIPH_SPI3_PLL1Q,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI3,    .PeriphIsr      = Spi_Spi3_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI3_TX,       .PeriphDmaRxReq = GPDMA_REQ_SPI3_RX,
      .Limited        = SPI_FUNCTION_INACTIVE,   .DataSizeMax    = SPI_DATA_SIZE_32BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_FULL,  .FifoBytes      = SPI_FIFO_BYTES_FULL },
#endif
#ifdef SPI4
    { .PeriphReg      = SPI4,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI4_PLL2Q, [SPI_CLK_SRC_ID_PLL3] = SPI_RCC_SPI4_PLL3,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_SPI4_PCLK2, [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_SPI4_HSI64, [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_SPI4_CSI,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_SPI4_HSE },
      .PeriphRccBase  = RCC_PERIPH_SPI4_PCLK2,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI4,      .PeriphIsr      = Spi_Spi4_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI4_TX,         .PeriphDmaRxReq = GPDMA_REQ_SPI4_RX,
      .Limited        = SPI_FUNCTION_ACTIVE,       .DataSizeMax    = SPI_DATA_SIZE_16BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_LIMITED, .FifoBytes      = SPI_FIFO_BYTES_LIMITED },
#endif
#ifdef SPI5
    { .PeriphReg      = SPI5,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI5_PLL2Q, [SPI_CLK_SRC_ID_PLL3] = RCC_PERIPH_SPI5_PLL3Q,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_SPI5_PCLK3, [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_SPI5_HSI64, [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_SPI5_CSI,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_SPI5_HSE },
      .PeriphRccBase  = RCC_PERIPH_SPI5_PCLK3,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI5,      .PeriphIsr      = Spi_Spi5_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI5_TX,         .PeriphDmaRxReq = GPDMA_REQ_SPI5_RX,
      .Limited        = SPI_FUNCTION_ACTIVE,       .DataSizeMax    = SPI_DATA_SIZE_16BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_LIMITED, .FifoBytes      = SPI_FIFO_BYTES_LIMITED },
#endif
#ifdef SPI6
    { .PeriphReg      = SPI6,
      .PeriphRcc      = { [SPI_CLK_SRC_ID_PLL1Q] = RCC_PERIPH_ID_CNT,     [SPI_CLK_SRC_ID_PLL2] = RCC_PERIPH_SPI6_PLL2Q, [SPI_CLK_SRC_ID_PLL3] = RCC_PERIPH_SPI6_PLL3Q,
                          [SPI_CLK_SRC_ID_PCLK]  = RCC_PERIPH_SPI6_PCLK2, [SPI_CLK_SRC_ID_HSI]  = RCC_PERIPH_SPI6_HSI64, [SPI_CLK_SRC_ID_CSI]  = RCC_PERIPH_SPI6_CSI,
                          [SPI_CLK_SRC_ID_HSE]   = RCC_PERIPH_SPI6_HSE },
      .PeriphRccBase  = RCC_PERIPH_SPI6_PCLK2,
      .PeriphNvic     = NVIC_PERIPH_IRQ_SPI6,      .PeriphIsr      = Spi_Spi6_IsrHandler,
      .PeriphDmaTxReq = GPDMA_REQ_SPI6_TX,         .PeriphDmaRxReq = GPDMA_REQ_SPI6_RX,
      .Limited        = SPI_FUNCTION_ACTIVE,       .DataSizeMax    = SPI_DATA_SIZE_16BIT,
      .XferSizeMax    = SPI_XFER_SIZE_MAX_LIMITED, .FifoBytes      = SPI_FIFO_BYTES_LIMITED },
#endif
};

_Static_assert( SPI_PERIPH_CNT == ( sizeof(spi_PeriphConf) / sizeof(spi_PeriphConfigStruct_t) ), "Spi: size of spi_PeriphConf is incorrect." );


/** \brief spi_ClockMode_t -> CFG2 CPOL / CPHA */
static const spi_RegValue_t spi_ClockModeLut[ SPI_CLOCK_MODE_CNT ] =
{
    [SPI_CLOCK_MODE_0] = ( LL_SPI_POLARITY_LOW  | LL_SPI_PHASE_1EDGE ),
    [SPI_CLOCK_MODE_1] = ( LL_SPI_POLARITY_LOW  | LL_SPI_PHASE_2EDGE ),
    [SPI_CLOCK_MODE_2] = ( LL_SPI_POLARITY_HIGH | LL_SPI_PHASE_1EDGE ),
    [SPI_CLOCK_MODE_3] = ( LL_SPI_POLARITY_HIGH | LL_SPI_PHASE_2EDGE ),
};


/** \brief spi_Direction_t -> CFG2 COMM (half-duplex direction HDDIR is set per transfer) */
static const spi_RegValue_t spi_DirectionLut[ SPI_DIRECTION_CNT ] =
{
    [SPI_DIRECTION_FULL_DUPLEX] = LL_SPI_FULL_DUPLEX,
    [SPI_DIRECTION_SIMPLEX_TX]  = LL_SPI_SIMPLEX_TX,
    [SPI_DIRECTION_SIMPLEX_RX]  = LL_SPI_SIMPLEX_RX,
    [SPI_DIRECTION_HALF_DUPLEX] = SPI_CFG2_COMM,
};


/** \brief spi_FrameFormat_t -> CFG2 SP */
static const spi_RegValue_t spi_FrameFormatLut[ SPI_FRAME_FORMAT_CNT ] =
{
    [SPI_FRAME_FORMAT_MOTOROLA] = LL_SPI_PROTOCOL_MOTOROLA,
    [SPI_FRAME_FORMAT_TI]       = LL_SPI_PROTOCOL_TI,
};


/** \brief spi_PinSpeed_t -> GPIO output speed */
static const gpio_PinSpeed_t spi_PinSpeedLut[ SPI_PIN_SPEED_CNT ] =
{
    [SPI_PIN_SPEED_LOW]       = GPIO_PIN_SPEED_LOW,
    [SPI_PIN_SPEED_MEDIUM]    = GPIO_PIN_SPEED_MEDIUM,
    [SPI_PIN_SPEED_HIGH]      = GPIO_PIN_SPEED_HIGH,
    [SPI_PIN_SPEED_VERY_HIGH] = GPIO_PIN_SPEED_VERY_HIGH,
};


/** \brief spi_XferMode_t -> data transfer mode handler */
static const spi_XferModeIf_t spi_XferModeLut[ SPI_XFER_MODE_CNT ] =
{
    [SPI_XFER_MODE_NONE] = { .CheckConfig = Spi_None_Check_Config, .Init = Spi_None_Xfer,     .Deinit = Spi_None_Xfer,       .Start = Spi_None_Xfer,      .Run = Spi_None_Xfer,      .Stop = Spi_None_Xfer,      .CheckDone = Spi_None_Xfer      },
    [SPI_XFER_MODE_DMA]  = { .CheckConfig = Spi_Dma_Check_Config,  .Init = Spi_Dma_XferInit,  .Deinit = Spi_Dma_XferDeinit,  .Start = Spi_Dma_XferStart,  .Run = Spi_Dma_XferRun,    .Stop = Spi_Dma_XferStop,   .CheckDone = Spi_Dma_Check_Done },
    [SPI_XFER_MODE_ISR]  = { .CheckConfig = Spi_Isr_Check_Config,  .Init = Spi_Isr_XferInit,  .Deinit = Spi_Isr_XferDeinit,  .Start = Spi_Isr_XferStart,  .Run = Spi_Isr_XferRun,    .Stop = Spi_Isr_XferStop,   .CheckDone = Spi_None_Xfer      },
    [SPI_XFER_MODE_POLL] = { .CheckConfig = Spi_Poll_Check_Config, .Init = Spi_Poll_XferInit, .Deinit = Spi_Poll_XferDeinit, .Start = Spi_Poll_XferStart, .Run = Spi_None_Xfer,      .Stop = Spi_Poll_XferStop,  .CheckDone = Spi_None_Xfer      },
};


/** \brief Data handling runtime context per peripheral */
static spi_XferContext_t spi_XferContext[ SPI_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
spi_ModuleVersion_t Spi_Get_ModuleVersion( void )
{
    spi_ModuleVersion_t retVersion;

    retVersion.Major = SPI_MAJOR_VERSION;
    retVersion.Minor = SPI_MINOR_VERSION;
    retVersion.Patch = SPI_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief SPI peripheral initialization through configuration structure
 *
 * Data handling of a previous initialization is released, the kernel clock source is selected
 * and the clock enabled, the peripheral is reset, pins are configured, the peripheral is
 * configured (it stays disabled until a transfer is started) and the data handling is
 * initialized (if DataConfig is set). CRC is disabled and master idle timing is zero after the
 * initialization (Spi_Set_CrcConfig() / Spi_Set_MasterTiming()).
 *
 * \param spiConfig [in]: Pointer to configuration structure. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Init( const spi_Config_t * const spiConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Check_Config( spiConfig );

    if( SPI_REQUEST_OK == retState )
    {
        const spi_PeriphId_t periphId = spiConfig->PeriphId;
        const rcc_PeriphId_t rccId    = spi_PeriphConf[ periphId ].PeriphRcc[ SPI_CLK_SRC_BIT_MASK_DECODE_SOURCE( spiConfig->ClkSrc ) ];
        rcc_RequestState_t   rccState = RCC_REQUEST_ERROR;

        /*------------- Data handling of previous initialization -------------*/
        retState = Spi_Set_XferDeinit( periphId );

        /*------- Kernel clock source selection and clock activation --------*/
        if( SPI_REQUEST_OK == retState )
        {
            /* Clock multiplexer is configured by every activation request */
            rccState = Rcc_Set_PeriphActive( rccId );

            if( RCC_REQUEST_OK == rccState )
            {
                rccState = Rcc_Set_ResetActive( rccId );
            }
            else
            {
                /* Peripheral clock activation failed */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                rccState = Rcc_Set_ResetInactive( rccId );
            }
            else
            {
                /* Peripheral reset failed */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                retState = SPI_REQUEST_OK;
            }
            else
            {
                retState = SPI_REQUEST_ERROR;
            }
        }
        else
        {
            /* Data handling of previous initialization could not be released */
        }

        /*------------- Peripheral configuration (SPE = 0 after reset) -------*/
        /* NSS management before the role - with the reset NSS configuration (hardware NSS input)
         * the internal slave select of a master is active, HW signals mode fault (MODF) and
         * clears MASTER */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_NssConfig( periphId, &spiConfig->NssConfig );
        }
        else
        {
            /* Error during initialization process */
        }

        /* Role - alternate function control keeps master pins driven from now on */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Mode( periphId, spiConfig->Mode );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_FrameFormat( periphId, spiConfig->FrameFormat );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_ClockMode( periphId, spiConfig->ClockMode );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_DataSize( periphId, spiConfig->DataSize );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_BitOrder( periphId, spiConfig->BitOrder );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Direction( periphId, spiConfig->Direction );
        }
        else
        {
            /* Error during initialization process */
        }

        /* SCK is generated by the master only */
        if( ( SPI_REQUEST_OK  == retState        ) &&
            ( SPI_MODE_MASTER == spiConfig->Mode )    )
        {
            retState = Spi_Set_BusFreq( periphId, spiConfig->BusFreq );
        }
        else
        {
            /* Error during initialization process or slave mode */
        }

        /*------------------------ GPIO pins ---------------------------------*/
        /* Pins are configured after the role - master SCK idle level is already driven */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->SckPin, spiConfig->PinSpeed );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->MisoPin, spiConfig->PinSpeed );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->MosiPin, spiConfig->PinSpeed );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->NssPin, spiConfig->PinSpeed );
        }
        else
        {
            /* Error during initialization process */
        }

        /*------------------ Data handling initialization --------------------*/
        if( ( SPI_REQUEST_OK == retState              ) &&
            ( SPI_NULL_PTR  != spiConfig->DataConfig  )    )
        {
            retState = Spi_Set_DataConfig( periphId, spiConfig->DataConfig );
        }
        else
        {
            /* Error during initialization process or data handling is not used */
        }
    }
    else
    {
        /* Configuration is invalid, nothing is modified */
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief SPI peripheral de-initialization
 *
 * Running transfer is aborted, data handling resources (GPDMA channels, SPI interrupt) are
 * released, the peripheral is disabled, reset and its clock is disabled. All steps are
 * executed, any failure is reported. GPIO pins are not changed.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Deinit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        const rcc_PeriphId_t rccId = spi_PeriphConf[ periphId ].PeriphRccBase;

        /* -1- Data handling (transfer aborted, GPDMA channels and SPI interrupt released) */
        const spi_RequestState_t xferState = Spi_Set_XferDeinit( periphId );

        /* -2- Peripheral */
        const spi_RequestState_t periphState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );

        /* -3- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( rccId );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( rccId );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( rccId );

        if( ( SPI_REQUEST_OK == xferState     ) &&
            ( SPI_REQUEST_OK == periphState   ) &&
            ( RCC_REQUEST_OK == rstActState   ) &&
            ( RCC_REQUEST_OK == rstInactState ) &&
            ( RCC_REQUEST_OK == clkState      )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module Spi
 *
 * This function shall be called in the main loop of the application or the task scheduler.
 * It moves data of running transfers in SPI_XFER_MODE_POLL.
 *
 * \note  Master transmits only when its transmit FIFO contains data - a slow task only slows the
 *        transfer down. Simplex RX master and slave transfers are clocked independently of the
 *        task - overrun / underrun is reported if the task is not called often enough.
 */
void Spi_Task( void )
{
    for( spi_PeriphId_t periphId = (spi_PeriphId_t)0u; SPI_PERIPH_CNT > periphId; periphId ++ )
    {
        const spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];

        if( ( SPI_FUNCTION_ACTIVE == xferCtx->InitState       ) &&
            ( SPI_XFER_MODE_POLL  == xferCtx->Config.XferMode ) &&
            ( SPI_FUNCTION_ACTIVE == xferCtx->XferState       )    )
        {
            (void)Spi_Poll_Task( periphId );
        }
        else
        {
            /* No polling transfer is running on the peripheral */
        }
    }
}


/**
 * \brief Initialization of configuration structure to default values
 *
 * SPI_PERIPH_1, PLL1Q kernel clock, master, 1 MHz, clock mode 0, 8-bit frames, MSB first,
 * full-duplex, Motorola format, software NSS (active low, no pulse), no data handling, pins not
 * configured, high pin speed.
 *
 * \param spiConfig [out]: Pointer to configuration structure. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_DefaultConfig( spi_Config_t * const spiConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_NULL_PTR != spiConfig )
    {
        spiConfig->PeriphId           = (spi_PeriphId_t)0u;
        spiConfig->ClkSrc             = SPI_CLK_SRC_SPI1_PLL1Q;
        spiConfig->Mode               = SPI_MODE_MASTER;
        spiConfig->BusFreq            = SPI_DEFAULT_BUS_FREQ_HZ;
        spiConfig->ClockMode          = SPI_CLOCK_MODE_0;
        spiConfig->DataSize           = SPI_DATA_SIZE_8BIT;
        spiConfig->BitOrder           = SPI_BIT_ORDER_MSB_FIRST;
        spiConfig->Direction          = SPI_DIRECTION_FULL_DUPLEX;
        spiConfig->FrameFormat        = SPI_FRAME_FORMAT_MOTOROLA;
        spiConfig->NssConfig.Mode     = SPI_NSS_MODE_SOFT;
        spiConfig->NssConfig.Polarity = SPI_NSS_POLARITY_LOW;
        spiConfig->NssConfig.Pulse    = SPI_FUNCTION_INACTIVE;
        spiConfig->DataConfig         = SPI_NULL_PTR;
        spiConfig->SckPin             = SPI_SCK_PIN_UNUSED;
        spiConfig->MisoPin            = SPI_MISO_PIN_UNUSED;
        spiConfig->MosiPin            = SPI_MOSI_PIN_UNUSED;
        spiConfig->NssPin             = SPI_NSS_PIN_UNUSED;
        spiConfig->PinSpeed           = SPI_PIN_SPEED_HIGH;

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/**
 * \brief Reads SPI peripheral activation state (SPE) - the peripheral is enabled only while a
 *        transfer is running
 *
 * \param periphId     [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param periphState [out]: Pointer to store the activation state (\ref spi_FlagState_t).
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_PeriphState( spi_PeriphId_t periphId, spi_FlagState_t * const periphState )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId    ) &&
        ( SPI_NULL_PTR  != periphState )    )
    {
        const uint32_t regValue = LL_SPI_IsEnabled( spi_PeriphConf[ periphId ].PeriphReg );

        if( 0u != regValue )
        {
            *periphState = SPI_FLAG_ACTIVE;
        }
        else
        {
            *periphState = SPI_FLAG_INACTIVE;
        }

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures master / slave role
 *
 * \note  Master keeps control of its pins while the peripheral is disabled between transfers
 *        (AFCNTR) - SCK stays at the idle level of the clock mode.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param mode     [in]: Required role, value from \ref spi_Mode_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_Mode( spi_PeriphId_t periphId, spi_Mode_t mode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_MODE_CNT > mode )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        spi_RegValue_t      modeValue = 0u;

        if( SPI_MODE_MASTER == mode )
        {
            modeValue = SPI_CFG2_MODE_MASK;
        }
        else
        {
            /* Slave - pins are released while the peripheral is disabled */
        }

        /* Internal slave select level of the new role is applied before the role - a master
         * with active internal slave select signals mode fault (MODF), HW clears MASTER */
        const spi_RegValue_t cfgValue = ( LL_SPI_ReadReg( periphReg, CFG2 ) & ~SPI_CFG2_MODE_MASK ) | modeValue;

        retState = Spi_Set_SsLevel( periphId, cfgValue );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_RegField( &periphReg->CFG2, SPI_CFG2_MODE_MASK, modeValue );
        }
        else
        {
            /* Internal slave select configuration failed */
        }
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads master / slave role
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param mode    [out]: Pointer to store the role (\ref spi_Mode_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_Mode( spi_PeriphId_t periphId, spi_Mode_t * const mode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != mode     )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_MASTER );

        if( 0u != regValue )
        {
            *mode = SPI_MODE_MASTER;
        }
        else
        {
            *mode = SPI_MODE_SLAVE;
        }

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures SCK frequency - the smallest baudrate prescaler (division by 2 - 256) giving
 *        a frequency not higher than the required one is selected
 *
 * \note  Applies to master mode only. The kernel clock is read from RCC - after its change the
 *        frequency has to be configured again.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param busFreq  [in]: Required SCK frequency in Hz (non-zero)
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if the frequency is lower than
 *         kernel clock / 256) returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_BusFreq( spi_PeriphId_t periphId, spi_FreqHz_t busFreq )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_FreqHz_t       clkFreq  = 0u;

    if( 0u < busFreq )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_KernelClk( periphId, &clkFreq );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = SPI_REQUEST_ERROR;

        for( spi_RegValue_t prescIdx = 0u; SPI_PRESC_CNT > prescIdx; prescIdx ++ )
        {
            const spi_FreqHz_t sckFreq = clkFreq >> ( prescIdx + SPI_PRESC_SHIFT_OFFSET );

            if( busFreq >= sckFreq )
            {
                retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG1,
                                             SPI_CFG1_PRESC_MASK,
                                             ( prescIdx << SPI_CFG1_MBR_Pos ) & SPI_CFG1_MBR );
                break;
            }
            else
            {
                /* Frequency is too high, try the next prescaler */
            }
        }
    }
    else
    {
        /* Kernel clock frequency is not available */
    }

    return ( retState );
}


/**
 * \brief Returns SCK frequency (kernel clock divided by the configured prescaler)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param busFreq [out]: Pointer to store the SCK frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_BusFreq( spi_PeriphId_t periphId, spi_FreqHz_t * const busFreq )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_FreqHz_t       clkFreq  = 0u;

    if( SPI_NULL_PTR != busFreq )
    {
        retState = Spi_Get_KernelClk( periphId, &clkFreq );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_RegValue_t prescIdx = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG1, SPI_CFG1_MBR ) >> SPI_CFG1_MBR_Pos;

        *busFreq = clkFreq >> ( prescIdx + SPI_PRESC_SHIFT_OFFSET );
    }
    else
    {
        /* Kernel clock frequency is not available */
    }

    return ( retState );
}


/**
 * \brief Configures clock polarity and phase
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param clockMode [in]: Required clock mode, value from \ref spi_ClockMode_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_ClockMode( spi_PeriphId_t periphId, spi_ClockMode_t clockMode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_CLOCK_MODE_CNT > clockMode )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG2,
                                     ( SPI_CFG2_CPOL | SPI_CFG2_CPHA ),
                                     spi_ClockModeLut[ clockMode ] );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads clock polarity and phase
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param clockMode [out]: Pointer to store the clock mode (\ref spi_ClockMode_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_ClockMode( spi_PeriphId_t periphId, spi_ClockMode_t * const clockMode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != clockMode )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, ( SPI_CFG2_CPOL | SPI_CFG2_CPHA ) );

        for( spi_ClockMode_t modeIdx = SPI_CLOCK_MODE_0; SPI_CLOCK_MODE_CNT > modeIdx; modeIdx ++ )
        {
            if( spi_ClockModeLut[ modeIdx ] == regValue )
            {
                *clockMode = modeIdx;
                retState   = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock mode does not match, keep searching */
                retState = SPI_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures size of data frame
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataSize [in]: Required data size, value from \ref spi_DataSize_t. Must not exceed the
 *                       maximum of the peripheral and the CRC size (if CRC is enabled).
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_DataSize( spi_PeriphId_t periphId, spi_DataSize_t dataSize )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Check_ConfigAllowed( periphId );

    if( ( SPI_REQUEST_OK == retState                              ) &&
        ( spi_PeriphConf[ periphId ].DataSizeMax >= dataSize      )    )
    {
        SPI_TypeDef * const  periphReg  = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t crcEnabled = READ_BIT( periphReg->CFG1, SPI_CFG1_CRCEN );
        const spi_RegValue_t crcField   = READ_BIT( periphReg->CFG1, SPI_CFG1_CRCSIZE ) >> SPI_CFG1_CRCSIZE_Pos;
        const spi_RegValue_t dataField  = (spi_RegValue_t)dataSize + SPI_DATA_SIZE_FIELD_OFFSET;

        if( ( 0u == crcEnabled ) || ( crcField >= dataField ) )
        {
            retState = Spi_Set_RegField( &periphReg->CFG1, SPI_CFG1_DSIZE, dataField << SPI_CFG1_DSIZE_Pos );
        }
        else
        {
            /* CRC can not be shorter than the data frame */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads size of data frame
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataSize [out]: Pointer to store the data size (\ref spi_DataSize_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_DataSize( spi_PeriphId_t periphId, spi_DataSize_t * const dataSize )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != dataSize )    )
    {
        const spi_RegValue_t dataField = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG1, SPI_CFG1_DSIZE ) >> SPI_CFG1_DSIZE_Pos;

        if( ( SPI_DATA_SIZE_FIELD_OFFSET                                          <= dataField ) &&
            ( ( (spi_RegValue_t)SPI_DATA_SIZE_CNT + SPI_DATA_SIZE_FIELD_OFFSET )   > dataField )    )
        {
            *dataSize = (spi_DataSize_t)( dataField - SPI_DATA_SIZE_FIELD_OFFSET );
            retState  = SPI_REQUEST_OK;
        }
        else
        {
            /* DSIZE values below 4 bits are not allowed */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures bit order of data frame
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param bitOrder [in]: Required bit order, value from \ref spi_BitOrder_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_BitOrder( spi_PeriphId_t periphId, spi_BitOrder_t bitOrder )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_BIT_ORDER_CNT > bitOrder )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        spi_RegValue_t orderValue = LL_SPI_MSB_FIRST;

        if( SPI_BIT_ORDER_LSB_FIRST == bitOrder )
        {
            orderValue = LL_SPI_LSB_FIRST;
        }
        else
        {
            /* Most significant bit first */
        }

        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_LSBFRST, orderValue );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads bit order of data frame
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param bitOrder [out]: Pointer to store the bit order (\ref spi_BitOrder_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_BitOrder( spi_PeriphId_t periphId, spi_BitOrder_t * const bitOrder )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != bitOrder )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_LSBFRST );

        if( 0u != regValue )
        {
            *bitOrder = SPI_BIT_ORDER_LSB_FIRST;
        }
        else
        {
            *bitOrder = SPI_BIT_ORDER_MSB_FIRST;
        }

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures communication direction
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param direction [in]: Required direction, value from \ref spi_Direction_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_Direction( spi_PeriphId_t periphId, spi_Direction_t direction )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_DIRECTION_CNT > direction )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_COMM, spi_DirectionLut[ direction ] );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads communication direction
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param direction [out]: Pointer to store the direction (\ref spi_Direction_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_Direction( spi_PeriphId_t periphId, spi_Direction_t * const direction )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != direction )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_COMM );

        for( spi_Direction_t dirIdx = SPI_DIRECTION_FULL_DUPLEX; SPI_DIRECTION_CNT > dirIdx; dirIdx ++ )
        {
            if( spi_DirectionLut[ dirIdx ] == regValue )
            {
                *direction = dirIdx;
                retState   = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Direction does not match, keep searching */
                retState = SPI_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures frame format (Motorola / TI)
 *
 * \note  In TI format clock mode, bit order and NSS behavior are given by the protocol - master
 *        has to use hardware NSS.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param frameFormat [in]: Required frame format, value from \ref spi_FrameFormat_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_FrameFormat( spi_PeriphId_t periphId, spi_FrameFormat_t frameFormat )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_FRAME_FORMAT_CNT > frameFormat )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_SP, spi_FrameFormatLut[ frameFormat ] );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads frame format
 *
 * \param periphId     [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param frameFormat [out]: Pointer to store the frame format (\ref spi_FrameFormat_t). Must not
 *                           be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_FrameFormat( spi_PeriphId_t periphId, spi_FrameFormat_t * const frameFormat )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId    ) &&
        ( SPI_NULL_PTR  != frameFormat )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_SP );

        for( spi_FrameFormat_t formatIdx = SPI_FRAME_FORMAT_MOTOROLA; SPI_FRAME_FORMAT_CNT > formatIdx; formatIdx ++ )
        {
            if( spi_FrameFormatLut[ formatIdx ] == regValue )
            {
                *frameFormat = formatIdx;
                retState     = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Frame format does not match, keep searching */
                retState = SPI_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ----------------------------- NSS management ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Configures NSS management (software / hardware, polarity, pulse between frames)
 *
 * \note  Software NSS: master does not use the NSS pin (slave select is driven by application
 *        GPIO), slave is permanently selected. Hardware NSS: master drives NSS active while the
 *        peripheral is enabled (one transfer), slave is selected by NSS input.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param nssConfig [in]: Pointer to NSS configuration. Must not be NULL, pulse requires
 *                        hardware NSS.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_NssConfig( spi_PeriphId_t periphId, const spi_NssConfig_t * const nssConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Check_NssConfig( nssConfig );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        /* Configuration is invalid */
    }

    if( SPI_REQUEST_OK == retState )
    {
        spi_RegValue_t nssValue = 0u;

        if( SPI_NSS_MODE_SOFT == nssConfig->Mode )
        {
            nssValue |= LL_SPI_NSS_SOFT;
        }
        else
        {
            /* Output is enabled for master only by HW (slave uses NSS as input) */
            nssValue |= LL_SPI_NSS_HARD_OUTPUT;
        }

        if( SPI_NSS_POLARITY_HIGH == nssConfig->Polarity )
        {
            nssValue |= LL_SPI_NSS_POLARITY_HIGH;
        }
        else
        {
            /* NSS is active low */
        }

        if( SPI_FUNCTION_ACTIVE == nssConfig->Pulse )
        {
            nssValue |= SPI_CFG2_SSOM;
        }
        else
        {
            /* NSS is kept active for the whole transfer */
        }

        SPI_TypeDef * const  periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t cfgValue  = ( LL_SPI_ReadReg( periphReg, CFG2 ) & ~SPI_CFG2_NSS_MASK ) | nssValue;

        /* Internal slave select level of the new NSS configuration is applied first - master
         * with software NSS must not see active internal slave select (mode fault) */
        retState = Spi_Set_SsLevel( periphId, cfgValue );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_RegField( &periphReg->CFG2, SPI_CFG2_NSS_MASK, nssValue );
        }
        else
        {
            /* Internal slave select configuration failed */
        }
    }
    else
    {
        /* Configuration is invalid or transfer is running, nothing is modified */
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads NSS management configuration
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param nssConfig [out]: Pointer to store the NSS configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_NssConfig( spi_PeriphId_t periphId, spi_NssConfig_t * const nssConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != nssConfig )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_NSS_MASK );

        nssConfig->Mode     = ( 0u != ( SPI_CFG2_SSM   & regValue ) ) ? SPI_NSS_MODE_SOFT     : SPI_NSS_MODE_HARD;
        nssConfig->Polarity = ( 0u != ( SPI_CFG2_SSIOP & regValue ) ) ? SPI_NSS_POLARITY_HIGH : SPI_NSS_POLARITY_LOW;
        nssConfig->Pulse    = ( 0u != ( SPI_CFG2_SSOM  & regValue ) ) ? SPI_FUNCTION_ACTIVE   : SPI_FUNCTION_INACTIVE;

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures master idle timing - idle SCK cycles between data frames (MIDI) and between
 *        NSS activation and the first SCK edge (MSSI)
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId      [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param interDataIdle [in]: Idle cycles between frames (0 - \ref SPI_IDLE_CYCLES_MAX)
 * \param ssIdle        [in]: Idle cycles after NSS activation (0 - \ref SPI_IDLE_CYCLES_MAX)
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_MasterTiming( spi_PeriphId_t periphId, spi_IdleCycles_t interDataIdle, spi_IdleCycles_t ssIdle )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_IDLE_CYCLES_MAX >= interDataIdle ) &&
        ( SPI_IDLE_CYCLES_MAX >= ssIdle        )    )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_RegValue_t timingValue = ( ( (spi_RegValue_t)interDataIdle << SPI_CFG2_MIDI_Pos ) & SPI_CFG2_MIDI ) |
                                           ( ( (spi_RegValue_t)ssIdle        << SPI_CFG2_MSSI_Pos ) & SPI_CFG2_MSSI );

        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_TIMING_MASK, timingValue );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads master idle timing
 *
 * \param periphId       [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param interDataIdle [out]: Pointer to store idle cycles between frames. Must not be NULL.
 * \param ssIdle        [out]: Pointer to store idle cycles after NSS activation. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_MasterTiming( spi_PeriphId_t periphId, spi_IdleCycles_t * const interDataIdle, spi_IdleCycles_t * const ssIdle )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId      ) &&
        ( SPI_NULL_PTR  != interDataIdle ) &&
        ( SPI_NULL_PTR  != ssIdle        )    )
    {
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CFG2, SPI_CFG2_TIMING_MASK );

        *interDataIdle = (spi_IdleCycles_t)( ( regValue & SPI_CFG2_MIDI ) >> SPI_CFG2_MIDI_Pos );
        *ssIdle        = (spi_IdleCycles_t)( ( regValue & SPI_CFG2_MSSI ) >> SPI_CFG2_MSSI_Pos );

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ----------------------------------- CRC ---------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Configures hardware CRC (state, length, polynomial, initialization pattern)
 *
 * \note  If CRC is disabled, only the CRC calculation is switched off - other fields are not
 *        checked and not modified.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param crcConfig [in]: Pointer to CRC configuration (see \ref spi_CrcConfig_t for allowed
 *                        values). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_CrcConfig( spi_PeriphId_t periphId, const spi_CrcConfig_t * const crcConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_DataSize_t     dataSize = SPI_DATA_SIZE_8BIT;

    if( ( SPI_NULL_PTR         != crcConfig        ) &&
        ( SPI_FUNCTION_ACTIVE  >= crcConfig->State )    )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_DataSize( periphId, &dataSize );
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
    }

    if( ( SPI_REQUEST_OK      == retState         ) &&
        ( SPI_FUNCTION_ACTIVE == crcConfig->State )    )
    {
        const spi_PeriphConfigStruct_t * const periphConf = &spi_PeriphConf[ periphId ];
        const spi_RegValue_t                   crcBits    = (spi_RegValue_t)crcConfig->Size + SPI_DATA_SIZE_BITS_OFFSET;
        spi_FunctionState_t                    sizeValid  = SPI_FUNCTION_INACTIVE;
        spi_FunctionState_t                    polyValid  = SPI_FUNCTION_INACTIVE;

        /* CRC length: not shorter than data, limited instances support 8 and 16 bits only */
        if( ( dataSize                <= crcConfig->Size ) &&
            ( periphConf->DataSizeMax >= crcConfig->Size ) &&
            ( ( SPI_FUNCTION_ACTIVE != periphConf->Limited     ) ||
              ( SPI_DATA_SIZE_8BIT  == crcConfig->Size         ) ||
              ( SPI_DATA_SIZE_16BIT == crcConfig->Size         )    )    )
        {
            sizeValid = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* CRC length is not supported */
        }

        /* Polynomial: non-zero and fits into the CRC length */
        if( ( 0u != crcConfig->Polynomial ) &&
            ( ( SPI_CRC_BITS_MAX <= crcBits ) || ( 0u == ( crcConfig->Polynomial >> crcBits ) ) ) )
        {
            polyValid = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Polynomial is out of range */
        }

        if( ( SPI_FUNCTION_ACTIVE == sizeValid             ) &&
            ( SPI_FUNCTION_ACTIVE == polyValid             ) &&
            ( SPI_CRC_INIT_CNT     > crcConfig->InitValue  )    )
        {
            SPI_TypeDef * const  periphReg = periphConf->PeriphReg;
            const spi_RegValue_t crcField  = ( (spi_RegValue_t)crcConfig->Size + SPI_DATA_SIZE_FIELD_OFFSET ) << SPI_CFG1_CRCSIZE_Pos;
            spi_RegValue_t       initValue = 0u;

            if( SPI_CRC_INIT_ALL_ONES == crcConfig->InitValue )
            {
                initValue = SPI_CR1_CRCINI_MASK;
            }
            else
            {
                /* CRC is initialized with zeros */
            }

            retState = Spi_Set_RegField( &periphReg->CRCPOLY, SPI_CRCPOLY_CRCPOLY, crcConfig->Polynomial );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_CRCINI_MASK, initValue );
            }
            else
            {
                /* Polynomial configuration failed */
            }

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CFG1, ( SPI_CFG1_CRCSIZE | SPI_CFG1_CRCEN ), ( crcField | SPI_CFG1_CRCEN ) );
            }
            else
            {
                /* Initialization pattern configuration failed */
            }
        }
        else
        {
            /* CRC configuration is invalid, nothing is modified */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CFG1, SPI_CFG1_CRCEN, 0u );
    }
    else
    {
        /* Configuration change is not allowed */
    }

    return ( retState );
}


/**
 * \brief Reads hardware CRC configuration
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param crcConfig [out]: Pointer to store the CRC configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_CrcConfig( spi_PeriphId_t periphId, spi_CrcConfig_t * const crcConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != crcConfig )    )
    {
        SPI_TypeDef * const  periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t cfgValue  = LL_SPI_ReadReg( periphReg, CFG1 );
        const spi_RegValue_t crcField  = ( cfgValue & SPI_CFG1_CRCSIZE ) >> SPI_CFG1_CRCSIZE_Pos;
        const spi_RegValue_t initValue = READ_BIT( periphReg->CR1, SPI_CR1_TCRCINI );

        if( ( SPI_DATA_SIZE_FIELD_OFFSET                                          <= crcField ) &&
            ( ( (spi_RegValue_t)SPI_DATA_SIZE_CNT + SPI_DATA_SIZE_FIELD_OFFSET )   > crcField )    )
        {
            crcConfig->State      = ( 0u != ( SPI_CFG1_CRCEN & cfgValue ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
            crcConfig->Size       = (spi_DataSize_t)( crcField - SPI_DATA_SIZE_FIELD_OFFSET );
            crcConfig->Polynomial = (spi_CrcPoly_t)LL_SPI_GetCRCPolynomial( periphReg );
            crcConfig->InitValue  = ( 0u != initValue ) ? SPI_CRC_INIT_ALL_ONES : SPI_CRC_INIT_ALL_ZERO;

            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* CRCSIZE values below 4 bits are not allowed */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads CRC calculated by transmitter and receiver (valid after the end of transfer)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param txCrc   [out]: Pointer to store the transmitter CRC. Must not be NULL.
 * \param rxCrc   [out]: Pointer to store the receiver CRC. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_CrcValue( spi_PeriphId_t periphId, spi_CrcValue_t * const txCrc, spi_CrcValue_t * const rxCrc )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != txCrc    ) &&
        ( SPI_NULL_PTR  != rxCrc    )    )
    {
        SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;

        *txCrc   = (spi_CrcValue_t)LL_SPI_GetTxCRC( periphReg );
        *rxCrc   = (spi_CrcValue_t)LL_SPI_GetRxCRC( periphReg );
        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------ Data handling ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Changes data handling configuration - resources of the previous mode are released and
 *        the new mode is initialized (GPDMA channels, SPI interrupt in NVIC)
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and nothing
 *        is modified.
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration (copied). Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_DataConfig( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Check_DataConfig( periphId, dataConfig );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        /* Configuration is invalid */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_XferDeinit( periphId );
    }
    else
    {
        /* Configuration is invalid or transfer is running, nothing is modified */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_XferInit( periphId, dataConfig );
    }
    else
    {
        /* Previous data handling could not be released */
    }

    return ( retState );
}


/**
 * \brief Returns data handling configuration
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [out]: Pointer to store the configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if data handling is not initialized)
 *         returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_DataConfig( spi_PeriphId_t periphId, spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT      > periphId                                ) &&
        ( SPI_NULL_PTR       != dataConfig                              ) &&
        ( SPI_FUNCTION_ACTIVE == spi_XferContext[ periphId ].InitState  )    )
    {
        *dataConfig = spi_XferContext[ periphId ].Config;
        retState    = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts transfer of the request in the configured direction (master starts clocking,
 *        slave waits for the master)
 *
 * The end of the transfer is reported by XferCompleteCallback or ErrorCallback and can be read
 * by Spi_Get_XferState() / Spi_Get_XferError().
 *
 * \pre   Data handling is initialized with a transfer mode (not NONE) and no transfer is
 *        running. Otherwise \ref SPI_REQUEST_ERROR is returned and no transfer is started.
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param xferRequest [in]: Pointer to transfer request (copied, buffers are not). Must not be
 *                          NULL, see \ref spi_XferRequest_t for the rules.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if the transfer was started.
 *         Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferStart( spi_PeriphId_t periphId, const spi_XferRequest_t * const xferRequest )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    spi_Direction_t    direction = SPI_DIRECTION_FULL_DUPLEX;
    spi_DataSize_t     dataSize  = SPI_DATA_SIZE_8BIT;

    retState = Spi_Check_XferRequest( periphId, xferRequest );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        /* Request is invalid */
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_RequestState_t dirState  = Spi_Get_Direction( periphId, &direction );
        const spi_RequestState_t sizeState = Spi_Get_DataSize( periphId, &dataSize );

        if( ( SPI_REQUEST_OK      != dirState                                  ) ||
            ( SPI_REQUEST_OK      != sizeState                                 ) ||
            ( SPI_FUNCTION_ACTIVE != spi_XferContext[ periphId ].InitState      ) ||
            ( SPI_XFER_MODE_NONE  == spi_XferContext[ periphId ].Config.XferMode )    )
        {
            /* Configuration is not readable or data handling is not initialized */
            retState = SPI_REQUEST_ERROR;
        }
        else
        {
            /* Transfer can be started */
        }
    }
    else
    {
        /* Transfer is running */
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_PeriphConfigStruct_t * const periphConf = &spi_PeriphConf[ periphId ];
        SPI_TypeDef * const                    periphReg  = periphConf->PeriphReg;
        spi_XferContext_t * const              xferCtx    = &spi_XferContext[ periphId ];
        const spi_XferModeIf_t * const         modeIf     = &spi_XferModeLut[ xferCtx->Config.XferMode ];
        spi_Mode_t                             mode       = SPI_MODE_MASTER;

        xferCtx->Request    = *xferRequest;
        xferCtx->FrameBytes = Spi_Get_FrameBytes( dataSize );
        xferCtx->RxFifoCnt  = (spi_DataCnt_t)( periphConf->FifoBytes / xferCtx->FrameBytes );
        xferCtx->TxIdx      = 0u;
        xferCtx->RxIdx      = 0u;
        xferCtx->XferError  = SPI_XFER_ERROR_NONE;
        xferCtx->TxUsed     = SPI_FUNCTION_INACTIVE;
        xferCtx->RxUsed     = SPI_FUNCTION_INACTIVE;

        if( ( SPI_DIRECTION_FULL_DUPLEX == direction ) ||
            ( SPI_DIRECTION_SIMPLEX_TX  == direction ) ||
            ( ( SPI_DIRECTION_HALF_DUPLEX == direction ) && ( SPI_NULL_PTR != xferRequest->TxData ) ) )
        {
            xferCtx->TxUsed = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Nothing is transmitted - transmit buffer is not used */
            xferCtx->Request.TxData = SPI_NULL_PTR;
        }

        if( ( SPI_DIRECTION_FULL_DUPLEX == direction ) ||
            ( SPI_DIRECTION_SIMPLEX_RX  == direction ) ||
            ( ( SPI_DIRECTION_HALF_DUPLEX == direction ) && ( SPI_NULL_PTR == xferRequest->TxData ) ) )
        {
            xferCtx->RxUsed = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Nothing is received - receive buffer is not used */
            xferCtx->Request.RxData = SPI_NULL_PTR;
        }

        /* Flags of a previous transfer are cleared (write 1 to clear - not verified by read-back) */
        LL_SPI_WriteReg( periphReg, IFCR, SPI_IFCR_ALL );

        /* Count of frames of the transfer (writable only while SPE = 0) */
        retState = Spi_Set_RegField( &periphReg->CR2, SPI_CR2_TSIZE, (spi_RegValue_t)xferRequest->XferSize );

        /* Half-duplex: direction of the single data line */
        if( ( SPI_REQUEST_OK            == retState  ) &&
            ( SPI_DIRECTION_HALF_DUPLEX == direction )    )
        {
            const spi_RegValue_t hdDir = ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed ) ? SPI_CR1_HDDIR : 0u;

            retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_HDDIR, hdDir );
        }
        else
        {
            /* Previous step failed or other direction than half-duplex */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Get_Mode( periphId, &mode );
        }
        else
        {
            /* Transfer configuration failed */
        }

        /* Transfer is marked as running before interrupts can occur */
        if( SPI_REQUEST_OK == retState )
        {
            xferCtx->XferState = SPI_FUNCTION_ACTIVE;

            retState = modeIf->Start( periphId );
        }
        else
        {
            /* Transfer configuration failed */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Enable( periphId, SPI_FUNCTION_ACTIVE );
        }
        else
        {
            /* Transfer mode resources could not be started */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = modeIf->Run( periphId );
        }
        else
        {
            /* Peripheral could not be enabled */
        }

        if( SPI_REQUEST_OK != retState )
        {
            /* Transfer was not started - mode resources are released, peripheral is disabled */
            (void)Spi_Set_XferAbort( periphId );
        }
        else if( SPI_MODE_MASTER == mode )
        {
            /* Master starts clocking (CSTART is cleared by HW at the end of transfer) */
            LL_SPI_StartMasterTransfer( periphReg );
        }
        else
        {
            /* Slave transfer is clocked by the master */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Aborts running transfer - transfer mode resources are stopped, master transfer is
 *        suspended and the peripheral is disabled (FIFOs are flushed). No callback is called.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (also if no transfer was running). Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferStop( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        if( SPI_FUNCTION_ACTIVE == spi_XferContext[ periphId ].XferState )
        {
            retState = Spi_Set_XferAbort( periphId );

            spi_XferContext[ periphId ].XferError = SPI_XFER_ERROR_NONE;
        }
        else
        {
            /* No transfer is running */
            retState = SPI_REQUEST_OK;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads transfer state
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param xferState [out]: Pointer to store the state - \ref SPI_FUNCTION_ACTIVE while the transfer
 *                         is running. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_XferState( spi_PeriphId_t periphId, spi_FunctionState_t * const xferState )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != xferState )    )
    {
        *xferState = spi_XferContext[ periphId ].XferState;
        retState   = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads error of the last finished transfer
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param xferError [out]: Pointer to store the error - \ref SPI_XFER_ERROR_NONE if the last
 *                         transfer finished successfully. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_XferError( spi_PeriphId_t periphId, spi_XferErrorId_t * const xferError )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != xferError )    )
    {
        *xferError = spi_XferContext[ periphId ].XferError;
        retState   = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------- Interrupts ------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Configures priority of SPI interrupt
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param irqPrio  [in]: Interrupt priority
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_IrqPriority( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( spi_PeriphConf[ periphId ].PeriphNvic, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads priority of SPI interrupt
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param irqPrio [out]: Pointer to store the interrupt priority. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_IrqPriority( spi_PeriphId_t periphId, spi_IrqPrio_t * const irqPrio )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != irqPrio  )    )
    {
        nvic_IrqPrio_t            nvicPrio  = 0u;
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( spi_PeriphConf[ periphId ].PeriphNvic, &nvicPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            *irqPrio = (spi_IrqPrio_t)nvicPrio;
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------- Common services for data transfer handlers ----------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Returns registers of SPI peripheral
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param periphReg [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_PeriphReg( spi_PeriphId_t periphId, SPI_TypeDef ** const periphReg )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != periphReg )    )
    {
        *periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        retState   = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns GPDMA request identifications of SPI peripheral
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param txRequest [out]: Pointer to store transmit request. Must not be NULL.
 * \param rxRequest [out]: Pointer to store receive request. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_PeriphDmaReq( spi_PeriphId_t periphId, gpdma_PeriphReqId_t * const txRequest, gpdma_PeriphReqId_t * const rxRequest )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId  ) &&
        ( SPI_NULL_PTR  != txRequest ) &&
        ( SPI_NULL_PTR  != rxRequest )    )
    {
        *txRequest = spi_PeriphConf[ periphId ].PeriphDmaTxReq;
        *rxRequest = spi_PeriphConf[ periphId ].PeriphDmaRxReq;
        retState   = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns data handling context of SPI peripheral
 *
 * \param periphId     [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param xferContext [out]: Pointer to store the context pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Get_XferContext( spi_PeriphId_t periphId, spi_XferContext_t ** const xferContext )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId    ) &&
        ( SPI_NULL_PTR  != xferContext )    )
    {
        *xferContext = &spi_XferContext[ periphId ];
        retState     = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes register field and verifies it by read-back
 *
 * \param regAddr    [in]: Pointer to the register. Must not be NULL.
 * \param fieldMask  [in]: Mask of the modified field(s)
 * \param fieldValue [in]: Value of the field(s), bits out of fieldMask are ignored
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if the field reads back the
 *         written value. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_RegField( volatile uint32_t * const regAddr, spi_RegValue_t fieldMask, spi_RegValue_t fieldValue )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_NULL_PTR != regAddr )
    {
        const spi_RegValue_t expected = fieldValue & fieldMask;

        MODIFY_REG( *regAddr, fieldMask, expected );

        for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const spi_RegValue_t regValue = READ_BIT( *regAddr, fieldMask );

            if( expected == regValue )
            {
                retState = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Field has not yet been applied, keep return state as error */
                retState = SPI_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns if the next frame of the running transfer may be written to the transmit FIFO
 *        (ISR / POLL mode)
 *
 * In receiving transfers the count of transmitted, not yet received frames is limited by the
 * receive FIFO size - the receive FIFO can never overflow.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return \ref SPI_FUNCTION_ACTIVE if a frame may be written, otherwise \ref SPI_FUNCTION_INACTIVE
 */
spi_FunctionState_t Spi_Get_TxPending( spi_PeriphId_t periphId )
{
    spi_FunctionState_t txPending = SPI_FUNCTION_INACTIVE;

    if( SPI_PERIPH_CNT > periphId )
    {
        const spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];
        const spi_DataCnt_t             txIdx   = xferCtx->TxIdx;
        const spi_DataCnt_t             rxIdx   = xferCtx->RxIdx;

        if( ( SPI_FUNCTION_ACTIVE == xferCtx->XferState        ) &&
            ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed           ) &&
            ( xferCtx->Request.XferSize > txIdx                ) &&
            ( ( SPI_FUNCTION_ACTIVE != xferCtx->RxUsed                      ) ||
              ( xferCtx->RxFifoCnt   > (spi_DataCnt_t)( txIdx - rxIdx )     )    )    )
        {
            txPending = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Transmission finished or receive FIFO could overflow */
        }
    }
    else
    {
        /* Invalid peripheral identification */
    }

    return ( txPending );
}


/**
 * \brief Moves frames of the running transfer (ISR / POLL mode): received frames are read while
 *        RXP is set, frames are written while TXP is set and \ref Spi_Get_TxPending allows it
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferDataStep( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const       periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];

        if( SPI_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            /* Received frames (one FIFO at most per call) */
            for( spi_DataCnt_t frameCnt = 0u; xferCtx->RxFifoCnt > frameCnt; frameCnt ++ )
            {
                const uint32_t rxReady = LL_SPI_IsActiveFlag_RXP( periphReg );

                if( ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed        ) &&
                    ( xferCtx->Request.XferSize > xferCtx->RxIdx    ) &&
                    ( 0u != rxReady                                 )    )
                {
                    Spi_Get_FrameRead( periphId );
                }
                else
                {
                    /* No received frame */
                    break;
                }
            }

            /* Frames to be transmitted (one FIFO at most per call) */
            for( spi_DataCnt_t frameCnt = 0u; xferCtx->RxFifoCnt > frameCnt; frameCnt ++ )
            {
                const uint32_t            txReady   = LL_SPI_IsActiveFlag_TXP( periphReg );
                const spi_FunctionState_t txPending = Spi_Get_TxPending( periphId );

                if( ( SPI_FUNCTION_ACTIVE == txPending ) &&
                    ( 0u                  != txReady   )    )
                {
                    Spi_Set_FrameWrite( periphId );
                }
                else
                {
                    /* Transmit FIFO is full or nothing may be transmitted */
                    break;
                }
            }
        }
        else
        {
            /* No transfer is running */
        }

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Processes end of transfer and error events of the running transfer (all modes)
 *
 * - MODF / OVR / UDR / TIFRE: transfer ends immediately with error
 * - EOT: remaining received frames are read (ISR / POLL), CRC is checked and the transfer ends
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param statusFlags [in]: Snapshot of SPI SR register
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferEvents( spi_PeriphId_t periphId, spi_StatusFlags_t statusFlags )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const             periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];
        spi_StatusFlags_t               errMask   = ( LL_SPI_SR_MODF | LL_SPI_SR_TIFRE );
        spi_XferErrorId_t               errorId   = SPI_XFER_ERROR_NONE;

        /* Overrun is relevant for reception, underrun for slave transmission only */
        if( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed )
        {
            errMask |= LL_SPI_SR_OVR;
        }
        else
        {
            /* Receive FIFO is not used */
        }

        if( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed )
        {
            errMask |= LL_SPI_SR_UDR;
        }
        else
        {
            /* Transmit FIFO is not used */
        }

        retState = SPI_REQUEST_OK;

        if( SPI_FUNCTION_ACTIVE != xferCtx->XferState )
        {
            /* No transfer is running */
        }
        else if( 0u != ( errMask & statusFlags ) )
        {
            if( 0u != ( LL_SPI_SR_MODF & statusFlags ) )
            {
                errorId = SPI_XFER_ERROR_MODE_FAULT;
            }
            else if( 0u != ( LL_SPI_SR_OVR & errMask & statusFlags ) )
            {
                errorId = SPI_XFER_ERROR_OVERRUN;
            }
            else if( 0u != ( LL_SPI_SR_UDR & errMask & statusFlags ) )
            {
                errorId = SPI_XFER_ERROR_UNDERRUN;
            }
            else
            {
                errorId = SPI_XFER_ERROR_FRAME;
            }

            retState = Spi_Set_XferError( periphId, errorId );
        }
        else if( 0u != ( LL_SPI_SR_EOT & statusFlags ) )
        {
            const uint32_t crcEnabled = LL_SPI_IsEnabledCRC( periphReg );

            if( SPI_XFER_MODE_DMA != xferCtx->Config.XferMode )
            {
                /* Last frames are still in the receive FIFO */
                retState = Spi_Set_XferDataStep( periphId );

                if( ( ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed ) && ( xferCtx->Request.XferSize > xferCtx->RxIdx ) ) ||
                    ( ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed ) && ( xferCtx->Request.XferSize > xferCtx->TxIdx ) )    )
                {
                    errorId = SPI_XFER_ERROR_INCOMPLETE;
                }
                else
                {
                    /* All frames were moved */
                }
            }
            else
            {
                /* Frames moved by DMA are checked at the transfer end */
            }

            if( ( SPI_XFER_ERROR_NONE == errorId                         ) &&
                ( 0u                  != crcEnabled                      ) &&
                ( 0u                  != LL_SPI_IsActiveFlag_CRCERR( periphReg ) )    )
            {
                errorId = SPI_XFER_ERROR_CRC;
            }
            else
            {
                /* CRC is not used or matches */
            }

            const spi_RequestState_t endState = Spi_Set_XferEnd( periphId, errorId );

            if( ( SPI_REQUEST_OK == retState ) &&
                ( SPI_REQUEST_OK == endState )    )
            {
                retState = SPI_REQUEST_OK;
            }
            else
            {
                retState = SPI_REQUEST_ERROR;
            }
        }
        else
        {
            /* No end of transfer or error event */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Terminates the running transfer by an error - the transfer is aborted, the error is
 *        stored and ErrorCallback is called
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param errorId  [in]: Error identification, value from \ref spi_XferErrorId_t (not NONE)
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferError( spi_PeriphId_t periphId, spi_XferErrorId_t errorId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT      > periphId ) &&
        ( SPI_XFER_ERROR_CNT  > errorId  ) &&
        ( SPI_XFER_ERROR_NONE != errorId )    )
    {
        spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];

        if( SPI_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            retState = Spi_Set_XferAbort( periphId );
        }
        else
        {
            /* Transfer already ended - error is only reported */
            retState = SPI_REQUEST_OK;
        }

        xferCtx->XferError = errorId;

        if( SPI_NULL_PTR != xferCtx->Config.ErrorCallback )
        {
            xferCtx->Config.ErrorCallback( errorId );
        }
        else
        {
            /* Error callback is not used */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks peripheral configuration structure (values in range, features supported by the
 *        peripheral, pins of the peripheral)
 *
 * \param spiConfig [in]: Pointer to configuration structure
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_Config( const spi_Config_t * const spiConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_NULL_PTR          != spiConfig              ) &&
        ( SPI_PERIPH_CNT         > spiConfig->PeriphId    ) &&
        ( (uint32_t)spiConfig->PeriphId == SPI_CLK_SRC_BIT_MASK_DECODE_PERIPH( spiConfig->ClkSrc ) ) &&
        ( SPI_CLK_SRC_ID_CNT             > SPI_CLK_SRC_BIT_MASK_DECODE_SOURCE( spiConfig->ClkSrc )  ) &&
        ( SPI_MODE_CNT           > spiConfig->Mode        ) &&
        ( SPI_CLOCK_MODE_CNT     > spiConfig->ClockMode   ) &&
        ( SPI_BIT_ORDER_CNT      > spiConfig->BitOrder    ) &&
        ( SPI_DIRECTION_CNT      > spiConfig->Direction   ) &&
        ( SPI_FRAME_FORMAT_CNT   > spiConfig->FrameFormat ) &&
        ( SPI_PIN_SPEED_CNT      > spiConfig->PinSpeed    ) &&
        ( ( SPI_MODE_SLAVE == spiConfig->Mode ) || ( 0u < spiConfig->BusFreq ) )    )
    {
        const spi_PeriphConfigStruct_t * const periphConf = &spi_PeriphConf[ spiConfig->PeriphId ];

        const spi_RequestState_t nssState  = Spi_Check_NssConfig( &spiConfig->NssConfig );
        const spi_RequestState_t sckState  = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->SckPin );
        const spi_RequestState_t misoState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->MisoPin );
        const spi_RequestState_t mosiState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->MosiPin );
        const spi_RequestState_t nssPState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->NssPin );

        if( ( RCC_PERIPH_ID_CNT       != periphConf->PeriphRcc[ SPI_CLK_SRC_BIT_MASK_DECODE_SOURCE( spiConfig->ClkSrc ) ] ) &&
            ( periphConf->DataSizeMax >= spiConfig->DataSize                        ) &&
            ( SPI_REQUEST_OK          == nssState                                   ) &&
            ( SPI_REQUEST_OK          == sckState                                   ) &&
            ( SPI_REQUEST_OK          == misoState                                  ) &&
            ( SPI_REQUEST_OK          == mosiState                                  ) &&
            ( SPI_REQUEST_OK          == nssPState                                  )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* Feature is not supported by the peripheral or pin configuration is invalid */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks encoded pin - unused pin or pin of the configured peripheral with valid port, pin
 *        and alternate function
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param pinCode  [in]: Encoded pin, value from \ref spi_SckPin_t, \ref spi_MisoPin_t,
 *                      \ref spi_MosiPin_t or \ref spi_NssPin_t (\ref SPI_PIN_UNUSED - unused pin)
 *
 * \return Returns \ref SPI_REQUEST_OK if the pin is unused or valid for the peripheral.
 *         Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_Pin( spi_PeriphId_t periphId, spi_PinCode_t pinCode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    const spi_PinCode_t pinPeriph = SPI_BIT_MASK_DECODE_PERIPH( pinCode );
    const spi_PinCode_t pinPort   = SPI_BIT_MASK_DECODE_PORT( pinCode );
    const spi_PinCode_t pinId     = SPI_BIT_MASK_DECODE_PIN( pinCode );
    const spi_PinCode_t pinAf     = SPI_BIT_MASK_DECODE_AF( pinCode );

    if( SPI_PIN_UNUSED == pinCode )
    {
        /* Pin is not configured by the module */
        retState = SPI_REQUEST_OK;
    }
    else if( ( (spi_PinCode_t)periphId          == pinPeriph ) &&
             ( (spi_PinCode_t)GPIO_PORT_CNT      > pinPort   ) &&
             ( (spi_PinCode_t)GPIO_PIN_ID_CNT    > pinId     ) &&
             ( (spi_PinCode_t)GPIO_ALT_FUNC_CNT  > pinAf     )    )
    {
        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks NSS configuration (values in range, pulse only with hardware NSS)
 *
 * \param nssConfig [in]: Pointer to NSS configuration
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_NssConfig( const spi_NssConfig_t * const nssConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_NULL_PTR         != nssConfig           ) &&
        ( SPI_NSS_MODE_CNT      > nssConfig->Mode     ) &&
        ( SPI_NSS_POLARITY_CNT  > nssConfig->Polarity ) &&
        ( SPI_FUNCTION_ACTIVE  >= nssConfig->Pulse    ) &&
        ( ( SPI_FUNCTION_INACTIVE == nssConfig->Pulse ) || ( SPI_NSS_MODE_HARD == nssConfig->Mode ) ) )
    {
        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the configuration may be changed - no transfer is running and the
 *        peripheral is disabled (CFG1, CFG2 and CR2 are write protected while SPE = 1)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration may be changed. Otherwise (or for an
 *         invalid periphId) returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_ConfigAllowed( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        const uint32_t periphEnabled = LL_SPI_IsEnabled( spi_PeriphConf[ periphId ].PeriphReg );

        if( ( SPI_FUNCTION_INACTIVE == spi_XferContext[ periphId ].XferState ) &&
            ( 0u                    == periphEnabled                         )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* Transfer is running */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks transfer request - size range of the peripheral and buffers required by the
 *        configured communication direction
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param xferRequest [in]: Pointer to transfer request
 *
 * \return Returns \ref SPI_REQUEST_OK if the request is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_XferRequest( spi_PeriphId_t periphId, const spi_XferRequest_t * const xferRequest )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    spi_Direction_t    direction = SPI_DIRECTION_FULL_DUPLEX;

    if( SPI_NULL_PTR != xferRequest )
    {
        retState = Spi_Get_Direction( periphId, &direction );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_FunctionState_t txSet = ( SPI_NULL_PTR != xferRequest->TxData ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        const spi_FunctionState_t rxSet = ( SPI_NULL_PTR != xferRequest->RxData ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        spi_FunctionState_t       bufOk = SPI_FUNCTION_INACTIVE;

        switch( direction )
        {
            case SPI_DIRECTION_FULL_DUPLEX:
                bufOk = ( ( SPI_FUNCTION_ACTIVE == txSet ) || ( SPI_FUNCTION_ACTIVE == rxSet ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
                break;

            case SPI_DIRECTION_SIMPLEX_TX:
                bufOk = txSet;
                break;

            case SPI_DIRECTION_SIMPLEX_RX:
                bufOk = rxSet;
                break;

            default:
                /* Half-duplex - exactly one direction */
                bufOk = ( txSet != rxSet ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
                break;
        }

        if( ( SPI_FUNCTION_ACTIVE                     == bufOk                 ) &&
            ( 0u                                       < xferRequest->XferSize ) &&
            ( spi_PeriphConf[ periphId ].XferSizeMax  >= xferRequest->XferSize )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* Size out of range or missing buffer */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks data handling configuration (common part and transfer mode specific part)
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_DataConfig( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT     > periphId             ) &&
        ( SPI_NULL_PTR      != dataConfig           ) &&
        ( SPI_XFER_MODE_CNT  > dataConfig->XferMode )    )
    {
        retState = spi_XferModeLut[ dataConfig->XferMode ].CheckConfig( periphId, dataConfig );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables the peripheral (SPE) with read-back verification
 *
 * \note  SPE = 0 flushes FIFOs and releases pins of a slave (master pins stay driven - AFCNTR).
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param enableState [in]: Required state, value from \ref spi_FunctionState_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_Enable( spi_PeriphId_t periphId, spi_FunctionState_t enableState )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        uint32_t            expected  = 0u;

        if( SPI_FUNCTION_ACTIVE == enableState )
        {
            LL_SPI_Enable( periphReg );
            expected = 1u;
        }
        else
        {
            LL_SPI_Disable( periphReg );
            expected = 0u;
        }

        for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_SPI_IsEnabled( periphReg );

            if( expected == regValue )
            {
                retState = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Peripheral state has not yet been applied, keep return state as error */
                retState = SPI_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures internal slave select level (SSI) used with software NSS: master is never
 *        selected by another master (no mode fault), slave is permanently selected
 *
 * \note  Called before CFG2 is written - the level of the new configuration is applied while
 *        the old one is still active (SSI is used by HW only with software NSS).
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param cfgValue [in]: CFG2 value the level is derived from (role MASTER, NSS polarity SSIOP)
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_SsLevel( spi_PeriphId_t periphId, spi_RegValue_t cfgValue )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const  periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t isMaster  = cfgValue & SPI_CFG2_MASTER;
        const spi_RegValue_t activeHi  = cfgValue & SPI_CFG2_SSIOP;
        spi_RegValue_t       ssiValue  = 0u;

        /* Master: SSI = inactive level, slave: SSI = active level */
        if( ( ( 0u != isMaster ) && ( 0u == activeHi ) ) ||
            ( ( 0u == isMaster ) && ( 0u != activeHi ) )    )
        {
            ssiValue = SPI_CR1_SSI;
        }
        else
        {
            /* Internal slave select is low */
        }

        retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_SSI, ssiValue );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures pin as push-pull alternate function (GPIO port clock is enabled by
 *        Gpio_Init())
 *
 * \param pinCode  [in]: Encoded pin, checked by \ref Spi_Check_Pin. Unused pin is skipped.
 * \param pinSpeed [in]: Output speed, value from \ref spi_PinSpeed_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_Pin( spi_PinCode_t pinCode, spi_PinSpeed_t pinSpeed )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PIN_UNUSED == pinCode )
    {
        /* Pin is not configured by the module */
        retState = SPI_REQUEST_OK;
    }
    else if( SPI_PIN_SPEED_CNT > pinSpeed )
    {
        gpio_Config_t gpioConfig;

        gpioConfig.PortId         = (gpio_PortId_t)SPI_BIT_MASK_DECODE_PORT( pinCode );
        gpioConfig.PinId          = (gpio_PinId_t)SPI_BIT_MASK_DECODE_PIN( pinCode );
        gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
        gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
        gpioConfig.PinSpeed       = spi_PinSpeedLut[ pinSpeed ];
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = (gpio_AltFunction_t)SPI_BIT_MASK_DECODE_AF( pinCode );
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        /* Port clock activation, pin configuration and read-back verification is done by Gpio_Init() */
        const gpio_RequestState_t gpioState = Gpio_Init( &gpioConfig );

        if( GPIO_REQUEST_OK == gpioState )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns frequency of the selected SPI kernel clock
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param clkFreq [out]: Pointer to store the frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (non-zero frequency). Otherwise (also for a clock
 *         source with unknown frequency, e.g. external audio clock) returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Get_KernelClk( spi_PeriphId_t periphId, spi_FreqHz_t * const clkFreq )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != clkFreq  )    )
    {
        rcc_PeriphId_t     clkSrcId = RCC_PERIPH_ID_CNT;
        rcc_FreqHz_t       rccFreq  = 0u;
        rcc_RequestState_t rccState = Rcc_Get_PeriphClkSrc( spi_PeriphConf[ periphId ].PeriphRccBase, &clkSrcId );

        if( RCC_REQUEST_OK == rccState )
        {
            rccState = Rcc_Get_PeriphClk( clkSrcId, &rccFreq );
        }
        else
        {
            /* Selected clock source is not known */
        }

        if( ( RCC_REQUEST_OK == rccState ) &&
            ( 0u              < rccFreq  )    )
        {
            *clkFreq = (spi_FreqHz_t)rccFreq;
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns count of buffer bytes occupied by one frame of the data size
 *
 * \param dataSize [in]: Data size, value from \ref spi_DataSize_t
 *
 * \return 1 (up to 8 bits), 2 (up to 16 bits) or 4 (up to 32 bits)
 */
static spi_FrameBytes_t Spi_Get_FrameBytes( spi_DataSize_t dataSize )
{
    spi_FrameBytes_t frameBytes = SPI_FRAME_BYTES_32BIT;

    if( SPI_DATA_SIZE_8BIT >= dataSize )
    {
        frameBytes = SPI_FRAME_BYTES_8BIT;
    }
    else if( SPI_DATA_SIZE_16BIT >= dataSize )
    {
        frameBytes = SPI_FRAME_BYTES_16BIT;
    }
    else
    {
        /* Frame up to 32 bits */
    }

    return ( frameBytes );
}


/**
 * \brief Initializes data handling - configuration is copied, transfer mode resources are
 *        initialized and SPI interrupt is enabled in NVIC (DMA / ISR mode)
 *
 * \note  On failure the partially initialized data handling is released.
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to checked data handling configuration
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_XferInit( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT     > periphId             ) &&
        ( SPI_NULL_PTR      != dataConfig           ) &&
        ( SPI_XFER_MODE_CNT  > dataConfig->XferMode )    )
    {
        spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];

        xferCtx->Config    = *dataConfig;
        xferCtx->XferState = SPI_FUNCTION_INACTIVE;
        xferCtx->XferError = SPI_XFER_ERROR_NONE;
        xferCtx->InitState = SPI_FUNCTION_ACTIVE;

        retState = spi_XferModeLut[ dataConfig->XferMode ].Init( periphId );

        const spi_FunctionState_t irqUsed = Spi_Get_IrqUsed( dataConfig );

        if( ( SPI_REQUEST_OK      == retState ) &&
            ( SPI_FUNCTION_ACTIVE == irqUsed  )    )
        {
            retState = Spi_Set_IrqInit( periphId, dataConfig->IrqPriority );
        }
        else
        {
            /* Mode initialization failed or interrupts are not used */
        }

        if( SPI_REQUEST_OK != retState )
        {
            (void)Spi_Set_XferDeinit( periphId );
        }
        else
        {
            /* Data handling is initialized */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes data handling - running transfer is aborted, transfer mode resources are
 *        released and SPI interrupt is disabled in NVIC
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (also if data handling was not initialized).
 *         Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_XferDeinit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];

        if( SPI_FUNCTION_ACTIVE == xferCtx->InitState )
        {
            spi_RequestState_t abortState = SPI_REQUEST_OK;

            if( SPI_FUNCTION_ACTIVE == xferCtx->XferState )
            {
                abortState = Spi_Set_XferAbort( periphId );
            }
            else
            {
                /* No transfer is running */
            }

            const spi_RequestState_t deinitState = spi_XferModeLut[ xferCtx->Config.XferMode ].Deinit( periphId );
            const spi_RequestState_t irqState    = Spi_Set_IrqDeinit( periphId );

            xferCtx->InitState = SPI_FUNCTION_INACTIVE;

            if( ( SPI_REQUEST_OK == abortState  ) &&
                ( SPI_REQUEST_OK == deinitState ) &&
                ( SPI_REQUEST_OK == irqState    )    )
            {
                retState = SPI_REQUEST_OK;
            }
            else
            {
                retState = SPI_REQUEST_ERROR;
            }
        }
        else
        {
            /* Data handling is not initialized */
            retState = SPI_REQUEST_OK;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns if the data handling configuration uses SPI interrupt (DMA or ISR mode)
 *
 * \param dataConfig [in]: Pointer to data handling configuration
 *
 * \return \ref SPI_FUNCTION_ACTIVE if SPI interrupt is used, otherwise \ref SPI_FUNCTION_INACTIVE
 */
static spi_FunctionState_t Spi_Get_IrqUsed( const spi_DataConfig_t * const dataConfig )
{
    spi_FunctionState_t irqUsed = SPI_FUNCTION_INACTIVE;

    if( ( SPI_NULL_PTR != dataConfig ) &&
        ( ( SPI_XFER_MODE_DMA == dataConfig->XferMode ) ||
          ( SPI_XFER_MODE_ISR == dataConfig->XferMode )    )    )
    {
        irqUsed = SPI_FUNCTION_ACTIVE;
    }
    else
    {
        irqUsed = SPI_FUNCTION_INACTIVE;
    }

    return ( irqUsed );
}


/**
 * \brief Registers the interrupt handler, configures priority and enables SPI interrupt in NVIC
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param irqPrio  [in]: Interrupt priority
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_IrqInit( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        const spi_PeriphConfigStruct_t * const periphConf = &spi_PeriphConf[ periphId ];

        const nvic_RequestState_t handlerState = Nvic_Set_PeriphIrq_Handler( periphConf->PeriphNvic, periphConf->PeriphIsr );
        const spi_RequestState_t  prioState    = Spi_Set_IrqPriority( periphId, irqPrio );
        const nvic_RequestState_t actState     = Nvic_Set_PeriphIrq_Active( periphConf->PeriphNvic );

        if( ( NVIC_REQUEST_OK == handlerState ) &&
            ( SPI_REQUEST_OK  == prioState    ) &&
            ( NVIC_REQUEST_OK == actState     )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables SPI interrupt in NVIC
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_IrqDeinit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( spi_PeriphConf[ periphId ].PeriphNvic );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Ends the running transfer - moved frames are checked (DMA), transfer mode resources are
 *        stopped, flags are cleared, the peripheral is disabled and XferCompleteCallback /
 *        ErrorCallback is called
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param errorId  [in]: Result of the transfer, value from \ref spi_XferErrorId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_XferEnd( spi_PeriphId_t periphId, spi_XferErrorId_t errorId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT     > periphId ) &&
        ( SPI_XFER_ERROR_CNT > errorId  )    )
    {
        spi_XferContext_t * const      xferCtx = &spi_XferContext[ periphId ];
        const spi_XferModeIf_t * const modeIf  = &spi_XferModeLut[ xferCtx->Config.XferMode ];
        spi_XferErrorId_t              xferErr = errorId;

        /* Last received frames may still be moved by DMA - checked before the channels are stopped */
        const spi_RequestState_t doneState   = modeIf->CheckDone( periphId );

        /* Peripheral is disabled before the transfer mode resources are stopped - CFG1 (DMA
         * requests) is write protected while SPE = 1 */
        const spi_RequestState_t enableState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );
        const spi_RequestState_t stopState   = modeIf->Stop( periphId );

        LL_SPI_WriteReg( spi_PeriphConf[ periphId ].PeriphReg, IFCR, SPI_IFCR_ALL );

        if( ( SPI_XFER_ERROR_NONE == xferErr   ) &&
            ( SPI_REQUEST_OK      != doneState )    )
        {
            /* End of transfer was reached, but DMA did not move all frames */
            xferErr = SPI_XFER_ERROR_INCOMPLETE;
        }
        else
        {
            /* Result is kept */
        }

        xferCtx->XferError = xferErr;
        xferCtx->XferState = SPI_FUNCTION_INACTIVE;

        if( SPI_XFER_ERROR_NONE == xferErr )
        {
            if( SPI_NULL_PTR != xferCtx->Config.XferCompleteCallback )
            {
                xferCtx->Config.XferCompleteCallback( );
            }
            else
            {
                /* Transfer complete callback is not used */
            }
        }
        else
        {
            if( SPI_NULL_PTR != xferCtx->Config.ErrorCallback )
            {
                xferCtx->Config.ErrorCallback( xferErr );
            }
            else
            {
                /* Error callback is not used */
            }
        }

        if( ( SPI_REQUEST_OK == stopState   ) &&
            ( SPI_REQUEST_OK == enableState )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Aborts the running transfer without callback - running master transfer is suspended,
 *        the peripheral is disabled, transfer mode resources are stopped and flags are cleared
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_XferAbort( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const       periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];
        const uint32_t            masterRun = LL_SPI_IsActiveMasterTransfer( periphReg );

        xferCtx->XferState = SPI_FUNCTION_INACTIVE;

        /* Running master transfer is suspended first (recommended disable procedure) */
        if( 0u != masterRun )
        {
            LL_SPI_SuspendMasterTransfer( periphReg );

            for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const spi_StatusFlags_t statusFlags = LL_SPI_ReadReg( periphReg, SR );

                if( 0u != ( ( LL_SPI_SR_SUSP | LL_SPI_SR_EOT ) & statusFlags ) )
                {
                    break;
                }
                else
                {
                    /* Transfer is not yet suspended - the peripheral is disabled after the timeout anyway */
                }
            }
        }
        else
        {
            /* Master is not clocking */
        }

        /* Peripheral is disabled before the transfer mode resources are stopped - CFG1 (DMA
         * requests) is write protected while SPE = 1 */
        const spi_RequestState_t enableState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );
        const spi_RequestState_t stopState   = spi_XferModeLut[ xferCtx->Config.XferMode ].Stop( periphId );

        LL_SPI_WriteReg( periphReg, IFCR, SPI_IFCR_ALL );

        if( ( SPI_REQUEST_OK == stopState   ) &&
            ( SPI_REQUEST_OK == enableState )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the next frame of the running transfer to TXDR (zero frame if the transmit buffer
 *        is not set). Access width corresponds to the frame size (data packing is not used).
 *
 * \param periphId [in]: SPI peripheral identification, checked by the caller
 */
static void Spi_Set_FrameWrite( spi_PeriphId_t periphId )
{
    SPI_TypeDef * const       periphReg = spi_PeriphConf[ periphId ].PeriphReg;
    spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];
    const spi_DataCnt_t       txIdx     = xferCtx->TxIdx;
    uint32_t                  frameData = 0u;

    if( SPI_NULL_PTR != xferCtx->Request.TxData )
    {
        const spi_Data_t * const frameBuf = &xferCtx->Request.TxData[ (uint32_t)txIdx * xferCtx->FrameBytes ];

        /* Frame is stored little endian */
        for( spi_FrameBytes_t byteIdx = xferCtx->FrameBytes; 0u < byteIdx; byteIdx -- )
        {
            frameData = ( frameData << SPI_BITS_PER_BYTE ) | frameBuf[ byteIdx - 1u ];
        }
    }
    else
    {
        /* Transmit buffer is not set - zero frame is transmitted */
    }

    if( SPI_FRAME_BYTES_8BIT == xferCtx->FrameBytes )
    {
        LL_SPI_TransmitData8( periphReg, (uint8_t)frameData );
    }
    else if( SPI_FRAME_BYTES_16BIT == xferCtx->FrameBytes )
    {
        LL_SPI_TransmitData16( periphReg, (uint16_t)frameData );
    }
    else
    {
        LL_SPI_TransmitData32( periphReg, frameData );
    }

    xferCtx->TxIdx = txIdx + 1u;
}


/**
 * \brief Reads the next frame of the running transfer from RXDR and stores it (discarded if the
 *        receive buffer is not set). Access width corresponds to the frame size.
 *
 * \param periphId [in]: SPI peripheral identification, checked by the caller
 */
static void Spi_Get_FrameRead( spi_PeriphId_t periphId )
{
    SPI_TypeDef * const       periphReg = spi_PeriphConf[ periphId ].PeriphReg;
    spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];
    const spi_DataCnt_t       rxIdx     = xferCtx->RxIdx;
    uint32_t                  frameData = 0u;

    if( SPI_FRAME_BYTES_8BIT == xferCtx->FrameBytes )
    {
        frameData = LL_SPI_ReceiveData8( periphReg );
    }
    else if( SPI_FRAME_BYTES_16BIT == xferCtx->FrameBytes )
    {
        frameData = LL_SPI_ReceiveData16( periphReg );
    }
    else
    {
        frameData = LL_SPI_ReceiveData32( periphReg );
    }

    if( SPI_NULL_PTR != xferCtx->Request.RxData )
    {
        spi_Data_t * const frameBuf = &xferCtx->Request.RxData[ (uint32_t)rxIdx * xferCtx->FrameBytes ];

        /* Frame is stored little endian */
        for( spi_FrameBytes_t byteIdx = 0u; xferCtx->FrameBytes > byteIdx; byteIdx ++ )
        {
            frameBuf[ byteIdx ] = (spi_Data_t)frameData;
            frameData           = frameData >> SPI_BITS_PER_BYTE;
        }
    }
    else
    {
        /* Receive buffer is not set - frame is discarded */
    }

    xferCtx->RxIdx = rxIdx + 1u;
}


/**
 * \brief Checks configuration of SPI_XFER_MODE_NONE (no mode specific resources)
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the parameters are valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_None_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId   ) &&
        ( SPI_NULL_PTR  != dataConfig )    )
    {
        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Transfer mode handler without resources (NONE mode, Run of POLL mode, CheckDone of ISR /
 *        POLL mode)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_REQUEST_OK for a valid periphId. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_None_Xfer( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

#ifdef SPI1
/**
 * \brief SPI1 global interrupt service routine
 */
static void Spi_Spi1_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_1 );
}
#endif /* SPI1 */

#ifdef SPI2
/**
 * \brief SPI2 global interrupt service routine
 */
static void Spi_Spi2_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_2 );
}
#endif /* SPI2 */

#ifdef SPI3
/**
 * \brief SPI3 global interrupt service routine
 */
static void Spi_Spi3_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_3 );
}
#endif /* SPI3 */

#ifdef SPI4
/**
 * \brief SPI4 global interrupt service routine
 */
static void Spi_Spi4_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_4 );
}
#endif /* SPI4 */

#ifdef SPI5
/**
 * \brief SPI5 global interrupt service routine
 */
static void Spi_Spi5_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_5 );
}
#endif /* SPI5 */

#ifdef SPI6
/**
 * \brief SPI6 global interrupt service routine
 */
static void Spi_Spi6_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_6 );
}
#endif /* SPI6 */

/* ================================ TASKS =================================== */
