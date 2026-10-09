/**
 * \author Mr.Nobody
 * \file Spi.c
 * \ingroup Spi
 * \brief Serial Peripheral Interface (SPI) MCAL module common functionality
 *
 * The module drives SPI peripherals of STM32F4 in master or slave mode:
 * - configuration: SCK frequency (baudrate prescaler is calculated from the APB clock), clock
 *   mode, data size (8 / 16 bits), bit order, communication direction, Motorola / TI frame
 *   format, NSS management, hardware CRC, SCK / MISO / MOSI / NSS pins
 * - transfers: full-duplex, simplex and half-duplex transfers of \ref spi_XferRequest_t
 *
 * STM32F4 SPI has no transfer size counter and no end of transfer flag - frames are counted by
 * the module (ISR / POLL) or by DMA, the end of transfer is detected by the module:
 * - start:  direction of the transfer is configured, CRC is reset, mode resources are armed and
 *           the peripheral is enabled (SPE) for the duration of one transfer
 * - end:    all frames were moved, CRC frame is received and checked, the transmission finished
 *           (TXE = 1, BSY = 0) and the peripheral is disabled
 * - errors: OVR / MODF / CRCERR / FRE terminate the transfer immediately
 *
 * Hardware configuration of the communication directions (the configured direction is stored
 * by the module, STM32F4 has no simplex transmit configuration):
 * - full-duplex:        2 lines (BIDIMODE = 0, RXONLY = 0)
 * - simplex TX master:  bidirectional output (BIDIMODE = 1, BIDIOE = 1)
 * - simplex TX slave:   2 lines, received frames are discarded (end of transfer is counted)
 * - simplex RX master:  2 lines, zero frames are transmitted (master clocks only while it
 *                       transmits, MOSI pin is not configured by the application)
 * - simplex RX slave:   receive only (RXONLY = 1)
 * - half-duplex:        bidirectional line, BIDIOE given by the request. Master reception clocks
 *                       continuously - the peripheral is disabled during the last frame
 *                       (reference manual procedure "disabling the SPI").
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
/* ============================== TYPEDEFS ================================== */

/** \brief Configuration of one SPI peripheral */
typedef struct
{
    SPI_TypeDef          *PeriphReg;  /**< Peripheral registers                        */
    rcc_PeriphId_t        PeriphRcc;  /**< RCC identification (clock, reset, frequency) */
    nvic_PeriphIrqList_t  PeriphNvic; /**< Interrupt NVIC identification               */
    nvic_IsrCallback_t    PeriphIsr;  /**< Interrupt service routine                   */
}   spi_PeriphConfigStruct_t;


/** \brief Runtime configuration of one SPI peripheral not stored in its registers */
typedef struct
{
    spi_Direction_t       Direction;  /**< Configured communication direction          */
    spi_FunctionState_t   SckPinUsed; /**< SCK pin was configured by Spi_Init()        */
    spi_PinCode_t         SckPin;     /**< SCK pin (pull follows the clock polarity)   */
    spi_PinSpeed_t        SckSpeed;   /**< Output speed of the configured SCK pin      */
}   spi_PeriphRuntime_t;

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
static spi_RequestState_t  Spi_Check_DataSize       ( spi_DataSize_t dataSize );
static spi_RequestState_t  Spi_Check_ConfigAllowed  ( spi_PeriphId_t periphId );
static spi_RequestState_t  Spi_Check_XferRequest    ( spi_PeriphId_t periphId, const spi_XferRequest_t * const xferRequest );
static spi_RequestState_t  Spi_Check_DataConfig     ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );

static spi_RequestState_t  Spi_Set_Enable           ( spi_PeriphId_t periphId, spi_FunctionState_t enableState );
static spi_RequestState_t  Spi_Set_RoleReg          ( spi_PeriphId_t periphId, spi_Mode_t mode, spi_NssMode_t nssMode );
static spi_RequestState_t  Spi_Set_SlaveRxReset     ( spi_PeriphId_t periphId, spi_Mode_t mode );
static spi_RequestState_t  Spi_Check_SckSpeed       ( spi_PeriphId_t periphId, spi_PinSpeed_t pinSpeed );
static spi_XferErrorId_t   Spi_Set_SlaveTxStop      ( spi_PeriphId_t periphId );
static spi_RequestState_t  Spi_Set_DirectionReg     ( spi_PeriphId_t periphId, spi_Direction_t direction, spi_FunctionState_t txUsed );
static spi_RequestState_t  Spi_Set_Pin              ( spi_PinCode_t pinCode, spi_PinSpeed_t pinSpeed, gpio_PinPullCfg_t pinPull );
static gpio_PinPullCfg_t   Spi_Get_SckPull          ( spi_ClockMode_t clockMode );
static spi_RequestState_t  Spi_Get_KernelClk        ( spi_PeriphId_t periphId, spi_FreqHz_t * const clkFreq );
static spi_RequestState_t  Spi_Get_SckCycles        ( spi_PeriphId_t periphId, spi_TimeoutCnt_t * const sckCycles );
static void                Spi_Set_SckWait          ( spi_TimeoutCnt_t coreCycles );
static void                Spi_Set_FlagsClear       ( spi_PeriphId_t periphId );
static spi_FunctionState_t Spi_Get_FlagWait         ( spi_PeriphId_t periphId, spi_StatusFlags_t flagMask, spi_StatusFlags_t flagValue );

static spi_RequestState_t  Spi_Set_XferInit         ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
static spi_RequestState_t  Spi_Set_XferDeinit       ( spi_PeriphId_t periphId );
static spi_FunctionState_t Spi_Get_IrqUsed          ( const spi_DataConfig_t * const dataConfig );
static spi_RequestState_t  Spi_Set_IrqInit          ( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio );
static spi_RequestState_t  Spi_Set_IrqDeinit        ( spi_PeriphId_t periphId );

static spi_RequestState_t  Spi_Set_XferEnd          ( spi_PeriphId_t periphId, spi_XferErrorId_t errorId );
static spi_RequestState_t  Spi_Set_XferAbort        ( spi_PeriphId_t periphId );
static void                Spi_Set_FrameWrite       ( spi_PeriphId_t periphId );
static void                Spi_Get_FrameRead        ( spi_PeriphId_t periphId );
static void                Spi_Set_CrcNext          ( spi_PeriphId_t periphId );

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

/** Count of baudrate prescaler values (BR field, division by 2 - 256) */
#define SPI_PRESC_CNT                   ( 8u )

/** Offset between BR field value and division shift (BR = 0 -> division by 2) */
#define SPI_PRESC_SHIFT_OFFSET          ( 1u )

/** Transmitted, not yet received frames of master (master clocks only when it transmits) */
#define SPI_IN_FLIGHT_MASTER            ( 1u )

/** Transmitted, not yet received frames of slave (data register + shift register) */
#define SPI_IN_FLIGHT_SLAVE             ( 2u )

/** Buffer bytes of 8-bit frame */
#define SPI_FRAME_BYTES_8BIT            ( 1u )

/** Buffer bytes of 16-bit frame */
#define SPI_FRAME_BYTES_16BIT           ( 2u )

/** Count of bits in one byte (frame assembly) */
#define SPI_BITS_PER_BYTE               ( 8u )

/** Count of bits of 8-bit CRC (polynomial range check) */
#define SPI_CRC_BITS_8                  ( 8u )

/** Count of bits of 16-bit CRC (polynomial range check) */
#define SPI_CRC_BITS_16                 ( 16u )

/** Lowest bit of CRC polynomial - CRC calculation is wrong with an even polynomial (device
 *  errata "Wrong CRC calculation when the polynomial is even") */
#define SPI_CRC_POLY_ODD_MASK           ( 1u )

/** Maximal kernel clock of master with low speed SCK pin (device errata, ES table at 30 pF) */
#define SPI_SCK_LOW_MAX_CLK_HZ          ( 25000000u )

/** Maximal kernel clock of master with medium speed SCK pin (device errata, ES table at 30 pF) */
#define SPI_SCK_MEDIUM_MAX_CLK_HZ       ( 75000000u )

/** Kernel clock of master is not limited by the SCK pin speed */
#define SPI_SCK_NO_MAX_CLK_HZ           ( UINT32_MAX )

/** Minimal count of CPU cycles of one iteration of the SCK busy-wait loop (SysTick stopped) */
#define SPI_SCK_LOOP_CYCLES             ( 4u )

/** Divider of SysTick external clock source (core clock / 8) */
#define SPI_SYSTICK_EXT_DIV             ( 8u )

/* =============================== MACROS =================================== */

/** CR1 fields of master / slave role and software NSS */
#define SPI_CR1_ROLE_MASK               ( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI )

/** CR1 fields of communication direction */
#define SPI_CR1_DIR_MASK                ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY )

/** CR1 fields of clock polarity and phase */
#define SPI_CR1_CLOCK_MASK              ( SPI_CR1_CPOL | SPI_CR1_CPHA )

/** Mask of polynomial register */
#define SPI_CRCPR_MASK                  ( SPI_CRCPR_CRCPOLY )

/** CR1 configuration restored after the peripheral reset of a slave receiver becoming master
 *  (role, direction, enable and CRCNEXT are not restored) */
#define SPI_CR1_RESTORE_MASK            ( SPI_CR1_CLOCK_MASK | SPI_CR1_BR | SPI_CR1_LSBFIRST | \
                                          SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_DFF | SPI_CR1_CRCEN )

/** CR2 configuration restored after the peripheral reset of a slave receiver becoming master */
#define SPI_CR2_RESTORE_MASK            ( SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_SSOE | \
                                          SPI_CR2_FRF | SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief SPI peripherals configuration array */
static const spi_PeriphConfigStruct_t spi_PeriphConf[ ] =
{
#ifdef SPI1
    { .PeriphReg = SPI1, .PeriphRcc = RCC_PERIPH_SPI1, .PeriphNvic = NVIC_PERIPH_IRQ_SPI1, .PeriphIsr = Spi_Spi1_IsrHandler },
#endif
#ifdef SPI2
    { .PeriphReg = SPI2, .PeriphRcc = RCC_PERIPH_SPI2, .PeriphNvic = NVIC_PERIPH_IRQ_SPI2, .PeriphIsr = Spi_Spi2_IsrHandler },
#endif
#ifdef SPI3
    { .PeriphReg = SPI3, .PeriphRcc = RCC_PERIPH_SPI3, .PeriphNvic = NVIC_PERIPH_IRQ_SPI3, .PeriphIsr = Spi_Spi3_IsrHandler },
#endif
#ifdef SPI4
    { .PeriphReg = SPI4, .PeriphRcc = RCC_PERIPH_SPI4, .PeriphNvic = NVIC_PERIPH_IRQ_SPI4, .PeriphIsr = Spi_Spi4_IsrHandler },
#endif
#ifdef SPI5
    { .PeriphReg = SPI5, .PeriphRcc = RCC_PERIPH_SPI5, .PeriphNvic = NVIC_PERIPH_IRQ_SPI5, .PeriphIsr = Spi_Spi5_IsrHandler },
#endif
#ifdef SPI6
    { .PeriphReg = SPI6, .PeriphRcc = RCC_PERIPH_SPI6, .PeriphNvic = NVIC_PERIPH_IRQ_SPI6, .PeriphIsr = Spi_Spi6_IsrHandler },
#endif
};

_Static_assert( SPI_PERIPH_CNT == ( sizeof(spi_PeriphConf) / sizeof(spi_PeriphConfigStruct_t) ), "Spi: size of spi_PeriphConf is incorrect." );


/** \brief spi_ClockMode_t -> CR1 CPOL / CPHA */
static const spi_RegValue_t spi_ClockModeLut[ SPI_CLOCK_MODE_CNT ] =
{
    [SPI_CLOCK_MODE_0] = ( LL_SPI_POLARITY_LOW  | LL_SPI_PHASE_1EDGE ),
    [SPI_CLOCK_MODE_1] = ( LL_SPI_POLARITY_LOW  | LL_SPI_PHASE_2EDGE ),
    [SPI_CLOCK_MODE_2] = ( LL_SPI_POLARITY_HIGH | LL_SPI_PHASE_1EDGE ),
    [SPI_CLOCK_MODE_3] = ( LL_SPI_POLARITY_HIGH | LL_SPI_PHASE_2EDGE ),
};


/** \brief spi_FrameFormat_t -> CR2 FRF */
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


/** \brief spi_PinSpeed_t -> maximal SPI kernel (APB) clock of master with the SCK pin speed -
 *         device errata "Corrupted last bit of data and/or CRC received in Master mode with
 *         delayed SCK feedback" (ES table at 30 pF SCK load: low 25 MHz, medium 75 MHz, high /
 *         very high 84 MHz - the fastest pads are not limited, see \ref spi_PinSpeed_t) */
static const spi_FreqHz_t spi_SckSpeedMaxClkLut[ SPI_PIN_SPEED_CNT ] =
{
    [SPI_PIN_SPEED_LOW]       = SPI_SCK_LOW_MAX_CLK_HZ,
    [SPI_PIN_SPEED_MEDIUM]    = SPI_SCK_MEDIUM_MAX_CLK_HZ,
    [SPI_PIN_SPEED_HIGH]      = SPI_SCK_NO_MAX_CLK_HZ,
    [SPI_PIN_SPEED_VERY_HIGH] = SPI_SCK_NO_MAX_CLK_HZ,
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

/** \brief Runtime configuration per peripheral */
static spi_PeriphRuntime_t spi_PeriphRuntime[ SPI_PERIPH_CNT ];

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
 * Data handling of a previous initialization is released, the peripheral clock is enabled, the
 * peripheral is reset, configured (it stays disabled until a transfer is started), pins are
 * configured and the data handling is initialized (if DataConfig is set). CRC is disabled after
 * the initialization (Spi_Set_CrcConfig()).
 *
 * \note  SCK pin gets pull-down (CPOL = 0) or pull-up (CPOL = 1) - the line keeps the idle level
 *        while the peripheral is disabled between transfers. NSS pin gets pull-up (active low).
 *
 * \note  Device errata ("Corrupted last bit of data and/or CRC received in Master mode with
 *        delayed SCK feedback"): master with SCK pin is refused if the pin speed is too slow for
 *        the kernel clock (see \ref spi_PinSpeed_t).
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
        const rcc_PeriphId_t rccId    = spi_PeriphConf[ periphId ].PeriphRcc;
        rcc_RequestState_t   rccState = RCC_REQUEST_ERROR;

        /*------------- Data handling of previous initialization -------------*/
        retState = Spi_Set_XferDeinit( periphId );

        /*------------------ Clock activation and reset -----------------------*/
        if( SPI_REQUEST_OK == retState )
        {
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

        /* Peripheral registers are in reset state - runtime configuration as well */
        spi_PeriphRuntime[ periphId ].Direction  = SPI_DIRECTION_FULL_DUPLEX;
        spi_PeriphRuntime[ periphId ].SckPinUsed = SPI_FUNCTION_INACTIVE;
        spi_PeriphRuntime[ periphId ].SckPin     = SPI_PIN_UNUSED;
        spi_PeriphRuntime[ periphId ].SckSpeed   = SPI_PIN_SPEED_HIGH;

        /*------------- Peripheral configuration (SPE = 0 after reset) -------*/
        /* NSS management first - role is configured together with the internal slave select */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_NssConfig( periphId, &spiConfig->NssConfig );
        }
        else
        {
            /* Error during initialization process */
        }

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

        /* Device errata: SCK pin of master has to be fast enough for the kernel clock */
        if( ( SPI_REQUEST_OK     == retState          ) &&
            ( SPI_MODE_MASTER    == spiConfig->Mode   ) &&
            ( SPI_SCK_PIN_UNUSED != spiConfig->SckPin )    )
        {
            retState = Spi_Check_SckSpeed( periphId, spiConfig->PinSpeed );
        }
        else
        {
            /* Error during initialization process, slave mode or SCK pin not configured */
        }

        /*------------------------ GPIO pins ---------------------------------*/
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->SckPin, spiConfig->PinSpeed, Spi_Get_SckPull( spiConfig->ClockMode ) );

            if( ( SPI_REQUEST_OK     == retState          ) &&
                ( SPI_SCK_PIN_UNUSED != spiConfig->SckPin )    )
            {
                spi_PeriphRuntime[ periphId ].SckPinUsed = SPI_FUNCTION_ACTIVE;
                spi_PeriphRuntime[ periphId ].SckPin     = (spi_PinCode_t)spiConfig->SckPin;
                spi_PeriphRuntime[ periphId ].SckSpeed   = spiConfig->PinSpeed;
            }
            else
            {
                /* SCK pin is not configured by the module */
            }
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->MisoPin, spiConfig->PinSpeed, GPIO_PIN_PULL_NONE );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->MosiPin, spiConfig->PinSpeed, GPIO_PIN_PULL_NONE );
        }
        else
        {
            /* Error during initialization process */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_Pin( (spi_PinCode_t)spiConfig->NssPin, spiConfig->PinSpeed, GPIO_PIN_PULL_UP );
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
 * Running transfer is aborted, data handling resources (DMA streams, SPI interrupt) are
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
        const rcc_PeriphId_t rccId = spi_PeriphConf[ periphId ].PeriphRcc;

        /* -1- Data handling (transfer aborted, DMA streams and SPI interrupt released) */
        const spi_RequestState_t xferState = Spi_Set_XferDeinit( periphId );

        /* -2- Peripheral */
        const spi_RequestState_t periphState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );

        /* -3- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( rccId );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( rccId );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( rccId );

        spi_PeriphRuntime[ periphId ].Direction  = SPI_DIRECTION_FULL_DUPLEX;
        spi_PeriphRuntime[ periphId ].SckPinUsed = SPI_FUNCTION_INACTIVE;
        spi_PeriphRuntime[ periphId ].SckPin     = SPI_PIN_UNUSED;
        spi_PeriphRuntime[ periphId ].SckSpeed   = SPI_PIN_SPEED_HIGH;

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
 * \note  Master transmits only when the task writes a frame - a slow task only slows the
 *        transfer down. Master half-duplex reception and slave transfers are clocked
 *        independently of the task - overrun is reported if the task is not called at least
 *        once per frame.
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
 * SPI_PERIPH_1, APB kernel clock, master, 1 MHz, clock mode 0, 8-bit frames, MSB first,
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
        spiConfig->ClkSrc             = SPI_CLK_SRC_PCLK;
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
 * \note  Internal slave select (software NSS) and NSS output (hardware NSS) follow the role.
 *
 * \note  Device errata ("Anticipated communication upon SPI transit from slave receiver to
 *        master"): a slave configured as receiver (RXONLY, or BIDIMODE without BIDIOE) is reset
 *        by RCC before it becomes master, the configuration is restored.
 *
 * \note  Device errata ("Corrupted last bit of data and/or CRC received in Master mode with
 *        delayed SCK feedback"): master is refused if the SCK pin configured by Spi_Init() is
 *        too slow for the kernel clock (see \ref spi_PinSpeed_t).
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
        const spi_RegValue_t softNss = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_SSM );
        const spi_NssMode_t  nssMode = ( 0u != softNss ) ? SPI_NSS_MODE_SOFT : SPI_NSS_MODE_HARD;

        /* Device errata: SCK pin configured by Spi_Init() has to be fast enough for a master */
        if( ( SPI_MODE_MASTER     == mode                                     ) &&
            ( SPI_FUNCTION_ACTIVE == spi_PeriphRuntime[ periphId ].SckPinUsed )    )
        {
            retState = Spi_Check_SckSpeed( periphId, spi_PeriphRuntime[ periphId ].SckSpeed );
        }
        else
        {
            /* Slave or SCK pin is not configured by the module */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_SlaveRxReset( periphId, mode );
        }
        else
        {
            /* SCK pin is too slow for the kernel clock */
        }

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_RoleReg( periphId, mode, nssMode );
        }
        else
        {
            /* SCK pin is too slow or peripheral reset failed */
        }

        /* Simplex reception is configured differently for master and slave */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_DirectionReg( periphId, spi_PeriphRuntime[ periphId ].Direction, SPI_FUNCTION_INACTIVE );
        }
        else
        {
            /* Role configuration failed */
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_MSTR );

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
 * \note  Applies to master mode only. The APB clock is read from RCC - after its change the
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
 *         APB clock / 256) returns \ref SPI_REQUEST_ERROR.
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
                retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR1,
                                             SPI_CR1_BR,
                                             ( prescIdx << SPI_CR1_BR_Pos ) & SPI_CR1_BR );
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
 * \brief Returns SCK frequency (APB clock divided by the configured prescaler)
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
        const spi_RegValue_t prescIdx = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_BR ) >> SPI_CR1_BR_Pos;

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
 * \note  Pull of the SCK pin configured by Spi_Init() follows the clock polarity.
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
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR1,
                                     SPI_CR1_CLOCK_MASK,
                                     spi_ClockModeLut[ clockMode ] );

        if( ( SPI_REQUEST_OK      == retState                                  ) &&
            ( SPI_FUNCTION_ACTIVE == spi_PeriphRuntime[ periphId ].SckPinUsed  )    )
        {
            const spi_PinCode_t       sckPin    = spi_PeriphRuntime[ periphId ].SckPin;
            const gpio_RequestState_t gpioState = Gpio_Set_PinPull( (gpio_PortId_t)SPI_BIT_MASK_DECODE_PORT( sckPin ),
                                                                    (gpio_PinId_t)SPI_BIT_MASK_DECODE_PIN( sckPin ),
                                                                    Spi_Get_SckPull( clockMode ) );

            retState = ( GPIO_REQUEST_OK == gpioState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
        }
        else
        {
            /* Configuration failed or SCK pin is not configured by the module */
        }
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_CLOCK_MASK );

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
 * \brief Configures size of data frame (8 or 16 bits)
 *
 * \note  CRC length follows the data size - with CRC enabled the polynomial has to fit into the
 *        new size.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataSize [in]: Required data size - SPI_DATA_SIZE_8BIT or SPI_DATA_SIZE_16BIT
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_DataSize( spi_PeriphId_t periphId, spi_DataSize_t dataSize )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Check_DataSize( dataSize );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        /* Data size is not supported */
    }

    if( SPI_REQUEST_OK == retState )
    {
        SPI_TypeDef * const  periphReg  = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t crcEnabled = READ_BIT( periphReg->CR1, SPI_CR1_CRCEN );
        const spi_RegValue_t crcPoly    = READ_BIT( periphReg->CRCPR, SPI_CRCPR_MASK );
        const spi_RegValue_t sizeValue  = ( SPI_DATA_SIZE_16BIT == dataSize ) ? LL_SPI_DATAWIDTH_16BIT : LL_SPI_DATAWIDTH_8BIT;

        if( ( 0u                  == crcEnabled                ) ||
            ( SPI_DATA_SIZE_16BIT == dataSize                  ) ||
            ( 0u                  == ( crcPoly >> SPI_CRC_BITS_8 ) )    )
        {
            retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_DFF, sizeValue );
        }
        else
        {
            /* Polynomial of the enabled CRC does not fit into 8-bit CRC */
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_DFF );

        *dataSize = ( 0u != regValue ) ? SPI_DATA_SIZE_16BIT : SPI_DATA_SIZE_8BIT;
        retState  = SPI_REQUEST_OK;
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
        const spi_RegValue_t orderValue = ( SPI_BIT_ORDER_LSB_FIRST == bitOrder ) ? LL_SPI_LSB_FIRST : LL_SPI_MSB_FIRST;

        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_LSBFIRST, orderValue );
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_LSBFIRST );

        *bitOrder = ( 0u != regValue ) ? SPI_BIT_ORDER_LSB_FIRST : SPI_BIT_ORDER_MSB_FIRST;
        retState  = SPI_REQUEST_OK;
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
 * \note  The direction is stored by the module, hardware direction (BIDIMODE / BIDIOE / RXONLY)
 *        is configured according to the role and the request at the start of every transfer.
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
        retState = Spi_Set_DirectionReg( periphId, direction, SPI_FUNCTION_INACTIVE );

        if( SPI_REQUEST_OK == retState )
        {
            spi_PeriphRuntime[ periphId ].Direction = direction;
        }
        else
        {
            /* Direction configuration failed, stored direction is kept */
        }
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
        *direction = spi_PeriphRuntime[ periphId ].Direction;
        retState   = SPI_REQUEST_OK;
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
 * \note  In TI format clock mode, bit order and NSS behavior are given by the protocol.
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
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR2, SPI_CR2_FRF, spi_FrameFormatLut[ frameFormat ] );
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR2, SPI_CR2_FRF );

        *frameFormat = ( 0u != regValue ) ? SPI_FRAME_FORMAT_TI : SPI_FRAME_FORMAT_MOTOROLA;
        retState     = SPI_REQUEST_OK;
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
 * \brief Configures NSS management (software / hardware)
 *
 * \note  Software NSS: master does not use the NSS pin (slave select is driven by application
 *        GPIO), slave is permanently selected. Hardware NSS: master drives NSS low while the
 *        peripheral is enabled (one transfer), slave is selected by NSS input. STM32F4 supports
 *        active low NSS without pulses only.
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param nssConfig [in]: Pointer to NSS configuration. Must not be NULL, polarity has to be
 *                        low and pulse inactive.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_NssConfig( spi_PeriphId_t periphId, const spi_NssConfig_t * const nssConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_Mode_t         mode     = SPI_MODE_MASTER;

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
        retState = Spi_Get_Mode( periphId, &mode );
    }
    else
    {
        /* Configuration is invalid or transfer is running, nothing is modified */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RoleReg( periphId, mode, nssConfig->Mode );
    }
    else
    {
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
        const spi_RegValue_t regValue = READ_BIT( spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_SSM );

        nssConfig->Mode     = ( 0u != regValue ) ? SPI_NSS_MODE_SOFT : SPI_NSS_MODE_HARD;
        nssConfig->Polarity = SPI_NSS_POLARITY_LOW;
        nssConfig->Pulse    = SPI_FUNCTION_INACTIVE;

        retState = SPI_REQUEST_OK;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures master idle timing - STM32F4 has no idle cycles between data frames or
 *        after NSS activation, only zero cycles are accepted
 *
 * \pre   No transfer may be running. Otherwise \ref SPI_REQUEST_ERROR is returned.
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
        /* Nothing to be configured - the request is accepted while the configuration may be changed */
        retState = Spi_Check_ConfigAllowed( periphId );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads master idle timing (always zero cycles on STM32F4)
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
        *interDataIdle = 0u;
        *ssIdle        = 0u;

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
 * \brief Configures hardware CRC (state and polynomial)
 *
 * \note  If CRC is disabled, only the CRC calculation is switched off - other fields are not
 *        checked and not modified. CRC length is given by the data size (Size has to be equal to
 *        it), CRC is always initialized with zeros.
 *
 * \note  Device errata: the CRC calculation is wrong if the polynomial is even - only odd
 *        polynomials (bit 0 set) are accepted.
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
        const spi_RegValue_t crcBits = ( SPI_DATA_SIZE_16BIT == dataSize ) ? SPI_CRC_BITS_16 : SPI_CRC_BITS_8;

        if( ( dataSize               == crcConfig->Size                    ) &&
            ( 0u                     != ( crcConfig->Polynomial & SPI_CRC_POLY_ODD_MASK ) ) &&
            ( 0u                     == ( crcConfig->Polynomial >> crcBits ) ) &&
            ( SPI_CRC_INIT_ALL_ZERO   == crcConfig->InitValue               )    )
        {
            SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;

            retState = Spi_Set_RegField( &periphReg->CRCPR, SPI_CRCPR_MASK, crcConfig->Polynomial );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_CRCEN, SPI_CR1_CRCEN );
            }
            else
            {
                /* Polynomial configuration failed */
            }
        }
        else
        {
            /* CRC configuration is not supported, nothing is modified */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_CRCEN, 0u );
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
        const spi_RegValue_t cr1Value  = LL_SPI_ReadReg( periphReg, CR1 );

        crcConfig->State      = ( 0u != ( SPI_CR1_CRCEN & cr1Value ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        crcConfig->Size       = ( 0u != ( SPI_CR1_DFF   & cr1Value ) ) ? SPI_DATA_SIZE_16BIT : SPI_DATA_SIZE_8BIT;
        crcConfig->Polynomial = (spi_CrcPoly_t)LL_SPI_GetCRCPolynomial( periphReg );
        crcConfig->InitValue  = SPI_CRC_INIT_ALL_ZERO;

        retState = SPI_REQUEST_OK;
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
 *        the new mode is initialized (DMA streams, SPI interrupt in NVIC)
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
 * \note  CRC is not supported in master half-duplex reception (the peripheral is disabled before
 *        the last frame - the CRC phase can not be clocked).
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
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_DataSize_t     dataSize = SPI_DATA_SIZE_8BIT;
    spi_Mode_t         mode     = SPI_MODE_MASTER;

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
        const spi_RequestState_t modeState = Spi_Get_Mode( periphId, &mode );
        const spi_RequestState_t sizeState = Spi_Get_DataSize( periphId, &dataSize );

        if( ( SPI_REQUEST_OK      != modeState                                  ) ||
            ( SPI_REQUEST_OK      != sizeState                                  ) ||
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
        SPI_TypeDef * const            periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        spi_XferContext_t * const      xferCtx   = &spi_XferContext[ periphId ];
        const spi_XferModeIf_t * const modeIf    = &spi_XferModeLut[ xferCtx->Config.XferMode ];
        const spi_Direction_t          direction = spi_PeriphRuntime[ periphId ].Direction;
        const spi_FunctionState_t      isMaster  = ( SPI_MODE_MASTER == mode ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        const spi_FunctionState_t      halfTx    = ( ( SPI_DIRECTION_HALF_DUPLEX == direction ) && ( SPI_NULL_PTR != xferRequest->TxData ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        const spi_FunctionState_t      halfRx    = ( ( SPI_DIRECTION_HALF_DUPLEX == direction ) && ( SPI_NULL_PTR == xferRequest->TxData ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;

        xferCtx->Request     = *xferRequest;
        xferCtx->FrameBytes  = ( SPI_DATA_SIZE_16BIT == dataSize ) ? SPI_FRAME_BYTES_16BIT : SPI_FRAME_BYTES_8BIT;
        xferCtx->InFlightMax = ( SPI_FUNCTION_ACTIVE == isMaster ) ? SPI_IN_FLIGHT_MASTER : SPI_IN_FLIGHT_SLAVE;
        xferCtx->TxIdx       = 0u;
        xferCtx->RxIdx       = 0u;
        xferCtx->XferError   = SPI_XFER_ERROR_NONE;
        xferCtx->TxUsed      = SPI_FUNCTION_INACTIVE;
        xferCtx->RxUsed      = SPI_FUNCTION_INACTIVE;
        xferCtx->CrcUsed     = ( 0u != LL_SPI_IsEnabledCRC( periphReg ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;
        xferCtx->CrcCheck    = ( SPI_DIRECTION_SIMPLEX_TX != direction ) ? xferCtx->CrcUsed : SPI_FUNCTION_INACTIVE;
        xferCtx->CrcNext     = SPI_FUNCTION_INACTIVE;
        xferCtx->RxStopUsed  = SPI_FUNCTION_INACTIVE;
        xferCtx->RxStopped   = SPI_FUNCTION_INACTIVE;
        xferCtx->CpuRxTail   = SPI_FUNCTION_INACTIVE;
        xferCtx->SckCycles   = 0u;

        /* Frames are transmitted: transmitting directions, simplex reception of master (zero
           frames generate the clock) */
        if( ( SPI_DIRECTION_FULL_DUPLEX == direction ) ||
            ( SPI_DIRECTION_SIMPLEX_TX  == direction ) ||
            ( SPI_FUNCTION_ACTIVE       == halfTx    ) ||
            ( ( SPI_DIRECTION_SIMPLEX_RX == direction ) && ( SPI_FUNCTION_ACTIVE == isMaster ) ) )
        {
            xferCtx->TxUsed = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Nothing is transmitted */
        }

        /* Frames are received: receiving directions, simplex transmission of slave (received
           frames are discarded - they count the end of the transfer) */
        if( ( SPI_DIRECTION_FULL_DUPLEX == direction ) ||
            ( SPI_DIRECTION_SIMPLEX_RX  == direction ) ||
            ( SPI_FUNCTION_ACTIVE       == halfRx    ) ||
            ( ( SPI_DIRECTION_SIMPLEX_TX == direction ) && ( SPI_FUNCTION_INACTIVE == isMaster ) ) )
        {
            xferCtx->RxUsed = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Nothing is received */
        }

        /* Buffers of not requested directions are not used */
        if( ( SPI_DIRECTION_SIMPLEX_RX == direction ) || ( SPI_FUNCTION_ACTIVE == halfRx ) )
        {
            xferCtx->Request.TxData = SPI_NULL_PTR;
        }
        else
        {
            /* Transmit buffer of the request is used */
        }

        if( ( SPI_DIRECTION_SIMPLEX_TX == direction ) || ( SPI_FUNCTION_ACTIVE == halfTx ) )
        {
            xferCtx->Request.RxData = SPI_NULL_PTR;
        }
        else
        {
            /* Receive buffer of the request is used */
        }

        /* Master half-duplex reception clocks continuously - stopped during the last frame */
        if( ( SPI_FUNCTION_ACTIVE == halfRx ) && ( SPI_FUNCTION_ACTIVE == isMaster ) )
        {
            xferCtx->RxStopUsed = SPI_FUNCTION_ACTIVE;

            if( SPI_FUNCTION_ACTIVE == xferCtx->CrcUsed )
            {
                /* CRC phase can not be clocked */
                retState = SPI_REQUEST_ERROR;
            }
            else
            {
                retState = Spi_Get_SckCycles( periphId, &xferCtx->SckCycles );
            }
        }
        else
        {
            /* Transfer ends with the frames of the request */
        }

        /* Hardware direction of the transfer (writable while SPE = 0) */
        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Set_DirectionReg( periphId, direction, halfTx );
        }
        else
        {
            /* Transfer is not supported */
        }

        /* CRC is reset by disabling the calculation, CRC phase request of a previous transfer is cleared */
        if( ( SPI_REQUEST_OK      == retState         ) &&
            ( SPI_FUNCTION_ACTIVE == xferCtx->CrcUsed )    )
        {
            retState = Spi_Set_RegField( &periphReg->CR1, ( SPI_CR1_CRCEN | SPI_CR1_CRCNEXT ), 0u );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_CRCEN, SPI_CR1_CRCEN );
            }
            else
            {
                /* CRC reset failed */
            }
        }
        else
        {
            /* Previous step failed or CRC is not used */
        }

        /* Flags of a previous transfer are cleared */
        Spi_Set_FlagsClear( periphId );

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

        /* Transfer of one frame: the last frame is the first one (unless the interrupt has
           already finished the transfer) */
        if( ( SPI_REQUEST_OK      == retState           ) &&
            ( 1u                  == xferRequest->XferSize ) &&
            ( SPI_FUNCTION_ACTIVE == xferCtx->XferState  )    )
        {
            if( SPI_FUNCTION_ACTIVE == xferCtx->RxStopUsed )
            {
                retState = Spi_Set_XferRxStop( periphId );
            }
            else if( ( SPI_FUNCTION_ACTIVE   == xferCtx->CrcUsed         ) &&
                     ( SPI_FUNCTION_INACTIVE == xferCtx->TxUsed          ) &&
                     ( SPI_XFER_MODE_DMA     != xferCtx->Config.XferMode )    )
            {
                /* Receive only: CRC phase follows the only data frame */
                Spi_Set_CrcNext( periphId );
            }
            else
            {
                /* CRC phase is requested after the last data frame / by DMA */
            }
        }
        else
        {
            /* Start failed or several frames are transferred */
        }

        if( SPI_REQUEST_OK != retState )
        {
            /* Transfer was not started - mode resources are released, peripheral is disabled */
            (void)Spi_Set_XferAbort( periphId );
        }
        else
        {
            /* Master clocks with the first transmitted frame (half-duplex reception right away),
               slave transfer is clocked by the master */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Aborts running transfer - transfer mode resources are stopped and the peripheral is
 *        disabled. No callback is called.
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

        retState = ( NVIC_REQUEST_OK == nvicState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
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
 * \brief Returns if the next frame of the running transfer may be written to the data register
 *        (ISR / POLL mode)
 *
 * In receiving transfers the count of transmitted, not yet received frames is limited (master 1
 * frame - it clocks only while it transmits, slave 2 frames - data and shift register), the
 * received frames can never overrun because of the transmitter.
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
            ( ( SPI_FUNCTION_ACTIVE != xferCtx->RxUsed                       ) ||
              ( xferCtx->InFlightMax > (spi_DataCnt_t)( txIdx - rxIdx )      )    )    )
        {
            txPending = SPI_FUNCTION_ACTIVE;
        }
        else
        {
            /* Transmission finished or receiver could overrun */
        }
    }
    else
    {
        /* Invalid peripheral identification */
    }

    return ( txPending );
}


/**
 * \brief Moves frames of the running transfer (ISR / POLL mode, last frame of DMA master
 *        half-duplex reception): received frame is read if RXNE is set, frame is written if TXE
 *        is set and \ref Spi_Get_TxPending allows it. CRC phase is requested after the last
 *        transmitted frame (receive only: after the last but one received frame).
 *
 * \note  Flags are taken from the snapshot of SR read by the caller before the data step - a
 *        read of SR after the read of DR clears the overrun flag (OVR) before the caller could
 *        process it.
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param statusFlags [in]: Snapshot of SPI SR register read before the data step
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferDataStep( spi_PeriphId_t periphId, spi_StatusFlags_t statusFlags )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];

        retState = SPI_REQUEST_OK;

        if( SPI_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            const uint32_t rxReady = statusFlags & LL_SPI_SR_RXNE;

            /*------------------------- Received frame ---------------------------*/
            if( ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed        ) &&
                ( xferCtx->Request.XferSize > xferCtx->RxIdx    ) &&
                ( 0u != rxReady                                 )    )
            {
                Spi_Get_FrameRead( periphId );

                if( (spi_DataCnt_t)( xferCtx->Request.XferSize - 1u ) == xferCtx->RxIdx )
                {
                    if( ( SPI_FUNCTION_ACTIVE   == xferCtx->RxStopUsed ) &&
                        ( SPI_FUNCTION_INACTIVE == xferCtx->RxStopped  )    )
                    {
                        /* Last frame is being received - the clock is stopped after it */
                        retState = Spi_Set_XferRxStop( periphId );
                    }
                    else if( ( SPI_FUNCTION_ACTIVE   == xferCtx->CrcUsed ) &&
                             ( SPI_FUNCTION_INACTIVE == xferCtx->TxUsed  ) &&
                             ( SPI_FUNCTION_INACTIVE == xferCtx->CrcNext )    )
                    {
                        /* Receive only: CRC phase follows the last frame being received */
                        Spi_Set_CrcNext( periphId );
                    }
                    else
                    {
                        /* Nothing special for the last frame */
                    }
                }
                else
                {
                    /* More than one frame remains */
                }
            }
            else
            {
                /* No received frame */
            }

            /*------------------------ Transmitted frame -------------------------*/
            const uint32_t            txReady   = statusFlags & LL_SPI_SR_TXE;
            const spi_FunctionState_t txPending = Spi_Get_TxPending( periphId );

            if( ( SPI_FUNCTION_ACTIVE == txPending ) &&
                ( 0u                  != txReady   )    )
            {
                Spi_Set_FrameWrite( periphId );

                if( ( SPI_FUNCTION_ACTIVE == xferCtx->CrcUsed                ) &&
                    ( xferCtx->Request.XferSize == xferCtx->TxIdx            )    )
                {
                    /* CRC phase follows the last transmitted frame */
                    Spi_Set_CrcNext( periphId );
                }
                else
                {
                    /* More frames to be transmitted or CRC is not used */
                }
            }
            else
            {
                /* Data register is full or nothing may be transmitted */
            }
        }
        else
        {
            /* No transfer is running */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Processes error events and the end of the running transfer (all modes)
 *
 * - MODF / OVR / CRCERR / FRE: transfer ends immediately with error
 * - all frames were moved (ISR / POLL, last frame of DMA master half-duplex reception): the
 *   transfer is finished (\ref Spi_Set_XferFinish)
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
        spi_StatusFlags_t               errMask   = LL_SPI_SR_MODF;
        spi_XferErrorId_t               errorId   = SPI_XFER_ERROR_NONE;

        /* Overrun is relevant for reception (frames clocked after the stop of master half-duplex
           reception are not), CRC error for CRC phase, frame error for TI format */
        if( ( SPI_FUNCTION_ACTIVE   == xferCtx->RxUsed    ) &&
            ( SPI_FUNCTION_INACTIVE == xferCtx->RxStopped )    )
        {
            errMask |= LL_SPI_SR_OVR;
        }
        else
        {
            /* Receiver is not used */
        }

        if( SPI_FUNCTION_ACTIVE == xferCtx->CrcCheck )
        {
            errMask |= LL_SPI_SR_CRCERR;
        }
        else
        {
            /* CRC is not used */
        }

        if( 0u != READ_BIT( periphReg->CR2, SPI_CR2_FRF ) )
        {
            errMask |= LL_SPI_SR_FRE;
        }
        else
        {
            /* Motorola frame format */
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
            else if( 0u != ( LL_SPI_SR_CRCERR & errMask & statusFlags ) )
            {
                errorId = SPI_XFER_ERROR_CRC;
            }
            else
            {
                errorId = SPI_XFER_ERROR_FRAME;
            }

            retState = Spi_Set_XferError( periphId, errorId );
        }
        else if( ( SPI_XFER_MODE_DMA   != xferCtx->Config.XferMode ) ||
                 ( SPI_FUNCTION_ACTIVE == xferCtx->CpuRxTail       )    )
        {
            if( ( ( SPI_FUNCTION_ACTIVE != xferCtx->TxUsed ) || ( xferCtx->Request.XferSize == xferCtx->TxIdx ) ) &&
                ( ( SPI_FUNCTION_ACTIVE != xferCtx->RxUsed ) || ( xferCtx->Request.XferSize == xferCtx->RxIdx ) )    )
            {
                retState = Spi_Set_XferFinish( periphId );
            }
            else
            {
                /* Frames remain to be moved */
            }
        }
        else
        {
            /* End of DMA transfer is signaled by DMA */
        }
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Finishes the running transfer after all data frames were moved: CRC frame is received
 *        (and checked), the end of transmission is awaited (TXE = 1, BSY = 0) and the transfer
 *        ends (\ref Spi_Set_XferEnd)
 *
 * \note  Busy waits up to two frames (CRC frame, last transmitted frame) in the calling context.
 *
 * \note  Device errata ("BSY flag may stay high at the end of a data transfer in Slave mode"):
 *        slave transmitter without reception (half-duplex transmission) is disabled while the
 *        last frame is transmitted, BSY is awaited afterwards. With CRC the CRC frame follows the
 *        last data frame, the peripheral is not disabled before it (BSY may stay set).
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferFinish( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        SPI_TypeDef * const             periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_XferContext_t * const xferCtx   = &spi_XferContext[ periphId ];
        spi_XferErrorId_t               errorId   = SPI_XFER_ERROR_NONE;

        if( SPI_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            /* CRC frame is received after the data frames */
            if( ( SPI_FUNCTION_ACTIVE == xferCtx->CrcUsed ) &&
                ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed  )    )
            {
                const spi_FunctionState_t crcReceived = Spi_Get_FlagWait( periphId, LL_SPI_SR_RXNE, LL_SPI_SR_RXNE );

                if( SPI_FUNCTION_ACTIVE == crcReceived )
                {
                    /* CRC frame is not stored */
                    (void)LL_SPI_ReadReg( periphReg, DR );
                }
                else
                {
                    errorId = SPI_XFER_ERROR_INCOMPLETE;
                }
            }
            else
            {
                /* No CRC frame is received */
            }

            /* Transmission is finished: stopped master reception is already finished, slave
               which received all frames is finished too (BSY may stay set in slave mode -
               device errata, it is checked by slave transmit only transfers only, see below) */
            const spi_FunctionState_t isSlave = ( 0u == READ_BIT( periphReg->CR1, SPI_CR1_MSTR ) ) ? SPI_FUNCTION_ACTIVE : SPI_FUNCTION_INACTIVE;

            if( ( SPI_XFER_ERROR_NONE   == errorId             ) &&
                ( SPI_FUNCTION_INACTIVE == xferCtx->RxStopUsed ) &&
                ( ( SPI_FUNCTION_INACTIVE == isSlave         ) ||
                  ( SPI_FUNCTION_INACTIVE == xferCtx->RxUsed )    )    )
            {
                /* Device errata "BSY flag may stay high at the end of a data transfer in Slave
                   mode": slave transmitter is disabled while the last frame is transmitted
                   (TXE = 1), BSY signals the end of the frame correctly afterwards. CRC frame
                   follows the last data frame - the peripheral is not disabled before it. */
                if( ( SPI_FUNCTION_ACTIVE   == isSlave          ) &&
                    ( SPI_FUNCTION_INACTIVE == xferCtx->CrcUsed )    )
                {
                    errorId = Spi_Set_SlaveTxStop( periphId );
                }
                else
                {
                    /* Master or CRC frame is transmitted */
                }

                const spi_FunctionState_t txIdle = ( SPI_XFER_ERROR_NONE == errorId ) ? Spi_Get_FlagWait( periphId, ( LL_SPI_SR_TXE | LL_SPI_SR_BSY ), LL_SPI_SR_TXE )
                                                                                       : SPI_FUNCTION_INACTIVE;

                if( SPI_FUNCTION_ACTIVE != txIdle )
                {
                    errorId = SPI_XFER_ERROR_INCOMPLETE;
                }
                else
                {
                    /* Last frame was transferred */
                }
            }
            else
            {
                /* Error was detected or transmission is not used */
            }

            if( ( SPI_XFER_ERROR_NONE == errorId                          ) &&
                ( SPI_FUNCTION_ACTIVE == xferCtx->CrcCheck                ) &&
                ( 0u                  != LL_SPI_IsActiveFlag_CRCERR( periphReg ) )    )
            {
                errorId = SPI_XFER_ERROR_CRC;
            }
            else
            {
                /* CRC is not used or matches */
            }

            retState = Spi_Set_XferEnd( periphId, errorId );
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
 * \brief Stops the clock of master half-duplex reception during the last frame - waits one SCK
 *        period (the last frame has started) and disables the peripheral (the frame in progress
 *        is completed by the hardware)
 *
 * \note  The last but one frame has to be read within one frame period minus one SCK period,
 *        otherwise additional frames are clocked (they are discarded, overrun is not reported).
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Set_XferRxStop( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        spi_XferContext_t * const xferCtx = &spi_XferContext[ periphId ];

        Spi_Set_SckWait( xferCtx->SckCycles );

        xferCtx->RxStopped = SPI_FUNCTION_ACTIVE;

        retState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );
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
        ( SPI_CLK_SRC_CNT        > spiConfig->ClkSrc      ) &&
        ( SPI_MODE_CNT           > spiConfig->Mode        ) &&
        ( SPI_CLOCK_MODE_CNT     > spiConfig->ClockMode   ) &&
        ( SPI_BIT_ORDER_CNT      > spiConfig->BitOrder    ) &&
        ( SPI_DIRECTION_CNT      > spiConfig->Direction   ) &&
        ( SPI_FRAME_FORMAT_CNT   > spiConfig->FrameFormat ) &&
        ( SPI_PIN_SPEED_CNT      > spiConfig->PinSpeed    ) &&
        ( ( SPI_MODE_SLAVE == spiConfig->Mode ) || ( 0u < spiConfig->BusFreq ) )    )
    {
        const spi_RequestState_t sizeState = Spi_Check_DataSize( spiConfig->DataSize );
        const spi_RequestState_t nssState  = Spi_Check_NssConfig( &spiConfig->NssConfig );
        const spi_RequestState_t sckState  = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->SckPin );
        const spi_RequestState_t misoState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->MisoPin );
        const spi_RequestState_t mosiState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->MosiPin );
        const spi_RequestState_t nssPState = Spi_Check_Pin( spiConfig->PeriphId, (spi_PinCode_t)spiConfig->NssPin );

        if( ( SPI_REQUEST_OK == sizeState ) &&
            ( SPI_REQUEST_OK == nssState  ) &&
            ( SPI_REQUEST_OK == sckState  ) &&
            ( SPI_REQUEST_OK == misoState ) &&
            ( SPI_REQUEST_OK == mosiState ) &&
            ( SPI_REQUEST_OK == nssPState )    )
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
 * \brief Checks NSS configuration (values in range, STM32F4: active low without pulses)
 *
 * \param nssConfig [in]: Pointer to NSS configuration
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_NssConfig( const spi_NssConfig_t * const nssConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_NULL_PTR          != nssConfig           ) &&
        ( SPI_NSS_MODE_CNT       > nssConfig->Mode     ) &&
        ( SPI_NSS_POLARITY_LOW  == nssConfig->Polarity ) &&
        ( SPI_FUNCTION_INACTIVE == nssConfig->Pulse    )    )
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
 * \brief Checks data size supported by STM32F4 (8 or 16 bits)
 *
 * \param dataSize [in]: Data size, value from \ref spi_DataSize_t
 *
 * \return Returns \ref SPI_REQUEST_OK if the data size is supported. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_DataSize( spi_DataSize_t dataSize )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_DATA_SIZE_8BIT  == dataSize ) ||
        ( SPI_DATA_SIZE_16BIT == dataSize )    )
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
 *        peripheral is disabled
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
 * \brief Checks transfer request - size and buffers required by the configured communication
 *        direction
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

        if( ( SPI_FUNCTION_ACTIVE == bufOk                 ) &&
            ( 0u                   < xferRequest->XferSize )    )
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
 * \brief Configures role and NSS management registers: CR1 MSTR, SSM, SSI (software NSS: master
 *        is never selected by another master, slave is permanently selected) and CR2 SSOE
 *        (hardware NSS output of master)
 *
 * \note  Master: NSS output is configured before the role, slave: the role is configured before
 *        the NSS output is disabled - master never sees an active NSS input (mode fault).
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param mode     [in]: Role, value from \ref spi_Mode_t
 * \param nssMode  [in]: NSS management, value from \ref spi_NssMode_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_RoleReg( spi_PeriphId_t periphId, spi_Mode_t mode, spi_NssMode_t nssMode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT   > periphId ) &&
        ( SPI_MODE_CNT     > mode     ) &&
        ( SPI_NSS_MODE_CNT > nssMode  )    )
    {
        SPI_TypeDef * const  periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        spi_RegValue_t       roleValue = 0u;
        spi_RegValue_t       ssoeValue = 0u;

        if( SPI_MODE_MASTER == mode )
        {
            roleValue |= SPI_CR1_MSTR;
        }
        else
        {
            /* Slave */
        }

        if( SPI_NSS_MODE_SOFT == nssMode )
        {
            /* Internal slave select: master inactive (high), slave active (low) */
            roleValue |= ( SPI_MODE_MASTER == mode ) ? ( SPI_CR1_SSM | SPI_CR1_SSI ) : SPI_CR1_SSM;
        }
        else if( SPI_MODE_MASTER == mode )
        {
            ssoeValue = SPI_CR2_SSOE;
        }
        else
        {
            /* Slave is selected by NSS input */
        }

        if( SPI_MODE_MASTER == mode )
        {
            retState = Spi_Set_RegField( &periphReg->CR2, SPI_CR2_SSOE, ssoeValue );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_ROLE_MASK, roleValue );
            }
            else
            {
                /* NSS output configuration failed */
            }
        }
        else
        {
            retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_ROLE_MASK, roleValue );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR2, SPI_CR2_SSOE, ssoeValue );
            }
            else
            {
                /* Role configuration failed */
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
 * \brief Resets the peripheral by RCC before a slave receiver becomes master
 *
 * Device errata "Anticipated communication upon SPI transit from slave receiver to master": the
 * communication clock starts upon setting MSTR (even with SPE = 0) if the slave is configured as
 * receiver (RXONLY = 1, or BIDIMODE = 1 with BIDIOE = 0). The peripheral is reset and CRCPR, CR2
 * and CR1 (without role, direction and enable) are restored - the peripheral is a full-duplex
 * slave afterwards, role and direction are configured by the caller.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param mode     [in]: Required role, value from \ref spi_Mode_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if no reset is needed or the
 *         configuration was restored. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_SlaveRxReset( spi_PeriphId_t periphId, spi_Mode_t mode )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_MODE_CNT   > mode     )    )
    {
        SPI_TypeDef * const  periphReg = spi_PeriphConf[ periphId ].PeriphReg;
        const spi_RegValue_t cr1Reg    = READ_REG( periphReg->CR1 );
        const spi_RegValue_t bidiRx    = cr1Reg & ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE );

        if( ( SPI_MODE_MASTER  == mode                                        ) &&
            ( 0u               == ( cr1Reg & SPI_CR1_MSTR )                   ) &&
            ( ( 0u             != ( cr1Reg & SPI_CR1_RXONLY ) ) ||
              ( SPI_CR1_BIDIMODE == bidiRx                                  )    )    )
        {
            const spi_RegValue_t cr2Reg   = READ_REG( periphReg->CR2 );
            const spi_RegValue_t crcpReg  = READ_REG( periphReg->CRCPR );
            const rcc_PeriphId_t rccId    = spi_PeriphConf[ periphId ].PeriphRcc;
            rcc_RequestState_t   rccState = Rcc_Set_ResetActive( rccId );

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
                retState = Spi_Set_RegField( &periphReg->CRCPR, SPI_CRCPR_MASK, crcpReg );
            }
            else
            {
                retState = SPI_REQUEST_ERROR;
            }

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR2, SPI_CR2_RESTORE_MASK, cr2Reg );
            }
            else
            {
                /* Previous step failed */
            }

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_RESTORE_MASK, cr1Reg );
            }
            else
            {
                /* Previous step failed */
            }
        }
        else
        {
            /* No slave receiver becomes master, reset is not needed */
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
 * \brief Checks that the SCK pin of master is fast enough for the kernel clock
 *
 * Device errata "Corrupted last bit of data and/or CRC received in Master mode with delayed SCK
 * feedback": the last received bit is not captured when the internal SCK feedback is too slow
 * for the APB clock (see \ref spi_SckSpeedMaxClkLut).
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param pinSpeed [in]: Output speed of the SCK pin, value from \ref spi_PinSpeed_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if the kernel clock does not
 *         exceed the limit of the pin speed. Otherwise (also if the kernel clock is not
 *         available) returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Check_SckSpeed( spi_PeriphId_t periphId, spi_PinSpeed_t pinSpeed )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_FreqHz_t       clkFreq  = 0u;

    if( SPI_PIN_SPEED_CNT > pinSpeed )
    {
        retState = Spi_Get_KernelClk( periphId, &clkFreq );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( ( SPI_REQUEST_OK == retState                            ) &&
        ( spi_SckSpeedMaxClkLut[ pinSpeed ] < clkFreq )    )
    {
        /* Last received bit may be corrupted */
        retState = SPI_REQUEST_ERROR;
    }
    else
    {
        /* Pin is fast enough or the kernel clock is not available */
    }

    return ( retState );
}


/**
 * \brief Disables slave transmitter while its last frame is transmitted
 *
 * Device errata "BSY flag may stay high at the end of a data transfer in Slave mode" (transmit
 * workaround): the last frame was written, TXE is awaited (the last frame has started) and the
 * peripheral is disabled while the frame is still being transmitted - BSY works correctly
 * afterwards and reports the end of the frame.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_XFER_ERROR_NONE if the peripheral was disabled. Otherwise (TXE not
 *         set within the timeout, invalid periphId) returns \ref SPI_XFER_ERROR_INCOMPLETE.
 */
static spi_XferErrorId_t Spi_Set_SlaveTxStop( spi_PeriphId_t periphId )
{
    spi_XferErrorId_t errorId = SPI_XFER_ERROR_INCOMPLETE;

    if( SPI_PERIPH_CNT > periphId )
    {
        const spi_FunctionState_t txStarted = Spi_Get_FlagWait( periphId, LL_SPI_SR_TXE, LL_SPI_SR_TXE );

        if( SPI_FUNCTION_ACTIVE == txStarted )
        {
            const spi_RequestState_t disableState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );

            errorId = ( SPI_REQUEST_OK == disableState ) ? SPI_XFER_ERROR_NONE : SPI_XFER_ERROR_INCOMPLETE;
        }
        else
        {
            /* Last frame has not started */
            errorId = SPI_XFER_ERROR_INCOMPLETE;
        }
    }
    else
    {
        errorId = SPI_XFER_ERROR_INCOMPLETE;
    }

    return ( errorId );
}


/**
 * \brief Configures hardware direction (CR1 BIDIMODE, BIDIOE, RXONLY) of the communication
 *        direction for the configured role (see module description)
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param direction [in]: Communication direction, value from \ref spi_Direction_t
 * \param txUsed    [in]: Half-duplex - \ref SPI_FUNCTION_ACTIVE for transmission
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_DirectionReg( spi_PeriphId_t periphId, spi_Direction_t direction, spi_FunctionState_t txUsed )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_Mode_t         mode     = SPI_MODE_MASTER;

    if( SPI_DIRECTION_CNT > direction )
    {
        retState = Spi_Get_Mode( periphId, &mode );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        spi_RegValue_t dirValue = 0u;

        switch( direction )
        {
            case SPI_DIRECTION_SIMPLEX_TX:
                /* Master: bidirectional output, slave: 2 lines (received frames are counted) */
                dirValue = ( SPI_MODE_MASTER == mode ) ? ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE ) : 0u;
                break;

            case SPI_DIRECTION_SIMPLEX_RX:
                /* Master: 2 lines (zero frames generate the clock), slave: receive only */
                dirValue = ( SPI_MODE_MASTER == mode ) ? 0u : SPI_CR1_RXONLY;
                break;

            case SPI_DIRECTION_HALF_DUPLEX:
                dirValue = ( SPI_FUNCTION_ACTIVE == txUsed ) ? ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE ) : SPI_CR1_BIDIMODE;
                break;

            default:
                /* Full-duplex - 2 lines */
                dirValue = 0u;
                break;
        }

        retState = Spi_Set_RegField( &spi_PeriphConf[ periphId ].PeriphReg->CR1, SPI_CR1_DIR_MASK, dirValue );
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
 * \param pinPull  [in]: Pull of the pin
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Set_Pin( spi_PinCode_t pinCode, spi_PinSpeed_t pinSpeed, gpio_PinPullCfg_t pinPull )
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
        gpioConfig.PinPull        = pinPull;
        gpioConfig.PinSpeed       = spi_PinSpeedLut[ pinSpeed ];
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = (gpio_AltFunction_t)SPI_BIT_MASK_DECODE_AF( pinCode );
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        /* Port clock activation, pin configuration and read-back verification is done by Gpio_Init() */
        const gpio_RequestState_t gpioState = Gpio_Init( &gpioConfig );

        retState = ( GPIO_REQUEST_OK == gpioState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns pull of the SCK pin keeping the idle level of the clock mode
 *
 * \param clockMode [in]: Clock mode, value from \ref spi_ClockMode_t
 *
 * \return Pull-up for CPOL = 1, otherwise pull-down
 */
static gpio_PinPullCfg_t Spi_Get_SckPull( spi_ClockMode_t clockMode )
{
    gpio_PinPullCfg_t pinPull = GPIO_PIN_PULL_DOWN;

    if( ( SPI_CLOCK_MODE_2 == clockMode ) ||
        ( SPI_CLOCK_MODE_3 == clockMode )    )
    {
        pinPull = GPIO_PIN_PULL_UP;
    }
    else
    {
        /* SCK idle low */
    }

    return ( pinPull );
}


/**
 * \brief Returns frequency of the SPI kernel clock (APB clock of the peripheral)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param clkFreq [out]: Pointer to store the frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (non-zero frequency). Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Get_KernelClk( spi_PeriphId_t periphId, spi_FreqHz_t * const clkFreq )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) &&
        ( SPI_NULL_PTR  != clkFreq  )    )
    {
        rcc_FreqHz_t             rccFreq  = 0u;
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( spi_PeriphConf[ periphId ].PeriphRcc, &rccFreq );

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
 * \brief Returns count of core clock cycles of one SCK period (core clock / SCK frequency)
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param sckCycles [out]: Pointer to store the count of cycles. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Get_SckCycles( spi_PeriphId_t periphId, spi_TimeoutCnt_t * const sckCycles )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;
    spi_FreqHz_t       sckFreq  = 0u;
    rcc_FreqHz_t       coreFreq = 0u;

    if( SPI_NULL_PTR != sckCycles )
    {
        retState = Spi_Get_BusFreq( periphId, &sckFreq );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        const rcc_RequestState_t rccState = Rcc_Get_ClkBusClk( RCC_CLK_BUS_AHB1, &coreFreq );

        if( ( RCC_REQUEST_OK == rccState ) &&
            ( 0u              < sckFreq  )    )
        {
            *sckCycles = (spi_TimeoutCnt_t)( coreFreq / sckFreq ) + 1u;
            retState   = SPI_REQUEST_OK;
        }
        else
        {
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        /* SCK frequency is not available */
    }

    return ( retState );
}


/**
 * \brief Waits at least the count of core clock cycles. SysTick counter is used as time base if
 *        it runs (read only - its configuration is not changed), otherwise a busy-wait loop
 *        assuming \ref SPI_SCK_LOOP_CYCLES core cycles per iteration is used.
 *
 * \param coreCycles [in]: Count of core clock cycles
 */
static void Spi_Set_SckWait( spi_TimeoutCnt_t coreCycles )
{
    const uint32_t tickCtrl = SysTick->CTRL;
    const uint32_t tickLoad = SysTick->LOAD & SysTick_LOAD_RELOAD_Msk;

    if( ( 0u != ( tickCtrl & SysTick_CTRL_ENABLE_Msk ) ) &&
        ( 0u  < tickLoad                                 )    )
    {
        /* SysTick is clocked by the core clock or by the core clock / 8 */
        const uint32_t waitTicks = ( 0u != ( tickCtrl & SysTick_CTRL_CLKSOURCE_Msk ) ) ? coreCycles : ( ( coreCycles / SPI_SYSTICK_EXT_DIV ) + 1u );
        const uint32_t startVal  = SysTick->VAL;

        for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t actualVal = SysTick->VAL;
            const uint32_t elapsed   = ( startVal >= actualVal ) ? ( startVal - actualVal ) : ( ( startVal + tickLoad + 1u ) - actualVal );

            if( waitTicks <= elapsed )
            {
                break;
            }
            else
            {
                /* Waiting - the counter counts down */
            }
        }
    }
    else
    {
        const spi_TimeoutCnt_t loopCnt = ( coreCycles / SPI_SCK_LOOP_CYCLES ) + 1u;

        for( volatile spi_TimeoutCnt_t loopIdx = 0u; loopCnt > loopIdx; loopIdx ++ )
        {
            /* SysTick does not run - core cycles are estimated */
        }
    }
}


/**
 * \brief Clears status flags of the peripheral: received frame (RXNE) and overrun (read of DR
 *        followed by read of SR), CRC error (written to zero), mode fault (read of SR followed by
 *        write of CR1 - SPE is cleared) and TI frame error (read of SR)
 *
 * \param periphId [in]: SPI peripheral identification, checked by the caller
 */
static void Spi_Set_FlagsClear( spi_PeriphId_t periphId )
{
    SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;

    LL_SPI_ClearFlag_OVR( periphReg );
    LL_SPI_ClearFlag_CRCERR( periphReg );

    if( 0u != LL_SPI_IsActiveFlag_MODF( periphReg ) )
    {
        LL_SPI_ClearFlag_MODF( periphReg );
    }
    else
    {
        /* No mode fault - peripheral state is not changed */
    }
}


/**
 * \brief Waits until masked status flags of the peripheral reach the value (bounded busy-wait)
 *
 * \param periphId  [in]: SPI peripheral identification, checked by the caller
 * \param flagMask  [in]: Mask of SR flags
 * \param flagValue [in]: Required value of the masked flags
 *
 * \return \ref SPI_FUNCTION_ACTIVE if the flags reached the value, otherwise (timeout)
 *         \ref SPI_FUNCTION_INACTIVE
 */
static spi_FunctionState_t Spi_Get_FlagWait( spi_PeriphId_t periphId, spi_StatusFlags_t flagMask, spi_StatusFlags_t flagValue )
{
    SPI_TypeDef * const periphReg = spi_PeriphConf[ periphId ].PeriphReg;
    spi_FunctionState_t flagState = SPI_FUNCTION_INACTIVE;

    for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const spi_StatusFlags_t statusFlags = LL_SPI_ReadReg( periphReg, SR );

        if( flagValue == ( flagMask & statusFlags ) )
        {
            flagState = SPI_FUNCTION_ACTIVE;
            break;
        }
        else
        {
            /* Flags have not yet reached the value */
        }
    }

    return ( flagState );
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

        retState = ( NVIC_REQUEST_OK == nvicState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Ends the running transfer - moved frames are checked (DMA), transfer mode resources are
 *        stopped, the peripheral is disabled, flags are cleared and XferCompleteCallback /
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

        /* Frames moved by DMA are checked before the streams are stopped */
        const spi_RequestState_t doneState   = modeIf->CheckDone( periphId );
        const spi_RequestState_t stopState   = modeIf->Stop( periphId );
        const spi_RequestState_t enableState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );

        Spi_Set_FlagsClear( periphId );

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
 * \brief Aborts the running transfer without callback - transfer mode resources are stopped,
 *        the peripheral is disabled, CRC phase request and flags are cleared
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
        const spi_RequestState_t  stopState = spi_XferModeLut[ xferCtx->Config.XferMode ].Stop( periphId );

        xferCtx->XferState = SPI_FUNCTION_INACTIVE;

        const spi_RequestState_t enableState = Spi_Set_Enable( periphId, SPI_FUNCTION_INACTIVE );
        const spi_RequestState_t crcState    = Spi_Set_RegField( &periphReg->CR1, SPI_CR1_CRCNEXT, 0u );

        Spi_Set_FlagsClear( periphId );

        if( ( SPI_REQUEST_OK == stopState   ) &&
            ( SPI_REQUEST_OK == enableState ) &&
            ( SPI_REQUEST_OK == crcState    )    )
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
 * \brief Writes the next frame of the running transfer to DR (zero frame if the transmit buffer
 *        is not set). Access width corresponds to the frame size.
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
    else
    {
        LL_SPI_TransmitData16( periphReg, (uint16_t)frameData );
    }

    xferCtx->TxIdx = txIdx + 1u;
}


/**
 * \brief Reads the next frame of the running transfer from DR and stores it (discarded if the
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
    else
    {
        frameData = LL_SPI_ReceiveData16( periphReg );
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
 * \brief Requests the CRC phase after the frame in progress (CRCNEXT, cleared by hardware)
 *
 * \param periphId [in]: SPI peripheral identification, checked by the caller
 */
static void Spi_Set_CrcNext( spi_PeriphId_t periphId )
{
    LL_SPI_SetCRCNext( spi_PeriphConf[ periphId ].PeriphReg );

    spi_XferContext[ periphId ].CrcNext = SPI_FUNCTION_ACTIVE;
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
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI1 */

#ifdef SPI2
/**
 * \brief SPI2 global interrupt service routine
 */
static void Spi_Spi2_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_2 );
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI2 */

#ifdef SPI3
/**
 * \brief SPI3 global interrupt service routine
 */
static void Spi_Spi3_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_3 );
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI3 */

#ifdef SPI4
/**
 * \brief SPI4 global interrupt service routine
 */
static void Spi_Spi4_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_4 );
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI4 */

#ifdef SPI5
/**
 * \brief SPI5 global interrupt service routine
 */
static void Spi_Spi5_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_5 );
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI5 */

#ifdef SPI6
/**
 * \brief SPI6 global interrupt service routine
 */
static void Spi_Spi6_IsrHandler( void )
{
    (void)Spi_Isr_Handler( SPI_PERIPH_6 );
    __DSB();    /* Cortex-M4 erratum 838869: stores completed before the exception return */
}
#endif /* SPI6 */

/* ================================ TASKS =================================== */
