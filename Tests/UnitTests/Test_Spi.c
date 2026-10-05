/**
 * \author Mr.Nobody
 * \file Test_Spi.c
 * \ingroup Spi
 * \brief Unit tests of Serial Peripheral Interface (SPI) module.
 *
 * Spi.c, Spi_Dma.c, Spi_Isr.c and Spi_Poll.c are compiled unchanged with real
 * LL drivers. SPI registers are emulated by RegMem, RCC, NVIC, GPIO and DMA
 * modules are mocked by CMock. SPI ISR registered in NVIC and DMA callbacks
 * passed to Dma_Init() are captured by stubs and called directly.
 *
 * \note Emulated registers are plain memory. Status flags (SR) are not changed
 *       by data register accesses - tests preset the flags (TXE, RXNE, BSY,
 *       errors) and the received data (DR) before every transfer step.
 * \note Runtime state of the module is static - setUp() releases every
 *       peripheral by Spi_Deinit() with ignored mocks.
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Spi_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_spi.h"                      /* SPI registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Spi_Reset_Module         ( void );
static void                 Ut_Spi_Expect_PeriphClk     ( rcc_PeriphId_t periphId, rcc_FreqHz_t clockHz );
static void                 Ut_Spi_Expect_CoreClk       ( rcc_FreqHz_t clockHz );
static void                 Ut_Spi_Expect_GpioInit      ( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinPullCfg_t pull );
static void                 Ut_Spi_Expect_Activation    ( void );
static void                 Ut_Spi_Expect_IrqInit       ( void );
static void                 Ut_Spi_Expect_DmaInit       ( void );
static void                 Ut_Spi_Expect_DmaStop       ( void );
static void                 Ut_Spi_Expect_DmaTransfer   ( dma_ChannelId_t streamId, dma_TransferSize_t xferSize, dma_MemoryAddrInc_t addrInc, dma_MemoryAddr_t memAddr, dma_DataCount_t dataCount );
static void                 Ut_Spi_Expect_DmaRemaining  ( dma_ChannelId_t streamId, dma_DataCount_t remaining );
static spi_Config_t         Ut_Spi_Get_Config           ( void );
static void                 Ut_Spi_Init_Master          ( const spi_DataConfig_t * const dataConfig );
static void                 Ut_Spi_Init_Slave           ( const spi_DataConfig_t * const dataConfig );
static spi_DataConfig_t     Ut_Spi_Get_DataConfig       ( spi_XferMode_t xferMode );
static nvic_RequestState_t  Ut_Spi_Nvic_HandlerCallback ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int cmockNumCalls );
static dma_RequestState_t   Ut_Spi_Dma_InitCallback     ( dma_ConfigStruct_t * const dmaConfig, int cmockNumCalls );
static rcc_RequestState_t   Ut_Spi_Rcc_ResetCallback    ( rcc_PeriphId_t periphId, int cmockNumCalls );
static void                 Ut_Spi_HwModel_SlaveBusy    ( void );

static void                 Ut_Spi_XferCompleteCallback ( void );
static void                 Ut_Spi_ErrorCallback        ( spi_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** SPI peripheral used by tests (APB2) */
#define UT_SPI_BUS                          ( SPI_PERIPH_1 )
#define UT_SPI_REG                          ( SPI1 )
#define UT_SPI_RCC                          ( RCC_PERIPH_SPI1 )
#define UT_SPI_NVIC                         ( NVIC_PERIPH_IRQ_SPI1 )

/** APB2 clock returned by RCC mock [Hz] */
#define UT_SPI_CLK_HZ                       ( 84000000u )

/** Core clock returned by RCC mock [Hz] */
#define UT_SPI_CORE_CLK_HZ                  ( 168000000u )

/** Bus frequency of test configurations [Hz] (84 MHz / 128 = 656.25 kHz, BR = 6) */
#define UT_SPI_BUS_FREQ_HZ                  ( 1000000u )

/** BR field value of UT_SPI_BUS_FREQ_HZ */
#define UT_SPI_BR_1MHZ                      ( 6u )

/** Interrupt priority of test configurations */
#define UT_SPI_PRIO                         ( 6u )

/** DMA streams of SPI1 used by tests (DMA2 stream 3 / 0, channel 3) */
#define UT_SPI_DMA                          ( DMA_PERIPH_2 )
#define UT_SPI_DMA_TX_STREAM                ( DMA_STREAM_3 )
#define UT_SPI_DMA_RX_STREAM                ( DMA_STREAM_0 )

/** Offsets of DMA buffers in emulated SRAM */
#define UT_SPI_TX_BUF_OFFSET                ( 0x100u )
#define UT_SPI_RX_BUF_OFFSET                ( 0x200u )

/** Maximal count of captured DMA configurations */
#define UT_SPI_DMA_CFG_CNT                  ( 4u )

/** Reset value of SPI CRC polynomial register */
#define UT_SPI_CRCPR_RESET                  ( 0x0007u )

/** Maximal kernel clock of master with low / medium speed SCK pin (device errata) */
#define UT_SPI_SCK_LOW_MAX_HZ               ( 25000000u )
#define UT_SPI_SCK_MEDIUM_MAX_HZ            ( 75000000u )

/* ============================== MACROS ==================================== */

/** SCK, MISO, MOSI, NSS pins of SPI1 (PA5, PA6, PA7, PA4, AF5) */
#define UT_SPI_SCK_PIN      SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_ALT_FUNC_5 )
#define UT_SPI_MISO_PIN     SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_6, GPIO_ALT_FUNC_5 )
#define UT_SPI_MOSI_PIN     SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_7, GPIO_ALT_FUNC_5 )
#define UT_SPI_NSS_PIN      SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_4, GPIO_ALT_FUNC_5 )

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the SPI */
static nvic_IsrCallback_t   utSpi_Isr;

/** DMA configurations passed to Dma_Init() */
static dma_ConfigStruct_t   utSpi_DmaConfig[ UT_SPI_DMA_CFG_CNT ];

/** Count of Dma_Init() calls */
static uint32_t             utSpi_DmaInitCnt;

/** Counts of callback calls */
static uint32_t             utSpi_CompleteCnt;
static uint32_t             utSpi_ErrorCnt;

/** Error of the last error callback */
static spi_XferErrorId_t    utSpi_LastError;

/** Count of peripheral resets (Rcc_Set_ResetActive() callback) */
static uint32_t             utSpi_ResetCnt;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Spi_Reset_Module();

    utSpi_Isr         = NULL;
    utSpi_DmaInitCnt  = 0u;
    utSpi_CompleteCnt = 0u;
    utSpi_ErrorCnt    = 0u;
    utSpi_LastError   = SPI_XFER_ERROR_NONE;
    utSpi_ResetCnt    = 0u;

    (void)memset( utSpi_DmaConfig, 0, sizeof( utSpi_DmaConfig ) );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Spi_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Spi_Get_ModuleVersion_ReturnsVersion( void )
{
    spi_ModuleVersion_t version = Spi_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ========================= DEFAULT CONFIGURATION ========================== */

/**
 * \brief   Spi_Get_DefaultConfig() rejects null pointer.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR.
 */
void Ut_Spi_Get_DefaultConfig_NullPtr_Error( void )
{
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DefaultConfig( NULL ) );
}


/**
 * \brief   Spi_Get_DefaultConfig() fills every field of the configuration.
 *
 * \details Configuration structure is pre-filled with non default values.
 *
 * \par Expected results
 * - SPI1, APB clock, master 1 MHz, mode 0, 8-bit MSB first, full-duplex, Motorola,
 *   software NSS active low without pulse, no data handling, no pins, high speed.
 */
void Ut_Spi_Get_DefaultConfig_FillsDefaults( void )
{
    spi_Config_t config;

    (void)memset( &config, 0xA5, sizeof( config ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( SPI_PERIPH_1,              config.PeriphId );
    TEST_ASSERT_EQUAL( SPI_CLK_SRC_PCLK,          config.ClkSrc );
    TEST_ASSERT_EQUAL( SPI_MODE_MASTER,           config.Mode );
    TEST_ASSERT_EQUAL_UINT32( 1000000u,           config.BusFreq );
    TEST_ASSERT_EQUAL( SPI_CLOCK_MODE_0,          config.ClockMode );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_8BIT,        config.DataSize );
    TEST_ASSERT_EQUAL( SPI_BIT_ORDER_MSB_FIRST,   config.BitOrder );
    TEST_ASSERT_EQUAL( SPI_DIRECTION_FULL_DUPLEX, config.Direction );
    TEST_ASSERT_EQUAL( SPI_FRAME_FORMAT_MOTOROLA, config.FrameFormat );
    TEST_ASSERT_EQUAL( SPI_NSS_MODE_SOFT,         config.NssConfig.Mode );
    TEST_ASSERT_EQUAL( SPI_NSS_POLARITY_LOW,      config.NssConfig.Polarity );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE,     config.NssConfig.Pulse );
    TEST_ASSERT_NULL( config.DataConfig );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED,      config.SckPin );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED,      config.MisoPin );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED,      config.MosiPin );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED,      config.NssPin );
    TEST_ASSERT_EQUAL( SPI_PIN_SPEED_HIGH,        config.PinSpeed );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Spi_Init() rejects invalid configurations and features not available on STM32F4.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR for null pointer, invalid peripheral / clock source, data size
 *   other than 8 / 16 bits, active high NSS, NSS pulse, pin of other peripheral and
 *   master without bus frequency. No RCC access.
 */
void Ut_Spi_Init_InvalidConfig_Error( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( NULL ) );

    config.PeriphId = SPI_PERIPH_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config        = Ut_Spi_Get_Config();
    config.ClkSrc = SPI_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config          = Ut_Spi_Get_Config();
    config.DataSize = SPI_DATA_SIZE_12BIT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config          = Ut_Spi_Get_Config();
    config.DataSize = SPI_DATA_SIZE_32BIT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config                    = Ut_Spi_Get_Config();
    config.NssConfig.Polarity = SPI_NSS_POLARITY_HIGH;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config                 = Ut_Spi_Get_Config();
    config.NssConfig.Mode  = SPI_NSS_MODE_HARD;
    config.NssConfig.Pulse = SPI_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config        = Ut_Spi_Get_Config();
    config.SckPin = SPI_PIN_ENCODE( SPI_PERIPH_2, GPIO_PORT_B, GPIO_PIN_ID_13, GPIO_ALT_FUNC_5 );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config         = Ut_Spi_Get_Config();
    config.BusFreq = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
}


/**
 * \brief   Spi_Init() configures master with software NSS and its pins.
 *
 * \details SPI1 master, 1 MHz at 84 MHz APB2, mode 0, 8-bit, SCK / MISO / MOSI pins.
 *
 * \note    Device errata bug AB#667: the kernel clock is read twice - bus frequency calculation
 *          and the check of the SCK pin speed (high speed is not limited).
 *
 * \par Expected results
 * - Clock activated, peripheral reset, SCK pin with pull-down, MISO / MOSI without pull.
 * - CR1: MSTR, SSM, SSI, BR = 6, no CPOL / CPHA / DFF / LSBFIRST / BIDIMODE; SPE cleared.
 * - CR2: no SSOE, Motorola format.
 */
void Ut_Spi_Init_Master_ConfiguredAndPinsSet( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.SckPin  = UT_SPI_SCK_PIN;
    config.MisoPin = UT_SPI_MISO_PIN;
    config.MosiPin = UT_SPI_MOSI_PIN;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_PIN_PULL_DOWN );
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_6, GPIO_PIN_PULL_NONE );
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_7, GPIO_PIN_PULL_NONE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | ( UT_SPI_BR_1MHZ << SPI_CR1_BR_Pos ), UT_SPI_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 );
}


/**
 * \brief   Spi_Init() configures slave with hardware NSS input.
 *
 * \details Slave, mode 3, 16-bit, LSB first, TI format, SCK and NSS pins.
 *
 * \par Expected results
 * - No bus frequency calculation (slave), SCK pin with pull-up (CPOL = 1), NSS pin with pull-up.
 * - CR1: CPOL, CPHA, DFF, LSBFIRST, no MSTR / SSM / SSI. CR2: FRF, no SSOE.
 */
void Ut_Spi_Init_SlaveHardNss_RoleAndNssConfigured( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.Mode           = SPI_MODE_SLAVE;
    config.ClockMode      = SPI_CLOCK_MODE_3;
    config.DataSize       = SPI_DATA_SIZE_16BIT;
    config.BitOrder       = SPI_BIT_ORDER_LSB_FIRST;
    config.FrameFormat    = SPI_FRAME_FORMAT_TI;
    config.NssConfig.Mode = SPI_NSS_MODE_HARD;
    config.SckPin         = UT_SPI_SCK_PIN;
    config.NssPin         = UT_SPI_NSS_PIN;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_PIN_PULL_UP );
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_4, GPIO_PIN_PULL_UP );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF | SPI_CR1_LSBFIRST, UT_SPI_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_FRF, UT_SPI_REG->CR2 );
}


/**
 * \brief   Spi_Init() configures master with hardware NSS output.
 *
 * \par Expected results
 * - CR1: MSTR without SSM / SSI, CR2: SSOE. NSS configuration read back as hardware,
 *   active low, without pulse.
 */
void Ut_Spi_Init_MasterHardNss_NssOutputEnabled( void )
{
    spi_Config_t    config    = Ut_Spi_Get_Config();
    spi_NssConfig_t nssConfig = { .Mode = SPI_NSS_MODE_SOFT, .Polarity = SPI_NSS_POLARITY_HIGH, .Pulse = SPI_FUNCTION_ACTIVE };

    config.NssConfig.Mode = SPI_NSS_MODE_HARD;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_MSTR, UT_SPI_REG->CR1 & ( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_SSOE, UT_SPI_REG->CR2 );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_NssConfig( UT_SPI_BUS, &nssConfig ) );
    TEST_ASSERT_EQUAL( SPI_NSS_MODE_HARD,     nssConfig.Mode );
    TEST_ASSERT_EQUAL( SPI_NSS_POLARITY_LOW,  nssConfig.Polarity );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, nssConfig.Pulse );
}


/**
 * \brief   Spi_Init() reports failure of RCC and GPIO.
 *
 * \note    Device errata bug AB#667: master with SCK pin reads the kernel clock twice (bus
 *          frequency, SCK pin speed check) before the pins are configured.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR if the clock can not be activated, the bus clock is not known or a pin
 *   can not be configured.
 */
void Ut_Spi_Init_RccOrGpioError_Error( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    Ut_Spi_Expect_Activation();
    Rcc_Get_PeriphClk_ExpectAndReturn( UT_SPI_RCC, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config.SckPin = UT_SPI_SCK_PIN;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
}


/**
 * \brief   Spi_Init() and Spi_Set_Mode() refuse master with SCK pin too slow for the kernel
 *          clock.
 *
 * \note    Device errata bug AB#667 (ES0182 2.12.4 "Corrupted last bit of data and/or CRC
 *          received in Master mode with delayed SCK feedback"): the last received bit is not
 *          captured when the SCK pad is too slow for the APB clock (ES table at 30 pF: low speed
 *          up to 25 MHz, medium speed up to 75 MHz).
 *
 * \par Expected results
 * - Master, low / medium speed SCK pin, 84 MHz kernel clock: SPI_REQUEST_ERROR, no pin
 *   configured. Medium speed is accepted at 75 MHz.
 * - Slave with low speed SCK pin is accepted, Spi_Set_Mode() to master is refused at 84 MHz
 *   (role kept) and when the kernel clock is not available, accepted at 25 MHz.
 */
void Ut_Spi_Init_SckSpeedTooSlowForKernelClk_Error( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();
    spi_Mode_t   mode   = SPI_MODE_CNT;

    config.SckPin   = UT_SPI_SCK_PIN;
    config.PinSpeed = SPI_PIN_SPEED_LOW;

    /* Bus frequency calculation and SCK speed check read the kernel clock */
    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    config.PinSpeed = SPI_PIN_SPEED_MEDIUM;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_SCK_MEDIUM_MAX_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_SCK_MEDIUM_MAX_HZ );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    /* Slave does not generate SCK */
    config.Mode     = SPI_MODE_SLAVE;
    config.PinSpeed = SPI_PIN_SPEED_LOW;

    Ut_Spi_Expect_Activation();
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Mode( UT_SPI_BUS, &mode ) );
    TEST_ASSERT_EQUAL( SPI_MODE_SLAVE, mode );

    Rcc_Get_PeriphClk_ExpectAndReturn( UT_SPI_RCC, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_SCK_LOW_MAX_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Mode( UT_SPI_BUS, &mode ) );
    TEST_ASSERT_EQUAL( SPI_MODE_MASTER, mode );
}


/**
 * \brief   Spi_Deinit() resets the peripheral and disables its clock.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, reset pulse, clock deactivated, SPE cleared. Invalid peripheral and RCC
 *   failure return SPI_REQUEST_ERROR.
 */
void Ut_Spi_Deinit_ResetAndClockDisabled( void )
{
    UT_SPI_REG->CR1 = SPI_CR1_SPE;

    Rcc_Set_ResetActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( SPI_PERIPH_CNT ) );

    Rcc_Set_ResetActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
}

/* ======================== PERIPHERAL CONFIGURATION ======================== */

/**
 * \brief   Spi_Set_BusFreq() selects the smallest prescaler not exceeding the frequency.
 *
 * \details APB2 84 MHz: 42 MHz -> /2, 10 MHz -> /16 (5.25 MHz), 400 kHz -> /256 (328 kHz).
 *
 * \par Expected results
 * - BR field 0 / 3 / 7, Spi_Get_BusFreq() returns the real frequency.
 * - Frequency below APB2 / 256 and zero frequency are refused.
 */
void Ut_Spi_Set_BusFreq_PrescalerSelected( void )
{
    spi_FreqHz_t busFreq = 0u;

    Ut_Spi_Init_Master( NULL );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 42000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_BR );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 10000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 3u << SPI_CR1_BR_Pos, UT_SPI_REG->CR1 & SPI_CR1_BR );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( UT_SPI_BUS, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 5250000u, busFreq );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( 7u << SPI_CR1_BR_Pos, UT_SPI_REG->CR1 & SPI_CR1_BR );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BusFreq( UT_SPI_BUS, 100000u ) );
    TEST_ASSERT_EQUAL_HEX32( 7u << SPI_CR1_BR_Pos, UT_SPI_REG->CR1 & SPI_CR1_BR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BusFreq( UT_SPI_BUS, 0u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_BusFreq( UT_SPI_BUS, NULL ) );
}


/**
 * \brief   Spi_Set_ClockMode() configures CPOL / CPHA and the pull of the SCK pin.
 *
 * \note    Device errata bug AB#667: master initialization with SCK pin reads the kernel clock
 *          twice (bus frequency, SCK pin speed check).
 *
 * \par Expected results
 * - Every clock mode is read back, SCK pin configured by Spi_Init() gets pull-up for CPOL = 1
 *   and pull-down for CPOL = 0. Invalid clock mode is refused.
 */
void Ut_Spi_Set_ClockMode_RoundTripAndSckPull( void )
{
    spi_Config_t    config    = Ut_Spi_Get_Config();
    spi_ClockMode_t clockMode = SPI_CLOCK_MODE_CNT;

    config.SckPin = UT_SPI_SCK_PIN;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_PIN_PULL_DOWN );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    for( spi_ClockMode_t modeIdx = SPI_CLOCK_MODE_0; SPI_CLOCK_MODE_CNT > modeIdx; modeIdx++ )
    {
        const gpio_PinPullCfg_t pull = ( SPI_CLOCK_MODE_2 <= modeIdx ) ? GPIO_PIN_PULL_UP : GPIO_PIN_PULL_DOWN;

        Gpio_Set_PinPull_ExpectAndReturn( GPIO_PORT_A, GPIO_PIN_ID_5, pull, GPIO_REQUEST_OK );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_ClockMode( UT_SPI_BUS, modeIdx ) );
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_ClockMode( UT_SPI_BUS, &clockMode ) );
        TEST_ASSERT_EQUAL( modeIdx, clockMode );
    }

    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CPOL | SPI_CR1_CPHA, UT_SPI_REG->CR1 & ( SPI_CR1_CPOL | SPI_CR1_CPHA ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_ClockMode( UT_SPI_BUS, SPI_CLOCK_MODE_CNT ) );
}


/**
 * \brief   Spi_Set_DataSize() accepts 8 and 16 bits only.
 *
 * \par Expected results
 * - 16 bits sets DFF, 8 bits clears it, other sizes are refused and DFF is kept.
 */
void Ut_Spi_Set_DataSize_EightOrSixteenBits( void )
{
    spi_DataSize_t dataSize = SPI_DATA_SIZE_CNT;

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_DFF, UT_SPI_REG->CR1 & SPI_CR1_DFF );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataSize( UT_SPI_BUS, &dataSize ) );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_16BIT, dataSize );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_4BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_12BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_DFF, UT_SPI_REG->CR1 & SPI_CR1_DFF );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_8BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_DFF );
}


/**
 * \brief   Spi_Set_DataSize() keeps the polynomial of enabled CRC valid.
 *
 * \details 16-bit CRC with polynomial 0x1021 is enabled.
 *
 * \par Expected results
 * - Change to 8 bits is refused (polynomial does not fit), DFF is kept.
 */
void Ut_Spi_Set_DataSize_CrcPolynomialTooWide_Error( void )
{
    const spi_CrcConfig_t crcConfig = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_16BIT, .Polynomial = 0x1021u, .InitValue = SPI_CRC_INIT_ALL_ZERO };

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_8BIT ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_DFF, UT_SPI_REG->CR1 & SPI_CR1_DFF );
}


/**
 * \brief   Spi_Set_BitOrder() and Spi_Set_FrameFormat() are read back.
 *
 * \par Expected results
 * - LSBFIRST and FRF follow the configuration, invalid values are refused.
 */
void Ut_Spi_Set_BitOrderAndFrameFormat_RoundTrip( void )
{
    spi_BitOrder_t    bitOrder    = SPI_BIT_ORDER_CNT;
    spi_FrameFormat_t frameFormat = SPI_FRAME_FORMAT_CNT;

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_LSB_FIRST ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_LSBFIRST, UT_SPI_REG->CR1 & SPI_CR1_LSBFIRST );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BitOrder( UT_SPI_BUS, &bitOrder ) );
    TEST_ASSERT_EQUAL( SPI_BIT_ORDER_LSB_FIRST, bitOrder );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_CNT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_TI ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_FRF, UT_SPI_REG->CR2 & SPI_CR2_FRF );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_FrameFormat( UT_SPI_BUS, &frameFormat ) );
    TEST_ASSERT_EQUAL( SPI_FRAME_FORMAT_TI, frameFormat );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_CNT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_MOTOROLA ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & SPI_CR2_FRF );
}


/**
 * \brief   Spi_Set_Direction() stores the direction and configures BIDIMODE / BIDIOE / RXONLY
 *          according to the role.
 *
 * \par Expected results
 * - Master: simplex TX bidirectional output, simplex RX 2 lines, half-duplex BIDIMODE.
 * - Slave:  simplex TX 2 lines, simplex RX RXONLY. Spi_Set_Mode() updates the registers.
 * - Spi_Get_Direction() returns the stored direction.
 */
void Ut_Spi_Set_Direction_RegistersFollowRole( void )
{
    const uint32_t  dirMask   = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY;
    spi_Direction_t direction = SPI_DIRECTION_CNT;

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, UT_SPI_REG->CR1 & dirMask );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Direction( UT_SPI_BUS, &direction ) );
    TEST_ASSERT_EQUAL( SPI_DIRECTION_SIMPLEX_TX, direction );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE, UT_SPI_REG->CR1 & dirMask );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & dirMask );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_RXONLY, UT_SPI_REG->CR1 & dirMask );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SSM, UT_SPI_REG->CR1 & ( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & dirMask );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_CNT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Direction( UT_SPI_BUS, &direction ) );
    TEST_ASSERT_EQUAL( SPI_DIRECTION_SIMPLEX_TX, direction );
}


/**
 * \brief   Spi_Set_Mode() resets the peripheral before a slave receiver becomes master.
 *
 * \note    Device errata bug AB#666 (ES0182 2.12.2 "Anticipated communication upon SPI transit
 *          from slave receiver to master"): the clock starts upon setting MSTR even with SPE = 0
 *          when a slave receiver (RXONLY, or BIDIMODE without BIDIOE) becomes master - the SPI
 *          has to be reset by RCC before and its configuration restored.
 *
 * \par Expected results
 * - Slave simplex RX (RXONLY) -> master: RCC reset (registers cleared by the callback), CRCPR,
 *   CR2 and CR1 configuration restored, MSTR / SSM / SSI set, RXONLY cleared.
 * - Slave half-duplex receiver (BIDIMODE, BIDIOE = 0) -> master: RCC reset.
 * - Master -> slave and slave full-duplex -> master: no reset.
 * - Reset failure is reported, the role is not changed.
 */
void Ut_Spi_Set_Mode_SlaveReceiverToMaster_Reset( void )
{
    const uint32_t cr1Config = SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST | SPI_CR1_DFF | SPI_CR1_CRCEN;
    const uint32_t dirMask   = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY;

    Ut_Spi_Init_Slave( NULL );

    Rcc_Set_ResetActive_StubWithCallback( Ut_Spi_Rcc_ResetCallback );

    /* Configuration to be restored after the reset */
    UT_SPI_REG->CR1  |= cr1Config;
    UT_SPI_REG->CR2  |= SPI_CR2_ERRIE;
    UT_SPI_REG->CRCPR = 0x1021u;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_RXONLY, UT_SPI_REG->CR1 & dirMask );

    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI, UT_SPI_REG->CR1 & ( SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & dirMask );
    TEST_ASSERT_EQUAL_HEX32( cr1Config, UT_SPI_REG->CR1 & cr1Config );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_ERRIE, UT_SPI_REG->CR2 & SPI_CR2_ERRIE );
    TEST_ASSERT_EQUAL_HEX32( 0x1021u, UT_SPI_REG->CRCPR );

    /* Master -> slave and slave full-duplex -> master: no reset */
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_FULL_DUPLEX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ResetCnt );

    /* Slave half-duplex receiver -> master */
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE, UT_SPI_REG->CR1 & dirMask );

    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_MSTR, UT_SPI_REG->CR1 & SPI_CR1_MSTR );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE, UT_SPI_REG->CR1 & dirMask );

    /* Reset failure: role is not changed */
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );

    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_MSTR );
}


/**
 * \brief   NSS features not available on STM32F4 are refused, master idle timing accepts zero.
 *
 * \par Expected results
 * - Active high NSS and NSS pulse refused, master timing 0 / 0 accepted and read back,
 *   non-zero timing refused.
 */
void Ut_Spi_Set_NssAndTiming_UnsupportedRefused( void )
{
    spi_NssConfig_t  nssConfig     = { .Mode = SPI_NSS_MODE_HARD, .Polarity = SPI_NSS_POLARITY_HIGH, .Pulse = SPI_FUNCTION_INACTIVE };
    spi_IdleCycles_t interDataIdle = 5u;
    spi_IdleCycles_t ssIdle        = 5u;

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, &nssConfig ) );

    nssConfig.Polarity = SPI_NSS_POLARITY_LOW;
    nssConfig.Pulse    = SPI_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, &nssConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, NULL ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_MasterTiming( UT_SPI_BUS, 0u, 0u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 1u, 0u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 0u, 1u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_MasterTiming( UT_SPI_BUS, &interDataIdle, &ssIdle ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, interDataIdle );
    TEST_ASSERT_EQUAL_UINT8( 0u, ssIdle );
}


/**
 * \brief   Spi_Set_CrcConfig() configures polynomial and enables CRC.
 *
 * \note    Device errata bug AB#665 (ES0182 2.12.3 "Wrong CRC calculation when the polynomial is
 *          even"): the hardware CRC is wrong with an even polynomial, such polynomial (0x06) has
 *          to be refused and CRCPR kept.
 *
 * \par Expected results
 * - Valid 8-bit CRC: CRCPR = polynomial, CRCEN set, configuration read back.
 * - Size different from the data size, zero / even / too wide polynomial and initialization
 *   with ones are refused (even polynomial: device errata). Disabling clears CRCEN only.
 */
void Ut_Spi_Set_CrcConfig_ValidAndUnsupported( void )
{
    spi_CrcConfig_t crcConfig = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 0x07u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    spi_CrcConfig_t readBack;

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0x07u, UT_SPI_REG->CRCPR );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CRCEN, UT_SPI_REG->CR1 & SPI_CR1_CRCEN );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_ACTIVE,   readBack.State );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_8BIT,    readBack.Size );
    TEST_ASSERT_EQUAL_HEX32( 0x07u,           readBack.Polynomial );
    TEST_ASSERT_EQUAL( SPI_CRC_INIT_ALL_ZERO, readBack.InitValue );

    crcConfig.Size = SPI_DATA_SIZE_16BIT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    crcConfig.Size       = SPI_DATA_SIZE_8BIT;
    crcConfig.Polynomial = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    crcConfig.Polynomial = 0x107u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    /* Device errata: CRC calculation is wrong with an even polynomial */
    crcConfig.Polynomial = 0x06u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0x07u, UT_SPI_REG->CRCPR );

    crcConfig.Polynomial = 0x07u;
    crcConfig.InitValue  = SPI_CRC_INIT_ALL_ONES;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, NULL ) );

    crcConfig.State      = SPI_FUNCTION_INACTIVE;
    crcConfig.Polynomial = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_CRCEN );
    TEST_ASSERT_EQUAL_HEX32( 0x07u, UT_SPI_REG->CRCPR );
}


/**
 * \brief   Spi_Get_CrcValue() reads transmitter and receiver CRC registers.
 *
 * \par Expected results
 * - TXCRCR / RXCRCR values are returned, null pointers are refused.
 */
void Ut_Spi_Get_CrcValue_ReadsRegisters( void )
{
    spi_CrcValue_t txCrc = 0u;
    spi_CrcValue_t rxCrc = 0u;

    UT_SPI_REG->TXCRCR = 0x1234u;
    UT_SPI_REG->RXCRCR = 0x5678u;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( UT_SPI_BUS, &txCrc, &rxCrc ) );
    TEST_ASSERT_EQUAL_HEX32( 0x1234u, txCrc );
    TEST_ASSERT_EQUAL_HEX32( 0x5678u, rxCrc );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcValue( UT_SPI_BUS, NULL, &rxCrc ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcValue( SPI_PERIPH_CNT, &txCrc, &rxCrc ) );
}


/**
 * \brief   Configuration is refused while the peripheral is enabled.
 *
 * \par Expected results
 * - Setters return SPI_REQUEST_ERROR while SPE is set, Spi_Get_PeriphState() reports it.
 */
void Ut_Spi_Setters_PeriphEnabled_Error( void )
{
    spi_FlagState_t periphState = SPI_FLAG_INACTIVE;

    Ut_Spi_Init_Master( NULL );

    UT_SPI_REG->CR1 |= SPI_CR1_SPE;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_PeriphState( UT_SPI_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_FLAG_ACTIVE, periphState );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 0u, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & ( SPI_CR1_DFF | SPI_CR1_BIDIMODE ) );
}


/**
 * \brief   Getters reject null pointers and invalid peripherals.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR.
 */
void Ut_Spi_Getters_InvalidParams_Error( void )
{
    spi_Mode_t mode = SPI_MODE_MASTER;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_PeriphState( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_Mode( SPI_PERIPH_CNT, &mode ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_ClockMode( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataSize( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_BitOrder( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_Direction( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_FrameFormat( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_NssConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_MasterTiming( UT_SPI_BUS, NULL, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferState( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferError( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_IrqPriority( UT_SPI_BUS, NULL ) );
}

/* ============================ DATA HANDLING =============================== */

/**
 * \brief   ISR data handling registers the interrupt handler and enables the interrupt.
 *
 * \par Expected results
 * - Handler registered, priority set, interrupt enabled, configuration read back.
 * - Spi_Get_DataConfig() fails before the initialization.
 */
void Ut_Spi_Set_DataConfig_Isr_InterruptEnabled( void )
{
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    spi_DataConfig_t       readBack;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    Ut_Spi_Init_Master( NULL );

    Ut_Spi_Expect_IrqInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_ISR, readBack.XferMode );
    TEST_ASSERT_EQUAL_PTR( Ut_Spi_XferCompleteCallback, readBack.XferCompleteCallback );
}


/**
 * \brief   DMA data handling refuses streams not connected to the SPI requests.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR for transmit stream of other peripheral, receive stream equal to the
 *   transmit stream, other DMA peripheral and invalid priority. No DMA access.
 */
void Ut_Spi_Set_DataConfig_DmaInvalidStream_Error( void )
{
    spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );

    Ut_Spi_Init_Master( NULL );

    dataConfig.TxDmaChannelId = SPI_DMA_CHANNEL_4;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    dataConfig                = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.RxDmaChannelId = SPI_DMA_CHANNEL_3;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    dataConfig               = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.RxDmaPeriphId = SPI_DMA_PERIPH_1;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    dataConfig               = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.TxDmaPriority = (spi_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
}


/**
 * \brief   DMA data handling initializes both streams of SPI1.
 *
 * \details Transmit DMA2 stream 3, receive DMA2 stream 0 (channel 3 of both).
 *
 * \par Expected results
 * - Dma_Init() configures channel 3, peripheral address of DR, directions, priorities,
 *   transfer complete and error callbacks. Interrupts of both streams are enabled, streams
 *   are stopped, SPI interrupt is enabled.
 */
void Ut_Spi_Set_DataConfig_Dma_StreamsInitialized( void )
{
    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_DmaInit();

    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );

    TEST_ASSERT_EQUAL( UT_SPI_DMA,                    utSpi_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_TX_STREAM,          utSpi_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_3,             utSpi_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,      utSpi_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)&UT_SPI_REG->DR, utSpi_DmaConfig[ 0u ].PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,        utSpi_DmaConfig[ 0u ].PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,      utSpi_DmaConfig[ 0u ].TransferMode );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,              utSpi_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 0u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 0u ].TransferErrorCallback );
    TEST_ASSERT_NULL( utSpi_DmaConfig[ 0u ].HalfTransferCallback );

    TEST_ASSERT_EQUAL( UT_SPI_DMA,                    utSpi_DmaConfig[ 1u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_RX_STREAM,          utSpi_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_3,             utSpi_DmaConfig[ 1u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,      utSpi_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)&UT_SPI_REG->DR, utSpi_DmaConfig[ 1u ].PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,             utSpi_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 1u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 1u ].TransferErrorCallback );

    TEST_ASSERT_NOT_NULL( utSpi_Isr );
}

/* ============================== TRANSFERS ================================= */

/**
 * \brief   Spi_Set_XferStart() refuses invalid requests and missing data handling.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR without data handling, for null request, zero size, missing buffers
 *   and half-duplex request with both buffers. Peripheral stays disabled.
 */
void Ut_Spi_Set_XferStart_InvalidRequest_Error( void )
{
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    uint8_t                txBuf[ 2u ] = { 0u };
    uint8_t                rxBuf[ 2u ] = { 0u };
    spi_XferRequest_t      request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( NULL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, NULL ) );

    request.XferSize = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    request.XferSize = 2u;
    request.TxData   = NULL;
    request.RxData   = NULL;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    request.TxData = txBuf;
    request.RxData = rxBuf;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   POLL full-duplex 8-bit transfer is moved by Spi_Task() frame by frame.
 *
 * \details Master transmits the next frame after the previous one was received.
 *
 * \par Expected results
 * - SPE set while running, frames written in order, received frames stored.
 * - Transfer completes after TXE = 1 / BSY = 0: complete callback, no error, SPE cleared,
 *   configuration is allowed again.
 */
void Ut_Spi_Poll_FullDuplex_FramesMovedAndCompleted( void )
{
    const spi_DataConfig_t dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t          txBuf[ 3u ] = { 0x11u, 0x22u, 0x33u };
    uint8_t                rxBuf[ 3u ] = { 0u };
    const spi_XferRequest_t request    = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 3u };
    const uint8_t          rxData[ 3u ] = { 0xA1u, 0xB2u, 0xC3u };
    spi_FunctionState_t    xferState   = SPI_FUNCTION_INACTIVE;
    spi_XferErrorId_t      xferError   = SPI_XFER_ERROR_CNT;

    Ut_Spi_Init_Master( &dataConfig );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SPE, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_SPI_REG->DR );

    /* Only one frame is in flight - no frame is written without reception */
    UT_SPI_REG->DR = 0x00u;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x00u, UT_SPI_REG->DR );

    for( uint32_t idx = 0u; 3u > idx; idx++ )
    {
        UT_SPI_REG->DR = rxData[ idx ];
        UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
        Spi_Task();

        if( 2u > idx )
        {
            TEST_ASSERT_EQUAL_HEX32( txBuf[ idx + 1u ], UT_SPI_REG->DR );
        }
    }

    TEST_ASSERT_EQUAL_HEX8_ARRAY( rxData, rxBuf, 3u );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferError( UT_SPI_BUS, &xferError ) );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_NONE, xferError );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
}


/**
 * \brief   POLL 16-bit frames are stored little endian.
 *
 * \par Expected results
 * - Frame 0x1234 is written from buffer { 0x34, 0x12 }, received 0xBEEF is stored
 *   { 0xEF, 0xBE }.
 */
void Ut_Spi_Poll_SixteenBit_LittleEndianBuffers( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 2u ] = { 0x34u, 0x12u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x1234u, UT_SPI_REG->DR );

    UT_SPI_REG->DR = 0xBEEFu;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0xEFu, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xBEu, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL master simplex reception transmits zero frames (2 line configuration).
 *
 * \par Expected results
 * - No BIDIMODE / RXONLY, zero frame written although TxData is set, received frames stored.
 */
void Ut_Spi_Poll_MasterSimplexRx_ZeroFramesTransmitted( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 1u ] = { 0xFFu };
    uint8_t                 rxBuf[ 1u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & ( SPI_CR1_BIDIMODE | SPI_CR1_RXONLY ) );

    UT_SPI_REG->DR = 0x55u;
    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x00u, UT_SPI_REG->DR );

    UT_SPI_REG->DR = 0x5Au;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0x5Au, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL master simplex transmission (bidirectional output) waits for the end of the
 *          last frame.
 *
 * \details Frames are written without reception, BSY is set after the last frame.
 *
 * \par Expected results
 * - BIDIMODE and BIDIOE set, frames written back to back.
 * - With BSY set the transfer ends with SPI_XFER_ERROR_INCOMPLETE after the timeout, with
 *   BSY cleared it completes.
 */
void Ut_Spi_Poll_MasterSimplexTx_CompletesWhenNotBusy( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 2u ] = { 0x81u, 0x42u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 2u };

    Ut_Spi_Init_Master( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, UT_SPI_REG->CR1 & ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x81u, UT_SPI_REG->DR );

    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_BSY;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x42u, UT_SPI_REG->DR );

    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_INCOMPLETE, utSpi_LastError );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL slave simplex transmission counts the end of transfer by received frames.
 *
 * \details Two frames are written ahead (data and shift register).
 *
 * \par Expected results
 * - 2 line configuration, two frames written before reception, third after the first
 *   reception, transfer completes after the third received frame.
 */
void Ut_Spi_Poll_SlaveSimplexTx_TwoFramesAhead( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 3u ] = { 0x01u, 0x02u, 0x03u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 3u };

    Ut_Spi_Init_Slave( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & ( SPI_CR1_BIDIMODE | SPI_CR1_RXONLY ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x01u, UT_SPI_REG->DR );
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x02u, UT_SPI_REG->DR );
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x02u, UT_SPI_REG->DR );

    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x03u, UT_SPI_REG->DR );
    Spi_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );
    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL slave full-duplex transfer ends after the last received frame although BSY is
 *          set (BSY may stay set in slave mode - device errata).
 *
 * \par Expected results
 * - Transfer completes without error with BSY set, SPE cleared.
 */
void Ut_Spi_Poll_SlaveFullDuplex_CompletesWithBusySet( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 1u ] = { 0x6Bu };
    uint8_t                 rxBuf[ 1u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Slave( &dataConfig );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x6Bu, UT_SPI_REG->DR );

    UT_SPI_REG->DR = 0x96u;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_BSY;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0x96u, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   POLL slave half-duplex transmission disables the peripheral during the last frame.
 *
 * \details HW model keeps BSY set while SPE is set and clears it when SPE is cleared.
 *
 * \note    Device errata bug AB#668 (ES0182 2.12.5 "BSY flag may stay high at the end of a data
 *          transfer in Slave mode"): BSY of a slave may stay set after the last frame. Errata
 *          transmit workaround - SPE is cleared while the last frame is transmitted (TXE = 1),
 *          BSY is reliable afterwards. Without it the transfer ends with a false
 *          SPI_XFER_ERROR_INCOMPLETE.
 *
 * \par Expected results
 * - Transfer completes without error, SPE cleared, last frame written.
 */
void Ut_Spi_Poll_SlaveHalfDuplexTx_DisabledDuringLastFrame( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 1u ] = { 0xC3u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 1u };

    Ut_Spi_Init_Slave( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, UT_SPI_REG->CR1 & ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_Spi_HwModel_SlaveBusy ) );

    Spi_Task();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL_HEX32( 0xC3u, UT_SPI_REG->DR );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   POLL overrun and mode fault terminate the transfer.
 *
 * \par Expected results
 * - Error callback with SPI_XFER_ERROR_OVERRUN / SPI_XFER_ERROR_MODE_FAULT, transfer state
 *   inactive, SPE cleared, error read back.
 */
void Ut_Spi_Poll_OverrunAndModeFault_ErrorReported( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    uint8_t                 rxBuf[ 4u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 4u };
    spi_XferErrorId_t       xferError   = SPI_XFER_ERROR_NONE;

    Ut_Spi_Init_Master( &dataConfig );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_OVR;
    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_OVERRUN, utSpi_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferError( UT_SPI_BUS, &xferError ) );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_OVERRUN, xferError );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_MODF;
    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_MODE_FAULT, utSpi_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL full-duplex CRC: CRCNEXT after the last frame, CRC frame received and checked.
 *
 * \par Expected results
 * - CRC reset (CRCEN toggled) at start, CRCNEXT set after the last transmitted frame.
 * - Matching CRC completes the transfer, CRCERR terminates it with SPI_XFER_ERROR_CRC.
 */
void Ut_Spi_Poll_FullDuplexCrc_CrcNextAndCheck( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const spi_CrcConfig_t   crcConfig   = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 0x07u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    const uint8_t           txBuf[ 1u ] = { 0xC5u };
    uint8_t                 rxBuf[ 1u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    UT_SPI_REG->CR1 |= SPI_CR1_CRCNEXT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CRCEN, UT_SPI_REG->CR1 & ( SPI_CR1_CRCEN | SPI_CR1_CRCNEXT ) );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CRCNEXT, UT_SPI_REG->CR1 & SPI_CR1_CRCNEXT );

    UT_SPI_REG->DR = 0x3Cu;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0x3Cu, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    UT_SPI_REG->SR = SPI_SR_TXE;
    Spi_Task();
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_CRCERR;
    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_CRC, utSpi_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->SR & SPI_SR_CRCERR );
}


/**
 * \brief   POLL slave receive only CRC: CRCNEXT after the last but one received frame.
 *
 * \par Expected results
 * - RXONLY set, CRCNEXT set after the first of two frames, transfer completes.
 */
void Ut_Spi_Poll_SlaveRxOnlyCrc_CrcNextAfterLastButOne( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const spi_CrcConfig_t   crcConfig   = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 0x07u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Slave( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_RXONLY, UT_SPI_REG->CR1 & SPI_CR1_RXONLY );

    UT_SPI_REG->DR = 0x10u;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CRCNEXT, UT_SPI_REG->CR1 & SPI_CR1_CRCNEXT );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

    UT_SPI_REG->DR = 0x20u;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0x10u, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x20u, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   POLL master half-duplex reception stops the clock during the last frame.
 *
 * \details Two frames: the peripheral is disabled after the first frame was read. Overrun
 *          of frames clocked after the stop is ignored.
 *
 * \par Expected results
 * - BIDIMODE without BIDIOE, SPE cleared after the first frame, transfer completes after the
 *   second frame, both frames stored.
 * - CRC is refused in master half-duplex reception.
 */
void Ut_Spi_Poll_MasterHalfDuplexRx_StoppedInLastFrame( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const spi_CrcConfig_t   crcConfig   = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 0x07u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( &dataConfig );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_CoreClk( UT_SPI_CORE_CLK_HZ );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_BIDIMODE | SPI_CR1_SPE, UT_SPI_REG->CR1 & ( SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SPE ) );

    UT_SPI_REG->DR = 0x5Au;
    UT_SPI_REG->SR = SPI_SR_RXNE;
    Spi_Task();
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

    UT_SPI_REG->DR = 0xA5u;
    UT_SPI_REG->SR = SPI_SR_RXNE | SPI_SR_OVR | SPI_SR_BSY;
    Spi_Task();

    TEST_ASSERT_EQUAL_HEX8( 0x5Au, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xA5u, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   Spi_Set_XferStop() aborts the running transfer without callback.
 *
 * \par Expected results
 * - Transfer state inactive, SPE cleared, no callback, no error stored. Stop without running
 *   transfer succeeds.
 */
void Ut_Spi_Set_XferStop_AbortedWithoutCallback( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    uint8_t                 rxBuf[ 4u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 4u };
    spi_FunctionState_t     xferState   = SPI_FUNCTION_ACTIVE;
    spi_XferErrorId_t       xferError   = SPI_XFER_ERROR_CNT;

    Ut_Spi_Init_Master( &dataConfig );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferError( UT_SPI_BUS, &xferError ) );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_NONE, xferError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt + utSpi_ErrorCnt );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStop( SPI_PERIPH_CNT ) );
}


/**
 * \brief   ISR full-duplex transfer is moved by the SPI interrupt.
 *
 * \details TXE interrupt is enabled only while a frame may be written.
 *
 * \par Expected results
 * - TXEIE, RXNEIE, ERRIE enabled at start, TXEIE disabled while the frame is in flight.
 * - Frames moved, complete callback, interrupt sources disabled at the end.
 */
void Ut_Spi_Isr_FullDuplex_InterruptDrivesTransfer( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };
    const uint32_t          itMask      = SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE;

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_IrqInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( itMask, UT_SPI_REG->CR2 & itMask );

    UT_SPI_REG->SR = SPI_SR_TXE;
    utSpi_Isr();
    TEST_ASSERT_EQUAL_HEX32( 0x12u, UT_SPI_REG->DR );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_RXNEIE | SPI_CR2_ERRIE, UT_SPI_REG->CR2 & itMask );

    UT_SPI_REG->DR = 0xABu;
    UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_RXNE;
    utSpi_Isr();
    TEST_ASSERT_EQUAL_HEX32( 0x34u, UT_SPI_REG->DR );

    UT_SPI_REG->DR = 0xCDu;
    utSpi_Isr();

    TEST_ASSERT_EQUAL_HEX8( 0xABu, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xCDu, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & itMask );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   ISR TI frame error terminates the transfer, interrupt without transfer is ignored.
 *
 * \par Expected results
 * - SPI_XFER_ERROR_FRAME reported once, interrupt sources disabled; interrupt without running
 *   transfer calls no callback.
 */
void Ut_Spi_Isr_FrameError_ErrorReported( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Slave( NULL );
    Ut_Spi_Expect_IrqInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_TI ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->SR = SPI_SR_FRE;
    utSpi_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_FRAME, utSpi_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & ( SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE ) );

    utSpi_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
}


/**
 * \brief   DMA full-duplex transfer arms both streams and completes on receive transfer
 *          complete.
 *
 * \details 4 frames from / to emulated SRAM. Transmit transfer complete does not end the
 *          transfer.
 *
 * \par Expected results
 * - Streams configured (8-bit, increment, buffers, count 4) and started, RXDMAEN / TXDMAEN
 *   and ERRIE set, SPE set.
 * - Receive transfer complete with TXE = 1 / BSY = 0: remaining counts checked, streams
 *   stopped, DMA requests cleared, complete callback.
 */
void Ut_Spi_Dma_FullDuplex_CompletedByRxStream( void )
{
    const spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    uint8_t * const         txBuf      = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const         rxBuf      = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request    = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 4u };

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_RX_STREAM, DMA_TRANSFER_SIZE_8BIT, DMA_MEMORY_ADDR_INCREMENT, REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, 4u );
    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_TX_STREAM, DMA_TRANSFER_SIZE_8BIT, DMA_MEMORY_ADDR_INCREMENT, REGMEM_SRAM_BASE + UT_SPI_TX_BUF_OFFSET, 4u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE, UT_SPI_REG->CR2 );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SPE, UT_SPI_REG->CR1 & SPI_CR1_SPE );

    utSpi_DmaConfig[ 0u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Ut_Spi_Expect_DmaRemaining( UT_SPI_DMA_TX_STREAM, 0u );
    Ut_Spi_Expect_DmaRemaining( UT_SPI_DMA_RX_STREAM, 0u );
    Ut_Spi_Expect_DmaStop();

    utSpi_DmaConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   DMA simplex transmission of 16-bit frames ends with transmit transfer complete.
 *
 * \details Master simplex TX, TxData in SRAM; odd buffer address is refused.
 *
 * \par Expected results
 * - Only the transmit stream is armed (16-bit), transfer completes on transmit transfer
 *   complete, misaligned buffer returns SPI_REQUEST_ERROR and the transfer is not started.
 */
void Ut_Spi_Dma_SimplexTx16Bit_CompletedByTxStream( void )
{
    const spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    const spi_XferRequest_t request    = { .TxData = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET ), .RxData = NULL, .XferSize = 3u };
    const spi_XferRequest_t badRequest = { .TxData = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET + 1u ), .RxData = NULL, .XferSize = 3u };

    Ut_Spi_Init_Master( NULL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    Ut_Spi_Expect_DmaStop();
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &badRequest ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );

    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_TX_STREAM, DMA_TRANSFER_SIZE_16BIT, DMA_MEMORY_ADDR_INCREMENT, REGMEM_SRAM_BASE + UT_SPI_TX_BUF_OFFSET, 3u );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_TXDMAEN | SPI_CR2_ERRIE, UT_SPI_REG->CR2 );

    UT_SPI_REG->SR = SPI_SR_TXE;
    Ut_Spi_Expect_DmaRemaining( UT_SPI_DMA_TX_STREAM, 0u );
    Ut_Spi_Expect_DmaStop();

    utSpi_DmaConfig[ 0u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
}


/**
 * \brief   DMA transfer error terminates the transfer.
 *
 * \par Expected results
 * - SPI_XFER_ERROR_DMA_TRANSFER reported, streams stopped. Error without running transfer is
 *   not reported.
 */
void Ut_Spi_Dma_TransferError_ErrorReported( void )
{
    const spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    const spi_XferRequest_t request    = { .TxData = NULL, .RxData = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET ), .XferSize = 2u };

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_RX_STREAM, DMA_TRANSFER_SIZE_8BIT, DMA_MEMORY_ADDR_INCREMENT, REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, 2u );
    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_TX_STREAM, DMA_TRANSFER_SIZE_8BIT, DMA_MEMORY_ADDR_STATIC, 0u, 2u );
    Dma_Set_MemoryAddr_IgnoreArg_memoryAddr();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Expect_DmaStop();
    utSpi_DmaConfig[ 1u ].TransferErrorCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_DMA_TRANSFER, utSpi_LastError );

    utSpi_DmaConfig[ 0u ].TransferErrorCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
}


/**
 * \brief   DMA master half-duplex reception: the stream moves all frames but the last one,
 *          the last frame is received by the SPI interrupt.
 *
 * \par Expected results
 * - Receive stream armed for 2 of 3 frames, no transmit stream.
 * - Receive transfer complete: SPE and RXDMAEN cleared, RXNEIE enabled.
 * - SPI interrupt stores the last frame and completes the transfer.
 */
void Ut_Spi_Dma_MasterHalfDuplexRx_LastFrameByCpu( void )
{
    const spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    uint8_t * const         rxBuf      = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request    = { .TxData = NULL, .RxData = rxBuf, .XferSize = 3u };

    Ut_Spi_Init_Master( NULL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    Ut_Spi_Expect_CoreClk( UT_SPI_CORE_CLK_HZ );
    Ut_Spi_Expect_DmaTransfer( UT_SPI_DMA_RX_STREAM, DMA_TRANSFER_SIZE_8BIT, DMA_MEMORY_ADDR_INCREMENT, REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, 2u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_RXDMAEN | SPI_CR2_ERRIE, UT_SPI_REG->CR2 );

    utSpi_DmaConfig[ 1u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR2_RXNEIE | SPI_CR2_ERRIE, UT_SPI_REG->CR2 );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

    UT_SPI_REG->DR = 0x77u;
    UT_SPI_REG->SR = SPI_SR_RXNE | SPI_SR_OVR;
    Ut_Spi_Expect_DmaRemaining( UT_SPI_DMA_RX_STREAM, 0u );
    Ut_Spi_Expect_DmaStop();

    utSpi_Isr();

    TEST_ASSERT_EQUAL_HEX8( 0x77u, rxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );
}


/**
 * \brief   Spi_Set_IrqPriority() / Spi_Get_IrqPriority() use NVIC.
 *
 * \par Expected results
 * - Priority passed to / read from NVIC, NVIC failure and invalid peripheral reported.
 */
void Ut_Spi_IrqPriority_PassedToNvic( void )
{
    static nvic_IrqPrio_t nvicPrio = 9u;
    spi_IrqPrio_t         irqPrio  = 0u;

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, 3u, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_IrqPriority( UT_SPI_BUS, 3u ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, 3u, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_IrqPriority( UT_SPI_BUS, 3u ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &nvicPrio );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_IrqPriority( UT_SPI_BUS, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( 9u, irqPrio );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_IrqPriority( SPI_PERIPH_CNT, 3u ) );
}

/* ===================== DATA HANDLING RELEASE / INSTANCES ================== */

/** Count of Dma_Set_InterruptInactive() calls (DMA stream released) */
static uint32_t             utSpi_DmaIrqOffCnt;

/** Return value of Dma_Set_InterruptInactive() stub */
static dma_RequestState_t   utSpi_DmaIrqOffState;

/** Count of Nvic_Set_PeriphIrq_Inactive() calls (SPI interrupt disabled) */
static uint32_t             utSpi_NvicIrqOffCnt;

/** Return value of Nvic_Set_PeriphIrq_Inactive() stub */
static nvic_RequestState_t  utSpi_NvicIrqOffState;

/** Dma_Set_InterruptInactive() stub counting the calls */
static dma_RequestState_t Ut_Spi_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int cmockNumCalls )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)cmockNumCalls;

    utSpi_DmaIrqOffCnt++;

    return ( utSpi_DmaIrqOffState );
}


/** Nvic_Set_PeriphIrq_Inactive() stub counting the calls */
static nvic_RequestState_t Ut_Spi_NvicIrqOffStub( nvic_PeriphIrqList_t irqId, int cmockNumCalls )
{
    (void)irqId;
    (void)cmockNumCalls;

    utSpi_NvicIrqOffCnt++;

    return ( utSpi_NvicIrqOffState );
}


/** RCC peripheral clock stub of any SPI peripheral - returns \ref UT_SPI_CLK_HZ */
static rcc_RequestState_t Ut_Spi_RccAnyClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int cmockNumCalls )
{
    (void)periphId;
    (void)cmockNumCalls;

    *periphClk = UT_SPI_CLK_HZ;

    return ( RCC_REQUEST_OK );
}


/** NVIC handler registration stub of any SPI peripheral - stores the handler */
static nvic_RequestState_t Ut_Spi_NvicAnyHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int cmockNumCalls )
{
    TEST_ASSERT_NOT_NULL( irqHandler );

    return ( Ut_Spi_Nvic_HandlerCallback( irqId, irqHandler, cmockNumCalls ) );
}


/**
 * \brief Verifies and resets the mocks, ignores all RCC / NVIC / GPIO / DMA calls of
 *        peripheral configuration and data handling of any SPI peripheral. Release of DMA
 *        streams and of the SPI interrupt is counted by \ref Ut_Spi_DmaIrqOffStub and
 *        \ref Ut_Spi_NvicIrqOffStub (counters cleared, OK returned).
 *
 * \note  CMock memory is common for all mocks - a single mock must not be destroyed after
 *        the ignores are set.
 */
static void Ut_Spi_Ignore_PeriphMocks( void )
{
    MockRcc_Port_Verify();
    MockNvic_Port_Verify();
    MockGpio_Port_Verify();
    MockDma_Port_Verify();

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();

    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Spi_RccAnyClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Spi_NvicAnyHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_StubWithCallback( Ut_Spi_NvicIrqOffStub );
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_StubWithCallback( Ut_Spi_Dma_InitCallback );
    Dma_Set_TransferCompleteIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_PeriphTransferSize_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryTransferSize_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddrIncrement_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_DataCount_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_StubWithCallback( Ut_Spi_DmaIrqOffStub );

    utSpi_DmaIrqOffCnt    = 0u;
    utSpi_DmaIrqOffState  = DMA_REQUEST_OK;
    utSpi_NvicIrqOffCnt   = 0u;
    utSpi_NvicIrqOffState = NVIC_REQUEST_OK;
}


/**
 * \brief   Spi_Deinit() releases both DMA streams and the SPI interrupt of DMA mode.
 *
 * \details SPI1 master in DMA mode (DMA2 TX stream 3, RX stream 0), Spi_Deinit() twice.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, Dma_Set_InterruptInactive() for both streams (2x), SPI interrupt
 *   disabled in NVIC (1x), TXDMAEN / RXDMAEN cleared, data handling not initialized any more.
 * - Second Spi_Deinit(): SPI_REQUEST_OK, streams and NVIC not touched again.
 */
void Ut_Spi_Dma_Deinit_StreamsAndIrqReleased( void )
{
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    UT_SPI_REG->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    Ut_Spi_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicIrqOffCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & ( SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicIrqOffCnt );
}


/**
 * \brief   Failures of DMA stream and NVIC release are reported by Spi_Deinit().
 *
 * \details SPI1 in DMA mode, Dma_Set_InterruptInactive() returns DMA_REQUEST_ERROR; SPI1 in
 *          ISR mode, Nvic_Set_PeriphIrq_Inactive() returns NVIC_REQUEST_ERROR.
 *
 * \par Expected results
 * - DMA failure: SPI_REQUEST_ERROR, both streams released anyway (2 calls), data handling not
 *   initialized any more.
 * - NVIC failure: SPI_REQUEST_ERROR, data handling not initialized any more.
 */
void Ut_Spi_Deinit_ReleaseError_ReturnsError( void )
{
    const spi_DataConfig_t dmaConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    const spi_DataConfig_t isrConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    spi_Config_t           config    = Ut_Spi_Get_Config();
    spi_DataConfig_t       readBack;

    Ut_Spi_Ignore_PeriphMocks();
    config.DataConfig = &dmaConfig;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    utSpi_DmaIrqOffState = DMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    utSpi_DmaIrqOffState = DMA_REQUEST_OK;
    config.DataConfig    = &isrConfig;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    utSpi_NvicIrqOffCnt   = 0u;
    utSpi_NvicIrqOffState = NVIC_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicIrqOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
}


/**
 * \brief   Spi_Set_DataConfig() releases the previous data handling.
 *
 * \details SPI1 master in DMA mode reconfigured to POLL, ISR and NONE mode.
 *
 * \par Expected results
 * - DMA -> POLL: SPI_REQUEST_OK, both DMA streams released, SPI interrupt disabled in NVIC.
 * - POLL -> ISR: SPI_REQUEST_OK, SPI interrupt registered.
 * - ISR -> NONE: SPI_REQUEST_OK, interrupt sources disabled, SPI interrupt disabled in NVIC,
 *   read back mode is NONE.
 */
void Ut_Spi_Set_DataConfig_Reconfigured_PreviousModeReleased( void )
{
    const spi_DataConfig_t dmaConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    const spi_DataConfig_t pollConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const spi_DataConfig_t isrConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    const spi_DataConfig_t noneConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_NONE );
    const uint32_t         itMask     = SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_DmaInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dmaConfig ) );

    Ut_Spi_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &pollConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicIrqOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_POLL, readBack.XferMode );

    utSpi_Isr = NULL;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &isrConfig ) );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_NvicIrqOffCnt );

    UT_SPI_REG->CR2 |= itMask;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &noneConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & itMask );
    TEST_ASSERT_EQUAL_UINT32( 3u, utSpi_NvicIrqOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_NONE, readBack.XferMode );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
}


/**
 * \brief   Spi_Deinit() aborts the running ISR transfer.
 *
 * \details SPI1 master in ISR mode, 2 frame transfer started, Spi_Deinit().
 *
 * \par Expected results
 * - SPI_REQUEST_OK, interrupt sources disabled, SPE cleared, no callback, transfer state
 *   INACTIVE.
 */
void Ut_Spi_Isr_Deinit_RunningTransferAborted( void )
{
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 2u };
    const uint32_t          itMask      = SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
    spi_FunctionState_t     xferState   = SPI_FUNCTION_ACTIVE;

    Ut_Spi_Init_Master( NULL );
    Ut_Spi_Expect_IrqInit();
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_NOT_EQUAL( 0u, UT_SPI_REG->CR2 & itMask );

    Ut_Spi_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 & itMask );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicIrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
}


/**
 * \brief   Other SPI peripherals use their own SPI interrupt and DMA callbacks.
 *
 * \details Every SPI peripheral of the MCU except SPI1 in DMA mode (first streams of the
 *          request map). Captured SPI interrupt without transfer, then a 2 frame reception is
 *          started and DMA callbacks of the peripheral are called (RX error, TX error, TX /
 *          RX done). MCUs with SPI1 only: test ignored.
 *
 * \par Expected results
 * - SPI interrupt without transfer: no callback.
 * - RX error of running transfer: one error callback SPI_XFER_ERROR_DMA_TRANSFER; TX error,
 *   TX / RX done without transfer: no further callback.
 * - Spi_Deinit(): SPI_REQUEST_OK, both streams released.
 */
void Ut_Spi_Dma_OtherPeriphCallbacks_OwnPeripheralReported( void )
{
#if defined(SPI2) || defined(SPI3) || defined(SPI4) || defined(SPI5) || defined(SPI6)
    const struct
    {
        spi_PeriphId_t       PeriphId;
        SPI_TypeDef *        PeriphReg;
        spi_DmaPeriphId_t    DmaId;
        spi_DmaChannelId_t   TxStream;
        spi_DmaChannelId_t   RxStream;
    }   periphLut[] =
    {
#ifdef SPI2
        { SPI_PERIPH_2, SPI2, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_4, SPI_DMA_CHANNEL_3 },
#endif /* SPI2 */
#ifdef SPI3
        { SPI_PERIPH_3, SPI3, SPI_DMA_PERIPH_1, SPI_DMA_CHANNEL_5, SPI_DMA_CHANNEL_0 },
#endif /* SPI3 */
#ifdef SPI4
        { SPI_PERIPH_4, SPI4, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_1, SPI_DMA_CHANNEL_0 },
#endif /* SPI4 */
#ifdef SPI5
        { SPI_PERIPH_5, SPI5, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_4, SPI_DMA_CHANNEL_3 },
#endif /* SPI5 */
#ifdef SPI6
        { SPI_PERIPH_6, SPI6, SPI_DMA_PERIPH_2, SPI_DMA_CHANNEL_5, SPI_DMA_CHANNEL_6 },
#endif /* SPI6 */
    };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
        spi_Config_t     config     = Ut_Spi_Get_Config();

        dataConfig.TxDmaPeriphId  = periphLut[ idx ].DmaId;
        dataConfig.TxDmaChannelId = periphLut[ idx ].TxStream;
        dataConfig.RxDmaPeriphId  = periphLut[ idx ].DmaId;
        dataConfig.RxDmaChannelId = periphLut[ idx ].RxStream;
        config.PeriphId           = periphLut[ idx ].PeriphId;
        config.DataConfig         = &dataConfig;

        Ut_Spi_Ignore_PeriphMocks();
        utSpi_Isr         = NULL;
        utSpi_DmaInitCnt  = 0u;
        utSpi_ErrorCnt    = 0u;
        utSpi_CompleteCnt = 0u;

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );
        TEST_ASSERT_NOT_NULL( utSpi_Isr );

        periphLut[ idx ].PeriphReg->SR = 0u;
        utSpi_Isr();
        TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt + utSpi_CompleteCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphLut[ idx ].PeriphId, &request ) );

        utSpi_DmaConfig[ 1u ].TransferErrorCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
        TEST_ASSERT_EQUAL( SPI_XFER_ERROR_DMA_TRANSFER, utSpi_LastError );

        utSpi_DmaConfig[ 0u ].TransferErrorCallback();
        utSpi_DmaConfig[ 0u ].TransferCompleteCallback();
        utSpi_DmaConfig[ 1u ].TransferCompleteCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
        TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( periphLut[ idx ].PeriphId ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaIrqOffCnt );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU with SPI1 only" );
#endif /* SPI2 OR SPI3 OR SPI4 OR SPI5 OR SPI6 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Releases every peripheral of the module (static runtime state of previous tests) by
 *        Spi_Deinit() with ignored mocks, mocks are reset afterwards.
 */
static void Ut_Spi_Reset_Module( void )
{
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );

    for( spi_PeriphId_t periphId = (spi_PeriphId_t)0u; SPI_PERIPH_CNT > periphId; periphId++ )
    {
        (void)Spi_Deinit( periphId );
    }

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();

    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
}


/**
 * \brief Expects read of peripheral clock frequency.
 *
 * \param periphId [in]: RCC peripheral
 * \param clockHz  [in]: Reported clock frequency
 */
static void Ut_Spi_Expect_PeriphClk( rcc_PeriphId_t periphId, rcc_FreqHz_t clockHz )
{
    static rcc_FreqHz_t reportedClk[ 8u ];
    static uint32_t     reportedIdx = 0u;

    /* Every expectation keeps own storage - more expectations can be queued */
    reportedIdx                = ( reportedIdx + 1u ) % 8u;
    reportedClk[ reportedIdx ] = clockHz;

    Rcc_Get_PeriphClk_ExpectAndReturn( periphId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &reportedClk[ reportedIdx ] );
}


/**
 * \brief Expects read of core (AHB) clock frequency.
 *
 * \param clockHz [in]: Reported clock frequency
 */
static void Ut_Spi_Expect_CoreClk( rcc_FreqHz_t clockHz )
{
    static rcc_FreqHz_t reportedClk = 0u;

    reportedClk = clockHz;

    Rcc_Get_ClkBusClk_ExpectAndReturn( RCC_CLK_BUS_AHB1, NULL, RCC_REQUEST_OK );
    Rcc_Get_ClkBusClk_IgnoreArg_clkBusFreq();
    Rcc_Get_ClkBusClk_ReturnThruPtr_clkBusFreq( &reportedClk );
}


/**
 * \brief Expects initialization of GPIO pin in push-pull alternate function 5 mode.
 *
 * \param portId [in]: GPIO port
 * \param pinId  [in]: GPIO pin
 * \param pull   [in]: Pull resistor configuration
 */
static void Ut_Spi_Expect_GpioInit( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinPullCfg_t pull )
{
    static gpio_Config_t expectedConfig[ 4u ];
    static uint32_t      expectedIdx = 0u;

    expectedIdx = ( expectedIdx + 1u ) % 4u;

    (void)memset( &expectedConfig[ expectedIdx ], 0, sizeof( gpio_Config_t ) );

    expectedConfig[ expectedIdx ].PortId         = portId;
    expectedConfig[ expectedIdx ].PinId          = pinId;
    expectedConfig[ expectedIdx ].PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expectedConfig[ expectedIdx ].PinPull        = pull;
    expectedConfig[ expectedIdx ].PinSpeed       = GPIO_PIN_SPEED_HIGH;
    expectedConfig[ expectedIdx ].PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expectedConfig[ expectedIdx ].PinAltFunction = GPIO_ALT_FUNC_5;
    expectedConfig[ expectedIdx ].PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( &expectedConfig[ expectedIdx ], GPIO_REQUEST_OK );
}


/**
 * \brief Expects clock activation and reset of the peripheral.
 */
static void Ut_Spi_Expect_Activation( void )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_OK );
}


/**
 * \brief Expects registration of the SPI interrupt handler (captured), priority and enable.
 */
static void Ut_Spi_Expect_IrqInit( void )
{
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( UT_SPI_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Handler_AddCallback( Ut_Spi_Nvic_HandlerCallback );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, UT_SPI_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_SPI_NVIC, NVIC_REQUEST_OK );
}


/**
 * \brief Expects initialization of DMA data handling of SPI1 (both streams, captured by
 *        stub), stop of the streams and SPI interrupt initialization.
 */
static void Ut_Spi_Expect_DmaInit( void )
{
    Dma_Get_DefaultConfig_ExpectAnyArgsAndReturn( DMA_REQUEST_OK );
    Dma_Init_ExpectAnyArgsAndReturn( DMA_REQUEST_OK );
    Dma_Init_AddCallback( Ut_Spi_Dma_InitCallback );
    Dma_Set_TransferCompleteIrqActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_TX_STREAM, DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_TX_STREAM, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_TX_STREAM, DMA_REQUEST_OK );

    Dma_Get_DefaultConfig_ExpectAnyArgsAndReturn( DMA_REQUEST_OK );
    Dma_Init_ExpectAnyArgsAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_RX_STREAM, DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_RX_STREAM, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_RX_STREAM, DMA_REQUEST_OK );

    Ut_Spi_Expect_DmaStop();
    Ut_Spi_Expect_IrqInit();
}


/**
 * \brief Expects stop of both DMA streams of SPI1.
 */
static void Ut_Spi_Expect_DmaStop( void )
{
    Dma_Set_TransferInactive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_TX_STREAM, DMA_REQUEST_OK );
    Dma_Set_TransferInactive_ExpectAndReturn( UT_SPI_DMA, UT_SPI_DMA_RX_STREAM, DMA_REQUEST_OK );
}


/**
 * \brief Expects configuration and start of DMA stream for the transfer.
 *
 * \param streamId  [in]: DMA stream of DMA2
 * \param xferSize  [in]: Peripheral and memory transfer size
 * \param addrInc   [in]: Memory address increment
 * \param memAddr   [in]: Memory address
 * \param dataCount [in]: Count of frames
 */
static void Ut_Spi_Expect_DmaTransfer( dma_ChannelId_t streamId, dma_TransferSize_t xferSize, dma_MemoryAddrInc_t addrInc, dma_MemoryAddr_t memAddr, dma_DataCount_t dataCount )
{
    Dma_Set_PeriphTransferSize_ExpectAndReturn( UT_SPI_DMA, streamId, xferSize, DMA_REQUEST_OK );
    Dma_Set_MemoryTransferSize_ExpectAndReturn( UT_SPI_DMA, streamId, xferSize, DMA_REQUEST_OK );
    Dma_Set_MemoryAddrIncrement_ExpectAndReturn( UT_SPI_DMA, streamId, addrInc, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( UT_SPI_DMA, streamId, memAddr, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( UT_SPI_DMA, streamId, dataCount, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( UT_SPI_DMA, streamId, DMA_REQUEST_OK );
}


/**
 * \brief Expects read of remaining data count of DMA stream.
 *
 * \param streamId  [in]: DMA stream of DMA2
 * \param remaining [in]: Reported remaining count
 */
static void Ut_Spi_Expect_DmaRemaining( dma_ChannelId_t streamId, dma_DataCount_t remaining )
{
    static dma_DataCount_t reportedCnt[ 4u ];
    static uint32_t        reportedIdx = 0u;

    reportedIdx                = ( reportedIdx + 1u ) % 4u;
    reportedCnt[ reportedIdx ] = remaining;

    Dma_Get_DataCount_ExpectAndReturn( UT_SPI_DMA, streamId, NULL, DMA_REQUEST_OK );
    Dma_Get_DataCount_IgnoreArg_dataCount();
    Dma_Get_DataCount_ReturnThruPtr_dataCount( &reportedCnt[ reportedIdx ] );
}


/**
 * \brief Returns configuration of SPI1 master used by tests (default configuration, SPI1,
 *        1 MHz).
 *
 * \return Peripheral configuration
 */
static spi_Config_t Ut_Spi_Get_Config( void )
{
    spi_Config_t config;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DefaultConfig( &config ) );

    config.PeriphId = UT_SPI_BUS;
    config.BusFreq  = UT_SPI_BUS_FREQ_HZ;

    return ( config );
}


/**
 * \brief Initializes SPI1 master without pins (data handling initialization expected by the
 *        caller).
 *
 * \param dataConfig [in]: Data handling configuration or NULL
 */
static void Ut_Spi_Init_Master( const spi_DataConfig_t * const dataConfig )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.DataConfig = dataConfig;

    Ut_Spi_Expect_Activation();
    Ut_Spi_Expect_PeriphClk( UT_SPI_RCC, UT_SPI_CLK_HZ );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );
}


/**
 * \brief Initializes SPI1 slave with software NSS without pins (data handling initialization
 *        expected by the caller).
 *
 * \param dataConfig [in]: Data handling configuration or NULL
 */
static void Ut_Spi_Init_Slave( const spi_DataConfig_t * const dataConfig )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.Mode       = SPI_MODE_SLAVE;
    config.DataConfig = dataConfig;

    Ut_Spi_Expect_Activation();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );
}


/**
 * \brief Returns data handling configuration of tests (DMA2 streams 3 / 0 of SPI1, priority
 *        of tests, callbacks of tests).
 *
 * \param xferMode [in]: Data transfer mode
 * \return Data handling configuration
 */
static spi_DataConfig_t Ut_Spi_Get_DataConfig( spi_XferMode_t xferMode )
{
    const spi_DataConfig_t dataConfig =
    {
        .XferMode             = xferMode,
        .TxDmaPeriphId        = SPI_DMA_PERIPH_2,
        .TxDmaChannelId       = SPI_DMA_CHANNEL_3,
        .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
        .RxDmaPeriphId        = SPI_DMA_PERIPH_2,
        .RxDmaChannelId       = SPI_DMA_CHANNEL_0,
        .RxDmaPriority        = SPI_DMA_PRIORITY_HIGH,
        .IrqPriority          = UT_SPI_PRIO,
        .XferCompleteCallback = Ut_Spi_XferCompleteCallback,
        .ErrorCallback        = Ut_Spi_ErrorCallback,
    };

    return ( dataConfig );
}


/**
 * \brief NVIC mock callback capturing registered interrupt handler.
 */
static nvic_RequestState_t Ut_Spi_Nvic_HandlerCallback( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int cmockNumCalls )
{
    (void)irqId;
    (void)cmockNumCalls;

    utSpi_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief DMA mock callback capturing configurations of Dma_Init().
 */
static dma_RequestState_t Ut_Spi_Dma_InitCallback( dma_ConfigStruct_t * const dmaConfig, int cmockNumCalls )
{
    (void)cmockNumCalls;

    if( UT_SPI_DMA_CFG_CNT > utSpi_DmaInitCnt )
    {
        utSpi_DmaConfig[ utSpi_DmaInitCnt ] = *dmaConfig;
    }

    utSpi_DmaInitCnt++;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief RCC reset mock callback - registers of the tested SPI get their reset values, resets
 *        are counted.
 */
static rcc_RequestState_t Ut_Spi_Rcc_ResetCallback( rcc_PeriphId_t periphId, int cmockNumCalls )
{
    (void)cmockNumCalls;

    TEST_ASSERT_EQUAL( UT_SPI_RCC, periphId );

    UT_SPI_REG->CR1   = 0u;
    UT_SPI_REG->CR2   = 0u;
    UT_SPI_REG->CRCPR = UT_SPI_CRCPR_RESET;

    utSpi_ResetCnt++;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief HW model of slave transmitter affected by device errata "BSY flag may stay high at the
 *        end of a data transfer in Slave mode" - TXE is set, BSY stays set while SPE is set.
 */
static void Ut_Spi_HwModel_SlaveBusy( void )
{
    if( 0u != ( UT_SPI_REG->CR1 & SPI_CR1_SPE ) )
    {
        UT_SPI_REG->SR = SPI_SR_TXE | SPI_SR_BSY;
    }
    else
    {
        UT_SPI_REG->SR = SPI_SR_TXE;
    }
}


/** Transfer complete callback of tests */
static void Ut_Spi_XferCompleteCallback( void )
{
    utSpi_CompleteCnt++;
}


/** Error callback of tests */
static void Ut_Spi_ErrorCallback( spi_XferErrorId_t errorId )
{
    utSpi_ErrorCnt++;
    utSpi_LastError = errorId;
}
