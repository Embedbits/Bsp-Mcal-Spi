/**
 * \author Mr.Nobody
 * \file Test_Spi.c
 * \ingroup Spi
 * \brief Unit tests of Serial Peripheral Interface (SPI) module (STM32U5, SPI v2 with FIFO).
 *
 * Spi.c, Spi_Dma.c, Spi_Isr.c and Spi_Poll.c are compiled unchanged with real
 * LL drivers. SPI registers are emulated by RegMem, RCC, NVIC, GPIO and GPDMA
 * modules are mocked by CMock. SPI ISR registered in NVIC and GPDMA error
 * callbacks passed to Gpdma_Init() are captured by stubs and called directly.
 *
 * \note Emulated registers are plain memory. Status flags (SR) are not changed
 *       by data register accesses - tests preset the flags (TXP, RXP, EOT,
 *       errors, SUSP) and the received data (RXDR) before every transfer step.
 *       A preset RXP stays active, so one step reads all frames it may read.
 * \note Master transfer abort waits for SUSP / EOT - tests preset SUSP before
 *       an abort of a running master transfer.
 * \note DMA buffers are placed in emulated SRAM (\ref REGMEM_SRAM_PTR) - the
 *       module passes 32-bit memory addresses to GPDMA.
 * \note Runtime state of the module is static - setUp() releases every
 *       peripheral by Spi_Deinit() with ignored mocks.
 * \note Mode fault of a master (MODF) is emulated by HW model running in background
 *       thread (Ut_Spi_HwModel) - tests are executed serially (RUN_SERIAL).
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Spi_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
#include "Stm32_spi.h"                      /* SPI registers definition       */

/* ============================= TYPEDEFS =================================== */

/** \brief Record of GPDMA calls of one channel */
typedef struct
{
    uint32_t            ActiveCnt;      /**< Gpdma_Set_ChannelActive() calls     */
    uint32_t            InactiveCnt;    /**< Gpdma_Set_ChannelInactive() calls   */
    uint32_t            SpeAtInactive;  /**< CR1.SPE at the last Gpdma_Set_ChannelInactive() call */
    uint32_t            IrqOnCnt;       /**< Gpdma_Set_InterruptActive() calls   */
    uint32_t            IrqOffCnt;      /**< Gpdma_Set_InterruptInactive() calls */
    uint32_t            PrioCnt;        /**< Gpdma_Set_Priority() calls          */
    gpdma_Priority_t    Prio;           /**< Last channel priority               */
    gpdma_SrcAddr_t     SrcAddr;        /**< Last source address                 */
    gpdma_DstAddr_t     DstAddr;        /**< Last destination address            */
    gpdma_AddrMode_t    SrcMode;        /**< Last source address mode            */
    gpdma_AddrMode_t    DstMode;        /**< Last destination address mode       */
    gpdma_DataSize_t    SrcSize;        /**< Last source data size               */
    gpdma_DataSize_t    DstSize;        /**< Last destination data size          */
    gpdma_BlockSize_t   BlockSize;      /**< Last block size                     */
}   ut_SpiDmaChannel_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Spi_Reset_Mocks          ( void );
static void                 Ut_Spi_HwModel              ( void );
static void                 Ut_Spi_Ignore_PeriphMocks   ( void );
static spi_Config_t         Ut_Spi_Get_Config           ( void );
static spi_DataConfig_t     Ut_Spi_Get_DataConfig       ( spi_XferMode_t xferMode );
static void                 Ut_Spi_Init                 ( const spi_Config_t * const config );
static void                 Ut_Spi_Init_Master          ( spi_XferMode_t xferMode );
static void                 Ut_Spi_Expect_Activation    ( rcc_PeriphId_t rccId );
static void                 Ut_Spi_Expect_KernelClk     ( rcc_PeriphId_t clkSrcId, rcc_FreqHz_t clockHz );
static void                 Ut_Spi_Task                 ( uint32_t srFlags );
static void                 Ut_Spi_Call_Isr             ( uint32_t srFlags );
static void                 Ut_Spi_Check_XferEnd        ( spi_XferErrorId_t expError );
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel );

static rcc_RequestState_t   Ut_Spi_RccClkSrcStub        ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_Spi_RccClkStub           ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_Spi_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Spi_NvicHandlerStub      ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static nvic_RequestState_t  Ut_Spi_NvicInactiveStub     ( nvic_PeriphIrqList_t irqId, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaInitStub          ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaActiveStub        ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaInactiveStub      ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaIrqOnStub         ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaIrqOffStub        ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaPrioStub          ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_Priority_t channelPrio, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaSrcAddrStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_SrcAddr_t sourceAddr, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaDstAddrStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DstAddr_t destAddr, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaSrcModeStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t srcAddrMode, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaDstModeStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t destAddrMode, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaSrcSizeStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t srcDataSize, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaDstSizeStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t destDataSize, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaBlockStub         ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t blockSize, int callCnt );
static gpdma_RequestState_t Ut_Spi_DmaRemainingStub     ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t * const blockSize, int callCnt );

static void                 Ut_Spi_XferCompleteCallback ( void );
static void                 Ut_Spi_ErrorCallback        ( spi_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** SPI peripheral used by tests (available on all STM32U5 devices) */
#define UT_SPI_BUS                          ( SPI_PERIPH_1 )
#define UT_SPI_REG                          ( SPI1 )
#define UT_SPI_RCC                          ( RCC_PERIPH_SPI1_PCLK2 )
#define UT_SPI_NVIC                         ( NVIC_PERIPH_IRQ_SPI1 )

/** Kernel clock returned by RCC stub [Hz] */
#define UT_SPI_CLK_HZ                       ( 100000000u )

/** Bus frequency of test configuration [Hz] and MBR field of it (100 MHz / 128 = 781 kHz) */
#define UT_SPI_BUS_FREQ_HZ                  ( 1000000u )
#define UT_SPI_MBR_1MHZ                     ( 6u )

/** DSIZE field of 8-bit frames (bits - 1) */
#define UT_SPI_DSIZE_8BIT                   ( 7u )

/** Interrupt priority of test configurations */
#define UT_SPI_PRIO                         ( 6u )

/** GPDMA channels of tests */
#define UT_SPI_DMA                          ( GPDMA_PERIPH_1 )
#define UT_SPI_DMA_TX_CHANNEL               ( GPDMA_CHANNEL_0 )
#define UT_SPI_DMA_RX_CHANNEL               ( GPDMA_CHANNEL_1 )

/** Count of GPDMA configurations stored by Gpdma_Init() stub */
#define UT_SPI_DMA_CFG_CNT                  ( 4u )

/** Count of GPDMA channels recorded per GPDMA peripheral */
#define UT_SPI_DMA_CHANNELS                 ( 8u )

/** Offsets of DMA buffers in emulated SRAM */
#define UT_SPI_TX_BUF_OFFSET                ( 0x100u )
#define UT_SPI_RX_BUF_OFFSET                ( 0x200u )

/** Count of GPIO configurations stored by Gpio_Init() stub */
#define UT_SPI_GPIO_CFG_CNT                 ( 4u )

/** All IFCR clear bits */
#define UT_SPI_IFCR_ALL                     ( SPI_IFCR_EOTC  | SPI_IFCR_TXTFC  | SPI_IFCR_UDRC  | SPI_IFCR_OVRC | \
                                              SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC )

/** IER bits of end of transfer and errors */
#define UT_SPI_IER_EVENTS                   ( SPI_IER_EOTIE | SPI_IER_UDRIE | SPI_IER_OVRIE | SPI_IER_TIFREIE | SPI_IER_MODFIE )

/** All IER bits used by the module */
#define UT_SPI_IER_ALL                      ( UT_SPI_IER_EVENTS | SPI_IER_TXPIE | SPI_IER_RXPIE )

/** Test pins of SPI1 (AF5) */
#define UT_SPI_SCK_PIN      ( SPI_SCK_PIN_SPI1_PA5 )
#define UT_SPI_MISO_PIN     ( SPI_MISO_PIN_SPI1_PA6 )
#define UT_SPI_MOSI_PIN     ( SPI_MOSI_PIN_SPI1_PA7 )
#define UT_SPI_NSS_PIN      ( SPI_NSS_PIN_SPI1_PA4 )

/** Encoded pin from the peripheral index, port index, pin number and alternate function number
 *  (bit-fields written independently of SPI_PIN_ENCODE) */
#define UT_SPI_PIN_CODE( PERIPH, PORT, PIN, AF )    ( ( (PERIPH) << 15u ) | ( (PORT) << 10u ) | ( (PIN) << 5u ) | (AF) )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC */
static nvic_IsrCallback_t       utSpi_Isr;

/** Kernel clock returned by RCC stub */
static rcc_FreqHz_t             utSpi_ClkHz;

/** Count of Nvic_Set_PeriphIrq_Inactive() calls and its return value */
static uint32_t                 utSpi_NvicOffCnt;
static nvic_RequestState_t      utSpi_NvicOffState;

/** GPIO configurations of Gpio_Init() calls */
static gpio_Config_t            utSpi_GpioConfig[ UT_SPI_GPIO_CFG_CNT ];
static uint32_t                 utSpi_GpioInitCnt;

/** GPDMA configurations of Gpdma_Init() calls (structure and transfer configuration) */
static gpdma_ConfigStruct_t     utSpi_DmaConfig[ UT_SPI_DMA_CFG_CNT ];
static gpdma_TransferConfig_t   utSpi_DmaXferConfig[ UT_SPI_DMA_CFG_CNT ];
static uint32_t                 utSpi_DmaInitCnt;
static gpdma_RequestState_t     utSpi_DmaInitState;

/** Return value of Gpdma_Set_InterruptInactive() stub */
static gpdma_RequestState_t     utSpi_DmaIrqOffState;

/** Remaining block size returned by Gpdma_Get_BlockSize() stub */
static gpdma_BlockSize_t        utSpi_DmaRemaining;

/** Records of GPDMA channel calls */
static ut_SpiDmaChannel_t       utSpi_DmaChannel[ GPDMA_PERIPH_CNT ][ UT_SPI_DMA_CHANNELS ];

/** Counts of callback calls */
static uint32_t                 utSpi_CompleteCnt;
static uint32_t                 utSpi_ErrorCnt;

/** Error of the last error callback */
static spi_XferErrorId_t        utSpi_LastError;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Spi_Ignore_PeriphMocks();

    for( spi_PeriphId_t periphId = (spi_PeriphId_t)0u; SPI_PERIPH_CNT > periphId; periphId++ )
    {
        (void)Spi_Deinit( periphId );
    }

    Ut_Spi_Reset_Mocks();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utSpi_Isr            = NULL;
    utSpi_ClkHz          = UT_SPI_CLK_HZ;
    utSpi_NvicOffCnt     = 0u;
    utSpi_NvicOffState   = NVIC_REQUEST_OK;
    utSpi_GpioInitCnt    = 0u;
    utSpi_DmaInitCnt     = 0u;
    utSpi_DmaInitState   = GPDMA_REQUEST_OK;
    utSpi_DmaIrqOffState = GPDMA_REQUEST_OK;
    utSpi_DmaRemaining   = 0u;
    utSpi_CompleteCnt    = 0u;
    utSpi_ErrorCnt       = 0u;
    utSpi_LastError      = SPI_XFER_ERROR_NONE;

    (void)memset( utSpi_GpioConfig, 0, sizeof( utSpi_GpioConfig ) );
    (void)memset( utSpi_DmaConfig, 0, sizeof( utSpi_DmaConfig ) );
    (void)memset( utSpi_DmaXferConfig, 0, sizeof( utSpi_DmaXferConfig ) );
    (void)memset( utSpi_DmaChannel, 0, sizeof( utSpi_DmaChannel ) );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
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
    const spi_ModuleVersion_t version = Spi_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   Items of the pin tables carry peripheral, port, pin and alternate function of the pin.
 *
 * \details Expected values are written as (peripheral index, port index, pin number, alternate
 *          function number) of the datasheet alternate function mapping, independently of the
 *          encoding macro.
 *
 * \par Expected results
 * - SCK / MISO / MOSI / NSS items of every SPI peripheral carry the expected bit-fields.
 * - Unused items of the four tables equal SPI_PIN_UNUSED.
 */
void Ut_Spi_PinTables_Items_EncodePeriphPortPinAndAf( void )
{
    /* SPI1 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_1, 0u, 5u, 5u ), SPI_SCK_PIN_SPI1_PA5 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_1, 0u, 6u, 5u ), SPI_MISO_PIN_SPI1_PA6 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_1, 0u, 7u, 5u ), SPI_MOSI_PIN_SPI1_PA7 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_1, 0u, 4u, 5u ), SPI_NSS_PIN_SPI1_PA4 );

    /* SPI2 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_2, 1u, 13u, 5u ), SPI_SCK_PIN_SPI2_PB13 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_2, 1u, 14u, 5u ), SPI_MISO_PIN_SPI2_PB14 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_2, 1u, 15u, 5u ), SPI_MOSI_PIN_SPI2_PB15 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_2, 1u, 12u, 5u ), SPI_NSS_PIN_SPI2_PB12 );

    /* SPI3 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_3, 2u, 10u, 6u ), SPI_SCK_PIN_SPI3_PC10 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_3, 2u, 11u, 6u ), SPI_MISO_PIN_SPI3_PC11 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_3, 2u, 12u, 6u ), SPI_MOSI_PIN_SPI3_PC12 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_3, 0u, 15u, 6u ), SPI_NSS_PIN_SPI3_PA15 );

    /* Unused pin */
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED, SPI_SCK_PIN_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED, SPI_MISO_PIN_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED, SPI_MOSI_PIN_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( SPI_PIN_UNUSED, SPI_NSS_PIN_UNUSED );
}

/* ========================== INITIALIZATION ================================ */

/**
 * \brief   Spi_Get_DefaultConfig() fills the default configuration.
 *
 * \par Expected results
 * - SPI1, PCLK kernel clock, master 1 MHz, mode 0, 8-bit, MSB first, full duplex, Motorola,
 *   software NSS active low without pulse, no data handling, no pins, high pin speed.
 * - NULL pointer: SPI_REQUEST_ERROR.
 */
void Ut_Spi_Get_DefaultConfig_FillsDefaults( void )
{
    spi_Config_t config;

    (void)memset( &config, 0xA5, sizeof( config ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( (spi_PeriphId_t)0u, config.PeriphId );
    TEST_ASSERT_EQUAL( SPI_CLK_SRC_PCLK, config.ClkSrc );
    TEST_ASSERT_EQUAL( SPI_MODE_MASTER, config.Mode );
    TEST_ASSERT_EQUAL_UINT32( 1000000u, config.BusFreq );
    TEST_ASSERT_EQUAL( SPI_CLOCK_MODE_0, config.ClockMode );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_8BIT, config.DataSize );
    TEST_ASSERT_EQUAL( SPI_BIT_ORDER_MSB_FIRST, config.BitOrder );
    TEST_ASSERT_EQUAL( SPI_DIRECTION_FULL_DUPLEX, config.Direction );
    TEST_ASSERT_EQUAL( SPI_FRAME_FORMAT_MOTOROLA, config.FrameFormat );
    TEST_ASSERT_EQUAL( SPI_NSS_MODE_SOFT, config.NssConfig.Mode );
    TEST_ASSERT_EQUAL( SPI_NSS_POLARITY_LOW, config.NssConfig.Polarity );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, config.NssConfig.Pulse );
    TEST_ASSERT_NULL( config.DataConfig );
    TEST_ASSERT_EQUAL( SPI_SCK_PIN_UNUSED, config.SckPin );
    TEST_ASSERT_EQUAL( SPI_MISO_PIN_UNUSED, config.MisoPin );
    TEST_ASSERT_EQUAL( SPI_MOSI_PIN_UNUSED, config.MosiPin );
    TEST_ASSERT_EQUAL( SPI_NSS_PIN_UNUSED, config.NssPin );
    TEST_ASSERT_EQUAL( SPI_PIN_SPEED_HIGH, config.PinSpeed );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DefaultConfig( NULL ) );
}


/**
 * \brief   Spi_Init() rejects invalid configuration without any access.
 *
 * \details NULL, invalid peripheral, clock source, mode, clock mode, bit order, direction,
 *          frame format, pin speed, master with zero frequency, clock source not available
 *          for SPI1 (PCLK), NSS pulse with software NSS, invalid NSS mode, SCK / MISO / MOSI / NSS
 *          pin of other peripheral, pin with port / pin / alternate function out of range, data
 *          size out of range.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, no RCC / GPIO call (strict mocks), CFG1 / CFG2 / CR1 not written.
 */
void Ut_Spi_Init_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    spi_Config_t config;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( NULL ) );

    config = Ut_Spi_Get_Config(); config.PeriphId         = SPI_PERIPH_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.ClkSrc           = SPI_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.Mode             = SPI_MODE_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.ClockMode        = SPI_CLOCK_MODE_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.BitOrder         = SPI_BIT_ORDER_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.Direction        = SPI_DIRECTION_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.FrameFormat      = SPI_FRAME_FORMAT_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.PinSpeed         = SPI_PIN_SPEED_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.BusFreq          = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.NssConfig.Pulse  = SPI_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.NssConfig.Mode   = SPI_NSS_MODE_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.SckPin        = SPI_SCK_PIN_SPI2_PB13;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.MisoPin       = SPI_MISO_PIN_SPI2_PB14;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.MosiPin       = SPI_MOSI_PIN_SPI2_PB15;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.NssPin        = SPI_NSS_PIN_SPI2_PB12;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.SckPin        = (spi_SckPin_t)SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_CNT, GPIO_PIN_ID_5, GPIO_ALT_FUNC_5 );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.MisoPin       = (spi_MisoPin_t)SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_5 );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.MosiPin       = (spi_MosiPin_t)SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_7, GPIO_ALT_FUNC_CNT );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.DataSize         = SPI_DATA_SIZE_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 | UT_SPI_REG->CFG2 | UT_SPI_REG->CR1 );
}


/**
 * \brief   Spi_Init() activates the kernel clock and configures a master.
 *
 * \details Default configuration (SPI1 master 1 MHz, PCLK2 kernel clock 100 MHz, no pins,
 *          no data handling).
 *
 * \par Expected results
 * - RCC: clock of PCLK kernel clock source activated, reset pulse, kernel clock source and
 *   frequency read.
 * - CFG2: MASTER, AFCNTR, SSM (software NSS); CR1: SSI = 1 (master not deselected).
 * - CFG1: DSIZE = 7 (8 bits), MBR = 6 (100 MHz / 128). Peripheral not enabled.
 */
void Ut_Spi_Init_Master_ClockActivatedAndConfigured( void )
{
    const spi_Config_t config = Ut_Spi_Get_Config();

    Ut_Spi_Expect_Activation( UT_SPI_RCC );
    Ut_Spi_Expect_KernelClk( UT_SPI_RCC, UT_SPI_CLK_HZ );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR | SPI_CFG2_SSM,
                             UT_SPI_REG->CFG2 & ( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR | SPI_CFG2_SSM | SPI_CFG2_SSOE ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SSI, UT_SPI_REG->CR1 & ( SPI_CR1_SSI | SPI_CR1_SPE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_DSIZE_8BIT << SPI_CFG1_DSIZE_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_DSIZE );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_MBR_1MHZ << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
}


/**
 * \brief   Spi_Init() activates the clock of the selected kernel clock source.
 *
 * \details SPI1 with MSIK kernel clock source (48 MHz), kernel clock source read back as
 *          MSIK.
 *
 * \par Expected results
 * - Rcc_Set_PeriphActive / reset with RCC_PERIPH_SPI1_MSIK, frequency of MSIK used
 *   (MBR = 5: 48 MHz / 64).
 */
void Ut_Spi_Init_ClockSourceMsik_ClockOfSourceActivated( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.ClkSrc = SPI_CLK_SRC_MSIK;

    Ut_Spi_Expect_Activation( RCC_PERIPH_SPI1_MSIK );
    Ut_Spi_Expect_KernelClk( RCC_PERIPH_SPI1_MSIK, 48000000u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 5u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
}


/**
 * \brief   Spi_Init() configures the pins as push-pull alternate function outputs.
 *
 * \details SCK PA5, MISO PA6, MOSI PA7, NSS PA4 (AF5), very high pin speed.
 *
 * \par Expected results
 * - Gpio_Init() 4x in order SCK, MISO, MOSI, NSS: alternate mode, no pull, push-pull, AF5,
 *   very high speed.
 */
void Ut_Spi_Init_Pins_GpioAlternatePushPull( void )
{
    spi_Config_t        config  = Ut_Spi_Get_Config();
    const gpio_PinId_t  pinLut[ UT_SPI_GPIO_CFG_CNT ] = { GPIO_PIN_ID_5, GPIO_PIN_ID_6, GPIO_PIN_ID_7, GPIO_PIN_ID_4 };

    config.SckPin   = UT_SPI_SCK_PIN;
    config.MisoPin  = UT_SPI_MISO_PIN;
    config.MosiPin  = UT_SPI_MOSI_PIN;
    config.NssPin   = UT_SPI_NSS_PIN;
    config.PinSpeed = SPI_PIN_SPEED_VERY_HIGH;

    Ut_Spi_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( UT_SPI_GPIO_CFG_CNT, utSpi_GpioInitCnt );

    for( uint32_t pinIdx = 0u; UT_SPI_GPIO_CFG_CNT > pinIdx; pinIdx++ )
    {
        TEST_ASSERT_EQUAL( GPIO_PORT_A,               utSpi_GpioConfig[ pinIdx ].PortId );
        TEST_ASSERT_EQUAL( pinLut[ pinIdx ],          utSpi_GpioConfig[ pinIdx ].PinId );
        TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE,   utSpi_GpioConfig[ pinIdx ].PinMode );
        TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE,        utSpi_GpioConfig[ pinIdx ].PinPull );
        TEST_ASSERT_EQUAL( GPIO_PIN_SPEED_VERY_HIGH,  utSpi_GpioConfig[ pinIdx ].PinSpeed );
        TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_PUSHPULL,  utSpi_GpioConfig[ pinIdx ].PinOutType );
        TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_5,           utSpi_GpioConfig[ pinIdx ].PinAltFunction );
    }
}


/**
 * \brief   Spi_Init() configures a slave with hardware NSS without the bus frequency.
 *
 * \details Slave, hardware NSS active low, bus frequency 0 (not used by slave).
 *
 * \par Expected results
 * - No kernel clock read (strict mocks), MASTER / AFCNTR / SSM cleared, SSI = 0, MBR = 0.
 */
void Ut_Spi_Init_SlaveHardNss_NoBusFrequency( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.Mode           = SPI_MODE_SLAVE;
    config.BusFreq        = 0u;
    config.NssConfig.Mode = SPI_NSS_MODE_HARD;

    Ut_Spi_Expect_Activation( UT_SPI_RCC );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG2 & ( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR | SPI_CFG2_SSM ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SSI );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
}


/**
 * \brief   Spi_Init() configures a master without mode fault.
 *
 * \details HW model emulates mode fault: master (CFG2.MASTER) with active internal slave
 *          select (software NSS with SSI at the active level, or hardware NSS input) gets
 *          SR.MODF and MASTER is cleared by HW. Default master configuration (software
 *          NSS) is initialized from the reset state of the registers.
 * \note    Bug AB#1148: MASTER was written before the NSS configuration (reset state =
 *          hardware NSS input) and before SSI - mode fault, Spi_Init() of a master returned
 *          SPI_REQUEST_ERROR on target.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, CFG2.MASTER and SSM set, CR1.SSI set, SR.MODF = 0.
 */
void Ut_Spi_Init_MasterModeFaultModel_NoModeFault( void )
{
    const spi_Config_t config = Ut_Spi_Get_Config();

    Ut_Spi_Expect_Activation( UT_SPI_RCC );
    Ut_Spi_Expect_KernelClk( UT_SPI_RCC, UT_SPI_CLK_HZ );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_Spi_HwModel ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_MASTER | SPI_CFG2_SSM, UT_SPI_REG->CFG2 & ( SPI_CFG2_MASTER | SPI_CFG2_SSM ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SSI, UT_SPI_REG->CR1 & SPI_CR1_SSI );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->SR & SPI_SR_MODF );
}


/**
 * \brief   Spi_Init() reports failures of RCC and GPIO.
 *
 * \details Rcc_Set_PeriphActive() fails; then Gpio_Init() of SCK fails.
 *
 * \par Expected results
 * - RCC failure: SPI_REQUEST_ERROR, peripheral not configured (CFG2 = 0).
 * - GPIO failure: SPI_REQUEST_ERROR.
 */
void Ut_Spi_Init_RccOrGpioError_ReturnsError( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_SPI_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG2 );

    Ut_Spi_Ignore_PeriphMocks();
    Gpio_Init_Stub( NULL );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_ERROR );
    config.SckPin = UT_SPI_SCK_PIN;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
}


/**
 * \brief   Spi_Init() reports unavailable kernel clock of a master.
 *
 * \details RCC reports kernel clock 0 Hz.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, MBR not written.
 */
void Ut_Spi_Init_KernelClockZero_ReturnsError( void )
{
    const spi_Config_t config = Ut_Spi_Get_Config();

    Ut_Spi_Ignore_PeriphMocks();
    utSpi_ClkHz = 0u;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
}


/**
 * \brief   Spi_Deinit() disables the peripheral, resets it and disables its base clock.
 *
 * \details Enabled peripheral deinitialized, invalid peripheral, RCC clock disable failure.
 *
 * \par Expected results
 * - Reset pulse and clock disable of RCC_PERIPH_SPI1_PCLK2, SPE cleared, SPI_REQUEST_OK.
 * - Invalid peripheral and RCC failure: SPI_REQUEST_ERROR.
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
 * \details Kernel clock 100 MHz: 50 MHz -> /2, 10 MHz -> /16 (6.25 MHz), 400 kHz -> /256
 *          (390 kHz); then 300 kHz (below /256) and 0 Hz.
 *
 * \par Expected results
 * - MBR 0 / 3 / 7, Spi_Get_BusFreq() returns the real frequency.
 * - 300 kHz and 0 Hz: SPI_REQUEST_ERROR, MBR not changed.
 */
void Ut_Spi_Set_BusFreq_PrescalerSelectedAndReadBack( void )
{
    spi_FreqHz_t busFreq = 0u;

    Ut_Spi_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 50000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( UT_SPI_BUS, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 50000000u, busFreq );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 10000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 3u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( UT_SPI_BUS, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 6250000u, busFreq );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( 7u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( UT_SPI_BUS, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 390625u, busFreq );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BusFreq( UT_SPI_BUS, 300000u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BusFreq( UT_SPI_BUS, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( 7u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_BusFreq( UT_SPI_BUS, NULL ) );
}


/**
 * \brief   Clock mode is written to CPOL / CPHA and read back.
 *
 * \par Expected results
 * - Modes 0 - 3: CPOL / CPHA bits, getter returns the mode. Invalid mode rejected.
 */
void Ut_Spi_Set_ClockMode_RoundTrip( void )
{
    const uint32_t  bitLut[ SPI_CLOCK_MODE_CNT ] = { 0u, SPI_CFG2_CPHA, SPI_CFG2_CPOL, SPI_CFG2_CPOL | SPI_CFG2_CPHA };
    spi_ClockMode_t clockMode = SPI_CLOCK_MODE_CNT;

    for( spi_ClockMode_t modeIdx = SPI_CLOCK_MODE_0; SPI_CLOCK_MODE_CNT > modeIdx; modeIdx++ )
    {
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_ClockMode( UT_SPI_BUS, modeIdx ) );
        TEST_ASSERT_EQUAL_HEX32( bitLut[ modeIdx ], UT_SPI_REG->CFG2 & ( SPI_CFG2_CPOL | SPI_CFG2_CPHA ) );
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_ClockMode( UT_SPI_BUS, &clockMode ) );
        TEST_ASSERT_EQUAL( modeIdx, clockMode );
    }

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_ClockMode( UT_SPI_BUS, SPI_CLOCK_MODE_CNT ) );
}


/**
 * \brief   SPI3 (limited feature set on STM32U5) refuses data size above 16 bits.
 *
 * \details Sets 32-bit and 16-bit data size of SPI3 and 32-bit data size of SPI2.
 *
 * \par Expected results
 * - SPI3 32-bit: SPI_REQUEST_ERROR, CFG1 not written.
 * - SPI3 16-bit: SPI_REQUEST_OK, DSIZE = 15.
 * - SPI2 32-bit (full feature set): SPI_REQUEST_OK, DSIZE = 31.
 */
void Ut_Spi_Set_DataSize_Spi3Limited_RefusesAbove16Bit( void )
{
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( SPI_PERIPH_3, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SPI3->CFG1 );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( SPI_PERIPH_3, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 15u << SPI_CFG1_DSIZE_Pos, SPI3->CFG1 & SPI_CFG1_DSIZE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( SPI_PERIPH_2, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 31u << SPI_CFG1_DSIZE_Pos, SPI2->CFG1 & SPI_CFG1_DSIZE );
}


/**
 * \brief   Data size is written to DSIZE and limited by the CRC size.
 *
 * \details 4-bit, 32-bit and 16-bit frames; invalid size; CRC enabled with 8-bit CRC and
 *          16-bit frame requested; DSIZE field below 4 bits preset.
 *
 * \par Expected results
 * - DSIZE 3 / 31 / 15, getter returns the size.
 * - Invalid size, frame longer than CRC: SPI_REQUEST_ERROR. Getter of DSIZE 2: error.
 */
void Ut_Spi_Set_DataSize_RangeAndCrcLimit( void )
{
    spi_DataSize_t dataSize = SPI_DATA_SIZE_CNT;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_4BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 3u << SPI_CFG1_DSIZE_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_DSIZE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 31u << SPI_CFG1_DSIZE_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_DSIZE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataSize( UT_SPI_BUS, &dataSize ) );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_16BIT, dataSize );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_CNT ) );

    UT_SPI_REG->CFG1 = SPI_CFG1_CRCEN | ( 7u << SPI_CFG1_CRCSIZE_Pos ) | ( UT_SPI_DSIZE_8BIT << SPI_CFG1_DSIZE_Pos );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_8BIT ) );

    UT_SPI_REG->CFG1 = 2u << SPI_CFG1_DSIZE_Pos;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataSize( UT_SPI_BUS, &dataSize ) );
}


/**
 * \brief   Bit order, frame format and direction are written and read back.
 *
 * \par Expected results
 * - LSB first: LSBFRST; TI: SP = 001; directions: COMM = 0 / 1 / 2 / 3.
 * - Getters return the values, invalid values rejected.
 */
void Ut_Spi_Set_BitOrderFrameFormatDirection_RoundTrip( void )
{
    const uint32_t       commLut[ SPI_DIRECTION_CNT ] = { 0u, SPI_CFG2_COMM_0, SPI_CFG2_COMM_1, SPI_CFG2_COMM };
    spi_BitOrder_t       bitOrder    = SPI_BIT_ORDER_CNT;
    spi_FrameFormat_t    frameFormat = SPI_FRAME_FORMAT_CNT;
    spi_Direction_t      direction   = SPI_DIRECTION_CNT;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_LSB_FIRST ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_LSBFRST, UT_SPI_REG->CFG2 & SPI_CFG2_LSBFRST );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BitOrder( UT_SPI_BUS, &bitOrder ) );
    TEST_ASSERT_EQUAL( SPI_BIT_ORDER_LSB_FIRST, bitOrder );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_MSB_FIRST ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BitOrder( UT_SPI_BUS, &bitOrder ) );
    TEST_ASSERT_EQUAL( SPI_BIT_ORDER_MSB_FIRST, bitOrder );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_TI ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_SP_0, UT_SPI_REG->CFG2 & SPI_CFG2_SP );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_FrameFormat( UT_SPI_BUS, &frameFormat ) );
    TEST_ASSERT_EQUAL( SPI_FRAME_FORMAT_TI, frameFormat );

    for( spi_Direction_t dirIdx = SPI_DIRECTION_FULL_DUPLEX; SPI_DIRECTION_CNT > dirIdx; dirIdx++ )
    {
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, dirIdx ) );
        TEST_ASSERT_EQUAL_HEX32( commLut[ dirIdx ], UT_SPI_REG->CFG2 & SPI_CFG2_COMM );
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Direction( UT_SPI_BUS, &direction ) );
        TEST_ASSERT_EQUAL( dirIdx, direction );
    }

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_CNT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_CNT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_CNT ) );
}


/**
 * \brief   Master / slave mode is written and read back, SSI follows the role.
 *
 * \details Slave, then master; software NSS active low.
 *
 * \par Expected results
 * - Slave: MASTER / AFCNTR cleared, SSI = 0; master: MASTER / AFCNTR set, SSI = 1.
 * - Getter returns the mode, invalid mode rejected.
 */
void Ut_Spi_Set_Mode_RoleAndInternalSlaveSelect( void )
{
    spi_Mode_t mode = SPI_MODE_CNT;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG2 & ( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SSI );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Mode( UT_SPI_BUS, &mode ) );
    TEST_ASSERT_EQUAL( SPI_MODE_SLAVE, mode );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR, UT_SPI_REG->CFG2 & ( SPI_CFG2_MASTER | SPI_CFG2_AFCNTR ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SSI, UT_SPI_REG->CR1 & SPI_CR1_SSI );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_Mode( UT_SPI_BUS, &mode ) );
    TEST_ASSERT_EQUAL( SPI_MODE_MASTER, mode );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_CNT ) );
}


/**
 * \brief   NSS configuration is written to CFG2 and SSI follows the polarity.
 *
 * \details Master with hardware NSS active high with pulse; software NSS with pulse; NULL.
 *
 * \par Expected results
 * - SSOE | SSIOP | SSOM, SSM = 0, SSI = 0 (master, active high), getter returns the
 *   configuration.
 * - Pulse with software NSS and NULL: SPI_REQUEST_ERROR, CFG2 not changed.
 */
void Ut_Spi_Set_NssConfig_HardPulseActiveHigh( void )
{
    const spi_NssConfig_t hardConfig = { .Mode = SPI_NSS_MODE_HARD, .Polarity = SPI_NSS_POLARITY_HIGH, .Pulse = SPI_FUNCTION_ACTIVE };
    const spi_NssConfig_t softPulse  = { .Mode = SPI_NSS_MODE_SOFT, .Polarity = SPI_NSS_POLARITY_LOW,  .Pulse = SPI_FUNCTION_ACTIVE };
    const uint32_t        nssMask    = SPI_CFG2_SSM | SPI_CFG2_SSOE | SPI_CFG2_SSIOP | SPI_CFG2_SSOM;
    spi_NssConfig_t       readBack;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_MASTER ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_NssConfig( UT_SPI_BUS, &hardConfig ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_SSOE | SPI_CFG2_SSIOP | SPI_CFG2_SSOM, UT_SPI_REG->CFG2 & nssMask );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SSI );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_NssConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_NSS_MODE_HARD, readBack.Mode );
    TEST_ASSERT_EQUAL( SPI_NSS_POLARITY_HIGH, readBack.Polarity );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_ACTIVE, readBack.Pulse );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, &softPulse ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG2_SSOE | SPI_CFG2_SSIOP | SPI_CFG2_SSOM, UT_SPI_REG->CFG2 & nssMask );
}


/**
 * \brief   Master idle timing is written to MIDI / MSSI and read back.
 *
 * \par Expected results
 * - 15 / 3 cycles: MIDI = 15, MSSI = 3, getter returns the values.
 * - 16 cycles, NULL pointer: SPI_REQUEST_ERROR.
 */
void Ut_Spi_Set_MasterTiming_FieldsAndLimit( void )
{
    spi_IdleCycles_t interData = 0u;
    spi_IdleCycles_t ssIdle    = 0u;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_MasterTiming( UT_SPI_BUS, 15u, 3u ) );
    TEST_ASSERT_EQUAL_HEX32( ( 15u << SPI_CFG2_MIDI_Pos ) | ( 3u << SPI_CFG2_MSSI_Pos ), UT_SPI_REG->CFG2 & ( SPI_CFG2_MIDI | SPI_CFG2_MSSI ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_MasterTiming( UT_SPI_BUS, &interData, &ssIdle ) );
    TEST_ASSERT_EQUAL_UINT8( 15u, interData );
    TEST_ASSERT_EQUAL_UINT8( 3u, ssIdle );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 16u, 0u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 0u, 16u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_MasterTiming( UT_SPI_BUS, NULL, &ssIdle ) );
}


/**
 * \brief   CRC configuration is checked, written and read back.
 *
 * \details 8-bit frames: CRC 16-bit polynomial 0x8005 initialized with ones; polynomial
 *          longer than the CRC; zero polynomial; CRC shorter than the frame; invalid init
 *          value; 32-bit CRC; CRC disabled; NULL.
 *
 * \par Expected results
 * - CRCPOLY = 0x8005, CR1 TCRCINI | RCRCINI, CFG1 CRCSIZE = 15 | CRCEN, getter returns it.
 * - Invalid configurations: SPI_REQUEST_ERROR, configuration not changed.
 * - 32-bit CRC with full polynomial accepted, disable clears CRCEN only.
 */
void Ut_Spi_Set_CrcConfig_CheckedWrittenAndReadBack( void )
{
    spi_CrcConfig_t crcConfig = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_16BIT, .Polynomial = 0x8005u, .InitValue = SPI_CRC_INIT_ALL_ONES };
    spi_CrcConfig_t readBack;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_8BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0x8005u, UT_SPI_REG->CRCPOLY );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_TCRCINI | SPI_CR1_RCRCINI, UT_SPI_REG->CR1 & ( SPI_CR1_TCRCINI | SPI_CR1_RCRCINI ) );
    TEST_ASSERT_EQUAL_HEX32( ( 15u << SPI_CFG1_CRCSIZE_Pos ) | SPI_CFG1_CRCEN, UT_SPI_REG->CFG1 & ( SPI_CFG1_CRCSIZE | SPI_CFG1_CRCEN ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_ACTIVE, readBack.State );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_16BIT, readBack.Size );
    TEST_ASSERT_EQUAL_HEX32( 0x8005u, readBack.Polynomial );
    TEST_ASSERT_EQUAL( SPI_CRC_INIT_ALL_ONES, readBack.InitValue );

    crcConfig.Polynomial = 0x18005u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    crcConfig.Polynomial = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    crcConfig.Polynomial = 0x07u;
    crcConfig.Size       = SPI_DATA_SIZE_4BIT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    crcConfig.Size       = SPI_DATA_SIZE_8BIT;
    crcConfig.InitValue  = SPI_CRC_INIT_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0x8005u, UT_SPI_REG->CRCPOLY );

    crcConfig.Size       = SPI_DATA_SIZE_32BIT;
    crcConfig.Polynomial = 0xFFFFFFFFu;
    crcConfig.InitValue  = SPI_CRC_INIT_ALL_ZERO;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & ( SPI_CR1_TCRCINI | SPI_CR1_RCRCINI ) );

    crcConfig.State = SPI_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 31u << SPI_CFG1_CRCSIZE_Pos, UT_SPI_REG->CFG1 & ( SPI_CFG1_CRCSIZE | SPI_CFG1_CRCEN ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, NULL ) );
}


/**
 * \brief   Spi_Get_CrcValue() reads the CRC registers.
 *
 * \par Expected results
 * - TXCRC / RXCRC values returned, NULL pointers rejected.
 */
void Ut_Spi_Get_CrcValue_ReadsRegisters( void )
{
    spi_CrcValue_t txCrc = 0u;
    spi_CrcValue_t rxCrc = 0u;

    UT_SPI_REG->TXCRC = 0x1234u;
    UT_SPI_REG->RXCRC = 0xABCDu;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( UT_SPI_BUS, &txCrc, &rxCrc ) );
    TEST_ASSERT_EQUAL_HEX32( 0x1234u, txCrc );
    TEST_ASSERT_EQUAL_HEX32( 0xABCDu, rxCrc );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcValue( UT_SPI_BUS, NULL, &rxCrc ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcValue( UT_SPI_BUS, &txCrc, NULL ) );
}


/**
 * \brief   Configuration setters are refused while the peripheral is enabled.
 *
 * \details SPE set, every setter called.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR for all setters, CFG1 / CFG2 not changed.
 */
void Ut_Spi_Setters_PeriphEnabled_ReturnError( void )
{
    const spi_NssConfig_t  nssConfig  = { .Mode = SPI_NSS_MODE_SOFT, .Polarity = SPI_NSS_POLARITY_LOW, .Pulse = SPI_FUNCTION_INACTIVE };
    const spi_CrcConfig_t  crcConfig  = { .State = SPI_FUNCTION_INACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 7u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );

    UT_SPI_REG->CR1 = SPI_CR1_SPE;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Mode( UT_SPI_BUS, SPI_MODE_SLAVE ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BusFreq( UT_SPI_BUS, 1000000u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_ClockMode( UT_SPI_BUS, SPI_CLOCK_MODE_3 ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_BitOrder( UT_SPI_BUS, SPI_BIT_ORDER_LSB_FIRST ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_FrameFormat( UT_SPI_BUS, SPI_FRAME_FORMAT_TI ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_NssConfig( UT_SPI_BUS, &nssConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_MasterTiming( UT_SPI_BUS, 1u, 1u ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 | UT_SPI_REG->CFG2 );
}


/**
 * \brief   Getters and register access reject invalid parameters.
 *
 * \par Expected results
 * - Invalid peripheral or NULL pointer: SPI_REQUEST_ERROR.
 */
void Ut_Spi_Getters_InvalidParams_ReturnError( void )
{
    spi_Mode_t          mode;
    spi_ClockMode_t     clockMode;
    spi_DataSize_t      dataSize;
    spi_BitOrder_t      bitOrder;
    spi_Direction_t     direction;
    spi_FrameFormat_t   frameFormat;
    spi_NssConfig_t     nssConfig;
    spi_CrcConfig_t     crcConfig;
    spi_FlagState_t     periphState;
    spi_FunctionState_t xferState;
    spi_XferErrorId_t   xferError;
    spi_DataConfig_t    dataConfig;
    spi_IrqPrio_t       irqPrio;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_Mode( SPI_PERIPH_CNT, &mode ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_ClockMode( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_ClockMode( SPI_PERIPH_CNT, &clockMode ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataSize( SPI_PERIPH_CNT, &dataSize ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_BitOrder( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_BitOrder( SPI_PERIPH_CNT, &bitOrder ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_Direction( SPI_PERIPH_CNT, &direction ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_FrameFormat( SPI_PERIPH_CNT, &frameFormat ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_NssConfig( SPI_PERIPH_CNT, &nssConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_CrcConfig( UT_SPI_BUS, &crcConfig ) );   /* CRCSIZE 0 */
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_PeriphState( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_PeriphState( SPI_PERIPH_CNT, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferState( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferState( SPI_PERIPH_CNT, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferError( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_XferError( SPI_PERIPH_CNT, &xferError ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &dataConfig ) );   /* Not initialized */
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_IrqPriority( UT_SPI_BUS, NULL ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_IrqPriority( SPI_PERIPH_CNT, &irqPrio ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_IrqPriority( SPI_PERIPH_CNT, UT_SPI_PRIO ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStop( SPI_PERIPH_CNT ) );
}


/**
 * \brief   Spi_Get_PeriphState() reports the SPE bit.
 *
 * \par Expected results
 * - SPE = 0: inactive, SPE = 1: active.
 */
void Ut_Spi_Get_PeriphState_EnableBit( void )
{
    spi_FlagState_t periphState = SPI_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_PeriphState( UT_SPI_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_FLAG_INACTIVE, periphState );

    UT_SPI_REG->CR1 = SPI_CR1_SPE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_PeriphState( UT_SPI_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_FLAG_ACTIVE, periphState );
}


/**
 * \brief   Interrupt priority is passed to NVIC and read from it.
 *
 * \par Expected results
 * - Nvic_Set_PeriphIrq_Prio / Nvic_Get_PeriphIrq_Prio of SPI1 line, NVIC failures reported.
 */
void Ut_Spi_IrqPriority_PassedToNvic( void )
{
    nvic_IrqPrio_t nvicPrio = 9u;
    spi_IrqPrio_t  irqPrio  = 0u;

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, UT_SPI_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_IrqPriority( UT_SPI_BUS, UT_SPI_PRIO ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &nvicPrio );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_IrqPriority( UT_SPI_BUS, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( 9u, irqPrio );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, UT_SPI_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_IrqPriority( UT_SPI_BUS, UT_SPI_PRIO ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, NULL, NVIC_REQUEST_ERROR );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_IrqPriority( UT_SPI_BUS, &irqPrio ) );
}

/* ========================== DATA HANDLING CONFIG ========================== */

/**
 * \brief   ISR data handling registers and enables the SPI interrupt.
 *
 * \par Expected results
 * - Data configuration not available before the initialization.
 * - NVIC handler registered, priority and enable of SPI1 line, IER cleared.
 * - Spi_Get_DataConfig() returns the configuration.
 */
void Ut_Spi_Set_DataConfig_Isr_InterruptRegistered( void )
{
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    spi_DataConfig_t       readBack;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    UT_SPI_REG->IER = UT_SPI_IER_ALL;
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( UT_SPI_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Handler_AddCallback( Ut_Spi_NvicHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_SPI_NVIC, UT_SPI_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_SPI_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_ISR, readBack.XferMode );
    TEST_ASSERT_EQUAL_PTR( Ut_Spi_XferCompleteCallback, readBack.XferCompleteCallback );
    TEST_ASSERT_EQUAL_PTR( Ut_Spi_ErrorCallback, readBack.ErrorCallback );
}


/**
 * \brief   Invalid data handling configuration is rejected without access.
 *
 * \details NULL, invalid peripheral, invalid mode, DMA: TX and RX on the same channel,
 *          invalid DMA peripheral / channel / priority.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, no GPDMA / NVIC call (strict mocks).
 */
void Ut_Spi_Set_DataConfig_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    spi_DataConfig_t dataConfig;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, NULL ) );
    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( SPI_PERIPH_CNT, &dataConfig ) );
    dataConfig.XferMode = SPI_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.RxDmaChannelId = dataConfig.TxDmaChannelId;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.TxDmaPeriphId = SPI_DMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.RxDmaChannelId = SPI_DMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    dataConfig.TxDmaPriority = (spi_DmaPriority_t)GPDMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
}


/**
 * \brief   DMA data handling initializes both GPDMA channels.
 *
 * \details GPDMA1 channel 0 (TX, low priority) and channel 1 (RX, high priority).
 *
 * \par Expected results
 * - Gpdma_Init() 2x: TX memory to peripheral, request SPI1_TX, destination TXDR static,
 *   source increment; RX peripheral to memory, request SPI1_RX, source RXDR static,
 *   destination increment; 8-bit data, single request, one transfer, error mask of all
 *   GPDMA errors, error callback, no complete / half callback, priorities of the config.
 * - GPDMA interrupt of both channels enabled, both channels stopped, TXDMAEN / RXDMAEN
 *   cleared, IER cleared; SPI interrupt registered.
 */
void Ut_Spi_Set_DataConfig_Dma_ChannelsInitialized( void )
{
    UT_SPI_REG->CFG1 = SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );

    /* -1- Transmit channel */
    TEST_ASSERT_EQUAL( UT_SPI_DMA,                  utSpi_DmaConfig[ 0u ].PeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_TX_CHANNEL,       utSpi_DmaConfig[ 0u ].ChannelId );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_LOW,          utSpi_DmaConfig[ 0u ].ChannelPrio );
    TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH,  utSpi_DmaXferConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( GPDMA_REQ_SPI1_TX,           utSpi_DmaXferConfig[ 0u ].RequestSource );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_SPI_REG->TXDR, utSpi_DmaXferConfig[ 0u ].DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC,           utSpi_DmaXferConfig[ 0u ].DestinationAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT,        utSpi_DmaXferConfig[ 0u ].SourceAddrMode );

    /* -2- Receive channel */
    TEST_ASSERT_EQUAL( UT_SPI_DMA,                  utSpi_DmaConfig[ 1u ].PeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_RX_CHANNEL,       utSpi_DmaConfig[ 1u ].ChannelId );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_HIGH,         utSpi_DmaConfig[ 1u ].ChannelPrio );
    TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY,  utSpi_DmaXferConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( GPDMA_REQ_SPI1_RX,           utSpi_DmaXferConfig[ 1u ].RequestSource );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_SPI_REG->RXDR, utSpi_DmaXferConfig[ 1u ].SourceAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC,           utSpi_DmaXferConfig[ 1u ].SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT,        utSpi_DmaXferConfig[ 1u ].DestinationAddrMode );

    for( uint32_t cfgIdx = 0u; 2u > cfgIdx; cfgIdx++ )
    {
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS,     utSpi_DmaXferConfig[ cfgIdx ].SourceDataSize );
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS,     utSpi_DmaXferConfig[ cfgIdx ].DestinationDataSize );
        TEST_ASSERT_EQUAL( GPDMA_PERIPH_REQ_SINGLE,   utSpi_DmaXferConfig[ cfgIdx ].RequestMode );
        TEST_ASSERT_EQUAL_UINT32( 1u,                 utSpi_DmaConfig[ cfgIdx ].TransfersCount );
        TEST_ASSERT_EQUAL_HEX32( GPDMA_ERROR_TRANSFER | GPDMA_ERROR_CONFIG_UPDATE | GPDMA_ERROR_CONFIG_ERROR | GPDMA_ERROR_TRIG_OVERRUN,
                                 utSpi_DmaConfig[ cfgIdx ].ErrorMask );
        TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ cfgIdx ].ErrorIsr );
        TEST_ASSERT_NULL( utSpi_DmaConfig[ cfgIdx ].TransferCompleteIsr );
        TEST_ASSERT_NULL( utSpi_DmaConfig[ cfgIdx ].HalfTransferIsr );
        TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ cfgIdx ].XferList );
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );
}


/**
 * \brief   Repeated DMA configuration with the same channels only updates the priority.
 *
 * \details DMA data handling configured, then configured again with other priorities.
 *
 * \par Expected results
 * - Gpdma_Init() not called again, Gpdma_Set_Priority() of both channels with new
 *   priorities, channel interrupts enabled again.
 */
void Ut_Spi_Set_DataConfig_DmaSameChannels_PriorityUpdated( void )
{
    spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    dataConfig.TxDmaPriority = SPI_DMA_PRIORITY_VERYHIGH;
    dataConfig.RxDmaPriority = SPI_DMA_PRIORITY_MEDIUM;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->PrioCnt );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_VERYHIGH, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->Prio );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_MEDIUM, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->Prio );
    TEST_ASSERT_EQUAL_UINT32( 2u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOnCnt );
}


/**
 * \brief   GPDMA initialization failure is reported.
 *
 * \details Gpdma_Init() returns error.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, data handling not initialized.
 */
void Ut_Spi_Set_DataConfig_DmaInitFailure_ReturnsError( void )
{
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( SPI_XFER_MODE_NONE );
    utSpi_DmaInitState = GPDMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
}


/**
 * \brief   Spi_Set_DataConfig() releases the previous data handling.
 *
 * \details DMA mode reconfigured to POLL, then to NONE mode.
 *
 * \par Expected results
 * - DMA -> POLL: both GPDMA channels stopped and their interrupts disabled, SPI interrupt
 *   disabled in NVIC, mode POLL read back.
 * - POLL -> NONE: SPI_REQUEST_OK, mode NONE read back.
 */
void Ut_Spi_Set_DataConfig_Reconfigured_PreviousModeReleased( void )
{
    const spi_DataConfig_t pollConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const spi_DataConfig_t noneConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_NONE );
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &pollConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_POLL, readBack.XferMode );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &noneConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_XFER_MODE_NONE, readBack.XferMode );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_NvicOffCnt );
}


/**
 * \brief   Spi_Deinit() releases DMA data handling and reports release failures.
 *
 * \details DMA mode, Spi_Deinit() twice; DMA mode with failing Gpdma_Set_InterruptInactive();
 *          ISR mode with failing Nvic_Set_PeriphIrq_Inactive().
 *
 * \par Expected results
 * - SPI_REQUEST_OK, channel interrupts disabled (1x each), NVIC line disabled, data
 *   handling not initialized; second Spi_Deinit() does not touch GPDMA / NVIC again.
 * - Release failures: SPI_REQUEST_ERROR, data handling not initialized.
 */
void Ut_Spi_Deinit_DataHandlingReleasedAndFailuresReported( void )
{
    spi_DataConfig_t readBack;

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicOffCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_NvicOffCnt );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    utSpi_DmaIrqOffState = GPDMA_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    utSpi_DmaIrqOffState = GPDMA_REQUEST_OK;
    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );
    utSpi_NvicOffState = NVIC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
}

/* ============================ TRANSFER START ============================== */

/**
 * \brief   Spi_Set_XferStart() rejects invalid requests.
 *
 * \details POLL data handling; NULL request, invalid peripheral, size 0, size above the
 *          maximum, no buffer; simplex TX without transmit buffer; half duplex with both
 *          buffers.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, transfer not started (SPE = 0, CR2 = 0).
 */
void Ut_Spi_Set_XferStart_InvalidRequest_ReturnsError( void )
{
    uint8_t           rxBuf[ 2u ] = { 0u };
    const uint8_t     txBuf[ 2u ] = { 0u };
    spi_XferRequest_t request;

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, NULL ) );
    request = (spi_XferRequest_t){ .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( SPI_PERIPH_CNT, &request ) );
    request.XferSize = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    request.XferSize = 0xFFFFu;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    request = (spi_XferRequest_t){ .TxData = NULL, .RxData = NULL, .XferSize = 2u };
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_TX ) );
    request = (spi_XferRequest_t){ .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );
    request = (spi_XferRequest_t){ .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR2 );
}


/**
 * \brief   Transfer is not started without usable data handling.
 *
 * \details No data handling; NONE mode; POLL mode with enabled peripheral.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR in all cases, transfer state INACTIVE.
 */
void Ut_Spi_Set_XferStart_NotReady_ReturnsError( void )
{
    const uint8_t           txBuf[ 1u ] = { 0x55u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 1u };
    const spi_DataConfig_t  pollConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    spi_FunctionState_t     xferState   = SPI_FUNCTION_ACTIVE;

    Ut_Spi_Init_Master( SPI_XFER_MODE_CNT );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Init_Master( SPI_XFER_MODE_NONE );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &pollConfig ) );
    UT_SPI_REG->CR1 |= SPI_CR1_SPE;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
}

/* =========================== POLLING TRANSFERS ============================ */

/**
 * \brief   Polling full-duplex transfer moves the frames and completes at EOT.
 *
 * \details 2 frames of 8 bits: start, task with TXP, task with RXP (RXDR = 0xAB), task with
 *          EOT. Second start while the transfer runs.
 *
 * \par Expected results
 * - Start: IFCR all flags, TSIZE = 2, SPE and CSTART set, transfer ACTIVE; second start
 *   refused.
 * - TXP: both frames written to the FIFO (last TXDR byte 0x34).
 * - RXP: both frames stored (0xAB); EOT: complete callback, SPE cleared, IFCR written.
 */
void Ut_Spi_Poll_FullDuplex_FramesMovedAndCompleted( void )
{
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };
    spi_FunctionState_t     xferState   = SPI_FUNCTION_INACTIVE;

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IFCR_ALL, UT_SPI_REG->IFCR );
    TEST_ASSERT_EQUAL_HEX32( 2u, UT_SPI_REG->CR2 & SPI_CR2_TSIZE );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SPE | SPI_CR1_CSTART, UT_SPI_REG->CR1 & ( SPI_CR1_SPE | SPI_CR1_CSTART ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_ACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Task( SPI_SR_TXP );
    TEST_ASSERT_EQUAL_HEX8( 0x34u, (uint8_t)UT_SPI_REG->TXDR );

    UT_SPI_REG->RXDR = 0xABu;
    Ut_Spi_Task( SPI_SR_RXP );
    TEST_ASSERT_EQUAL_HEX8( 0xABu, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xABu, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt );

    UT_SPI_REG->IFCR = 0u;
    Ut_Spi_Task( SPI_SR_EOT );
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IFCR_ALL, UT_SPI_REG->IFCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   Frames of 9 - 16 and 17 - 32 bits use little endian buffers.
 *
 * \details 16-bit frame {0x12, 0x34} with RXDR 0xCDAB; 32-bit frame {0x01 .. 0x04} with
 *          RXDR 0x88776655.
 *
 * \par Expected results
 * - TXDR 0x3412, received {0xAB, 0xCD}; TXDR 0x04030201, received {0x55 .. 0x88}.
 */
void Ut_Spi_Poll_SixteenAndThirtyTwoBit_LittleEndianBuffers( void )
{
    const uint8_t     txBuf[ 4u ] = { 0x01u, 0x02u, 0x03u, 0x04u };
    const uint8_t     tx16[ 2u ]  = { 0x12u, 0x34u };
    uint8_t           rxBuf[ 4u ] = { 0u };
    spi_XferRequest_t request     = { .TxData = tx16, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    Ut_Spi_Task( SPI_SR_TXP );
    TEST_ASSERT_EQUAL_HEX16( 0x3412u, (uint16_t)UT_SPI_REG->TXDR );
    UT_SPI_REG->RXDR = 0xCDABu;
    Ut_Spi_Task( SPI_SR_RXP | SPI_SR_EOT );
    TEST_ASSERT_EQUAL_HEX8( 0xABu, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xCDu, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_32BIT ) );
    request.TxData = txBuf;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    Ut_Spi_Task( SPI_SR_TXP );
    TEST_ASSERT_EQUAL_HEX32( 0x04030201u, UT_SPI_REG->TXDR );
    UT_SPI_REG->RXDR = 0x88776655u;
    Ut_Spi_Task( SPI_SR_RXP | SPI_SR_EOT );
    TEST_ASSERT_EQUAL_HEX8( 0x55u, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x66u, rxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x77u, rxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x88u, rxBuf[ 3u ] );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_CompleteCnt );
}


/**
 * \brief   End of transfer before all frames were moved is reported as incomplete.
 *
 * \details 2 frames transmitted, EOT without received frames.
 *
 * \par Expected results
 * - Error callback SPI_XFER_ERROR_INCOMPLETE, transfer INACTIVE.
 */
void Ut_Spi_Poll_EotBeforeAllFrames_IncompleteError( void )
{
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Task( SPI_SR_TXP );
    Ut_Spi_Task( SPI_SR_EOT );

    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_INCOMPLETE );
}


/**
 * \brief   Error flags terminate the transfer and are reported by priority.
 *
 * \details Full-duplex master transfers ended by MODF together with OVR, OVR, UDR, TIFRE
 *          (SUSP preset - abort of the master transfer completes immediately).
 *
 * \par Expected results
 * - MODE_FAULT (has priority), OVERRUN, UNDERRUN, FRAME reported once each, CSUSP set
 *   (master transfer suspended), SPE cleared.
 */
void Ut_Spi_Poll_ErrorFlags_ReportedByPriority( void )
{
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };
    const struct
    {
        uint32_t            SrFlags;
        spi_XferErrorId_t   ErrorId;
    }   errorLut[] =
    {
        { SPI_SR_MODF | SPI_SR_OVR, SPI_XFER_ERROR_MODE_FAULT },
        { SPI_SR_OVR,               SPI_XFER_ERROR_OVERRUN    },
        { SPI_SR_UDR,               SPI_XFER_ERROR_UNDERRUN   },
        { SPI_SR_TIFRE,             SPI_XFER_ERROR_FRAME      },
    };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );

    for( uint32_t idx = 0u; ( sizeof( errorLut ) / sizeof( errorLut[ 0u ] ) ) > idx; idx++ )
    {
        utSpi_ErrorCnt   = 0u;
        UT_SPI_REG->CR1 &= ~SPI_CR1_CSUSP;

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
        Ut_Spi_Task( errorLut[ idx ].SrFlags | SPI_SR_SUSP );

        Ut_Spi_Check_XferEnd( errorLut[ idx ].ErrorId );
        TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CSUSP, UT_SPI_REG->CR1 & SPI_CR1_CSUSP );
        TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
    }
}


/**
 * \brief   CRC error is reported at the end of the transfer.
 *
 * \details 8-bit CRC enabled, 1 frame moved, EOT with CRCE.
 *
 * \par Expected results
 * - Error callback SPI_XFER_ERROR_CRC.
 */
void Ut_Spi_Poll_CrcError_ReportedAtEot( void )
{
    const spi_CrcConfig_t   crcConfig   = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = 0x07u, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    const uint8_t           txBuf[ 1u ] = { 0x12u };
    uint8_t                 rxBuf[ 1u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( UT_SPI_BUS, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Task( SPI_SR_TXP );
    Ut_Spi_Task( SPI_SR_RXP | SPI_SR_EOT | SPI_SR_CRCE );

    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_CRC );
}


/**
 * \brief   Master simplex reception does not write the transmit FIFO.
 *
 * \details Simplex RX, request with both buffers (transmit buffer ignored), 1 frame, task
 *          with TXP | RXP | EOT.
 *
 * \par Expected results
 * - TXDR not written, frame received, transfer completed.
 */
void Ut_Spi_Poll_SimplexRx_TransmitFifoNotWritten( void )
{
    const uint8_t           txBuf[ 1u ] = { 0x12u };
    uint8_t                 rxBuf[ 1u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 1u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->RXDR = 0x5Au;
    Ut_Spi_Task( SPI_SR_TXP | SPI_SR_RXP | SPI_SR_EOT );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->TXDR );
    TEST_ASSERT_EQUAL_HEX8( 0x5Au, rxBuf[ 0u ] );
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_NONE );
}


/**
 * \brief   Half-duplex direction follows the buffer of the request.
 *
 * \details Half duplex: transmission request, then reception request.
 *
 * \par Expected results
 * - Transmission: HDDIR = 1, frame written; reception: HDDIR = 0, frame received.
 */
void Ut_Spi_Poll_HalfDuplex_DirectionFromRequest( void )
{
    const uint8_t     txBuf[ 1u ] = { 0x12u };
    uint8_t           rxBuf[ 1u ] = { 0u };
    spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 1u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( UT_SPI_BUS, SPI_DIRECTION_HALF_DUPLEX ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_HDDIR, UT_SPI_REG->CR1 & SPI_CR1_HDDIR );
    Ut_Spi_Task( SPI_SR_TXP | SPI_SR_EOT );
    TEST_ASSERT_EQUAL_HEX8( 0x12u, (uint8_t)UT_SPI_REG->TXDR );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );

    request = (spi_XferRequest_t){ .TxData = NULL, .RxData = rxBuf, .XferSize = 1u };
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_HDDIR );
    UT_SPI_REG->RXDR = 0x77u;
    Ut_Spi_Task( SPI_SR_RXP | SPI_SR_EOT );
    TEST_ASSERT_EQUAL_HEX8( 0x77u, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_CompleteCnt );
}


/**
 * \brief   Slave transfer is not started by CSTART.
 *
 * \details Slave (software NSS), simplex transmission of 1 frame.
 *
 * \par Expected results
 * - SPE set, CSTART not set.
 */
void Ut_Spi_Poll_Slave_NoMasterStart( void )
{
    spi_Config_t            config      = Ut_Spi_Get_Config();
    const spi_DataConfig_t  dataConfig  = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    const uint8_t           txBuf[ 1u ] = { 0x12u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 1u };

    config.Mode       = SPI_MODE_SLAVE;
    config.Direction  = SPI_DIRECTION_SIMPLEX_TX;
    config.DataConfig = &dataConfig;
    Ut_Spi_Init( &config );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SPE, UT_SPI_REG->CR1 & ( SPI_CR1_SPE | SPI_CR1_CSTART ) );
}


/**
 * \brief   Spi_Set_XferStop() aborts the running transfer without callback.
 *
 * \details Master transfer running (CSTART), SUSP preset, Spi_Set_XferStop() twice.
 *
 * \par Expected results
 * - CSUSP set, SPE cleared, IFCR all flags, no callback, transfer INACTIVE, error NONE.
 * - Second stop: SPI_REQUEST_OK (no transfer).
 */
void Ut_Spi_Set_XferStop_RunningTransferAborted( void )
{
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 2u };
    spi_FunctionState_t     xferState   = SPI_FUNCTION_ACTIVE;
    spi_XferErrorId_t       xferError   = SPI_XFER_ERROR_CNT;

    Ut_Spi_Init_Master( SPI_XFER_MODE_POLL );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    UT_SPI_REG->SR   = SPI_SR_SUSP;
    UT_SPI_REG->IFCR = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CSUSP, UT_SPI_REG->CR1 & ( SPI_CR1_CSUSP | SPI_CR1_SPE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IFCR_ALL, UT_SPI_REG->IFCR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt + utSpi_ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferError( UT_SPI_BUS, &xferError ) );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_NONE, xferError );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );
}


/**
 * \brief   Spi_Set_XferStop() of a DMA transfer disables the peripheral before DMA is stopped.
 *
 * \details Master DMA transfer running (CSTART), SUSP preset, Spi_Set_XferStop().
 * \note    Bug AB#1161: GPDMA channels and DMA requests (CFG1, write protected while
 *          SPE = 1 on STM32H5) were stopped before SPE was cleared - abort failed on target.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, GPDMA channels stopped with SPE = 0, DMA requests and SPE cleared.
 */
void Ut_Spi_Set_XferStop_DmaTransfer_DisabledBeforeDmaStop( void )
{
    uint8_t * const           txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const           rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t   request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 3u };
    const ut_SpiDmaChannel_t *txChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL );
    const ut_SpiDmaChannel_t *rxChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_SPE, UT_SPI_REG->CR1 & SPI_CR1_SPE );

    UT_SPI_REG->SR = SPI_SR_SUSP;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, txChan->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxChan->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, txChan->SpeAtInactive );
    TEST_ASSERT_EQUAL_HEX32( 0u, rxChan->SpeAtInactive );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}

/* ========================== INTERRUPT TRANSFERS =========================== */

/**
 * \brief   ISR full-duplex transfer is moved by the SPI interrupt.
 *
 * \details 2 frames: start, ISR with TXP, ISR with RXP (RXDR = 0xCD), ISR with EOT.
 *
 * \par Expected results
 * - Start: IER = events | TXPIE | RXPIE.
 * - TXP: both frames written, TXPIE cleared (no further frame).
 * - RXP: frames stored; EOT: complete callback, IER cleared, SPE cleared.
 */
void Ut_Spi_Isr_FullDuplex_InterruptDrivesTransfer( void )
{
    const uint8_t           txBuf[ 2u ] = { 0x12u, 0x34u };
    uint8_t                 rxBuf[ 2u ] = { 0u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IER_ALL, UT_SPI_REG->IER );

    Ut_Spi_Call_Isr( SPI_SR_TXP );
    TEST_ASSERT_EQUAL_HEX8( 0x34u, (uint8_t)UT_SPI_REG->TXDR );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IER_EVENTS | SPI_IER_RXPIE, UT_SPI_REG->IER );

    UT_SPI_REG->RXDR = 0xCDu;
    Ut_Spi_Call_Isr( SPI_SR_RXP );
    TEST_ASSERT_EQUAL_HEX8( 0xCDu, rxBuf[ 1u ] );

    Ut_Spi_Call_Isr( SPI_SR_EOT );
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   Transmit interrupt is kept only while the FIFO limit allows a frame.
 *
 * \details 20 frames of 8 bits (receive FIFO limit 16 frames): ISR with TXP, then ISR with
 *          RXP.
 *
 * \par Expected results
 * - TXP: 16 frames written (last 15), TXPIE cleared (16 frames in flight).
 * - RXP: 16 frames read, TXPIE enabled again (4 frames pending).
 */
void Ut_Spi_Isr_FifoLimit_TransmitInterruptFollowsPendingFrames( void )
{
    uint8_t                 txBuf[ 20u ];
    uint8_t                 rxBuf[ 20u ] = { 0u };
    const spi_XferRequest_t request      = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 20u };

    for( uint32_t idx = 0u; 20u > idx; idx++ )
    {
        txBuf[ idx ] = (uint8_t)idx;
    }

    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Call_Isr( SPI_SR_TXP );
    TEST_ASSERT_EQUAL_HEX8( 15u, (uint8_t)UT_SPI_REG->TXDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER & SPI_IER_TXPIE );

    UT_SPI_REG->RXDR = 0x99u;
    Ut_Spi_Call_Isr( SPI_SR_RXP );
    TEST_ASSERT_EQUAL_HEX8( 0x99u, rxBuf[ 15u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x00u, rxBuf[ 16u ] );
    TEST_ASSERT_EQUAL_HEX32( SPI_IER_TXPIE, UT_SPI_REG->IER & SPI_IER_TXPIE );
}


/**
 * \brief   SPI interrupt without running transfer is ignored.
 *
 * \details ISR mode, no transfer, ISR with OVR | EOT.
 *
 * \par Expected results
 * - No callback.
 */
void Ut_Spi_Isr_NoTransfer_FlagsIgnored( void )
{
    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );

    Ut_Spi_Call_Isr( SPI_SR_OVR | SPI_SR_EOT );

    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_CompleteCnt + utSpi_ErrorCnt );
}


/**
 * \brief   Spi_Task() does not move frames of interrupt transfers.
 *
 * \details ISR transfer running, Spi_Task() with TXP.
 *
 * \par Expected results
 * - TXDR not written.
 */
void Ut_Spi_Task_IsrTransfer_NotProcessed( void )
{
    const uint8_t           txBuf[ 1u ] = { 0x12u };
    const spi_XferRequest_t request     = { .TxData = txBuf, .RxData = NULL, .XferSize = 1u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    Ut_Spi_Task( SPI_SR_TXP );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->TXDR );
}

/* ============================= DMA TRANSFERS ============================== */

/**
 * \brief   DMA full-duplex transfer arms both channels and completes at EOT.
 *
 * \details 3 frames of 8 bits, buffers in SRAM: start, SPI interrupt with EOT.
 *
 * \par Expected results
 * - RX channel: 8-bit sizes, block 3, destination = receive buffer (increment), active;
 *   TX channel: source = transmit buffer (increment), active; RXDMAEN | TXDMAEN; IER =
 *   events; CSTART.
 * - EOT: remaining 0 -> complete callback; both channels stopped, DMA requests and IER
 *   cleared, SPE cleared.
 */
void Ut_Spi_Dma_FullDuplex_ChannelsArmedAndEotCompletes( void )
{
    uint8_t * const           txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const           rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t   request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 3u };
    const ut_SpiDmaChannel_t *txChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL );
    const ut_SpiDmaChannel_t *rxChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS, rxChan->SrcSize );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS, rxChan->DstSize );
    TEST_ASSERT_EQUAL_UINT16( 3u, rxChan->BlockSize );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, rxChan->DstAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, rxChan->DstMode );
    TEST_ASSERT_EQUAL_UINT32( 1u, rxChan->ActiveCnt );
    TEST_ASSERT_EQUAL_UINT16( 3u, txChan->BlockSize );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_TX_BUF_OFFSET, txChan->SrcAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, txChan->SrcMode );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChan->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IER_EVENTS, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CSTART, UT_SPI_REG->CR1 & SPI_CR1_CSTART );

    Ut_Spi_Call_Isr( SPI_SR_EOT );

    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_UINT32( 2u, txChan->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxChan->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, txChan->SpeAtInactive );
    TEST_ASSERT_EQUAL_HEX32( 0u, rxChan->SpeAtInactive );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   DMA full-duplex reception without transmit buffer transmits a dummy frame.
 *
 * \details 16-bit frames, 2 frames, only receive buffer.
 *
 * \par Expected results
 * - TX channel: 16-bit sizes, block 4 bytes, static source address (dummy, not 0).
 * - RX channel: 16-bit sizes, destination = receive buffer.
 */
void Ut_Spi_Dma_ReceptionOnly_DummyTransmitSource( void )
{
    uint8_t * const           rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t   request = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };
    const ut_SpiDmaChannel_t *txChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL );
    const ut_SpiDmaChannel_t *rxChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, txChan->SrcSize );
    TEST_ASSERT_EQUAL_UINT16( 4u, txChan->BlockSize );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, txChan->SrcMode );
    TEST_ASSERT_NOT_EQUAL( 0u, txChan->SrcAddr );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, rxChan->DstSize );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, rxChan->DstAddr );
}


/**
 * \brief   DMA transfer with misaligned buffer of 16-bit frames is refused.
 *
 * \details 16-bit frames, transmit buffer at odd address.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, no channel activated, transfer INACTIVE, SPE = 0.
 */
void Ut_Spi_Dma_MisalignedBuffer_ReturnsError( void )
{
    uint8_t * const         txBuf     = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET + 1u );
    const spi_XferRequest_t request   = { .TxData = txBuf, .RxData = NULL, .XferSize = 2u };
    spi_FunctionState_t     xferState = SPI_FUNCTION_ACTIVE;

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->ActiveCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
}


/**
 * \brief   End of transfer with data left in a DMA channel is reported as incomplete.
 *
 * \details Full-duplex DMA transfer, GPDMA reports 1 remaining byte at EOT.
 *
 * \par Expected results
 * - Error callback SPI_XFER_ERROR_INCOMPLETE.
 */
void Ut_Spi_Dma_RemainingData_IncompleteError( void )
{
    uint8_t * const         txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const         rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    utSpi_DmaRemaining = 1u;
    Ut_Spi_Call_Isr( SPI_SR_EOT );

    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_INCOMPLETE );
}


/**
 * \brief   GPDMA error aborts the transfer and is reported once.
 *
 * \details Full-duplex DMA transfer: error callback of the RX channel with transfer error;
 *          new transfer: error callback of the TX channel with configuration error and
 *          trigger overrun (one GPDMA error event with two flags).
 *
 * \par Expected results
 * - Transfer error: one error callback SPI_XFER_ERROR_DMA_TRANSFER, transfer INACTIVE.
 * - Two flags of one event: one error callback (SPI_XFER_ERROR_DMA_CONFIG - first error of
 *   the priority list), transfer INACTIVE.
 */
void Ut_Spi_Dma_GpdmaError_OneErrorCallbackPerTransfer( void )
{
    uint8_t * const         txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const         rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 1u ].ErrorIsr );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    UT_SPI_REG->SR = SPI_SR_SUSP;
    utSpi_DmaConfig[ 1u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_DMA_TRANSFER );

    utSpi_ErrorCnt = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    utSpi_DmaConfig[ 0u ].ErrorIsr( (gpdma_ErrorMaskId_t)( GPDMA_ERROR_CONFIG_ERROR | GPDMA_ERROR_TRIG_OVERRUN ) );
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_DMA_CONFIG );
}

/* =========================== OTHER PERIPHERALS ============================ */

/**
 * \brief   Other SPI peripherals use their own interrupt and GPDMA error callbacks.
 *
 * \details Every SPI peripheral of the MCU except SPI1 in DMA mode (GPDMA1 channels 2 / 3):
 *          captured SPI interrupt without transfer, reception started, error callbacks of
 *          the RX and TX channel called. MCUs with SPI1 only: test ignored.
 *
 * \par Expected results
 * - GPDMA requests of the peripheral (TX / RX).
 * - Interrupt without transfer: no callback; GPDMA error of the running transfer: error
 *   callback SPI_XFER_ERROR_DMA_TRANSFER.
 * - Spi_Deinit(): SPI_REQUEST_OK.
 */
void Ut_Spi_OtherPeriph_OwnInterruptAndDmaCallbacks( void )
{
#if defined(SPI2) || \
    defined(SPI3) || \
    defined(SPI4) || \
    defined(SPI5) || \
    defined(SPI6)
    const struct
    {
        spi_PeriphId_t          PeriphId;
        SPI_TypeDef *           PeriphReg;
        spi_ClkSrc_t            ClkSrc;
        gpdma_PeriphReqId_t     TxReq;
        gpdma_PeriphReqId_t     RxReq;
    }   periphLut[] =
    {
#ifdef SPI2
        { SPI_PERIPH_2, SPI2, SPI_CLK_SRC_PCLK,  GPDMA_REQ_SPI2_TX, GPDMA_REQ_SPI2_RX },
#endif /* SPI2 */
#ifdef SPI3
        { SPI_PERIPH_3, SPI3, SPI_CLK_SRC_PCLK,  GPDMA_REQ_SPI3_TX, GPDMA_REQ_SPI3_RX },
#endif /* SPI3 */
#ifdef SPI4
        { SPI_PERIPH_4, SPI4, SPI_CLK_SRC_PCLK,  GPDMA_REQ_SPI4_TX, GPDMA_REQ_SPI4_RX },
#endif /* SPI4 */
#ifdef SPI5
        { SPI_PERIPH_5, SPI5, SPI_CLK_SRC_PCLK,  GPDMA_REQ_SPI5_TX, GPDMA_REQ_SPI5_RX },
#endif /* SPI5 */
#ifdef SPI6
        { SPI_PERIPH_6, SPI6, SPI_CLK_SRC_PCLK,  GPDMA_REQ_SPI6_TX, GPDMA_REQ_SPI6_RX },
#endif /* SPI6 */
    };
    uint8_t * const         rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
        spi_Config_t     config     = Ut_Spi_Get_Config();

        dataConfig.TxDmaChannelId = SPI_DMA_CHANNEL_2;
        dataConfig.RxDmaChannelId = SPI_DMA_CHANNEL_3;
        config.PeriphId           = periphLut[ idx ].PeriphId;
        config.ClkSrc             = periphLut[ idx ].ClkSrc;
        config.DataConfig         = &dataConfig;

        utSpi_Isr         = NULL;
        utSpi_DmaInitCnt  = 0u;
        utSpi_ErrorCnt    = 0u;
        utSpi_CompleteCnt = 0u;
        Ut_Spi_Init( &config );

        TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );
        TEST_ASSERT_EQUAL( periphLut[ idx ].TxReq, utSpi_DmaXferConfig[ 0u ].RequestSource );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RxReq, utSpi_DmaXferConfig[ 1u ].RequestSource );
        TEST_ASSERT_NOT_NULL( utSpi_Isr );

        periphLut[ idx ].PeriphReg->SR = 0u;
        utSpi_Isr();
        TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt + utSpi_CompleteCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphLut[ idx ].PeriphId, &request ) );
        periphLut[ idx ].PeriphReg->SR = SPI_SR_SUSP;
        utSpi_DmaConfig[ 1u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
        TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
        TEST_ASSERT_EQUAL( SPI_XFER_ERROR_DMA_TRANSFER, utSpi_LastError );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphLut[ idx ].PeriphId, &request ) );
        utSpi_DmaConfig[ 0u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
        TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_ErrorCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( periphLut[ idx ].PeriphId ) );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU with SPI1 only" );
#endif /* SPI2 OR SPI3 OR SPI4 OR SPI5 OR SPI6 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief HW model of the SPI mode fault (runs in background thread): master with active
 *        internal slave select gets SR.MODF, MASTER is cleared by HW.
 *
 * Internal slave select: software NSS - SSI at the active level (SSIOP), hardware NSS
 * input - active (NSS pin is not driven by the test), hardware NSS output - inactive.
 * CFG2 is read before CR1 - the module writes SSI before MASTER.
 */
static void Ut_Spi_HwModel( void )
{
    const uint32_t cfg2     = UT_SPI_REG->CFG2;
    const uint32_t ssi      = UT_SPI_REG->CR1 & SPI_CR1_SSI;
    const uint32_t activeHi = cfg2 & SPI_CFG2_SSIOP;
    uint32_t       ssActive = 0u;

    if( 0u != ( cfg2 & SPI_CFG2_SSM ) )
    {
        if( ( 0u != ssi      ) &&
            ( 0u != activeHi )    )
        {
            ssActive = 1u;
        }
        else if( ( 0u == ssi      ) &&
                 ( 0u == activeHi )    )
        {
            ssActive = 1u;
        }
        else
        {
            /* Software NSS at the inactive level */
            ssActive = 0u;
        }
    }
    else if( 0u == ( cfg2 & SPI_CFG2_SSOE ) )
    {
        /* Hardware NSS input - slave select is active */
        ssActive = 1u;
    }
    else
    {
        /* Hardware NSS output - master drives NSS */
        ssActive = 0u;
    }

    if( ( 0u != ( cfg2 & SPI_CFG2_MASTER ) ) &&
        ( 0u != ssActive                    )    )
    {
        (void)__atomic_and_fetch( &UT_SPI_REG->CFG2, ~SPI_CFG2_MASTER, __ATOMIC_SEQ_CST );
        (void)__atomic_or_fetch( &UT_SPI_REG->SR, SPI_SR_MODF, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* No mode fault */
    }
}


/**
 * \brief Verifies and resets all mocks (CMock memory is common for all mocks - a single mock
 *        is never reset alone).
 */
static void Ut_Spi_Reset_Mocks( void )
{
    MockRcc_Port_Verify();
    MockNvic_Port_Verify();
    MockGpio_Port_Verify();
    MockGpdma_Port_Verify();

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockGpdma_Port_Destroy();

    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockGpdma_Port_Init();
}


/**
 * \brief Resets the mocks and accepts any RCC / NVIC / GPIO / GPDMA call by recording stubs.
 */
static void Ut_Spi_Ignore_PeriphMocks( void )
{
    Ut_Spi_Reset_Mocks();

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_Stub( Ut_Spi_RccClkSrcStub );
    Rcc_Get_PeriphClk_Stub( Ut_Spi_RccClkStub );
    Gpio_Init_Stub( Ut_Spi_GpioInitStub );
    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Spi_NvicHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_Stub( Ut_Spi_NvicInactiveStub );
    Gpdma_Get_DefaultConfig_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_Stub( Ut_Spi_DmaInitStub );
    Gpdma_Set_ChannelActive_Stub( Ut_Spi_DmaActiveStub );
    Gpdma_Set_ChannelInactive_Stub( Ut_Spi_DmaInactiveStub );
    Gpdma_Set_InterruptActive_Stub( Ut_Spi_DmaIrqOnStub );
    Gpdma_Set_InterruptInactive_Stub( Ut_Spi_DmaIrqOffStub );
    Gpdma_Set_Priority_Stub( Ut_Spi_DmaPrioStub );
    Gpdma_Set_SourceAddr_Stub( Ut_Spi_DmaSrcAddrStub );
    Gpdma_Set_DestinationAddr_Stub( Ut_Spi_DmaDstAddrStub );
    Gpdma_Set_SourceAddrMode_Stub( Ut_Spi_DmaSrcModeStub );
    Gpdma_Set_DestinationAddrMode_Stub( Ut_Spi_DmaDstModeStub );
    Gpdma_Set_SourceDataSize_Stub( Ut_Spi_DmaSrcSizeStub );
    Gpdma_Set_DestinationDataSize_Stub( Ut_Spi_DmaDstSizeStub );
    Gpdma_Set_BlockSize_Stub( Ut_Spi_DmaBlockStub );
    Gpdma_Get_BlockSize_Stub( Ut_Spi_DmaRemainingStub );
}


/**
 * \brief Returns configuration of SPI1 master used by tests (default configuration, 1 MHz).
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
 * \brief Returns data handling configuration of tests (GPDMA1 channel 0 TX low priority,
 *        channel 1 RX high priority, test callbacks).
 *
 * \param xferMode [in]: Data transfer mode
 */
static spi_DataConfig_t Ut_Spi_Get_DataConfig( spi_XferMode_t xferMode )
{
    const spi_DataConfig_t dataConfig =
    {
        .XferMode             = xferMode,
        .TxDmaPeriphId        = SPI_DMA_PERIPH_1,
        .TxDmaChannelId       = SPI_DMA_CHANNEL_0,
        .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
        .RxDmaPeriphId        = SPI_DMA_PERIPH_1,
        .RxDmaChannelId       = SPI_DMA_CHANNEL_1,
        .RxDmaPriority        = SPI_DMA_PRIORITY_HIGH,
        .IrqPriority          = UT_SPI_PRIO,
        .XferCompleteCallback = Ut_Spi_XferCompleteCallback,
        .ErrorCallback        = Ut_Spi_ErrorCallback,
    };

    return ( dataConfig );
}


/**
 * \brief Initializes SPI with ignored (recorded) mocks.
 *
 * \param config [in]: Configuration
 */
static void Ut_Spi_Init( const spi_Config_t * const config )
{
    Ut_Spi_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( config ) );
}


/**
 * \brief Initializes SPI1 master (1 MHz) with data handling of the mode.
 *
 * \param xferMode [in]: Data transfer mode, SPI_XFER_MODE_CNT - no data handling
 */
static void Ut_Spi_Init_Master( spi_XferMode_t xferMode )
{
    spi_Config_t           config     = Ut_Spi_Get_Config();
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( xferMode );

    config.DataConfig = ( SPI_XFER_MODE_CNT > xferMode ) ? &dataConfig : NULL;

    Ut_Spi_Init( &config );
}


/**
 * \brief Expects clock activation and reset pulse of the RCC peripheral.
 *
 * \param rccId [in]: RCC peripheral (kernel clock source of the SPI)
 */
static void Ut_Spi_Expect_Activation( rcc_PeriphId_t rccId )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
}


/**
 * \brief Expects read of the kernel clock source of SPI1 and of its frequency.
 *
 * \param clkSrcId [in]: Reported kernel clock source
 * \param clockHz  [in]: Reported frequency
 */
static void Ut_Spi_Expect_KernelClk( rcc_PeriphId_t clkSrcId, rcc_FreqHz_t clockHz )
{
    static rcc_PeriphId_t reportedSrc;
    static rcc_FreqHz_t   reportedClk;

    reportedSrc = clkSrcId;
    reportedClk = clockHz;

    Rcc_Get_PeriphClkSrc_ExpectAndReturn( UT_SPI_RCC, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_IgnoreArg_periphClkSrc();
    Rcc_Get_PeriphClkSrc_ReturnThruPtr_periphClkSrc( &reportedSrc );
    Rcc_Get_PeriphClk_ExpectAndReturn( clkSrcId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &reportedClk );
}


/**
 * \brief Calls Spi_Task() with given status flags.
 *
 * \param srFlags [in]: Value of the SR register
 */
static void Ut_Spi_Task( uint32_t srFlags )
{
    UT_SPI_REG->SR = srFlags;
    Spi_Task();
}


/**
 * \brief Calls the captured SPI interrupt with given status flags.
 *
 * \param srFlags [in]: Value of the SR register
 */
static void Ut_Spi_Call_Isr( uint32_t srFlags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utSpi_Isr, "SPI ISR not registered" );

    UT_SPI_REG->SR = srFlags;
    utSpi_Isr();
}


/**
 * \brief Checks the end of the transfer - one callback of the expected result, transfer
 *        INACTIVE and the stored error.
 *
 * \param expError [in]: Expected result of the transfer
 */
static void Ut_Spi_Check_XferEnd( spi_XferErrorId_t expError )
{
    spi_FunctionState_t xferState = SPI_FUNCTION_ACTIVE;
    spi_XferErrorId_t   xferError = SPI_XFER_ERROR_CNT;

    if( SPI_XFER_ERROR_NONE == expError )
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, utSpi_ErrorCnt, "Error callback" );
        TEST_ASSERT_NOT_EQUAL_MESSAGE( 0u, utSpi_CompleteCnt, "Complete callback" );
    }
    else
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, utSpi_ErrorCnt, "Count of error callbacks" );
        TEST_ASSERT_EQUAL( expError, utSpi_LastError );
    }

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( UT_SPI_BUS, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferError( UT_SPI_BUS, &xferError ) );
    TEST_ASSERT_EQUAL( expError, xferError );
}


/** \brief Returns record of GPDMA channel calls */
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_TRUE( GPDMA_PERIPH_CNT > dmaBus );
    TEST_ASSERT_TRUE( UT_SPI_DMA_CHANNELS > (uint32_t)dmaChannel );

    return ( &utSpi_DmaChannel[ dmaBus ][ dmaChannel ] );
}


/** \brief Rcc_Get_PeriphClkSrc() stub - kernel clock source equals the base RCC peripheral */
static rcc_RequestState_t Ut_Spi_RccClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)callCnt;

    *periphClkSrc = periphId;

    return ( RCC_REQUEST_OK );
}


/** \brief Rcc_Get_PeriphClk() stub - returns \ref utSpi_ClkHz */
static rcc_RequestState_t Ut_Spi_RccClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClk = utSpi_ClkHz;

    return ( RCC_REQUEST_OK );
}


/** \brief Gpio_Init() stub - stores the configurations */
static gpio_RequestState_t Ut_Spi_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );

    if( UT_SPI_GPIO_CFG_CNT > utSpi_GpioInitCnt )
    {
        utSpi_GpioConfig[ utSpi_GpioInitCnt ] = *gpioConfig;
    }
    else
    {
        /* No action required */
    }

    utSpi_GpioInitCnt++;

    return ( GPIO_REQUEST_OK );
}


/** \brief Nvic_Set_PeriphIrq_Handler() stub - stores the handler */
static nvic_RequestState_t Ut_Spi_NvicHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utSpi_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** \brief Nvic_Set_PeriphIrq_Inactive() stub - counts the calls */
static nvic_RequestState_t Ut_Spi_NvicInactiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    utSpi_NvicOffCnt++;

    return ( utSpi_NvicOffState );
}


/** \brief Gpdma_Init() stub - stores the configuration and its transfer configuration */
static gpdma_RequestState_t Ut_Spi_DmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( configStruct );
    TEST_ASSERT_NOT_NULL( configStruct->TransferConfig );

    if( UT_SPI_DMA_CFG_CNT > utSpi_DmaInitCnt )
    {
        utSpi_DmaConfig[ utSpi_DmaInitCnt ]     = *configStruct;
        utSpi_DmaXferConfig[ utSpi_DmaInitCnt ] = *configStruct->TransferConfig;
    }
    else
    {
        /* No action required */
    }

    utSpi_DmaInitCnt++;

    return ( utSpi_DmaInitState );
}


/** \brief Gpdma_Set_ChannelActive() stub */
static gpdma_RequestState_t Ut_Spi_DmaActiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->ActiveCnt++;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_ChannelInactive() stub */
static gpdma_RequestState_t Ut_Spi_DmaInactiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->InactiveCnt++;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SpeAtInactive = UT_SPI_REG->CR1 & SPI_CR1_SPE;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_InterruptActive() stub */
static gpdma_RequestState_t Ut_Spi_DmaIrqOnStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOnCnt++;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_InterruptInactive() stub - returns \ref utSpi_DmaIrqOffState */
static gpdma_RequestState_t Ut_Spi_DmaIrqOffStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOffCnt++;
    return ( utSpi_DmaIrqOffState );
}


/** \brief Gpdma_Set_Priority() stub */
static gpdma_RequestState_t Ut_Spi_DmaPrioStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_Priority_t channelPrio, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->PrioCnt++;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->Prio = channelPrio;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_SourceAddr() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_SrcAddr_t sourceAddr, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcAddr = sourceAddr;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_DestinationAddr() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DstAddr_t destAddr, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstAddr = destAddr;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_SourceAddrMode() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcModeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t srcAddrMode, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcMode = srcAddrMode;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_DestinationAddrMode() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstModeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t destAddrMode, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstMode = destAddrMode;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_SourceDataSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcSizeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t srcDataSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcSize = srcDataSize;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_DestinationDataSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstSizeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t destDataSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstSize = destDataSize;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_BlockSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaBlockStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t blockSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->BlockSize = blockSize;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Get_BlockSize() stub - returns \ref utSpi_DmaRemaining */
static gpdma_RequestState_t Ut_Spi_DmaRemainingStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t * const blockSize, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    *blockSize = utSpi_DmaRemaining;

    return ( GPDMA_REQUEST_OK );
}


/** \brief Transfer complete callback of tests */
static void Ut_Spi_XferCompleteCallback( void )
{
    utSpi_CompleteCnt++;
}


/**
 * \brief Transfer error callback of tests.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_Spi_ErrorCallback( spi_XferErrorId_t errorId )
{
    utSpi_LastError = errorId;
    utSpi_ErrorCnt++;
}
