/**
 * \author Mr.Nobody
 * \file Test_Spi.c
 * \ingroup Spi
 * \brief Unit tests of Serial Peripheral Interface (SPI) module (STM32H7, SPI v2 with FIFO).
 *
 * Spi.c, Spi_Dma.c, Spi_Isr.c and Spi_Poll.c are compiled unchanged with real
 * LL drivers. SPI registers are emulated by RegMem, RCC, NVIC, GPIO and DMA
 * modules are mocked by CMock. SPI ISR registered in NVIC and DMA error
 * callbacks passed to Dma_Init() are captured by stubs and called directly.
 *
 * \note Emulated registers are plain memory. Status flags (SR) are not changed
 *       by data register accesses - tests preset the flags (TXP, RXP, EOT,
 *       errors, SUSP) and the received data (RXDR) before every transfer step.
 *       A preset RXP stays active, so one step reads all frames it may read.
 * \note Master transfer abort waits for SUSP / EOT - tests preset SUSP before
 *       an abort of a running master transfer.
 * \note DMA buffers are placed in emulated SRAM (\ref REGMEM_SRAM_PTR) - the
 *       module passes 32-bit memory addresses to DMA.
 * \note Runtime state of the module is static - setUp() releases every
 *       peripheral by Spi_Deinit() with ignored mocks.
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Spi_Port.h"                       /* Module under test              */
#include "Spi.h"                            /* Module private interface       */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "MockGpdma_Port.h"                 /* GPDMA module mock (STM32H7R / H7S) */
#include "Stm32_spi.h"                      /* SPI registers definition       */

/* ============================= TYPEDEFS =================================== */

/** \brief Record of DMA calls of one stream */
typedef struct
{
    uint32_t            ActiveCnt;      /**< Dma_Set_TransferActive() calls            */
    uint32_t            InactiveCnt;    /**< Dma_Set_TransferInactive() calls          */
    uint32_t            IrqOnCnt;       /**< Dma_Set_InterruptActive() calls           */
    uint32_t            IrqOffCnt;      /**< Dma_Set_InterruptInactive() calls         */
#if defined(STM32H7RS)
    uint32_t            PrioCnt;        /**< Gpdma_Set_Priority() calls          */
    gpdma_Priority_t    Prio;           /**< Last channel priority               */
    gpdma_SrcAddr_t     SrcAddr;        /**< Last source address                 */
    gpdma_DstAddr_t     DstAddr;        /**< Last destination address            */
    gpdma_AddrMode_t    SrcMode;        /**< Last source address mode            */
    gpdma_AddrMode_t    DstMode;        /**< Last destination address mode       */
    gpdma_DataSize_t    SrcSize;        /**< Last source data size               */
    gpdma_DataSize_t    DstSize;        /**< Last destination data size          */
    gpdma_BlockSize_t   BlockSize;      /**< Last block size                     */
#else
    uint32_t            TeIrqOnCnt;     /**< Dma_Set_TransferErrorIrqActive() calls    */
    uint32_t            TeIrqOffCnt;    /**< Dma_Set_TransferErrorIrqInactive() calls  */
    uint32_t            TeIsrNullCnt;   /**< Dma_Set_TransferErrorIsrHandler( NULL )   */
    dma_MemoryAddr_t    MemAddr;        /**< Last memory address                       */
    dma_MemoryAddrInc_t MemInc;         /**< Last memory address increment             */
    dma_TransferSize_t  PeriphSize;     /**< Last peripheral data size                 */
    dma_TransferSize_t  MemSize;        /**< Last memory data size                     */
    dma_DataCount_t     DataCount;      /**< Last data count                           */
#endif /* STM32H7RS */
}   ut_SpiDmaChannel_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Spi_Reset_Mocks          ( void );
static void                 Ut_Spi_Ignore_PeriphMocks   ( void );
static spi_Config_t         Ut_Spi_Get_Config           ( void );
static spi_DataConfig_t     Ut_Spi_Get_DataConfig       ( spi_XferMode_t xferMode );
static void                 Ut_Spi_Init                 ( const spi_Config_t * const config );
static void                 Ut_Spi_Init_Master          ( spi_XferMode_t xferMode );
#if defined(STM32H7RS)
static void                 Ut_Spi_Flush_DmaChannelState( void );
#endif /* STM32H7RS */
static void                 Ut_Spi_Expect_Activation    ( rcc_PeriphId_t rccId );
static void                 Ut_Spi_Expect_KernelClk     ( rcc_PeriphId_t clkSrcId, rcc_FreqHz_t clockHz );
static void                 Ut_Spi_Task                 ( uint32_t srFlags );
static void                 Ut_Spi_Call_Isr             ( uint32_t srFlags );
static void                 Ut_Spi_Check_XferEnd        ( spi_XferErrorId_t expError );
#if defined(STM32H7RS)
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel );
#else
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel       ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel );
#endif /* STM32H7RS */

static rcc_RequestState_t   Ut_Spi_RccClkSrcStub        ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_Spi_RccClkStub           ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_Spi_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Spi_NvicHandlerStub      ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static nvic_RequestState_t  Ut_Spi_NvicInactiveStub     ( nvic_PeriphIrqList_t irqId, int callCnt );
#if defined(STM32H7RS)
static gpdma_RequestState_t Ut_Spi_DmaInitStub          ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
#else
static dma_RequestState_t   Ut_Spi_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
#endif /* STM32H7RS */
#if defined(STM32H7RS)
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
#else
static dma_RequestState_t   Ut_Spi_DmaActiveStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaInactiveStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaIrqOnStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaTeIrqOnStub       ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaTeIrqOffStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaTeIsrStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_IsrCallback irqHandler, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaPeriphSizeStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferSize_t periphTransferSize, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaMemSizeStub       ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferSize_t memoryTransferSize, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaMemIncStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddrInc_t memoryAddrInc, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaMemAddrStub       ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaDataCountStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt );
static dma_RequestState_t   Ut_Spi_DmaRemainingStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
#endif /* STM32H7RS */

static void                 Ut_Spi_XferCompleteCallback ( void );
static void                 Ut_Spi_ErrorCallback        ( spi_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** SPI peripheral used by tests (available on all STM32H7 devices) */
#define UT_SPI_BUS                          ( SPI_PERIPH_1 )
#define UT_SPI_REG                          ( SPI1 )
#define UT_SPI_RCC                          ( RCC_PERIPH_SPI1_PLL1Q )
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

/** DMA streams of tests */
#if defined(STM32H7RS)
#define UT_SPI_DMA                          ( GPDMA_PERIPH_1 )
#define UT_SPI_DMA_TX_CHANNEL               ( GPDMA_CHANNEL_0 )
#define UT_SPI_DMA_RX_CHANNEL               ( GPDMA_CHANNEL_1 )
#else
#define UT_SPI_DMA                          ( DMA_PERIPH_1 )
#define UT_SPI_DMA_TX_CHANNEL               ( DMA_STREAM_0 )
#define UT_SPI_DMA_RX_CHANNEL               ( DMA_STREAM_1 )
#endif /* STM32H7RS */

/** Count of DMA configurations stored by Dma_Init() stub */
#define UT_SPI_DMA_CFG_CNT                  ( 4u )

/** Count of DMA streams recorded per DMA peripheral */
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

/** DMA configurations of Dma_Init() calls */
#if !defined(STM32H7RS)
static dma_ConfigStruct_t       utSpi_DmaConfig[ UT_SPI_DMA_CFG_CNT ];
#endif /* !STM32H7RS */
static uint32_t                 utSpi_DmaInitCnt;
#if !defined(STM32H7RS)
static dma_RequestState_t       utSpi_DmaInitState;
#endif /* !STM32H7RS */

/** Return value of Dma_Set_InterruptInactive() stub */
#if !defined(STM32H7RS)
static dma_RequestState_t       utSpi_DmaIrqOffState;
#endif /* !STM32H7RS */

/** Remaining data count returned by Dma_Get_DataCount() stub */
#if !defined(STM32H7RS)
static dma_DataCount_t          utSpi_DmaRemaining;
#endif /* !STM32H7RS */

/** Records of DMA stream calls */
#if !defined(STM32H7RS)
static ut_SpiDmaChannel_t       utSpi_DmaChannel[ DMA_PERIPH_CNT ][ UT_SPI_DMA_CHANNELS ];
#endif /* !STM32H7RS */

#if defined(STM32H7RS)
/** GPDMA configurations of Gpdma_Init() calls (structure and transfer configuration) */
static gpdma_ConfigStruct_t     utSpi_DmaConfig[ UT_SPI_DMA_CFG_CNT ];
static gpdma_TransferConfig_t   utSpi_DmaXferConfig[ UT_SPI_DMA_CFG_CNT ];
static uint32_t                 utSpi_DmaInitCnt;
static gpdma_RequestState_t     utSpi_DmaInitState;
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** Return value of Gpdma_Set_InterruptInactive() stub */
static gpdma_RequestState_t     utSpi_DmaIrqOffState;
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** Remaining block size returned by Gpdma_Get_BlockSize() stub */
static gpdma_BlockSize_t        utSpi_DmaRemaining;
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** Records of GPDMA channel calls */
static ut_SpiDmaChannel_t       utSpi_DmaChannel[ GPDMA_PERIPH_CNT ][ UT_SPI_DMA_CHANNELS ];
#endif /* STM32H7RS */

/** Counts of callback calls */
static uint32_t                 utSpi_CompleteCnt;
static uint32_t                 utSpi_ErrorCnt;

/** Error of the last error callback */
static spi_XferErrorId_t        utSpi_LastError;

/* ============================ TEST FIXTURE ================================ */

#if defined(STM32H7RS)
void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    /* Results of the stubs which the release of the previous test uses (the previous test may set them) */
    utSpi_ClkHz          = UT_SPI_CLK_HZ;
    utSpi_NvicOffState   = NVIC_REQUEST_OK;
    utSpi_DmaInitState   = GPDMA_REQUEST_OK;
    utSpi_DmaIrqOffState = GPDMA_REQUEST_OK;

    Ut_Spi_Ignore_PeriphMocks();

    for( spi_PeriphId_t periphId = (spi_PeriphId_t)0u; SPI_PERIPH_CNT > periphId; periphId++ )
    {
        (void)Spi_Deinit( periphId );
    }

    Ut_Spi_Flush_DmaChannelState();

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
#else
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
    utSpi_DmaInitState   = DMA_REQUEST_OK;
    utSpi_DmaIrqOffState = DMA_REQUEST_OK;
    utSpi_DmaRemaining   = 0u;
    utSpi_CompleteCnt    = 0u;
    utSpi_ErrorCnt       = 0u;
    utSpi_LastError      = SPI_XFER_ERROR_NONE;

    (void)memset( utSpi_GpioConfig, 0, sizeof( utSpi_GpioConfig ) );
    (void)memset( utSpi_DmaConfig, 0, sizeof( utSpi_DmaConfig ) );
    (void)memset( utSpi_DmaChannel, 0, sizeof( utSpi_DmaChannel ) );
}
#endif /* STM32H7RS */


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

    /* SPI4 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_4, 4u, 12u, 5u ), SPI_SCK_PIN_SPI4_PE12 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_4, 4u, 13u, 5u ), SPI_MISO_PIN_SPI4_PE13 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_4, 4u, 14u, 5u ), SPI_MOSI_PIN_SPI4_PE14 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_4, 4u, 11u, 5u ), SPI_NSS_PIN_SPI4_PE11 );

    /* SPI5 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_5, 5u, 7u, 5u ), SPI_SCK_PIN_SPI5_PF7 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_5, 5u, 8u, 5u ), SPI_MISO_PIN_SPI5_PF8 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_5, 5u, 9u, 5u ), SPI_MOSI_PIN_SPI5_PF9 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_5, 5u, 6u, 5u ), SPI_NSS_PIN_SPI5_PF6 );

    /* SPI6 */
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_6, 0u, 5u, 8u ), SPI_SCK_PIN_SPI6_PA5 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_6, 0u, 6u, 8u ), SPI_MISO_PIN_SPI6_PA6 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_6, 0u, 7u, 8u ), SPI_MOSI_PIN_SPI6_PA7 );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_PIN_CODE( SPI_PERIPH_6, 0u, 4u, 8u ), SPI_NSS_PIN_SPI6_PA4 );

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
 * - SPI1, PLL1Q kernel clock, master 1 MHz, mode 0, 8-bit, MSB first, full duplex, Motorola,
 *   software NSS active low without pulse, no data handling, no pins, high pin speed.
 * - NULL pointer: SPI_REQUEST_ERROR.
 */
void Ut_Spi_Get_DefaultConfig_FillsDefaults( void )
{
    spi_Config_t config;

    (void)memset( &config, 0xA5, sizeof( config ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( (spi_PeriphId_t)0u, config.PeriphId );
    TEST_ASSERT_EQUAL( SPI_CLK_SRC_SPI1_PLL1Q, config.ClkSrc );
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
    config = Ut_Spi_Get_Config(); config.ClkSrc           = (spi_ClkSrc_t)SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_CNT );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.ClkSrc           = SPI_CLK_SRC_SPI2_PLL1Q;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    config = Ut_Spi_Get_Config(); config.ClkSrc           = (spi_ClkSrc_t)SPI_CLK_SRC_ENCODE( SPI_PERIPH_1, SPI_CLK_SRC_ID_PCLK );
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
 * \details Default configuration (SPI1 master 1 MHz, PLL1Q kernel clock 100 MHz, no pins,
 *          no data handling).
 *
 * \par Expected results
 * - RCC: clock of PLL1Q kernel clock source activated, reset pulse, kernel clock source and
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
 * \details SPI1 with PLL2 kernel clock source (50 MHz), kernel clock source read back as
 *          PLL2P.
 *
 * \par Expected results
 * - Rcc_Set_PeriphActive / reset with RCC_PERIPH_SPI1_PLL2P, frequency of PLL2P used
 *   (MBR = 5: 50 MHz / 64).
 */
void Ut_Spi_Init_ClockSourcePll2_ClockOfSourceActivated( void )
{
    spi_Config_t config = Ut_Spi_Get_Config();

    config.ClkSrc = SPI_CLK_SRC_SPI1_PLL2P;

    Ut_Spi_Expect_Activation( RCC_PERIPH_SPI1_PLL2P );
    Ut_Spi_Expect_KernelClk( RCC_PERIPH_SPI1_PLL2P, 50000000u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 5u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & SPI_CFG1_MBR );
}


/**
 * \brief   Spi_Init() activates the clock of the RCC identification of every kernel clock source item.
 *
 * \details Every item of \ref spi_ClkSrc_t is passed in the configuration of its peripheral.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, Rcc_Set_PeriphActive / reset with the RCC identification of the item (strict mocks).
 */
void Ut_Spi_Init_ClockSourceItems_ActivateOwnRccId( void )
{
    static const struct
    {
        spi_PeriphId_t PeriphId;
        spi_ClkSrc_t   ClkSrc;
        rcc_PeriphId_t RccId;
    }   itemLut[] =
    {
#ifdef SPI1
        { SPI_PERIPH_1, SPI_CLK_SRC_SPI1_PLL1Q, RCC_PERIPH_SPI1_PLL1Q },
        { SPI_PERIPH_1, SPI_CLK_SRC_SPI1_PLL2P, RCC_PERIPH_SPI1_PLL2P },
        { SPI_PERIPH_1, SPI_CLK_SRC_SPI1_PLL3P, RCC_PERIPH_SPI1_PLL3P },
#endif /* SPI1 */
#ifdef SPI2
        { SPI_PERIPH_2, SPI_CLK_SRC_SPI2_PLL1Q, RCC_PERIPH_SPI2_PLL1Q },
        { SPI_PERIPH_2, SPI_CLK_SRC_SPI2_PLL2P, RCC_PERIPH_SPI2_PLL2P },
        { SPI_PERIPH_2, SPI_CLK_SRC_SPI2_PLL3P, RCC_PERIPH_SPI2_PLL3P },
#endif /* SPI2 */
#ifdef SPI3
        { SPI_PERIPH_3, SPI_CLK_SRC_SPI3_PLL1Q, RCC_PERIPH_SPI3_PLL1Q },
        { SPI_PERIPH_3, SPI_CLK_SRC_SPI3_PLL2P, RCC_PERIPH_SPI3_PLL2P },
        { SPI_PERIPH_3, SPI_CLK_SRC_SPI3_PLL3P, RCC_PERIPH_SPI3_PLL3P },
#endif /* SPI3 */
#ifdef SPI4
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_PLL2Q, RCC_PERIPH_SPI4_PLL2Q },
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_PLL3Q, RCC_PERIPH_SPI4_PLL3Q },
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_PCLK2, RCC_PERIPH_SPI4_PCLK2 },
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_HSI, RCC_PERIPH_SPI4_HSI },
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_CSI, RCC_PERIPH_SPI4_CSI },
        { SPI_PERIPH_4, SPI_CLK_SRC_SPI4_HSE, RCC_PERIPH_SPI4_HSE },
#endif /* SPI4 */
#ifdef SPI5
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_PLL2Q, RCC_PERIPH_SPI5_PLL2Q },
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_PLL3Q, RCC_PERIPH_SPI5_PLL3Q },
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_PCLK2, RCC_PERIPH_SPI5_PCLK2 },
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_HSI, RCC_PERIPH_SPI5_HSI },
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_CSI, RCC_PERIPH_SPI5_CSI },
        { SPI_PERIPH_5, SPI_CLK_SRC_SPI5_HSE, RCC_PERIPH_SPI5_HSE },
#endif /* SPI5 */
#ifdef SPI6
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_PLL2Q, RCC_PERIPH_SPI6_PLL2Q },
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_PLL3Q, RCC_PERIPH_SPI6_PLL3Q },
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_PCLK4, RCC_PERIPH_SPI6_PCLK4 },
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_HSI, RCC_PERIPH_SPI6_HSI },
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_CSI, RCC_PERIPH_SPI6_CSI },
        { SPI_PERIPH_6, SPI_CLK_SRC_SPI6_HSE, RCC_PERIPH_SPI6_HSE },
#endif /* SPI6 */
    };

    Rcc_Get_PeriphClkSrc_Stub( Ut_Spi_RccClkSrcStub );
    Rcc_Get_PeriphClk_Stub( Ut_Spi_RccClkStub );
    utSpi_ClkHz = UT_SPI_CLK_HZ;

    for( uint32_t idx = 0u; ( sizeof( itemLut ) / sizeof( itemLut[ 0u ] ) ) > idx; idx++ )
    {
        spi_Config_t config = Ut_Spi_Get_Config();

        config.PeriphId = itemLut[ idx ].PeriphId;
        config.ClkSrc   = itemLut[ idx ].ClkSrc;

        Ut_Spi_Expect_Activation( itemLut[ idx ].RccId );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( &config ) );
    }
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
 * - Reset pulse and clock disable of RCC_PERIPH_SPI1_PLL1Q, SPE cleared, SPI_REQUEST_OK.
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
 * \brief   Prescaler bypass (BPASS, STM32H7R / H7S) is cleared by the bus frequency setting.
 *
 * \details CFG1.BPASS preset, sets 10 MHz bus frequency.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, BPASS cleared, MBR = 3 (kernel clock / 16), bus frequency 6.25 MHz.
 * - Classic STM32H7 lines: ignored (no prescaler bypass).
 */
void Ut_Spi_Set_BusFreq_PrescalerBypassCleared( void )
{
#if defined(STM32H7RS)
    spi_FreqHz_t busFreq = 0u;

    Ut_Spi_Ignore_PeriphMocks();

    UT_SPI_REG->CFG1 = SPI_CFG1_BPASS;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_BusFreq( UT_SPI_BUS, 10000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 3u << SPI_CFG1_MBR_Pos, UT_SPI_REG->CFG1 & ( SPI_CFG1_MBR | SPI_CFG1_BPASS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( UT_SPI_BUS, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 6250000u, busFreq );
#else
    TEST_IGNORE_MESSAGE( "Classic STM32H7: no prescaler bypass (BPASS of STM32H7R / H7S)" );
#endif /* STM32H7RS */
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
 * - SPI_REQUEST_ERROR, no DMA / NVIC call (strict mocks).
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Set_DataConfig_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
#if defined(STM32H7RS)
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
#else
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
    dataConfig.TxDmaPriority = (spi_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA data handling initializes both DMA streams.
 *
 * \details DMA1 stream 0 (TX, low priority) and stream 1 (RX, high priority).
 *
 * \par Expected results
 * - Dma_Init() 2x: TX memory to peripheral, DMAMUX1 request SPI1_TX, peripheral address TXDR;
 *   RX peripheral to memory, request SPI1_RX, peripheral address RXDR; normal mode, peripheral
 *   address static, memory increment, 8-bit data, error callback, no complete / half callback,
 *   priorities of the config.
 * - Transfer error interrupt and NVIC line of both streams enabled, both streams stopped,
 *   TXDMAEN / RXDMAEN cleared, IER cleared; SPI interrupt registered.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Set_DataConfig_Dma_ChannelsInitialized( void )
{
#if defined(STM32H7RS)
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
#else
    UT_SPI_REG->CFG1 = SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_DmaInitCnt );

    /* -1- Transmit stream */
    TEST_ASSERT_EQUAL( UT_SPI_DMA,                 utSpi_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_TX_CHANNEL,      utSpi_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,           utSpi_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,   utSpi_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_REQ_SPI1_TX,            utSpi_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_SPI_REG->TXDR, utSpi_DmaConfig[ 0u ].PeriphAddress );

    /* -2- Receive stream */
    TEST_ASSERT_EQUAL( UT_SPI_DMA,                 utSpi_DmaConfig[ 1u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_SPI_DMA_RX_CHANNEL,      utSpi_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,          utSpi_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,   utSpi_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( DMA_REQ_SPI1_RX,            utSpi_DmaConfig[ 1u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_SPI_REG->RXDR, utSpi_DmaConfig[ 1u ].PeriphAddress );

    for( uint32_t cfgIdx = 0u; 2u > cfgIdx; cfgIdx++ )
    {
        TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,  utSpi_DmaConfig[ cfgIdx ].TransferMode );
        TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,    utSpi_DmaConfig[ cfgIdx ].PeriphAddrIncrement );
        TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, utSpi_DmaConfig[ cfgIdx ].MemoryAddrIncrement );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,    utSpi_DmaConfig[ cfgIdx ].PeriphTransferSize );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,    utSpi_DmaConfig[ cfgIdx ].MemoryTransferSize );
        TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ cfgIdx ].TransferErrorCallback );
        TEST_ASSERT_NULL( utSpi_DmaConfig[ cfgIdx ].TransferCompleteCallback );
        TEST_ASSERT_NULL( utSpi_DmaConfig[ cfgIdx ].HalfTransferCallback );
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->TeIrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->TeIrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );
#endif /* STM32H7RS */
}


/**
 * \brief   Repeated DMA configuration releases the streams and initializes them again.
 *
 * \details DMA data handling configured, then configured again with other priorities.
 *
 * \par Expected results
 * - Streams of the first configuration released: transfer error interrupt, NVIC line and
 *   error handler of both streams removed.
 * - Dma_Init() 2x more with the new priorities, stream interrupts enabled again.
 */
void Ut_Spi_Set_DataConfig_DmaReconfigured_StreamsReinitialized( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    dataConfig.TxDmaPriority = SPI_DMA_PRIORITY_VERYHIGH;
    dataConfig.RxDmaPriority = SPI_DMA_PRIORITY_MEDIUM;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL )->TeIrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->TeIsrNullCnt );
    TEST_ASSERT_EQUAL_UINT32( 4u, utSpi_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_VERYHIGH, utSpi_DmaConfig[ 2u ].Priority );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_MEDIUM, utSpi_DmaConfig[ 3u ].Priority );
    TEST_ASSERT_EQUAL_UINT32( 2u, Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL )->IrqOnCnt );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA initialization failure is reported.
 *
 * \details Dma_Init() returns error.
 *
 * \par Expected results
 * - SPI_REQUEST_ERROR, data handling not initialized.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Set_DataConfig_DmaInitFailure_ReturnsError( void )
{
#if defined(STM32H7RS)
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( SPI_XFER_MODE_NONE );
    utSpi_DmaInitState = GPDMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
#else
    const spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    spi_DataConfig_t       readBack;

    Ut_Spi_Init_Master( SPI_XFER_MODE_NONE );
    utSpi_DmaInitState = DMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataConfig( UT_SPI_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
#endif /* STM32H7RS */
}


/**
 * \brief   Spi_Set_DataConfig() releases the previous data handling.
 *
 * \details DMA mode reconfigured to POLL, then to NONE mode.
 *
 * \par Expected results
 * - DMA -> POLL: both DMA streams released and their interrupts disabled, SPI interrupt
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
 * \details DMA mode, Spi_Deinit() twice; DMA mode with failing Dma_Set_InterruptInactive();
 *          ISR mode with failing Nvic_Set_PeriphIrq_Inactive().
 *
 * \par Expected results
 * - SPI_REQUEST_OK, channel interrupts disabled (1x each), NVIC line disabled, data
 *   handling not initialized; second Spi_Deinit() does not touch DMA / NVIC again.
 * - Release failures: SPI_REQUEST_ERROR, data handling not initialized.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Deinit_DataHandlingReleasedAndFailuresReported( void )
{
#if defined(STM32H7RS)
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
#else
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
    utSpi_DmaIrqOffState = DMA_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );

    utSpi_DmaIrqOffState = DMA_REQUEST_OK;
    Ut_Spi_Init_Master( SPI_XFER_MODE_ISR );
    utSpi_NvicOffState = NVIC_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Deinit( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_DataConfig( UT_SPI_BUS, &readBack ) );
#endif /* STM32H7RS */
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

/**
 * \brief   SPI4 (limited feature set on STM32H7) refuses data size above 16 bits.
 *
 * \details Sets 32-bit and 16-bit data size of SPI4 and 32-bit data size of SPI2.
 *
 * \par Expected results
 * - SPI4 32-bit: SPI_REQUEST_ERROR, CFG1 not written.
 * - SPI4 16-bit: SPI_REQUEST_OK, DSIZE = 15.
 * - SPI2 32-bit (full feature set): SPI_REQUEST_OK, DSIZE = 31.
 */
void Ut_Spi_Set_DataSize_Spi4Limited_RefusesAbove16Bit( void )
{
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( SPI_PERIPH_4, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SPI4->CFG1 );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( SPI_PERIPH_4, SPI_DATA_SIZE_16BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 15u << SPI_CFG1_DSIZE_Pos, SPI4->CFG1 & SPI_CFG1_DSIZE );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( SPI_PERIPH_2, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 31u << SPI_CFG1_DSIZE_Pos, SPI2->CFG1 & SPI_CFG1_DSIZE );
}


/**
 * \brief   SPI6 data size range follows the feature set of the family.
 *
 * \details Sets 32-bit data size of SPI6.
 *
 * \par Expected results
 * - STM32H7R / H7S (full feature set): SPI_REQUEST_OK, DSIZE = 31.
 * - Classic STM32H7 lines (limited feature set): SPI_REQUEST_ERROR, CFG1 not written.
 */
void Ut_Spi_Set_DataSize_Spi6_FeatureSetOfFamily( void )
{
#if defined(STM32H7RS)
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( SPI_PERIPH_6, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 31u << SPI_CFG1_DSIZE_Pos, SPI6->CFG1 & SPI_CFG1_DSIZE );
#else
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_DataSize( SPI_PERIPH_6, SPI_DATA_SIZE_32BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SPI6->CFG1 );
#endif /* STM32H7RS */
}


/**
 * \brief   SPI4 (limited feature set) accepts transfers up to the full TSIZE range.
 *
 * \details POLL data handling of SPI4, transfer of 0xFFFE frames (above the 10-bit limit of the
 *          limited STM32H5 instances), then 0xFFFF frames.
 *
 * \par Expected results
 * - 0xFFFE frames: SPI_REQUEST_OK, CR2.TSIZE = 0xFFFE.
 * - 0xFFFF frames: SPI_REQUEST_ERROR (above the maximum of the module).
 *
 * \note    STM32H7R / H7S: TSIZE of SPI4 / SPI5 is limited to 10 bits - 0x3FE frames accepted
 *          (CR2.TSIZE = 0x3FE), 0x3FF frames refused.
 */
void Ut_Spi_Set_XferStart_Spi4Limited_FullTsize( void )
{
#if defined(STM32H7RS)
    uint8_t * const   rxBuf    = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    spi_Config_t      config     = Ut_Spi_Get_Config();
    spi_XferRequest_t request    = { .TxData = NULL, .RxData = rxBuf, .XferSize = 0x3FEu };

    config.PeriphId   = SPI_PERIPH_4;
    config.ClkSrc     = SPI_CLK_SRC_SPI4_PCLK2;
    config.DataConfig = &dataConfig;
    Ut_Spi_Init( &config );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( SPI_PERIPH_4, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0x3FEu, SPI4->CR2 & SPI_CR2_TSIZE );

    SPI4->SR = SPI_SR_SUSP;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( SPI_PERIPH_4 ) );
    request.XferSize = 0x3FFu;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( SPI_PERIPH_4, &request ) );
#else
    uint8_t * const   rxBuf    = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_POLL );
    spi_Config_t      config     = Ut_Spi_Get_Config();
    spi_XferRequest_t request    = { .TxData = NULL, .RxData = rxBuf, .XferSize = 0xFFFEu };

    config.PeriphId   = SPI_PERIPH_4;
    config.ClkSrc     = SPI_CLK_SRC_SPI4_PCLK2;
    config.DataConfig = &dataConfig;
    Ut_Spi_Init( &config );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( SPI_PERIPH_4, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0xFFFEu, SPI4->CR2 & SPI_CR2_TSIZE );

    SPI4->SR = SPI_SR_SUSP;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( SPI_PERIPH_4 ) );
    request.XferSize = 0xFFFFu;
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Set_XferStart( SPI_PERIPH_4, &request ) );
#endif /* STM32H7RS */
}

/* ============================= DMA TRANSFERS ============================== */

/**
 * \brief   DMA full-duplex transfer arms both streams and completes at EOT.
 *
 * \details 3 frames of 8 bits, buffers in SRAM: start, SPI interrupt with EOT.
 *
 * \par Expected results
 * - RX stream: 8-bit sizes, data count 3, memory = receive buffer (increment), active;
 *   TX stream: memory = transmit buffer (increment), data count 3, active; RXDMAEN | TXDMAEN;
 *   IER = events; CSTART.
 * - EOT: remaining 0 -> complete callback; both streams stopped, DMA requests and IER
 *   cleared, SPE cleared.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Dma_FullDuplex_ChannelsArmedAndEotCompletes( void )
{
#if defined(STM32H7RS)
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
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
#else
    uint8_t * const           txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const           rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t   request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 3u };
    const ut_SpiDmaChannel_t *txChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL );
    const ut_SpiDmaChannel_t *rxChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT, rxChan->PeriphSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT, rxChan->MemSize );
    TEST_ASSERT_EQUAL_UINT16( 3u, rxChan->DataCount );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, rxChan->MemAddr );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, rxChan->MemInc );
    TEST_ASSERT_EQUAL_UINT32( 1u, rxChan->ActiveCnt );
    TEST_ASSERT_EQUAL_UINT16( 3u, txChan->DataCount );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_TX_BUF_OFFSET, txChan->MemAddr );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, txChan->MemInc );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChan->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( UT_SPI_IER_EVENTS, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( SPI_CR1_CSTART, UT_SPI_REG->CR1 & SPI_CR1_CSTART );

    Ut_Spi_Call_Isr( SPI_SR_EOT );

    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_UINT32( 2u, txChan->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxChan->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CFG1 & ( SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->IER );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_SPI_REG->CR1 & SPI_CR1_SPE );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA full-duplex reception without transmit buffer transmits a dummy frame.
 *
 * \details 16-bit frames, 2 frames, only receive buffer; then 32-bit frames with transmit
 *          buffer only.
 *
 * \par Expected results
 * - TX stream: 16-bit sizes, data count 2 (frames), static memory address (dummy, not 0).
 * - RX stream: 16-bit sizes, memory = receive buffer.
 * - 32-bit frames: TX stream 32-bit sizes, memory increment; RX stream static dummy address.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_Dma_ReceptionOnly_DummyTransmitSource( void )
{
#if defined(STM32H7RS)
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
#else
    uint8_t * const           txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const           rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    spi_XferRequest_t         request = { .TxData = NULL, .RxData = rxBuf, .XferSize = 2u };
    const ut_SpiDmaChannel_t *txChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_TX_CHANNEL );
    const ut_SpiDmaChannel_t *rxChan  = Ut_Spi_Get_DmaChannel( UT_SPI_DMA, UT_SPI_DMA_RX_CHANNEL );

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_16BIT ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, txChan->PeriphSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, txChan->MemSize );
    TEST_ASSERT_EQUAL_UINT16( 2u, txChan->DataCount );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_STATIC, txChan->MemInc );
    TEST_ASSERT_NOT_EQUAL( 0u, txChan->MemAddr );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, rxChan->MemSize );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_RX_BUF_OFFSET, rxChan->MemAddr );

    /* -2- 32-bit frames, transmission only (received frames discarded) */
    UT_SPI_REG->SR = SPI_SR_SUSP;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( UT_SPI_BUS ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_DataSize( UT_SPI_BUS, SPI_DATA_SIZE_32BIT ) );
    request = (spi_XferRequest_t){ .TxData = txBuf, .RxData = NULL, .XferSize = 2u };

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );

    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_32BIT, txChan->PeriphSize );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, txChan->MemInc );
    TEST_ASSERT_EQUAL_HEX32( REGMEM_SRAM_BASE + UT_SPI_TX_BUF_OFFSET, txChan->MemAddr );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_STATIC, rxChan->MemInc );
#endif /* STM32H7RS */
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
 * \details Full-duplex DMA transfer, DMA reports 1 remaining frame at EOT.
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
 * \brief   DMA transfer error aborts the transfer and is reported once.
 *
 * \details Full-duplex DMA transfer: error callback of the RX stream; new transfer: error
 *          callback of the TX stream.
 *
 * \par Expected results
 * - Both cases: one error callback SPI_XFER_ERROR_DMA_TRANSFER, transfer INACTIVE.
 */
void Ut_Spi_Dma_DmaError_OneErrorCallbackPerTransfer( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    uint8_t * const         txBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_TX_BUF_OFFSET );
    uint8_t * const         rxBuf   = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    const spi_XferRequest_t request = { .TxData = txBuf, .RxData = rxBuf, .XferSize = 2u };

    Ut_Spi_Init_Master( SPI_XFER_MODE_DMA );
    TEST_ASSERT_NOT_NULL( utSpi_DmaConfig[ 1u ].TransferErrorCallback );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    UT_SPI_REG->SR = SPI_SR_SUSP;
    utSpi_DmaConfig[ 1u ].TransferErrorCallback();
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_DMA_TRANSFER );

    utSpi_ErrorCnt = 0u;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( UT_SPI_BUS, &request ) );
    utSpi_DmaConfig[ 0u ].TransferErrorCallback();
    Ut_Spi_Check_XferEnd( SPI_XFER_ERROR_DMA_TRANSFER );
#endif /* STM32H7RS */
}


/**
 * \brief   SPI6 refuses DMA mode (requests routed by DMAMUX2 to BDMA, not to DMA1 / DMA2).
 *
 * \details SPI6 initialized with DMA data handling, then with ISR data handling and a reception
 *          started in the captured SPI6 interrupt.
 *
 * \par Expected results
 * - DMA: SPI_REQUEST_ERROR, no Dma_Init() call, Spi_Get_PeriphDmaReq() of SPI6 returns error.
 * - ISR: SPI_REQUEST_OK, interrupt registered, dummy frame transmitted and received frame
 *   stored (TXP, RXP), complete callback at EOT.
 */
void Ut_Spi_Dma_Spi6_DmaModeRefused( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    uint8_t * const   rxBuf      = REGMEM_SRAM_PTR( uint8_t, UT_SPI_RX_BUF_OFFSET );
    spi_DataConfig_t  dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );
    spi_Config_t      config     = Ut_Spi_Get_Config();
    const spi_XferRequest_t request = { .TxData = NULL, .RxData = rxBuf, .XferSize = 1u };
    dma_PeriphReqId_t txRequest  = DMA_REQ_MEM2MEM;
    dma_PeriphReqId_t rxRequest  = DMA_REQ_MEM2MEM;

    config.PeriphId   = SPI_PERIPH_6;
    config.ClkSrc     = SPI_CLK_SRC_SPI6_PCLK4;
    config.DataConfig = &dataConfig;

    Ut_Spi_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_DmaInitCnt );
    TEST_ASSERT_EQUAL( SPI_REQUEST_ERROR, Spi_Get_PeriphDmaReq( SPI_PERIPH_6, &txRequest, &rxRequest ) );

    dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_ISR );
    utSpi_Isr  = NULL;
    Ut_Spi_Init( &config );
    TEST_ASSERT_NOT_NULL( utSpi_Isr );

    SPI6->RXDR = 0x5Au;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( SPI_PERIPH_6, &request ) );
    SPI6->SR = SPI_SR_TXP;
    utSpi_Isr();
    SPI6->SR = SPI_SR_RXP;
    utSpi_Isr();
    SPI6->SR = SPI_SR_EOT;
    utSpi_Isr();

    TEST_ASSERT_EQUAL_HEX8( 0x5Au, rxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_CompleteCnt );
#endif /* STM32H7RS */
}

/* =========================== OTHER PERIPHERALS ============================ */

/**
 * \brief   Other SPI peripherals use their own interrupt and DMA error callbacks.
 *
 * \details SPI2 - SPI5 in DMA mode (DMA1 streams 2 / 3, SPI6 has no DMA mode - see
 *          Ut_Spi_Dma_Spi6_DmaModeRefused): captured SPI interrupt without transfer, reception
 *          started, error callbacks of the RX and TX stream called.
 *
 * \par Expected results
 * - DMAMUX1 requests of the peripheral (TX / RX).
 * - Interrupt without transfer: no callback; DMA error of the running transfer: error
 *   callback SPI_XFER_ERROR_DMA_TRANSFER.
 * - Spi_Deinit(): SPI_REQUEST_OK.
 *
 * \note    STM32H7R / H7S (GPDMA): body of the STM32H5 test.
 */
void Ut_Spi_OtherPeriph_OwnInterruptAndDmaCallbacks( void )
{
#if defined(STM32H7RS)
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
        { SPI_PERIPH_2, SPI2, SPI_CLK_SRC_SPI2_PLL1Q, GPDMA_REQ_SPI2_TX, GPDMA_REQ_SPI2_RX },
#endif /* SPI2 */
#ifdef SPI3
        { SPI_PERIPH_3, SPI3, SPI_CLK_SRC_SPI3_PLL1Q, GPDMA_REQ_SPI3_TX, GPDMA_REQ_SPI3_RX },
#endif /* SPI3 */
#ifdef SPI4
        { SPI_PERIPH_4, SPI4, SPI_CLK_SRC_SPI4_PCLK2, GPDMA_REQ_SPI4_TX, GPDMA_REQ_SPI4_RX },
#endif /* SPI4 */
#ifdef SPI5
        { SPI_PERIPH_5, SPI5, SPI_CLK_SRC_SPI5_PCLK2, GPDMA_REQ_SPI5_TX, GPDMA_REQ_SPI5_RX },
#endif /* SPI5 */
#ifdef SPI6
        { SPI_PERIPH_6, SPI6, SPI_CLK_SRC_SPI6_PCLK4, GPDMA_REQ_SPI6_TX, GPDMA_REQ_SPI6_RX },
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
#else
    const struct
    {
        spi_PeriphId_t          PeriphId;
        SPI_TypeDef *           PeriphReg;
        spi_ClkSrc_t            ClkSrc;
        dma_PeriphReqId_t       TxReq;
        dma_PeriphReqId_t       RxReq;
    }   periphLut[] =
    {
        { SPI_PERIPH_2, SPI2, SPI_CLK_SRC_SPI2_PLL1Q, DMA_REQ_SPI2_TX, DMA_REQ_SPI2_RX },
        { SPI_PERIPH_3, SPI3, SPI_CLK_SRC_SPI3_PLL1Q, DMA_REQ_SPI3_TX, DMA_REQ_SPI3_RX },
        { SPI_PERIPH_4, SPI4, SPI_CLK_SRC_SPI4_PCLK2, DMA_REQ_SPI4_TX, DMA_REQ_SPI4_RX },
        { SPI_PERIPH_5, SPI5, SPI_CLK_SRC_SPI5_PCLK2, DMA_REQ_SPI5_TX, DMA_REQ_SPI5_RX },
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
        TEST_ASSERT_EQUAL( periphLut[ idx ].TxReq, utSpi_DmaConfig[ 0u ].PeripheralReqId );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RxReq, utSpi_DmaConfig[ 1u ].PeripheralReqId );
        TEST_ASSERT_NOT_NULL( utSpi_Isr );

        periphLut[ idx ].PeriphReg->SR = 0u;
        utSpi_Isr();
        TEST_ASSERT_EQUAL_UINT32( 0u, utSpi_ErrorCnt + utSpi_CompleteCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphLut[ idx ].PeriphId, &request ) );
        periphLut[ idx ].PeriphReg->SR = SPI_SR_SUSP;
        utSpi_DmaConfig[ 1u ].TransferErrorCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utSpi_ErrorCnt );
        TEST_ASSERT_EQUAL( SPI_XFER_ERROR_DMA_TRANSFER, utSpi_LastError );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphLut[ idx ].PeriphId, &request ) );
        utSpi_DmaConfig[ 0u ].TransferErrorCallback();
        TEST_ASSERT_EQUAL_UINT32( 2u, utSpi_ErrorCnt );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( periphLut[ idx ].PeriphId ) );
    }
#endif /* STM32H7RS */
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
#if defined(STM32H7RS)
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
#else
    TEST_IGNORE_MESSAGE( "Classic STM32H7: DMA streams (test of the STM32H7R / H7S GPDMA functionality)" );
#endif /* STM32H7RS */
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
#if defined(STM32H7RS)
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
#else
    TEST_IGNORE_MESSAGE( "Classic STM32H7: DMA streams (test of the STM32H7R / H7S GPDMA functionality)" );
#endif /* STM32H7RS */
}


/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Verifies and resets all mocks (CMock memory is common for all mocks - a single mock
 *        is never reset alone).
 */
static void Ut_Spi_Reset_Mocks( void )
{
    MockRcc_Port_Verify();
    MockNvic_Port_Verify();
    MockGpio_Port_Verify();
    MockDma_Port_Verify();
    MockGpdma_Port_Verify();

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();
    MockGpdma_Port_Destroy();

    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
    MockGpdma_Port_Init();
}


#if defined(STM32H7RS)
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
#else
/**
 * \brief Resets the mocks and accepts any RCC / NVIC / GPIO / DMA call by recording stubs.
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
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_Stub( Ut_Spi_DmaInitStub );
    Dma_Set_TransferActive_Stub( Ut_Spi_DmaActiveStub );
    Dma_Set_TransferInactive_Stub( Ut_Spi_DmaInactiveStub );
    Dma_Set_InterruptActive_Stub( Ut_Spi_DmaIrqOnStub );
    Dma_Set_InterruptInactive_Stub( Ut_Spi_DmaIrqOffStub );
    Dma_Set_TransferErrorIrqActive_Stub( Ut_Spi_DmaTeIrqOnStub );
    Dma_Set_TransferErrorIrqInactive_Stub( Ut_Spi_DmaTeIrqOffStub );
    Dma_Set_TransferErrorIsrHandler_Stub( Ut_Spi_DmaTeIsrStub );
    Dma_Set_PeriphTransferSize_Stub( Ut_Spi_DmaPeriphSizeStub );
    Dma_Set_MemoryTransferSize_Stub( Ut_Spi_DmaMemSizeStub );
    Dma_Set_MemoryAddrIncrement_Stub( Ut_Spi_DmaMemIncStub );
    Dma_Set_MemoryAddr_Stub( Ut_Spi_DmaMemAddrStub );
    Dma_Set_DataCount_Stub( Ut_Spi_DmaDataCountStub );
    Dma_Get_DataCount_Stub( Ut_Spi_DmaRemainingStub );
}
#endif /* STM32H7RS */


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
 * \brief Returns data handling configuration of tests (DMA1 stream 0 TX low priority,
 *        stream 1 RX high priority, test callbacks).
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


#if defined(STM32H7RS)
/**
 * \brief Makes the module forget the GPDMA channels of the previous test.
 *
 * The module keeps the GPDMA channels configured by Spi_Set_DataConfig() after Spi_Deinit()
 * (the channel is reused, only its priority is updated - GPDMA module can not release a single
 * channel). A test which expects Gpdma_Init() would depend on the tests executed before it
 * (all tests in one process). SPI1 is configured with DMA data handling on two GPDMA channels
 * which no test uses (the last two recorded channels), so the following test configures other channels
 * and Gpdma_Init() is called.
 */
static void Ut_Spi_Flush_DmaChannelState( void )
{
    spi_Config_t     config     = Ut_Spi_Get_Config();
    spi_DataConfig_t dataConfig = Ut_Spi_Get_DataConfig( SPI_XFER_MODE_DMA );

    dataConfig.TxDmaChannelId = (spi_DmaChannelId_t)( (uint32_t)UT_SPI_DMA_CHANNELS - 1u );
    dataConfig.RxDmaChannelId = (spi_DmaChannelId_t)( (uint32_t)UT_SPI_DMA_CHANNELS - 2u );
    config.DataConfig         = &dataConfig;

    Ut_Spi_Init( &config );
    (void)Spi_Deinit( UT_SPI_BUS );
}


#endif /* STM32H7RS */

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


#if defined(STM32H7RS)
/** \brief Returns record of GPDMA channel calls */
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_TRUE( GPDMA_PERIPH_CNT > dmaBus );
    TEST_ASSERT_TRUE( UT_SPI_DMA_CHANNELS > (uint32_t)dmaChannel );

    return ( &utSpi_DmaChannel[ dmaBus ][ dmaChannel ] );
}
#else
/** \brief Returns record of DMA stream calls */
static ut_SpiDmaChannel_t * Ut_Spi_Get_DmaChannel( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_TRUE( DMA_PERIPH_CNT > dmaBus );
    TEST_ASSERT_TRUE( UT_SPI_DMA_CHANNELS > (uint32_t)dmaChannel );

    return ( &utSpi_DmaChannel[ dmaBus ][ dmaChannel ] );
}
#endif /* STM32H7RS */


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
        /* Record buffer full */
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


#if defined(STM32H7RS)
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
#else
/** \brief Dma_Init() stub - stores the configuration */
static dma_RequestState_t Ut_Spi_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( dmaConfig );

    if( UT_SPI_DMA_CFG_CNT > utSpi_DmaInitCnt )
    {
        utSpi_DmaConfig[ utSpi_DmaInitCnt ] = *dmaConfig;
    }
    else
    {
        /* Record buffer full */
    }

    utSpi_DmaInitCnt++;

    return ( utSpi_DmaInitState );
}
#endif /* STM32H7RS */


#if defined(STM32H7RS)
/** \brief Gpdma_Set_ChannelActive() stub */
static gpdma_RequestState_t Ut_Spi_DmaActiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->ActiveCnt++;
    return ( GPDMA_REQUEST_OK );
}
#else
/** \brief Dma_Set_TransferActive() stub */
static dma_RequestState_t Ut_Spi_DmaActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->ActiveCnt++;
    return ( DMA_REQUEST_OK );
}
#endif /* STM32H7RS */


#if defined(STM32H7RS)
/** \brief Gpdma_Set_ChannelInactive() stub */
static gpdma_RequestState_t Ut_Spi_DmaInactiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->InactiveCnt++;
    return ( GPDMA_REQUEST_OK );
}
#else
/** \brief Dma_Set_TransferInactive() stub */
static dma_RequestState_t Ut_Spi_DmaInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->InactiveCnt++;
    return ( DMA_REQUEST_OK );
}
#endif /* STM32H7RS */


#if defined(STM32H7RS)
/** \brief Gpdma_Set_InterruptActive() stub */
static gpdma_RequestState_t Ut_Spi_DmaIrqOnStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOnCnt++;
    return ( GPDMA_REQUEST_OK );
}
#else
/** \brief Dma_Set_InterruptActive() stub */
static dma_RequestState_t Ut_Spi_DmaIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOnCnt++;
    return ( DMA_REQUEST_OK );
}
#endif /* STM32H7RS */


#if defined(STM32H7RS)
/** \brief Gpdma_Set_InterruptInactive() stub - returns \ref utSpi_DmaIrqOffState */
static gpdma_RequestState_t Ut_Spi_DmaIrqOffStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOffCnt++;
    return ( utSpi_DmaIrqOffState );
}
#else
/** \brief Dma_Set_InterruptInactive() stub - returns \ref utSpi_DmaIrqOffState */
static dma_RequestState_t Ut_Spi_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->IrqOffCnt++;
    return ( utSpi_DmaIrqOffState );
}
#endif /* STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_TransferErrorIrqActive() stub */
static dma_RequestState_t Ut_Spi_DmaTeIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->TeIrqOnCnt++;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_TransferErrorIrqInactive() stub */
static dma_RequestState_t Ut_Spi_DmaTeIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->TeIrqOffCnt++;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_TransferErrorIsrHandler() stub - counts removals of the handler */
static dma_RequestState_t Ut_Spi_DmaTeIsrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_IsrCallback irqHandler, int callCnt )
{
    (void)callCnt;

    if( NULL == irqHandler )
    {
        Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->TeIsrNullCnt++;
    }
    else
    {
        /* Handler registered */
    }

    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_PeriphTransferSize() stub */
static dma_RequestState_t Ut_Spi_DmaPeriphSizeStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferSize_t periphTransferSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->PeriphSize = periphTransferSize;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_MemoryTransferSize() stub */
static dma_RequestState_t Ut_Spi_DmaMemSizeStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferSize_t memoryTransferSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->MemSize = memoryTransferSize;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_MemoryAddrIncrement() stub */
static dma_RequestState_t Ut_Spi_DmaMemIncStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddrInc_t memoryAddrInc, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->MemInc = memoryAddrInc;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_MemoryAddr() stub */
static dma_RequestState_t Ut_Spi_DmaMemAddrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->MemAddr = memoryAddr;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/** \brief Dma_Set_DataCount() stub */
static dma_RequestState_t Ut_Spi_DmaDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DataCount = dataCount;
    return ( DMA_REQUEST_OK );
}
#endif /* !STM32H7RS */


#if defined(STM32H7RS)
/** \brief Gpdma_Set_Priority() stub */
static gpdma_RequestState_t Ut_Spi_DmaPrioStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_Priority_t channelPrio, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->PrioCnt++;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->Prio = channelPrio;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_SourceAddr() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_SrcAddr_t sourceAddr, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcAddr = sourceAddr;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_DestinationAddr() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DstAddr_t destAddr, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstAddr = destAddr;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_SourceAddrMode() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcModeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t srcAddrMode, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcMode = srcAddrMode;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_DestinationAddrMode() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstModeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_AddrMode_t destAddrMode, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstMode = destAddrMode;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_SourceDataSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaSrcSizeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t srcDataSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->SrcSize = srcDataSize;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_DestinationDataSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaDstSizeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DataSize_t destDataSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->DstSize = destDataSize;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Set_BlockSize() stub */
static gpdma_RequestState_t Ut_Spi_DmaBlockStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t blockSize, int callCnt )
{
    (void)callCnt;
    Ut_Spi_Get_DmaChannel( dmaBus, dmaChannel )->BlockSize = blockSize;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */

#if defined(STM32H7RS)
/** \brief Gpdma_Get_BlockSize() stub - returns \ref utSpi_DmaRemaining */
static gpdma_RequestState_t Ut_Spi_DmaRemainingStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t * const blockSize, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    *blockSize = utSpi_DmaRemaining;

    return ( GPDMA_REQUEST_OK );
}
#else
/** \brief Dma_Get_DataCount() stub - returns \ref utSpi_DmaRemaining */
static dma_RequestState_t Ut_Spi_DmaRemainingStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    *dataCount = utSpi_DmaRemaining;

    return ( DMA_REQUEST_OK );
}
#endif /* STM32H7RS */


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
