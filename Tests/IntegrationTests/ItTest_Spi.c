/**
 * \author Mr.Nobody
 * \file ItTest_Spi.c
 * \ingroup Spi
 * \brief Integration tests of Serial Peripheral Interface (SPI) module on target.
 *
 * Spi module runs on the MCU together with real RCC, NVIC, GPIO and DMA modules
 * and hardware. Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): clock activation, real bus frequency, data transfers by
 * polling, interrupts and DMA (FIFO, DMA streams), frames of 4 - 16 bits, end of
 * transfer detection, CRC calculation and check (8 / 16 bits), overrun, NSS
 * output / pulse and the stop of master half-duplex reception.
 *
 * Boards (named by the MCU as the detection of the connected boards does):
 * - STM32F745xG / STM32F746xG - 32F746GDISCOVERY (STM32F746NG); the detection
 *   names every board with ID 0x449 and 1 MB flash STM32F745xG
 * - STM32F722xE, STM32F756xG, STM32F765xI / STM32F767xI - NUCLEO-F722ZE,
 *   NUCLEO-F756ZG (board file override with the name STM32F756xG), NUCLEO-F767ZI
 *   (Nucleo-144 boards, common pinout)
 *
 * SPI5 (master, APB2, DMA2 stream 4 / 3 channel 2) and SPI2 (slave, APB1, DMA1
 * stream 4 / 3 channel 0) are wired together. The slave uses software NSS.
 * - 32F746GDISCOVERY (Arduino connector): SCK PF7 (A4) - PI1 (D13), MISO PF8 (A3) -
 *   PB14 (D12), MOSI PF9 (A2) - PB15 (D11); PI1 also drives LED LD1 (it blinks with
 *   the clock). Master NSS PF6 (A5).
 * - Nucleo-144: SCK PF7 - PB10, MISO PF8 - PC2, MOSI PF9 - PC3. Master NSS PF6.
 * The slave transfer is started before the master transfer. Master transfers
 * without gaps between frames (simplex / half-duplex) are received by the slave
 * by DMA - interrupts of both peripherals would share one CPU.
 *
 * Half-duplex: the bidirectional line of the master is MOSI (PF9), of the slave
 * MISO (wired to PF8) - master half-duplex transmission is received by slave
 * simplex reception, slave half-duplex transmission by master full-duplex
 * reception. Data of master half-duplex reception are not driven by anyone - the
 * slave counts the frames clocked by the master.
 *
 * \note The host halts the core for a few milliseconds on every mailbox access
 *       (probe-rs). Peripherals keep running - transfers of the master by DMA
 *       and the slave by interrupts / DMA are not affected, polled transfers are
 *       clocked by the master frame by frame.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Spi_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* SPI clock state verification   */
#include "Stm32_gpio.h"                     /* NSS pin level                  */
/* ============================= TYPEDEFS =================================== */

/** \brief Counters of callbacks of one peripheral */
typedef struct
{
    volatile uint32_t          CompleteCnt; /**< Count of transfer complete callbacks */
    volatile uint32_t          ErrorCnt;    /**< Count of error callbacks             */
    volatile spi_XferErrorId_t LastError;   /**< Error of the last error callback     */
}   itSpi_Events_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static spi_Config_t It_Spi_Get_Config       ( spi_PeriphId_t periphId );
static void         It_Spi_Init             ( spi_Config_t * const config, spi_XferMode_t xferMode );
static void         It_Spi_Start            ( spi_PeriphId_t periphId, const uint8_t * const txData, uint8_t * const rxData, uint16_t xferSize );
static void         It_Spi_Wait             ( spi_PeriphId_t periphId );
static void         It_Spi_ExpectComplete   ( spi_PeriphId_t periphId, uint32_t completeCnt );
static void         It_Spi_Check_Frames     ( const uint8_t * const txData, const uint8_t * const rxData, uint32_t frameCnt, uint32_t frameBytes, uint32_t frameMask );
static void         It_Spi_Delay            ( uint32_t loopCnt );
static void         It_Spi_MasterComplete   ( void );
static void         It_Spi_MasterError      ( spi_XferErrorId_t errorId );
static void         It_Spi_SlaveComplete    ( void );
static void         It_Spi_SlaveError       ( spi_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32F745xG) || \
    defined(IT_BOARD_STM32F746xG)

    /* 32F746GDISCOVERY */

    /** SPI2 slave (APB1): SCK PI1 (D13, LED LD1), MISO PB14 (D12), MOSI PB15 (D11) (AF5) */
    #define IT_SPI_SLAVE_SCK                ( SPI_SCK_PIN_SPI2_PI1 )
    #define IT_SPI_SLAVE_MISO               ( SPI_MISO_PIN_SPI2_PB14 )
    #define IT_SPI_SLAVE_MOSI               ( SPI_MOSI_PIN_SPI2_PB15 )

#elif defined(IT_BOARD_STM32F722xE) || \
      defined(IT_BOARD_STM32F756xG) || \
      defined(IT_BOARD_STM32F765xI) || \
      defined(IT_BOARD_STM32F767xI)

    /* NUCLEO-F722ZE / NUCLEO-F756ZG / NUCLEO-F767ZI (Nucleo-144) */

    /** SPI2 slave (APB1): SCK PB10, MISO PC2, MOSI PC3 (AF5) */
    #define IT_SPI_SLAVE_SCK                ( SPI_SCK_PIN_SPI2_PB10 )
    #define IT_SPI_SLAVE_MISO               ( SPI_MISO_PIN_SPI2_PC2 )
    #define IT_SPI_SLAVE_MOSI               ( SPI_MOSI_PIN_SPI2_PC3 )

#else
    #error "Board of Spi integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** SPI5 master (APB2): SCK PF7, MISO PF8, MOSI PF9, NSS PF6 (AF5) - Arduino A4 / A3 / A2 / A5 of
 *  32F746GDISCOVERY, Zio / morpho connectors of Nucleo-144 */
#define IT_SPI_MASTER                       ( SPI_PERIPH_5 )
#define IT_SPI_MASTER_RCC                   ( RCC_PERIPH_SPI5 )
#define IT_SPI_MASTER_SCK                   ( SPI_SCK_PIN_SPI5_PF7 )
#define IT_SPI_MASTER_MISO                  ( SPI_MISO_PIN_SPI5_PF8 )
#define IT_SPI_MASTER_MOSI                  ( SPI_MOSI_PIN_SPI5_PF9 )
#define IT_SPI_MASTER_NSS                   ( SPI_NSS_PIN_SPI5_PF6 )
#define IT_SPI_MASTER_NSS_PORT              ( GPIOF )
#define IT_SPI_MASTER_NSS_PIN               ( 6u )

/** SPI5 DMA streams (DMA2 stream 4 TX / stream 3 RX, channel 2) */
#define IT_SPI_MASTER_DMA_TX                ( SPI_TX_DMA_SPI5_DMA2_STREAM4 )
#define IT_SPI_MASTER_DMA_RX                ( SPI_RX_DMA_SPI5_DMA2_STREAM3 )

/** SPI2 slave */
#define IT_SPI_SLAVE                        ( SPI_PERIPH_2 )

/** SPI2 DMA streams (DMA1 stream 4 TX / stream 3 RX, channel 0) */
#define IT_SPI_SLAVE_DMA_TX                 ( SPI_TX_DMA_SPI2_DMA1_STREAM4 )
#define IT_SPI_SLAVE_DMA_RX                 ( SPI_RX_DMA_SPI2_DMA1_STREAM3 )

/** Bus frequency of polled / interrupt transfers [Hz] */
#define IT_SPI_BUS_FREQ_HZ                  ( 1000000u )

/** Bus frequency of DMA transfers [Hz] */
#define IT_SPI_FAST_FREQ_HZ                 ( 6000000u )

/** Bus frequency of polled master half-duplex reception [Hz] (frame longer than Spi_Task()), not lower
 *  than the lowest bus frequency (APB clock / 256 = 421.9 kHz at 108 MHz of APB2) */
#define IT_SPI_SLOW_FREQ_HZ                 ( 500000u )

/** Size of transfer buffers (bytes) */
#define IT_SPI_BUFFER_SIZE                  ( 32u )

/** Count of frames of half-duplex reception tests */
#define IT_SPI_HALF_RX_FRAMES               ( 3u )

/** Count of frames of the overrun test (receive FIFO holds 4 frames of 8 bits - the 5th frame overruns) */
#define IT_SPI_OVERRUN_FRAMES               ( 5u )

/** CRC-8 polynomials x^8 + x^2 + x + 1 / x^8 + x^5 + x^4 + 1 */
#define IT_SPI_CRC8_POLY                    ( 0x07u )
#define IT_SPI_CRC8_POLY_OTHER              ( 0x31u )

/** CRC-16 polynomial x^16 + x^15 + x^2 + 1 */
#define IT_SPI_CRC16_POLY                   ( 0x8005u )

/** Significant bits of 12-bit and 5-bit frames */
#define IT_SPI_MASK_12BIT                   ( 0x0FFFu )
#define IT_SPI_MASK_5BIT                    ( 0x001Fu )

/** Maximal count of polling loops until the event occurs (> 100 ms) */
#define IT_SPI_WAIT_LOOPS                   ( 2000000u )

/** Busy loops longer than the transfer of several frames */
#define IT_SPI_IDLE_LOOPS                   ( 50000u )

/** Interrupt priority of the tests */
#define IT_SPI_IRQ_PRIO                     ( 5u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configurations of master and slave */
static spi_DataConfig_t     itSpi_MasterData;
static spi_DataConfig_t     itSpi_SlaveData;

/** Callback counters of master and slave */
static itSpi_Events_t       itSpi_Master;
static itSpi_Events_t       itSpi_Slave;

/** Buffers (aligned for 16-bit DMA frames) */
static uint8_t              itSpi_MasterTx[ IT_SPI_BUFFER_SIZE ] __attribute__(( aligned( 4 ) ));
static uint8_t              itSpi_MasterRx[ IT_SPI_BUFFER_SIZE ] __attribute__(( aligned( 4 ) ));
static uint8_t              itSpi_SlaveTx[ IT_SPI_BUFFER_SIZE ]  __attribute__(( aligned( 4 ) ));
static uint8_t              itSpi_SlaveRx[ IT_SPI_BUFFER_SIZE ]  __attribute__(( aligned( 4 ) ));

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    for( uint32_t idx = 0u; IT_SPI_BUFFER_SIZE > idx; idx++ )
    {
        itSpi_MasterTx[ idx ] = (uint8_t)( 0x5Au + ( idx * 7u ) );
        itSpi_SlaveTx[ idx ]  = (uint8_t)( 0xC3u - ( idx * 13u ) );
        itSpi_MasterRx[ idx ] = 0u;
        itSpi_SlaveRx[ idx ]  = 0u;
    }

    itSpi_Master = (itSpi_Events_t){ 0u, 0u, SPI_XFER_ERROR_NONE };
    itSpi_Slave  = (itSpi_Events_t){ 0u, 0u, SPI_XFER_ERROR_NONE };
}


void tearDown( void )
{
    (void)Spi_Deinit( IT_SPI_MASTER );
    (void)Spi_Deinit( IT_SPI_SLAVE );
}

/* =============================== TESTS ==================================== */

/*----------------------------- Initialization -------------------------------*/

/**
 * \brief   Spi_Init() activates clock and configures the bus frequency.
 *
 * \details SPI5 clock is inactive after reset. 1 MHz required.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, clock active, peripheral disabled until a transfer starts.
 * - Bus frequency is the closest lower prescaler frequency (> 1/2 of the requirement).
 */
void It_Spi_Init_ClockActivatedAndBusFreqSet( void )
{
    spi_Config_t        config      = It_Spi_Get_Config( IT_SPI_MASTER );
    rcc_FunctionState_t clockState  = RCC_FUNCTION_ACTIVE;
    spi_FlagState_t     periphState = SPI_FLAG_ACTIVE;
    spi_FreqHz_t        busFreq     = 0u;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_SPI_MASTER_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clockState );

    It_Spi_Init( &config, SPI_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_SPI_MASTER_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_ACTIVE, clockState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_PeriphState( IT_SPI_MASTER, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_FLAG_INACTIVE, periphState );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_BusFreq( IT_SPI_MASTER, &busFreq ) );
    TEST_ASSERT_LESS_OR_EQUAL_UINT32( IT_SPI_BUS_FREQ_HZ, busFreq );
    TEST_ASSERT_GREATER_THAN_UINT32( IT_SPI_BUS_FREQ_HZ / 2u, busFreq );
}


/**
 * \brief   Spi_Deinit() deactivates clock of the peripheral.
 *
 * \par Expected results
 * - SPI_REQUEST_OK, clock inactive.
 */
void It_Spi_Deinit_ClockDeactivated( void )
{
    spi_Config_t        config     = It_Spi_Get_Config( IT_SPI_MASTER );
    rcc_FunctionState_t clockState = RCC_FUNCTION_ACTIVE;

    It_Spi_Init( &config, SPI_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Deinit( IT_SPI_MASTER ) );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_SPI_MASTER_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clockState );
}

/*--------------------------------- Polling ----------------------------------*/

/**
 * \brief   Polled full-duplex transfer exchanges data between master and slave.
 *
 * \details Both transfers are moved by Spi_Task(), 16 frames, mode 0. The slave preloads
 *          its first frames by one Spi_Task() call before the master starts.
 *
 * \par Expected results
 * - Master received slave data and vice versa, both transfers complete without error.
 */
void It_Spi_Poll_FullDuplex_DataExchanged( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    It_Spi_Init( &master, SPI_XFER_MODE_POLL );
    It_Spi_Init( &slave, SPI_XFER_MODE_POLL );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 16u );
    Spi_Task();
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 16u );

    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 16u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 16u );
}


/**
 * \brief   Slave which does not read received frames reports overrun.
 *
 * \details Master transfers 5 frames by interrupts (one more than the receive FIFO holds), polled
 *          slave transfer is not serviced (Spi_Task() is not called) until the master finished.
 *
 * \par Expected results
 * - Master completes, slave reports SPI_XFER_ERROR_OVERRUN with the next Spi_Task() call.
 */
void It_Spi_Poll_SlaveNotServiced_OverrunReported( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_ACTIVE;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_POLL );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, IT_SPI_OVERRUN_FRAMES );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, NULL, IT_SPI_OVERRUN_FRAMES );

    for( uint32_t loopCnt = 0u;
         ( IT_SPI_WAIT_LOOPS > loopCnt ) &&
         ( SPI_FUNCTION_INACTIVE != xferState );
         loopCnt++ )
    {
        (void)Spi_Get_XferState( IT_SPI_MASTER, &xferState );
    }

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );

    Spi_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, itSpi_Slave.ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_OVERRUN, itSpi_Slave.LastError );
}


/**
 * \brief   Polled master half-duplex reception clocks exactly the requested frames.
 *
 * \details Master half-duplex reception of 3 frames (slow clock, Spi_Task() loop), slave
 *          simplex reception counts the clocked frames: request of 3 frames completes, request
 *          of 4 frames stays running.
 *
 * \par Expected results
 * - Master completes both transfers without error, slave completes the 3 frame request and
 *   its 4 frame request is still running afterwards.
 */
void It_Spi_Poll_MasterHalfDuplexRx_ExactFrameCount( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_INACTIVE;

    master.Direction = SPI_DIRECTION_HALF_DUPLEX;
    master.BusFreq   = IT_SPI_SLOW_FREQ_HZ;
    slave.Direction  = SPI_DIRECTION_SIMPLEX_RX;

    It_Spi_Init( &master, SPI_XFER_MODE_POLL );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, IT_SPI_HALF_RX_FRAMES );
    It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, IT_SPI_HALF_RX_FRAMES );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, IT_SPI_HALF_RX_FRAMES + 1u );
    It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, IT_SPI_HALF_RX_FRAMES );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 2u );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( IT_SPI_SLAVE, &xferState ) );
    TEST_ASSERT_EQUAL_MESSAGE( SPI_FUNCTION_ACTIVE, xferState, "Master clocked more frames" );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( IT_SPI_SLAVE ) );
}

/*-------------------------------- Interrupts --------------------------------*/

/**
 * \brief   Interrupt driven full-duplex transfer of 16-bit frames, LSB first, mode 1.
 *
 * \par Expected results
 * - 8 frames exchanged in both directions, both transfers complete without error.
 */
void It_Spi_Isr_FullDuplex16BitLsbFirst_DataExchanged( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.DataSize  = SPI_DATA_SIZE_16BIT;
    master.BitOrder  = SPI_BIT_ORDER_LSB_FIRST;
    master.ClockMode = SPI_CLOCK_MODE_1;
    slave.DataSize   = SPI_DATA_SIZE_16BIT;
    slave.BitOrder   = SPI_BIT_ORDER_LSB_FIRST;
    slave.ClockMode  = SPI_CLOCK_MODE_1;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_ISR );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 16u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 16u );
}


/**
 * \brief   Master simplex transmission (bidirectional output) to slave simplex reception
 *          (receive only) and master simplex reception from slave simplex transmission.
 *
 * \par Expected results
 * - Slave received master data, master received slave data, all transfers complete.
 */
void It_Spi_Isr_Simplex_DataTransferred( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.Direction = SPI_DIRECTION_SIMPLEX_TX;
    slave.Direction  = SPI_DIRECTION_SIMPLEX_RX;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, 12u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, NULL, 12u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 12u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( IT_SPI_MASTER, SPI_DIRECTION_SIMPLEX_RX ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_Direction( IT_SPI_SLAVE, SPI_DIRECTION_SIMPLEX_TX ) );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, NULL, 12u );
    It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, 12u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 12u );
    It_Spi_ExpectComplete( IT_SPI_MASTER, 2u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 2u );
}


/**
 * \brief   Master half-duplex transmission is received by slave simplex reception.
 *
 * \par Expected results
 * - Slave received master data, both transfers complete.
 */
void It_Spi_Isr_MasterHalfDuplexTx_SlaveReceives( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.Direction = SPI_DIRECTION_HALF_DUPLEX;
    slave.Direction  = SPI_DIRECTION_SIMPLEX_RX;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, 10u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, NULL, 10u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 10u );
}


/**
 * \brief   Interrupt driven master half-duplex reception clocks exactly the requested frames.
 *
 * \details Master half-duplex reception of 1 and 3 frames, slave simplex reception counts the
 *          clocked frames (request one frame longer stays running).
 *
 * \par Expected results
 * - Master completes without error, slave requests of the same size complete, longer slave
 *   request is still running.
 */
void It_Spi_Isr_MasterHalfDuplexRx_ExactFrameCount( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_INACTIVE;

    master.Direction = SPI_DIRECTION_HALF_DUPLEX;
    slave.Direction  = SPI_DIRECTION_SIMPLEX_RX;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    for( uint16_t frameCnt = 1u; IT_SPI_HALF_RX_FRAMES >= frameCnt; frameCnt += 2u )
    {
        It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, frameCnt );
        It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, frameCnt );
        It_Spi_Wait( IT_SPI_MASTER );
        It_Spi_Wait( IT_SPI_SLAVE );

        It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, (uint16_t)( frameCnt + 1u ) );
        It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, frameCnt );
        It_Spi_Wait( IT_SPI_MASTER );
        It_Spi_Delay( IT_SPI_IDLE_LOOPS );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( IT_SPI_SLAVE, &xferState ) );
        TEST_ASSERT_EQUAL_MESSAGE( SPI_FUNCTION_ACTIVE, xferState, "Master clocked more frames" );
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( IT_SPI_SLAVE ) );
    }

    It_Spi_ExpectComplete( IT_SPI_MASTER, 4u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 2u );
}


/**
 * \brief   Slave transfer without master is aborted by Spi_Set_XferStop().
 *
 * \details Master is not initialized - no clock is received, the slave transfer keeps running.
 *
 * \par Expected results
 * - Transfer state stays active, Spi_Set_XferStop() ends it without callback and disables
 *   the peripheral.
 */
void It_Spi_Isr_SlaveWithoutMaster_StoppedByApplication( void )
{
    spi_Config_t        slave       = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState   = SPI_FUNCTION_INACTIVE;
    spi_FlagState_t     periphState = SPI_FLAG_ACTIVE;

    It_Spi_Init( &slave, SPI_XFER_MODE_ISR );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 4u );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( IT_SPI_SLAVE, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_ACTIVE, xferState );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( IT_SPI_SLAVE ) );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( IT_SPI_SLAVE, &xferState ) );
    TEST_ASSERT_EQUAL( SPI_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_PeriphState( IT_SPI_SLAVE, &periphState ) );
    TEST_ASSERT_EQUAL( SPI_FLAG_INACTIVE, periphState );
    TEST_ASSERT_EQUAL_UINT32( 0u, itSpi_Slave.CompleteCnt + itSpi_Slave.ErrorCnt );
}

/*----------------------------------- DMA ------------------------------------*/

/**
 * \brief   DMA full-duplex transfer at higher bus frequency.
 *
 * \details 32 frames, both master and slave by DMA, twice (channels are re-armed).
 *
 * \par Expected results
 * - Data exchanged in both directions, all transfers complete without error.
 */
void It_Spi_Dma_FullDuplex_DataExchanged( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.BusFreq = IT_SPI_FAST_FREQ_HZ;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    for( uint32_t xferIdx = 0u; 2u > xferIdx; xferIdx++ )
    {
        It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );
        It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, IT_SPI_BUFFER_SIZE );
        It_Spi_Wait( IT_SPI_MASTER );
        It_Spi_Wait( IT_SPI_SLAVE );

        TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, IT_SPI_BUFFER_SIZE );
        TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );

        for( uint32_t idx = 0u; IT_SPI_BUFFER_SIZE > idx; idx++ )
        {
            itSpi_MasterTx[ idx ] = (uint8_t)~itSpi_MasterTx[ idx ];
            itSpi_SlaveTx[ idx ]  = (uint8_t)~itSpi_SlaveTx[ idx ];
            itSpi_MasterRx[ idx ] = 0u;
            itSpi_SlaveRx[ idx ]  = 0u;
        }
    }

    It_Spi_ExpectComplete( IT_SPI_MASTER, 2u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 2u );
}


/**
 * \brief   Hardware CRC is transmitted after the data and checked by the receiver.
 *
 * \details CRC-8 on both sides, master and slave by DMA (CRC phase by hardware), 8 frames.
 *          Second exchange with different polynomial of the slave. CRC phase of polled /
 *          interrupt transfers is verified by unit tests - the CRC frame follows the last data
 *          frame without gap, end of transfer of one peripheral (CRC frame busy-wait) would
 *          block the other one on the same CPU.
 *
 * \par Expected results
 * - Same polynomial: both transfers complete, data exchanged, transmit CRC of each side equals
 *   receive CRC of the other side.
 * - Different polynomial: both sides report SPI_XFER_ERROR_CRC.
 */
void It_Spi_Dma_Crc_CheckedByReceiver( void )
{
    spi_Config_t    master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t    slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_CrcConfig_t crcConfig = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_8BIT, .Polynomial = IT_SPI_CRC8_POLY, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    spi_CrcValue_t  masterTx  = 0u;
    spi_CrcValue_t  masterRx  = 0u;
    spi_CrcValue_t  slaveTx   = 0u;
    spi_CrcValue_t  slaveRx   = 0u;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( IT_SPI_MASTER, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( IT_SPI_SLAVE, &crcConfig ) );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 8u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 8u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( IT_SPI_MASTER, &masterTx, &masterRx ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( IT_SPI_SLAVE, &slaveTx, &slaveRx ) );
    TEST_ASSERT_EQUAL_HEX32( masterTx, slaveRx );
    TEST_ASSERT_EQUAL_HEX32( slaveTx, masterRx );

    crcConfig.Polynomial = IT_SPI_CRC8_POLY_OTHER;
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( IT_SPI_SLAVE, &crcConfig ) );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, itSpi_Master.ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_CRC, itSpi_Master.LastError );
    TEST_ASSERT_EQUAL_UINT32( 1u, itSpi_Slave.ErrorCnt );
    TEST_ASSERT_EQUAL( SPI_XFER_ERROR_CRC, itSpi_Slave.LastError );
}


/**
 * \brief   Slave half-duplex transmission (DMA) is received by master full-duplex reception.
 *
 * \details Master transfers continuously by DMA (no gap between frames), slave 16-bit frames.
 *
 * \par Expected results
 * - Master received slave data, both transfers complete.
 */
void It_Spi_Dma_SlaveHalfDuplexTx16Bit_MasterReceives( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.DataSize = SPI_DATA_SIZE_16BIT;
    slave.DataSize  = SPI_DATA_SIZE_16BIT;
    slave.Direction = SPI_DIRECTION_HALF_DUPLEX;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, NULL, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 16u );
}


/**
 * \brief   DMA master half-duplex reception clocks exactly the requested frames.
 *
 * \details 3 frames (DMA 2 frames + last frame by interrupt) and 1 frame (interrupt only),
 *          slave simplex reception by DMA counts the frames.
 *
 * \par Expected results
 * - Master completes without error, slave requests of the same size complete, longer slave
 *   request is still running.
 */
void It_Spi_Dma_MasterHalfDuplexRx_ExactFrameCount( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_INACTIVE;

    master.Direction = SPI_DIRECTION_HALF_DUPLEX;
    slave.Direction  = SPI_DIRECTION_SIMPLEX_RX;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    for( uint16_t frameCnt = 1u; IT_SPI_HALF_RX_FRAMES >= frameCnt; frameCnt += 2u )
    {
        It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, frameCnt );
        It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, frameCnt );
        It_Spi_Wait( IT_SPI_MASTER );
        It_Spi_Wait( IT_SPI_SLAVE );

        It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, (uint16_t)( frameCnt + 1u ) );
        It_Spi_Start( IT_SPI_MASTER, NULL, itSpi_MasterRx, frameCnt );
        It_Spi_Wait( IT_SPI_MASTER );
        It_Spi_Delay( IT_SPI_IDLE_LOOPS );

        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_XferState( IT_SPI_SLAVE, &xferState ) );
        TEST_ASSERT_EQUAL_MESSAGE( SPI_FUNCTION_ACTIVE, xferState, "Master clocked more frames" );
        TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStop( IT_SPI_SLAVE ) );
    }

    It_Spi_ExpectComplete( IT_SPI_MASTER, 4u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 2u );
}


/**
 * \brief   Hardware NSS of the master is active (low) during the transfer only.
 *
 * \details Master with hardware NSS output (PF6, pull-up), 32 frames by DMA.
 *
 * \par Expected results
 * - NSS high before the transfer, low while the transfer is running, high afterwards.
 * - Data received by the slave.
 */
void It_Spi_Dma_HardNss_ActiveDuringTransfer( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_ACTIVE;
    uint32_t            nssLow    = 0u;

    master.NssConfig.Mode = SPI_NSS_MODE_HARD;
    master.NssPin         = IT_SPI_MASTER_NSS;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    TEST_ASSERT_BIT_HIGH_MESSAGE( IT_SPI_MASTER_NSS_PIN, IT_SPI_MASTER_NSS_PORT->IDR, "NSS active before transfer" );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, NULL, IT_SPI_BUFFER_SIZE );

    for( uint32_t loopCnt = 0u;
         ( IT_SPI_WAIT_LOOPS > loopCnt ) &&
         ( SPI_FUNCTION_INACTIVE != xferState );
         loopCnt++ )
    {
        if( 0u == ( IT_SPI_MASTER_NSS_PORT->IDR & ( 1u << IT_SPI_MASTER_NSS_PIN ) ) )
        {
            nssLow++;
        }
        else
        {
            /* No action required */
        }

        (void)Spi_Get_XferState( IT_SPI_MASTER, &xferState );
    }

    It_Spi_Wait( IT_SPI_SLAVE );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE( 0u, nssLow, "NSS not active during transfer" );
    TEST_ASSERT_BIT_HIGH_MESSAGE( IT_SPI_MASTER_NSS_PIN, IT_SPI_MASTER_NSS_PORT->IDR, "NSS active after transfer" );
    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );
}

/*---------------------- STM32F7 frame sizes, CRC, NSS -----------------------*/

/**
 * \brief   DMA full-duplex transfer of 12-bit frames (2 bytes per frame, 16-bit DMA accesses).
 *
 * \details 16 frames of 12 bits in both directions by DMA at the higher bus frequency.
 *
 * \par Expected results
 * - Lower 12 bits of every frame are exchanged in both directions, both transfers complete.
 */
void It_Spi_Dma_FullDuplex12Bit_DataExchanged( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.BusFreq  = IT_SPI_FAST_FREQ_HZ;
    master.DataSize = SPI_DATA_SIZE_12BIT;
    slave.DataSize  = SPI_DATA_SIZE_12BIT;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE / 2u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, IT_SPI_BUFFER_SIZE / 2u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    It_Spi_Check_Frames( itSpi_SlaveTx, itSpi_MasterRx, IT_SPI_BUFFER_SIZE / 2u, 2u, IT_SPI_MASK_12BIT );
    It_Spi_Check_Frames( itSpi_MasterTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE / 2u, 2u, IT_SPI_MASK_12BIT );
}


/**
 * \brief   Interrupt driven full-duplex transfer of 5-bit frames (1 byte per frame).
 *
 * \par Expected results
 * - Lower 5 bits of 8 frames are exchanged in both directions, both transfers complete.
 */
void It_Spi_Isr_FullDuplex5Bit_DataExchanged( void )
{
    spi_Config_t master = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t slave  = It_Spi_Get_Config( IT_SPI_SLAVE );

    master.DataSize = SPI_DATA_SIZE_5BIT;
    slave.DataSize  = SPI_DATA_SIZE_5BIT;

    It_Spi_Init( &master, SPI_XFER_MODE_ISR );
    It_Spi_Init( &slave, SPI_XFER_MODE_ISR );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    It_Spi_Check_Frames( itSpi_SlaveTx, itSpi_MasterRx, 8u, 1u, IT_SPI_MASK_5BIT );
    It_Spi_Check_Frames( itSpi_MasterTx, itSpi_SlaveRx, 8u, 1u, IT_SPI_MASK_5BIT );
}


/**
 * \brief   16-bit hardware CRC of 8-bit frames is transferred as two frames and checked.
 *
 * \details CRC-16 (polynomial 0x8005) on both sides, master and slave by DMA, 8 frames. The
 *          received CRC occupies two 8-bit frames of the receive FIFO (read by the module).
 *
 * \par Expected results
 * - Both transfers complete without CRC error, data exchanged, transmit CRC of each side equals
 *   receive CRC of the other side. CRC configuration is read back with 16-bit length.
 */
void It_Spi_Dma_Crc16EightBitFrames_CheckedByReceiver( void )
{
    spi_Config_t    master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t    slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_CrcConfig_t crcConfig = { .State = SPI_FUNCTION_ACTIVE, .Size = SPI_DATA_SIZE_16BIT, .Polynomial = IT_SPI_CRC16_POLY, .InitValue = SPI_CRC_INIT_ALL_ZERO };
    spi_CrcConfig_t readBack;
    spi_CrcValue_t  masterTx  = 0u;
    spi_CrcValue_t  masterRx  = 0u;
    spi_CrcValue_t  slaveTx   = 0u;
    spi_CrcValue_t  slaveRx   = 0u;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( IT_SPI_MASTER, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_CrcConfig( IT_SPI_SLAVE, &crcConfig ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcConfig( IT_SPI_MASTER, &readBack ) );
    TEST_ASSERT_EQUAL( SPI_DATA_SIZE_16BIT, readBack.Size );

    It_Spi_Start( IT_SPI_SLAVE, itSpi_SlaveTx, itSpi_SlaveRx, 8u );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, itSpi_MasterRx, 8u );
    It_Spi_Wait( IT_SPI_MASTER );
    It_Spi_Wait( IT_SPI_SLAVE );

    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_SlaveTx, itSpi_MasterRx, 8u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, 8u );

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( IT_SPI_MASTER, &masterTx, &masterRx ) );
    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Get_CrcValue( IT_SPI_SLAVE, &slaveTx, &slaveRx ) );
    TEST_ASSERT_EQUAL_HEX32( masterTx, slaveRx );
    TEST_ASSERT_EQUAL_HEX32( slaveTx, masterRx );
}


/**
 * \brief   NSS pulse of the master releases NSS between the frames.
 *
 * \details Master with hardware NSS output and NSS pulse (PF6, mode 0), 32 frames by DMA at the
 *          slow bus frequency, NSS level sampled while the transfer is running.
 *
 * \par Expected results
 * - NSS sampled low (frames) and high (pulses between the frames) during the transfer, high
 *   after the transfer.
 * - Data received by the slave, both transfers complete.
 */
void It_Spi_Dma_HardNssPulse_ReleasedBetweenFrames( void )
{
    spi_Config_t        master    = It_Spi_Get_Config( IT_SPI_MASTER );
    spi_Config_t        slave     = It_Spi_Get_Config( IT_SPI_SLAVE );
    spi_FunctionState_t xferState = SPI_FUNCTION_ACTIVE;
    uint32_t            nssLow    = 0u;
    uint32_t            nssPulse  = 0u;

    master.BusFreq         = IT_SPI_SLOW_FREQ_HZ;
    master.NssConfig.Mode  = SPI_NSS_MODE_HARD;
    master.NssConfig.Pulse = SPI_FUNCTION_ACTIVE;
    master.NssPin          = IT_SPI_MASTER_NSS;

    It_Spi_Init( &master, SPI_XFER_MODE_DMA );
    It_Spi_Init( &slave, SPI_XFER_MODE_DMA );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    It_Spi_Start( IT_SPI_SLAVE, NULL, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );
    It_Spi_Start( IT_SPI_MASTER, itSpi_MasterTx, NULL, IT_SPI_BUFFER_SIZE );

    for( uint32_t loopCnt = 0u;
         ( IT_SPI_WAIT_LOOPS > loopCnt ) &&
         ( SPI_FUNCTION_INACTIVE != xferState );
         loopCnt++ )
    {
        const uint32_t nssHigh = IT_SPI_MASTER_NSS_PORT->IDR & ( 1u << IT_SPI_MASTER_NSS_PIN );

        if( 0u == nssHigh )
        {
            nssLow++;
        }
        else if( 0u != nssLow )
        {
            /* NSS released after the first frame */
            nssPulse++;
        }
        else
        {
            /* First frame has not started */
        }

        (void)Spi_Get_XferState( IT_SPI_MASTER, &xferState );
    }

    It_Spi_Wait( IT_SPI_SLAVE );
    It_Spi_Delay( IT_SPI_IDLE_LOOPS );

    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE( 0u, nssLow, "NSS not active during transfer" );
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE( 0u, nssPulse, "NSS not released between frames" );
    TEST_ASSERT_BIT_HIGH_MESSAGE( IT_SPI_MASTER_NSS_PIN, IT_SPI_MASTER_NSS_PORT->IDR, "NSS active after transfer" );
    It_Spi_ExpectComplete( IT_SPI_MASTER, 1u );
    It_Spi_ExpectComplete( IT_SPI_SLAVE, 1u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itSpi_MasterTx, itSpi_SlaveRx, IT_SPI_BUFFER_SIZE );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Returns configuration of the master (SPI5) or the slave (SPI2) with pins of the board
 *        (default configuration: 1 MHz, mode 0, 8-bit, MSB first, full-duplex, software NSS).
 *
 * \param periphId [in]: IT_SPI_MASTER or IT_SPI_SLAVE
 * \return Peripheral configuration
 */
static spi_Config_t It_Spi_Get_Config( spi_PeriphId_t periphId )
{
    spi_Config_t config;

    (void)Spi_Get_DefaultConfig( &config );

    config.PeriphId = periphId;
    config.BusFreq  = IT_SPI_BUS_FREQ_HZ;

    if( IT_SPI_MASTER == periphId )
    {
        config.Mode    = SPI_MODE_MASTER;
        config.SckPin  = IT_SPI_MASTER_SCK;
        config.MisoPin = IT_SPI_MASTER_MISO;
        config.MosiPin = IT_SPI_MASTER_MOSI;
    }
    else
    {
        config.Mode    = SPI_MODE_SLAVE;
        config.SckPin  = IT_SPI_SLAVE_SCK;
        config.MisoPin = IT_SPI_SLAVE_MISO;
        config.MosiPin = IT_SPI_SLAVE_MOSI;
    }

    return ( config );
}


/**
 * \brief Initializes the peripheral of the configuration with data handling of the transfer
 *        mode (DMA channels of the board, callbacks of the master / slave).
 *
 * \param config   [in]: Configuration prepared by \ref It_Spi_Get_Config
 * \param xferMode [in]: Data transfer mode
 */
static void It_Spi_Init( spi_Config_t * const config, spi_XferMode_t xferMode )
{
    spi_DataConfig_t * dataConfig = &itSpi_SlaveData;

    if( IT_SPI_MASTER == config->PeriphId )
    {
        dataConfig = &itSpi_MasterData;
    }
    else
    {
        dataConfig = &itSpi_SlaveData;
    }

    dataConfig->XferMode      = xferMode;
    dataConfig->IrqPriority   = IT_SPI_IRQ_PRIO;
    dataConfig->TxDmaPriority = SPI_DMA_PRIORITY_MEDIUM;
    dataConfig->RxDmaPriority = SPI_DMA_PRIORITY_HIGH;

    if( IT_SPI_MASTER == config->PeriphId )
    {
        dataConfig->TxDma                = IT_SPI_MASTER_DMA_TX;
        dataConfig->RxDma                = IT_SPI_MASTER_DMA_RX;
        dataConfig->XferCompleteCallback = It_Spi_MasterComplete;
        dataConfig->ErrorCallback        = It_Spi_MasterError;
    }
    else
    {
        dataConfig->TxDma                = IT_SPI_SLAVE_DMA_TX;
        dataConfig->RxDma                = IT_SPI_SLAVE_DMA_RX;
        dataConfig->XferCompleteCallback = It_Spi_SlaveComplete;
        dataConfig->ErrorCallback        = It_Spi_SlaveError;
    }

    config->DataConfig = dataConfig;

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Init( config ) );
}


/**
 * \brief Starts transfer of the request.
 *
 * \param periphId [in]: Peripheral
 * \param txData   [in]: Transmitted data or NULL
 * \param rxData  [out]: Received data or NULL
 * \param xferSize [in]: Count of frames
 */
static void It_Spi_Start( spi_PeriphId_t periphId, const uint8_t * const txData, uint8_t * const rxData, uint16_t xferSize )
{
    const spi_XferRequest_t request = { .TxData = txData, .RxData = rxData, .XferSize = xferSize };

    TEST_ASSERT_EQUAL( SPI_REQUEST_OK, Spi_Set_XferStart( periphId, &request ) );
}


/**
 * \brief Waits for the end of the running transfer of the peripheral (Spi_Task() moves polled
 *        transfers of both peripherals).
 *
 * \param periphId [in]: Peripheral
 */
static void It_Spi_Wait( spi_PeriphId_t periphId )
{
    spi_FunctionState_t xferState = SPI_FUNCTION_ACTIVE;

    for( uint32_t loopCnt = 0u;
         ( IT_SPI_WAIT_LOOPS > loopCnt ) &&
         ( SPI_FUNCTION_INACTIVE != xferState );
         loopCnt++ )
    {
        Spi_Task();

        (void)Spi_Get_XferState( periphId, &xferState );
    }

    if( IT_SPI_MASTER == periphId )
    {
        TEST_ASSERT_EQUAL_INT_MESSAGE( SPI_FUNCTION_INACTIVE, xferState, "Master transfer not finished" );
    }
    else
    {
        TEST_ASSERT_EQUAL_INT_MESSAGE( SPI_FUNCTION_INACTIVE, xferState, "Slave transfer not finished" );
    }
}


/**
 * \brief Checks that the transfers of the peripheral completed without error.
 *
 * \param periphId    [in]: Peripheral
 * \param completeCnt [in]: Expected count of complete callbacks
 */
static void It_Spi_ExpectComplete( spi_PeriphId_t periphId, uint32_t completeCnt )
{
    const itSpi_Events_t * events = &itSpi_Slave;

    if( IT_SPI_MASTER == periphId )
    {
        events = &itSpi_Master;
    }
    else
    {
        events = &itSpi_Slave;
    }

    const char * name = "Slave";

    if( IT_SPI_MASTER == periphId )
    {
        name = "Master";
    }
    else
    {
        name = "Slave";
    }

    UNITY_TEST_ASSERT_EQUAL_INT( SPI_XFER_ERROR_NONE, events->LastError, __LINE__, name );
    UNITY_TEST_ASSERT_EQUAL_UINT32( 0u, events->ErrorCnt, __LINE__, name );
    UNITY_TEST_ASSERT_EQUAL_UINT32( completeCnt, events->CompleteCnt, __LINE__, name );
}


/**
 * \brief Checks significant bits of received frames (stored little endian, 1 or 2 bytes per
 *        frame) against the transmitted frames.
 *
 * \param txData     [in]: Transmitted frames
 * \param rxData     [in]: Received frames
 * \param frameCnt   [in]: Count of frames
 * \param frameBytes [in]: Bytes per frame (1 or 2)
 * \param frameMask  [in]: Significant bits of the frame
 */
static void It_Spi_Check_Frames( const uint8_t * const txData, const uint8_t * const rxData, uint32_t frameCnt, uint32_t frameBytes, uint32_t frameMask )
{
    for( uint32_t frameIdx = 0u; frameCnt > frameIdx; frameIdx++ )
    {
        uint32_t txFrame = txData[ frameIdx * frameBytes ];
        uint32_t rxFrame = rxData[ frameIdx * frameBytes ];

        if( 2u == frameBytes )
        {
            txFrame |= (uint32_t)txData[ ( frameIdx * frameBytes ) + 1u ] << 8u;
            rxFrame |= (uint32_t)rxData[ ( frameIdx * frameBytes ) + 1u ] << 8u;
        }
        else
        {
            /* Frame occupies one byte */
        }

        TEST_ASSERT_EQUAL_HEX32_MESSAGE( txFrame & frameMask, rxFrame & frameMask, "Received frame" );
    }
}


/**
 * \brief Busy-wait.
 *
 * \param loopCnt [in]: Count of loops
 */
static void It_Spi_Delay( uint32_t loopCnt )
{
    for( volatile uint32_t loopIdx = 0u; loopCnt > loopIdx; loopIdx++ )
    {
        /* Waiting */
    }
}


/** Master transfer complete callback */
static void It_Spi_MasterComplete( void )
{
    itSpi_Master.CompleteCnt++;
}


/** Master error callback */
static void It_Spi_MasterError( spi_XferErrorId_t errorId )
{
    itSpi_Master.ErrorCnt++;
    itSpi_Master.LastError = errorId;
}


/** Slave transfer complete callback */
static void It_Spi_SlaveComplete( void )
{
    itSpi_Slave.CompleteCnt++;
}


/** Slave error callback */
static void It_Spi_SlaveError( spi_XferErrorId_t errorId )
{
    itSpi_Slave.ErrorCnt++;
    itSpi_Slave.LastError = errorId;
}

