/**
 * \author Mr.Nobody
 * \file Spi_Dma.c
 * \ingroup Spi
 * \brief Spi module DMA data transfer handler
 *
 * SPI_XFER_MODE_DMA - data frames are moved by DMA streams (memory to DR, DR to memory). The
 * stream configured by the application has to be connected to the SPI request (request mapping
 * table of the reference manual), its channel selection (CHSEL) is set by the handler.
 *
 * End of transfer is signaled by the transfer complete interrupt of the receive stream (or the
 * transmit stream if nothing is received) - the transfer is finished by Spi.c
 * (Spi_Set_XferFinish: CRC frame, end of transmission). Errors of the peripheral (OVR, MODF,
 * CRCERR, FRE) are processed in the SPI interrupt, DMA transfer errors in the DMA interrupt.
 *
 * CRC phase is handled by the hardware in DMA mode (CRCNEXT is set after the last DMA frame).
 *
 * Master half-duplex reception: the receive stream moves all frames but the last one, the
 * peripheral is disabled during the last frame and the last frame is received by the SPI
 * interrupt (RXNE).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Spi_Dma.h"                        /* Self include                   */
#include "Spi_Isr.h"                        /* SPI interrupt handler          */
#include "Spi.h"                            /* Module private interface       */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief Direction of DMA transfer of SPI */
typedef enum
{
    SPI_DMA_DIR_TX = 0u, /**< Transmission (memory to DR) */
    SPI_DMA_DIR_RX,      /**< Reception (DR to memory)    */
    SPI_DMA_DIR_CNT      /**< Count of directions         */
}   spi_DmaDir_t;


/** \brief DMA stream connected to SPI request */
typedef struct
{
    dma_PeriphId_t    DmaId;      /**< DMA peripheral                        */
    dma_ChannelId_t   StreamId;   /**< DMA stream                            */
    dma_PeriphReqId_t ChannelSel; /**< Channel selection (request) of stream */
}   spi_DmaStream_t;


/** \brief DMA configuration of one SPI peripheral */
typedef struct
{
    spi_DmaStream_t  TxStream[ 3u ];  /**< Streams of transmit request (last one repeated if fewer exist) */
    spi_DmaStream_t  RxStream[ 3u ];  /**< Streams of receive request (last one repeated if fewer exist)  */
    dma_IsrCallback  TxDoneIsr;       /**< Transmit stream transfer complete handler              */
    dma_IsrCallback  RxDoneIsr;       /**< Receive stream transfer complete handler               */
    dma_IsrCallback  TxErrorIsr;      /**< Transmit stream transfer error handler                 */
    dma_IsrCallback  RxErrorIsr;      /**< Receive stream transfer error handler                  */
}   spi_DmaPeriphConfig_t;


/** \brief Initialized DMA stream of one direction */
typedef struct
{
    spi_FunctionState_t Initialized; /**< DMA stream was initialized for the direction */
    spi_FunctionState_t Armed;       /**< DMA stream moves frames of running transfer  */
    dma_PeriphId_t      DmaId;       /**< Initialized DMA peripheral                   */
    dma_ChannelId_t     StreamId;    /**< Initialized DMA stream                       */
}   spi_DmaChannelState_t;


/** \brief Dummy frame used when a buffer of the request is not set */
typedef uint16_t spi_DmaDummy_t;

/* ======================== FORWARD DECLARATIONS ============================ */

/** Declares DMA interrupt handlers of one SPI peripheral */
#define SPI_DMA_DECLARE_HANDLERS( name )                                        \
    static void Spi_Dma_##name##_TxDone  ( void );                              \
    static void Spi_Dma_##name##_RxDone  ( void );                              \
    static void Spi_Dma_##name##_TxError ( void );                              \
    static void Spi_Dma_##name##_RxError ( void )

/** Defines DMA interrupt handlers of one SPI peripheral */
#define SPI_DMA_DEFINE_HANDLERS( name, periphId )                                                                     \
    static void Spi_Dma_##name##_TxDone( void )  { (void)Spi_Dma_XferDone( periphId, SPI_DMA_DIR_TX ); }              \
    static void Spi_Dma_##name##_RxDone( void )  { (void)Spi_Dma_XferDone( periphId, SPI_DMA_DIR_RX ); }              \
    static void Spi_Dma_##name##_TxError( void ) { (void)Spi_Dma_XferError( periphId ); }                             \
    static void Spi_Dma_##name##_RxError( void ) { (void)Spi_Dma_XferError( periphId ); }

/** DMA interrupt handlers of one SPI peripheral (configuration table entry) */
#define SPI_DMA_ISR_CONFIG( name )                                                                                    \
    .TxDoneIsr = Spi_Dma_##name##_TxDone, .RxDoneIsr  = Spi_Dma_##name##_RxDone,                                      \
    .TxErrorIsr = Spi_Dma_##name##_TxError, .RxErrorIsr = Spi_Dma_##name##_RxError

static spi_RequestState_t Spi_Dma_Get_Stream      ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_DmaCode_t dmaCode, dma_PeriphReqId_t * const channelSel );
static spi_RequestState_t Spi_Dma_Set_ChannelInit ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Set_ChannelOff  ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Check_Buffers   ( const spi_XferContext_t * const xferCtx );
static spi_RequestState_t Spi_Dma_Set_Transfer    ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, dma_DataCount_t dataCount );
static spi_RequestState_t Spi_Dma_Set_Request     ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_FunctionState_t reqState );
static spi_RequestState_t Spi_Dma_Set_Stop        ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_XferDone        ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_XferError       ( spi_PeriphId_t periphId );

#ifdef SPI1
SPI_DMA_DECLARE_HANDLERS( Spi1 );
#endif /* SPI1 */
#ifdef SPI2
SPI_DMA_DECLARE_HANDLERS( Spi2 );
#endif /* SPI2 */
#ifdef SPI3
SPI_DMA_DECLARE_HANDLERS( Spi3 );
#endif /* SPI3 */
#ifdef SPI4
SPI_DMA_DECLARE_HANDLERS( Spi4 );
#endif /* SPI4 */
#ifdef SPI5
SPI_DMA_DECLARE_HANDLERS( Spi5 );
#endif /* SPI5 */
#ifdef SPI6
SPI_DMA_DECLARE_HANDLERS( Spi6 );
#endif /* SPI6 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Count of streams connected to one SPI request (\ref spi_DmaPeriphConfig_t) */
#define SPI_DMA_STREAM_OPTIONS      ( 3u )

/** Buffer bytes of 16-bit frame (buffer alignment) */
#define SPI_DMA_FRAME_BYTES_16BIT   ( 2u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/**
 * \brief DMA streams of SPI requests (RM0090 / RM0383 / RM0390 DMA request mapping, STM32CubeMX
 *        database; the third stream of SPI1_TX / SPI4_RX / SPI5_TX exists on the newer devices only)
 */
static const spi_DmaPeriphConfig_t spi_DmaPeriphConfig[ ] =
{
#ifdef SPI1
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_3, DMA_REQ_CHANNEL_3 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_3 },
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
                    { DMA_PERIPH_2, DMA_STREAM_2, DMA_REQ_CHANNEL_2 } },
#else
                    { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_3 } },
#endif
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_0, DMA_REQ_CHANNEL_3 }, { DMA_PERIPH_2, DMA_STREAM_2, DMA_REQ_CHANNEL_3 }, { DMA_PERIPH_2, DMA_STREAM_2, DMA_REQ_CHANNEL_3 } },
      SPI_DMA_ISR_CONFIG( Spi1 ) },
#endif /* SPI1 */
#ifdef SPI2
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_0 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_0 } },
      SPI_DMA_ISR_CONFIG( Spi2 ) },
#endif /* SPI2 */
#ifdef SPI3
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_5, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_7, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_7, DMA_REQ_CHANNEL_0 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_0, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_2, DMA_REQ_CHANNEL_0 }, { DMA_PERIPH_1, DMA_STREAM_2, DMA_REQ_CHANNEL_0 } },
      SPI_DMA_ISR_CONFIG( Spi3 ) },
#endif /* SPI3 */
#ifdef SPI4
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_1, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_2, DMA_STREAM_4, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_2, DMA_STREAM_4, DMA_REQ_CHANNEL_5 } },
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_0, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_2, DMA_STREAM_3, DMA_REQ_CHANNEL_5 },
#if defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
                    { DMA_PERIPH_2, DMA_STREAM_4, DMA_REQ_CHANNEL_4 } },
#else
                    { DMA_PERIPH_2, DMA_STREAM_3, DMA_REQ_CHANNEL_5 } },
#endif
      SPI_DMA_ISR_CONFIG( Spi4 ) },
#endif /* SPI4 */
#ifdef SPI5
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_4, DMA_REQ_CHANNEL_2 }, { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_7 },
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
                    { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_5 } },
#else
                    { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_7 } },
#endif
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_3, DMA_REQ_CHANNEL_2 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_7 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_7 } },
      SPI_DMA_ISR_CONFIG( Spi5 ) },
#endif /* SPI5 */
#ifdef SPI6
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_1 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_1 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_1 } },
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_1 }, { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_1 }, { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_1 } },
      SPI_DMA_ISR_CONFIG( Spi6 ) },
#endif /* SPI6 */
};

_Static_assert( SPI_PERIPH_CNT == ( sizeof(spi_DmaPeriphConfig) / sizeof(spi_DmaPeriphConfig_t) ), "Spi: spi_DmaPeriphConfig has incorrect size." );


/** \brief Initialized DMA streams per SPI peripheral and direction */
static spi_DmaChannelState_t spi_DmaChannelState[ SPI_PERIPH_CNT ][ SPI_DMA_DIR_CNT ];

/** \brief Direction whose transfer complete ends the running transfer (SPI_DMA_DIR_CNT - none) */
static spi_DmaDir_t spi_DmaEndDir[ SPI_PERIPH_CNT ];

/** \brief Zero frame transmitted when the transmit buffer is not set */
static spi_DmaDummy_t spi_DmaDummyTx = 0u;

/** \brief Frame storage when the receive buffer is not set (frames are discarded) */
static spi_DmaDummy_t spi_DmaDummyRx = 0u;

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data handling configuration - priorities, both streams
 *        are items of the lists \ref spi_TxDma_t / \ref spi_RxDma_t of the SPI peripheral (the
 *        streams of the SPI requests of their direction, transmission and reception never share
 *        a stream)
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState   = SPI_REQUEST_ERROR;
    dma_PeriphReqId_t  channelSel = DMA_REQ_CHANNEL_0;

    if( ( SPI_PERIPH_CNT > periphId   ) &&
        ( SPI_NULL_PTR  != dataConfig )    )
    {
        if( ( (uint32_t)DMA_PRIORITY_CNT > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( (uint32_t)DMA_PRIORITY_CNT > (uint32_t)dataConfig->RxDmaPriority )    )
        {
            retState = Spi_Dma_Get_Stream( periphId, SPI_DMA_DIR_TX, (spi_DmaCode_t)dataConfig->TxDma, &channelSel );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Dma_Get_Stream( periphId, SPI_DMA_DIR_RX, (spi_DmaCode_t)dataConfig->RxDma, &channelSel );
            }
            else
            {
                /* Transmit stream is not connected to the SPI request */
            }
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
 * \brief Initializes DMA data transfer - transmit and receive streams are configured (channel
 *        selection, peripheral address, callbacks), their interrupts are enabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferInit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Dma_Set_ChannelInit( periphId, SPI_DMA_DIR_TX );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Dma_Set_ChannelInit( periphId, SPI_DMA_DIR_RX );
    }
    else
    {
        /* Transmit stream initialization failed */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Dma_XferStop( periphId );
    }
    else
    {
        /* Stream initialization failed */
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer - streams are stopped, their interrupts and callbacks
 *        are released
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferDeinit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    const spi_RequestState_t stopState = Spi_Dma_XferStop( periphId );
    const spi_RequestState_t txState   = Spi_Dma_Set_ChannelOff( periphId, SPI_DMA_DIR_TX );
    const spi_RequestState_t rxState   = Spi_Dma_Set_ChannelOff( periphId, SPI_DMA_DIR_RX );

    if( ( SPI_REQUEST_OK == stopState ) &&
        ( SPI_REQUEST_OK == txState   ) &&
        ( SPI_REQUEST_OK == rxState   )    )
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
 * \brief Starts DMA data transfer before the peripheral is enabled - receive request and stream,
 *        transmit stream (its request is enabled by \ref Spi_Dma_XferRun)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise (also for buffers not aligned to the frame
 *         size) returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferStart( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( SPI_REQUEST_OK == retState )
    {
        spi_DmaEndDir[ periphId ] = SPI_DMA_DIR_CNT;

        retState = Spi_Dma_Check_Buffers( xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    /* Reception (master half-duplex reception: last frame is received by the CPU) */
    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed )    )
    {
        const dma_DataCount_t rxCount = ( SPI_FUNCTION_ACTIVE == xferCtx->RxStopUsed ) ? ( (dma_DataCount_t)xferCtx->Request.XferSize - 1u )
                                                                                         : (dma_DataCount_t)xferCtx->Request.XferSize;

        if( 0u < rxCount )
        {
            retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_RX, SPI_FUNCTION_ACTIVE );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_RX, rxCount );
            }
            else
            {
                /* Receive request could not be enabled */
            }

            spi_DmaEndDir[ periphId ] = SPI_DMA_DIR_RX;
        }
        else
        {
            /* The only frame is received by the CPU */
            xferCtx->CpuRxTail = SPI_FUNCTION_ACTIVE;
        }
    }
    else
    {
        /* Previous step failed or nothing is received */
    }

    /* Transmission (request is enabled after the peripheral - Spi_Dma_XferRun) */
    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed )    )
    {
        retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_TX, (dma_DataCount_t)xferCtx->Request.XferSize );

        if( SPI_DMA_DIR_CNT == spi_DmaEndDir[ periphId ] )
        {
            spi_DmaEndDir[ periphId ] = SPI_DMA_DIR_TX;
        }
        else
        {
            /* End of transfer is signaled by reception */
        }
    }
    else
    {
        /* Previous step failed or nothing is transmitted */
    }

    return ( retState );
}


/**
 * \brief Runs DMA data transfer after the peripheral is enabled - SPI error interrupt (and RXNE
 *        interrupt if the only frame is received by the CPU) and the transmit DMA request are
 *        enabled
 *
 * \note  The transmit request is enabled while the peripheral is enabled - DMA moves the frames
 *        to the data register only when the peripheral can shift them out (otherwise transfer
 *        complete of the transmit stream would precede the transmission). The request is
 *        enabled without read-back - the end of a short transfer (DMA interrupt) disables it
 *        right away.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferRun( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState  = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx   = SPI_NULL_PTR;
    SPI_TypeDef *       periphReg = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_PeriphReg( periphId, &periphReg );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( SPI_REQUEST_OK == retState )
    {
        const spi_IsrItMask_t itMask = ( SPI_FUNCTION_ACTIVE == xferCtx->CpuRxTail ) ? ( SPI_ISR_IT_EVENTS | SPI_ISR_IT_RXNE )
                                                                                      : SPI_ISR_IT_EVENTS;

        retState = Spi_Isr_Set_ItStart( periphId, itMask );

        if( ( SPI_REQUEST_OK      == retState                                            ) &&
            ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ SPI_DMA_DIR_TX ].Armed )    )
        {
            LL_SPI_EnableDMAReq_TX( periphReg );
        }
        else
        {
            /* Interrupt enable failed or nothing is transmitted */
        }
    }
    else
    {
        /* Invalid peripheral identification */
    }

    return ( retState );
}


/**
 * \brief Stops DMA data transfer - SPI interrupts are disabled, DMA streams are stopped and DMA
 *        requests of the peripheral are disabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferStop( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    const spi_RequestState_t itState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_ALL );
    const spi_RequestState_t txState = Spi_Dma_Set_Stop( periphId, SPI_DMA_DIR_TX );
    const spi_RequestState_t rxState = Spi_Dma_Set_Stop( periphId, SPI_DMA_DIR_RX );

    if( ( SPI_REQUEST_OK == itState ) &&
        ( SPI_REQUEST_OK == txState ) &&
        ( SPI_REQUEST_OK == rxState )    )
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
 * \brief Checks that DMA streams moved all frames of the running transfer (remaining data count
 *        of armed streams is zero)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_REQUEST_OK if all frames were moved. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Done( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_PERIPH_CNT > periphId )
    {
        retState = SPI_REQUEST_OK;

        for( spi_DmaDir_t dmaDir = SPI_DMA_DIR_TX; ( SPI_REQUEST_OK == retState ) && ( SPI_DMA_DIR_CNT > dmaDir ); dmaDir ++ )
        {
            const spi_DmaChannelState_t * const chState = &spi_DmaChannelState[ periphId ][ dmaDir ];

            if( SPI_FUNCTION_ACTIVE == chState->Armed )
            {
                dma_DataCount_t          remaining = 1u;
                const dma_RequestState_t dmaState  = Dma_Get_DataCount( chState->DmaId, chState->StreamId, &remaining );

                if( ( DMA_REQUEST_OK == dmaState  ) &&
                    ( 0u             == remaining )    )
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
                /* Direction is not moved by DMA */
            }
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
 * \brief Looks up DMA stream connected to the SPI request
 *
 * The DMA peripheral and the stream are decoded from the item of the DMA stream list, the item has to
 * belong to the SPI peripheral.
 *
 * \param periphId    [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir      [in]: Direction, value from \ref spi_DmaDir_t
 * \param dmaCode     [in]: Item of \ref spi_TxDma_t / \ref spi_RxDma_t (encoded DMA stream)
 * \param channelSel [out]: Pointer to store channel selection of the stream. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the stream is connected to the request. Otherwise
 *         returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Get_Stream( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_DmaCode_t dmaCode, dma_PeriphReqId_t * const channelSel )
{
    spi_RequestState_t retState   = SPI_REQUEST_ERROR;
    const uint32_t     codePeriph = SPI_DMA_BIT_MASK_DECODE_PERIPH( dmaCode );
    const uint32_t     codeDmaId  = SPI_DMA_BIT_MASK_DECODE_DMA( dmaCode );
    const uint32_t     codeStream = SPI_DMA_BIT_MASK_DECODE_STREAM( dmaCode );

    if( ( SPI_PERIPH_CNT  >  periphId           ) &&
        ( SPI_DMA_DIR_CNT >  dmaDir             ) &&
        ( codePeriph      == (uint32_t)periphId ) &&
        ( SPI_NULL_PTR    != channelSel         )    )
    {
        const spi_DmaStream_t * const streams = ( SPI_DMA_DIR_TX == dmaDir ) ? spi_DmaPeriphConfig[ periphId ].TxStream
                                                                            : spi_DmaPeriphConfig[ periphId ].RxStream;

        for( uint32_t streamIdx = 0u; SPI_DMA_STREAM_OPTIONS > streamIdx; streamIdx ++ )
        {
            if( ( (dma_PeriphId_t)codeDmaId   == streams[ streamIdx ].DmaId    ) &&
                ( (dma_ChannelId_t)codeStream == streams[ streamIdx ].StreamId )    )
            {
                *channelSel = streams[ streamIdx ].ChannelSel;
                retState    = SPI_REQUEST_OK;
                break;
            }
            else
            {
                /* Stream does not match, keep searching */
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
 * \brief Initializes DMA stream of one direction (normal mode, peripheral address DR, channel
 *        selection, priority, transfer complete and transfer error callbacks and interrupts)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref spi_DmaDir_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelInit( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState   = SPI_REQUEST_ERROR;
    SPI_TypeDef *       periphReg  = SPI_NULL_PTR;
    spi_XferContext_t * xferCtx    = SPI_NULL_PTR;
    dma_PeriphReqId_t   channelSel = DMA_REQ_CHANNEL_0;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_XferContext( periphId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( SPI_REQUEST_OK == retState ) && ( SPI_DMA_DIR_CNT > dmaDir ) )
    {
        const spi_DmaPeriphConfig_t * const periphConf = &spi_DmaPeriphConfig[ periphId ];
        spi_DmaChannelState_t * const       chState    = &spi_DmaChannelState[ periphId ][ dmaDir ];
        dma_ConfigStruct_t                  dmaConfig;
        dma_RequestState_t                  dmaState   = DMA_REQUEST_ERROR;
        spi_DmaCode_t                       dmaCode    = (spi_DmaCode_t)xferCtx->Config.TxDma;
        spi_DmaPriority_t                   dmaPrio    = xferCtx->Config.TxDmaPriority;

        if( SPI_DMA_DIR_RX == dmaDir )
        {
            dmaCode = (spi_DmaCode_t)xferCtx->Config.RxDma;
            dmaPrio = xferCtx->Config.RxDmaPriority;
        }
        else
        {
            /* Transmit stream */
        }

        retState = Spi_Dma_Get_Stream( periphId, dmaDir, dmaCode, &channelSel );
        dmaState = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.DmaPeriphId         = (dma_PeriphId_t)SPI_DMA_BIT_MASK_DECODE_DMA( dmaCode );
        dmaConfig.DmaChannel          = (dma_ChannelId_t)SPI_DMA_BIT_MASK_DECODE_STREAM( dmaCode );
        dmaConfig.PeripheralReqId     = channelSel;
        dmaConfig.TransferMode        = DMA_TRANSFER_MODE_NORMAL;
        dmaConfig.PeriphAddress       = (dma_PeriphAddr_t)LL_SPI_DMA_GetRegAddr( periphReg );
        dmaConfig.PeriphAddrIncrement = DMA_PERIPH_ADDR_STATIC;
        dmaConfig.MemoryAddrIncrement = DMA_MEMORY_ADDR_INCREMENT;
        dmaConfig.PeriphTransferSize  = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.MemoryTransferSize  = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.DataCount           = 0u;
        dmaConfig.Priority            = (dma_Priority_t)dmaPrio;
        dmaConfig.HalfTransferCallback = DMA_NULL_PTR;

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            dmaConfig.Direction                = DMA_DIR_MEMORY_TO_PERIPH;
            dmaConfig.MemoryAddress            = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyTx;
            dmaConfig.TransferCompleteCallback = periphConf->TxDoneIsr;
            dmaConfig.TransferErrorCallback    = periphConf->TxErrorIsr;
        }
        else
        {
            dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
            dmaConfig.MemoryAddress            = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyRx;
            dmaConfig.TransferCompleteCallback = periphConf->RxDoneIsr;
            dmaConfig.TransferErrorCallback    = periphConf->RxErrorIsr;
        }

        if( ( SPI_REQUEST_OK == retState ) &&
            ( DMA_REQUEST_OK == dmaState )    )
        {
            dmaState = Dma_Init( &dmaConfig );
        }
        else
        {
            /* Stream is not connected to the request */
            dmaState = DMA_REQUEST_ERROR;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Initialized = SPI_FUNCTION_ACTIVE;
            chState->Armed       = SPI_FUNCTION_INACTIVE;
            chState->DmaId       = dmaConfig.DmaPeriphId;
            chState->StreamId    = dmaConfig.DmaChannel;

            dmaState = Dma_Set_TransferCompleteIrqActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Stream initialization failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferErrorIrqActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Transfer complete interrupt could not be enabled */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Transfer error interrupt could not be enabled */
        }

        retState = ( DMA_REQUEST_OK == dmaState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases DMA stream of one direction - stream is disabled, its interrupts and callbacks
 *        are released
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref spi_DmaDir_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (also if the stream was not initialized). Otherwise
 *         returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelOff( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) && ( SPI_DMA_DIR_CNT > dmaDir ) )
    {
        spi_DmaChannelState_t * const chState = &spi_DmaChannelState[ periphId ][ dmaDir ];

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_RequestState_t xferState = Dma_Set_TransferInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t tcState   = Dma_Set_TransferCompleteIrqInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t teState   = Dma_Set_TransferErrorIrqInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t nvicState = Dma_Set_InterruptInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t tcIsr     = Dma_Set_TransferCompleteIsrHandler( chState->DmaId, chState->StreamId, DMA_NULL_PTR );
            const dma_RequestState_t teIsr     = Dma_Set_TransferErrorIsrHandler( chState->DmaId, chState->StreamId, DMA_NULL_PTR );

            chState->Initialized = SPI_FUNCTION_INACTIVE;
            chState->Armed       = SPI_FUNCTION_INACTIVE;

            if( ( DMA_REQUEST_OK == xferState ) &&
                ( DMA_REQUEST_OK == tcState   ) &&
                ( DMA_REQUEST_OK == teState   ) &&
                ( DMA_REQUEST_OK == nvicState ) &&
                ( DMA_REQUEST_OK == tcIsr     ) &&
                ( DMA_REQUEST_OK == teIsr     )    )
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
            /* Stream was not initialized */
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
 * \brief Checks buffers of the request for DMA - 16-bit frames require buffers aligned to 2 bytes
 *
 * \param xferCtx [in]: Pointer to data handling context
 *
 * \return Returns \ref SPI_REQUEST_OK if the buffers can be used. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Check_Buffers( const spi_XferContext_t * const xferCtx )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( SPI_NULL_PTR != xferCtx )
    {
        const uintptr_t txAddr = (uintptr_t)xferCtx->Request.TxData;
        const uintptr_t rxAddr = (uintptr_t)xferCtx->Request.RxData;

        if( ( SPI_DMA_FRAME_BYTES_16BIT != xferCtx->FrameBytes ) ||
            ( ( 0u == ( txAddr % SPI_DMA_FRAME_BYTES_16BIT ) ) &&
              ( 0u == ( rxAddr % SPI_DMA_FRAME_BYTES_16BIT ) )    )    )
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
 * \brief Configures and starts DMA stream of one direction for the running transfer (frame size,
 *        buffer of the request or dummy frame, count of frames)
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir    [in]: Direction, value from \ref spi_DmaDir_t
 * \param dataCount [in]: Count of frames moved by the stream
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Transfer( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, dma_DataCount_t dataCount )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( ( SPI_REQUEST_OK      == retState                                              ) &&
        ( SPI_DMA_DIR_CNT      > dmaDir                                                ) &&
        ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ dmaDir ].Initialized )    )
    {
        spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const dma_TransferSize_t      xferSize = ( SPI_DMA_FRAME_BYTES_16BIT == xferCtx->FrameBytes ) ? DMA_TRANSFER_SIZE_16BIT : DMA_TRANSFER_SIZE_8BIT;
        dma_MemoryAddrInc_t           addrInc  = DMA_MEMORY_ADDR_INCREMENT;
        dma_MemoryAddr_t              memAddr  = 0u;
        dma_RequestState_t            dmaState = DMA_REQUEST_ERROR;

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            if( SPI_NULL_PTR != xferCtx->Request.TxData )
            {
                memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Request.TxData;
            }
            else
            {
                /* Zero frames are transmitted */
                memAddr = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyTx;
                addrInc = DMA_MEMORY_ADDR_STATIC;
            }
        }
        else
        {
            if( SPI_NULL_PTR != xferCtx->Request.RxData )
            {
                memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Request.RxData;
            }
            else
            {
                /* Received frames are discarded */
                memAddr = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyRx;
                addrInc = DMA_MEMORY_ADDR_STATIC;
            }
        }

        dmaState = Dma_Set_PeriphTransferSize( chState->DmaId, chState->StreamId, xferSize );

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryTransferSize( chState->DmaId, chState->StreamId, xferSize );
        }
        else
        {
            /* Configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddrIncrement( chState->DmaId, chState->StreamId, addrInc );
        }
        else
        {
            /* Configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddr( chState->DmaId, chState->StreamId, memAddr );
        }
        else
        {
            /* Configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( chState->DmaId, chState->StreamId, dataCount );
        }
        else
        {
            /* Configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Armed = SPI_FUNCTION_ACTIVE;

            dmaState = Dma_Set_TransferActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Configuration failed */
        }

        retState = ( DMA_REQUEST_OK == dmaState ) ? SPI_REQUEST_OK : SPI_REQUEST_ERROR;
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables DMA request of the peripheral (CR2 TXDMAEN / RXDMAEN)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref spi_DmaDir_t
 * \param reqState [in]: Required state, value from \ref spi_FunctionState_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Request( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_FunctionState_t reqState )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *      periphReg = SPI_NULL_PTR;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( ( SPI_REQUEST_OK == retState ) && ( SPI_DMA_DIR_CNT > dmaDir ) )
    {
        const spi_RegValue_t reqMask  = ( SPI_DMA_DIR_TX == dmaDir ) ? SPI_CR2_TXDMAEN : SPI_CR2_RXDMAEN;
        const spi_RegValue_t reqValue = ( SPI_FUNCTION_ACTIVE == reqState ) ? reqMask : 0u;

        retState = Spi_Set_RegField( &periphReg->CR2, reqMask, reqValue );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops DMA stream of one direction and disables the DMA request of the peripheral
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref spi_DmaDir_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Stop( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId ) && ( SPI_DMA_DIR_CNT > dmaDir ) )
    {
        spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        dma_RequestState_t            dmaState = DMA_REQUEST_OK;

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Stream is not initialized */
        }

        chState->Armed = SPI_FUNCTION_INACTIVE;

        const spi_RequestState_t reqState = Spi_Dma_Set_Request( periphId, dmaDir, SPI_FUNCTION_INACTIVE );

        if( ( DMA_REQUEST_OK == dmaState ) &&
            ( SPI_REQUEST_OK == reqState )    )
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
 * \brief Transfer complete of DMA stream (DMA interrupt): the stream signaling the end of the
 *        transfer finishes it (Spi_Set_XferFinish), master half-duplex reception continues with
 *        the last frame received by the CPU (peripheral disabled, RXNE interrupt)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Direction of the stream, value from \ref spi_DmaDir_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_XferDone( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( ( SPI_REQUEST_OK      == retState                  ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->XferState        ) &&
        ( SPI_XFER_MODE_DMA   == xferCtx->Config.XferMode  ) &&
        ( dmaDir              == spi_DmaEndDir[ periphId ] )    )
    {
        if( ( SPI_DMA_DIR_RX      == dmaDir              ) &&
            ( SPI_FUNCTION_ACTIVE == xferCtx->RxStopUsed )    )
        {
            /* Last but one frame was received - the clock is stopped after the last frame */
            retState = Spi_Set_XferRxStop( periphId );

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_RX, SPI_FUNCTION_INACTIVE );
            }
            else
            {
                /* Peripheral could not be disabled */
            }

            xferCtx->RxIdx     = (spi_DataCnt_t)( xferCtx->Request.XferSize - 1u );
            xferCtx->CpuRxTail = SPI_FUNCTION_ACTIVE;

            if( SPI_REQUEST_OK == retState )
            {
                retState = Spi_Isr_Set_ItActive( periphId, SPI_ISR_IT_RXNE );
            }
            else
            {
                (void)Spi_Set_XferError( periphId, SPI_XFER_ERROR_INCOMPLETE );
            }
        }
        else
        {
            retState = Spi_Set_XferFinish( periphId );
        }
    }
    else
    {
        /* Stream does not signal the end of a running DMA transfer */
    }

    return ( retState );
}


/**
 * \brief Transfer error of DMA stream (DMA interrupt) - the running transfer is terminated by
 *        \ref SPI_XFER_ERROR_DMA_TRANSFER
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_XferError( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( ( SPI_REQUEST_OK      == retState           ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->XferState )    )
    {
        retState = Spi_Set_XferError( periphId, SPI_XFER_ERROR_DMA_TRANSFER );
    }
    else
    {
        /* No transfer is running */
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

#ifdef SPI1
SPI_DMA_DEFINE_HANDLERS( Spi1, SPI_PERIPH_1 )
#endif /* SPI1 */
#ifdef SPI2
SPI_DMA_DEFINE_HANDLERS( Spi2, SPI_PERIPH_2 )
#endif /* SPI2 */
#ifdef SPI3
SPI_DMA_DEFINE_HANDLERS( Spi3, SPI_PERIPH_3 )
#endif /* SPI3 */
#ifdef SPI4
SPI_DMA_DEFINE_HANDLERS( Spi4, SPI_PERIPH_4 )
#endif /* SPI4 */
#ifdef SPI5
SPI_DMA_DEFINE_HANDLERS( Spi5, SPI_PERIPH_5 )
#endif /* SPI5 */
#ifdef SPI6
SPI_DMA_DEFINE_HANDLERS( Spi6, SPI_PERIPH_6 )
#endif /* SPI6 */

/* ================================ TASKS =================================== */
