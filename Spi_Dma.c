/**
 * \author Mr.Nobody
 * \file Spi_Dma.c
 * \ingroup Spi
 * \brief Spi module DMA data transfer handler
 *
 * SPI_XFER_MODE_DMA - data frames are moved between the request buffers and SPI data registers
 * by two DMA streams of DMA1 / DMA2 (STM32H7 - requests routed by DMAMUX1, direct mode with data
 * width equal to the frame size, one transfer of XferSize frames):
 * - Transmission: memory -> TXDR (zero frames from a static word if TxData is not set)
 * - Reception:    RXDR -> memory (frames are discarded into a static word if RxData is not set)
 *
 * Streams of the used directions are armed before the peripheral is enabled (RXDMAEN, streams,
 * TXDMAEN - RM sequence). End of transfer and errors are processed by the SPI interrupt
 * (Spi_Isr.c -> Spi.c), DMA interrupts report only transfer errors. At the end of transfer the
 * last received frames may still be moved - Spi_Dma_Check_Done waits for them.
 *
 * SPI6 requests are routed by DMAMUX2 to BDMA (not supported by the Dma module) - DMA mode of
 * SPI6 is refused by Spi_Dma_Check_Config(). Buffers shall be accessible by DMA1 / DMA2 (AXI
 * SRAM, SRAM1 - SRAM4, not DTCM / ITCM).
 *
 * DMA callbacks have no parameter, so every peripheral has its own set of handlers generated
 * by SPI_DMA_DEFINE_HANDLERS().
 *
 */
/* ============================== INCLUDES ================================== *
 * \note  STM32H7R / STM32H7S (Ral family STM32H7RS, no DMA1 / DMA2 streams): the GPDMA implementation of
 *        the STM32H5 module is compiled (whole file family switch).
 */
#include "Stm32.h"                          /* MCU device header (family macro STM32H7RS) */

#if defined(STM32H7RS)
#include "Spi_Dma.h"                        /* Self include                   */
#include "Spi_Isr.h"                        /* SPI interrupt handler          */
#include "Spi.h"                            /* Module private interface       */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a GPDMA channel */
typedef enum
{
    SPI_DMA_DIR_TX = 0u, /**< Transmission (memory to TXDR) */
    SPI_DMA_DIR_RX,      /**< Reception (RXDR to memory)    */
    SPI_DMA_DIR_CNT      /**< Count of directions           */
}   spi_DmaDir_t;


/** \brief GPDMA handlers of one peripheral (registered in GPDMA module) */
typedef struct
{
    gpdma_IsrErrCallback *TxErrorIsr; /**< Transmission transfer error handler */
    gpdma_IsrErrCallback *RxErrorIsr; /**< Reception transfer error handler    */
}   spi_DmaIsrConfig_t;


/** \brief GPDMA channel ownership (GPDMA channel can not be de-initialized separately) */
typedef struct
{
    spi_FunctionState_t Initialized; /**< GPDMA channel was initialized for the direction */
    spi_DmaPeriphId_t   PeriphId;    /**< Initialized GPDMA peripheral                    */
    spi_DmaChannelId_t  ChannelId;   /**< Initialized GPDMA channel                       */
}   spi_DmaChannelState_t;


/** \brief GPDMA error bit reporting */
typedef struct
{
    gpdma_ErrorMaskId_t DmaError; /**< GPDMA error bit            */
    spi_XferErrorId_t   ErrorId;  /**< Error reported to the user */
}   spi_DmaErrorConfig_t;


/** \brief Index of \ref spi_DmaErrorConfig_t items */
typedef enum
{
    SPI_DMA_ERR_IDX_TRANSFER = 0u,  /**< Transfer error           */
    SPI_DMA_ERR_IDX_CONFIG,         /**< Configuration error      */
    SPI_DMA_ERR_IDX_CONFIG_UPDATE,  /**< Configuration update     */
    SPI_DMA_ERR_IDX_TRIG_OVERRUN,   /**< Trigger overrun          */
    SPI_DMA_ERR_IDX_CNT             /**< Count of reported errors */
}   spi_DmaErrIdx_t;


/** \brief Count of bytes of one GPDMA block (BNDT field) */
typedef uint32_t spi_DmaByteCnt_t;


/** \brief Static word used as source of zero frames / destination of discarded frames */
typedef uint32_t spi_DmaDummy_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares GPDMA handlers of one SPI peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Spi1)
 */
#define SPI_DMA_DECLARE_HANDLERS( name )                                                \
    static void Spi_Dma_##name##_TxError ( gpdma_ErrorMaskId_t errorMask );             \
    static void Spi_Dma_##name##_RxError ( gpdma_ErrorMaskId_t errorMask )

/**
 * \brief Defines GPDMA handlers of one SPI peripheral - every handler forwards the event with the
 *        peripheral identification to the common processing function
 *
 * \param name     [in]: Peripheral name used in handler names (e.g. Spi1)
 * \param periphId [in]: Peripheral identification, value from \ref spi_PeriphId_t
 */
#define SPI_DMA_DEFINE_HANDLERS( name, periphId )                                                          \
    static void Spi_Dma_##name##_TxError( gpdma_ErrorMaskId_t errorMask ) { (void)Spi_Dma_XferError( periphId, errorMask ); } \
    static void Spi_Dma_##name##_RxError( gpdma_ErrorMaskId_t errorMask ) { (void)Spi_Dma_XferError( periphId, errorMask ); }

/**
 * \brief Handler table entry of one SPI peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Spi1)
 */
#define SPI_DMA_ISR_CONFIG( name )                                                      \
    { .TxErrorIsr = Spi_Dma_##name##_TxError, .RxErrorIsr = Spi_Dma_##name##_RxError }

/* ======================== FORWARD DECLARATIONS ============================ */

static spi_RequestState_t Spi_Dma_Set_ChannelInit ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Set_ChannelOff  ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Check_Buffers   ( const spi_XferContext_t * const xferCtx );
static spi_RequestState_t Spi_Dma_Set_Transfer    ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Set_Request     ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_FunctionState_t reqState );
static spi_RequestState_t Spi_Dma_Set_Stop        ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Get_Remaining   ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, gpdma_BlockSize_t * const remaining );
static spi_RequestState_t Spi_Dma_XferError       ( spi_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask );

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

/** Count of data items transferred per DMA request (single transfer) */
#define SPI_DMA_BURST_LEN            ( 1u )

/** Count of transfers in GPDMA transfer list (one block) */
#define SPI_DMA_TRANSFERS_CNT        ( 1u )

/** Maximum count of bytes of one GPDMA block */
#define SPI_DMA_BLOCK_BYTES_MAX      ( 0xFFFFu )

/** Buffer bytes of 8-bit frame */
#define SPI_DMA_FRAME_BYTES_8BIT     ( 1u )

/** Buffer bytes of 16-bit frame */
#define SPI_DMA_FRAME_BYTES_16BIT    ( 2u )

/** GPDMA errors reported to the user */
#define SPI_DMA_ERROR_MASK           ( GPDMA_ERROR_TRANSFER      | \
                                       GPDMA_ERROR_CONFIG_UPDATE | \
                                       GPDMA_ERROR_CONFIG_ERROR  | \
                                       GPDMA_ERROR_TRIG_OVERRUN    )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief spi_PeriphId_t -> GPDMA handlers */
static const spi_DmaIsrConfig_t spi_DmaIsrConfig[ ] =
{
#ifdef SPI1
    SPI_DMA_ISR_CONFIG( Spi1 ),
#endif /* SPI1 */
#ifdef SPI2
    SPI_DMA_ISR_CONFIG( Spi2 ),
#endif /* SPI2 */
#ifdef SPI3
    SPI_DMA_ISR_CONFIG( Spi3 ),
#endif /* SPI3 */
#ifdef SPI4
    SPI_DMA_ISR_CONFIG( Spi4 ),
#endif /* SPI4 */
#ifdef SPI5
    SPI_DMA_ISR_CONFIG( Spi5 ),
#endif /* SPI5 */
#ifdef SPI6
    SPI_DMA_ISR_CONFIG( Spi6 ),
#endif /* SPI6 */
};

_Static_assert( SPI_PERIPH_CNT == ( sizeof(spi_DmaIsrConfig) / sizeof(spi_DmaIsrConfig_t) ), "Spi: spi_DmaIsrConfig has incorrect size." );


/** \brief GPDMA error bits and reported errors */
static const spi_DmaErrorConfig_t spi_DmaErrorConfig[ SPI_DMA_ERR_IDX_CNT ] =
{
    [SPI_DMA_ERR_IDX_TRANSFER]      = { .DmaError = GPDMA_ERROR_TRANSFER,      .ErrorId = SPI_XFER_ERROR_DMA_TRANSFER        },
    [SPI_DMA_ERR_IDX_CONFIG]        = { .DmaError = GPDMA_ERROR_CONFIG_ERROR,  .ErrorId = SPI_XFER_ERROR_DMA_CONFIG          },
    [SPI_DMA_ERR_IDX_CONFIG_UPDATE] = { .DmaError = GPDMA_ERROR_CONFIG_UPDATE, .ErrorId = SPI_XFER_ERROR_DMA_CONFIG_UPDATE   },
    [SPI_DMA_ERR_IDX_TRIG_OVERRUN]  = { .DmaError = GPDMA_ERROR_TRIG_OVERRUN,  .ErrorId = SPI_XFER_ERROR_DMA_TRIGGER_OVERRUN },
};


/** \brief GPDMA transfer lists (must be static, used by GPDMA HW) */
static gpdma_XferList_t spi_DmaXferList[ SPI_PERIPH_CNT ][ SPI_DMA_DIR_CNT ];


/** \brief GPDMA channel ownership per peripheral and direction */
static spi_DmaChannelState_t spi_DmaChannelState[ SPI_PERIPH_CNT ][ SPI_DMA_DIR_CNT ];


/** \brief Source of zero frames (transmission without TxData) - never written */
static spi_DmaDummy_t spi_DmaDummyTx = 0u;


/** \brief Destination of discarded frames (reception without RxData) */
static spi_DmaDummy_t spi_DmaDummyRx = 0u;

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data handling configuration
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if DMA identifications and priorities are valid and the
 *         transmit channel differs from the receive channel. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT > periphId   ) &&
        ( SPI_NULL_PTR  != dataConfig )    )
    {
        if( ( SPI_DMA_PERIPH_CNT            > dataConfig->TxDmaPeriphId           ) &&
            ( SPI_DMA_CHANNEL_CNT           > dataConfig->TxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT  > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( SPI_DMA_PERIPH_CNT            > dataConfig->RxDmaPeriphId           ) &&
            ( SPI_DMA_CHANNEL_CNT           > dataConfig->RxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT  > (uint32_t)dataConfig->RxDmaPriority ) &&
            ( ( dataConfig->TxDmaPeriphId  != dataConfig->RxDmaPeriphId  ) ||
              ( dataConfig->TxDmaChannelId != dataConfig->RxDmaChannelId )    )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* DMA configuration is invalid or both directions share one channel */
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
 * \brief Initializes DMA data transfer - GPDMA channels of both directions, DMA requests and SPI
 *        interrupt sources are disabled until the transfer start
 *
 * \note  A GPDMA channel already initialized for the direction by a previous initialization is
 *        reused (GPDMA module does not support de-initialization of a single channel).
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
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
        /* Transmit channel initialization failed */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Dma_XferStop( periphId );
    }
    else
    {
        /* Receive channel initialization failed */
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer - GPDMA channels and their interrupts, DMA requests and
 *        SPI interrupt sources are disabled. All steps are executed, any failure is reported.
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
 * \brief Starts DMA data transfer before the peripheral is enabled - receive DMA request is
 *        enabled, GPDMA channels of the used directions are armed and transmit DMA request is
 *        enabled (RM sequence)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if the buffers are not aligned to the
 *         frame size or the transfer exceeds one GPDMA block) returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferStart( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Dma_Check_Buffers( xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed )    )
    {
        retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_RX, SPI_FUNCTION_ACTIVE );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_RX );
        }
        else
        {
            /* Receive DMA request could not be enabled */
        }
    }
    else
    {
        /* Previous step failed or nothing is received */
    }

    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed )    )
    {
        retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_TX );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_TX, SPI_FUNCTION_ACTIVE );
        }
        else
        {
            /* Transmit channel could not be armed */
        }
    }
    else
    {
        /* Previous step failed or nothing is transmitted */
    }

    return ( retState );
}


/**
 * \brief Runs DMA data transfer after the peripheral is enabled - end of transfer and error
 *        interrupts are enabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferRun( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItActive( periphId, SPI_ISR_IT_EVENTS );

    return ( retState );
}


/**
 * \brief Stops DMA data transfer - GPDMA channels, SPI DMA requests and SPI interrupt sources
 *        are disabled. All steps are executed, any failure is reported.
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
 * \brief Checks that GPDMA channels moved all frames of the transfer (remaining block size of
 *        the used directions is zero). The last received frames are moved after the end of
 *        transfer - the check is repeated up to SPI_TIMEOUT_RAW times.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_REQUEST_OK if all frames were moved. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Done( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    for( spi_DmaDir_t dmaDir = SPI_DMA_DIR_TX;
         ( SPI_REQUEST_OK == retState ) &&
         ( SPI_DMA_DIR_CNT > dmaDir );
         dmaDir ++ )
    {
        const spi_FunctionState_t dirUsed = ( SPI_DMA_DIR_TX == dmaDir ) ? xferCtx->TxUsed : xferCtx->RxUsed;

        if( SPI_FUNCTION_ACTIVE == dirUsed )
        {
            retState = SPI_REQUEST_ERROR;

            for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                gpdma_BlockSize_t        remaining = 0u;
                const spi_RequestState_t remState  = Spi_Dma_Get_Remaining( periphId, dmaDir, &remaining );

                if( SPI_REQUEST_OK != remState )
                {
                    /* Remaining count is not available */
                    break;
                }
                else if( 0u == remaining )
                {
                    retState = SPI_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Frames are still being moved, keep return state as error */
                }
            }
        }
        else
        {
            /* Direction is not used */
        }
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Initializes (or reuses) the GPDMA channel of one direction and enables its interrupt
 *        (only error handler is registered). Data width, addresses and block size are set by
 *        every transfer start.
 *
 * - Transmission: memory -> TXDR (static)
 * - Reception:    RXDR (static) -> memory
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelInit( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *       periphReg = SPI_NULL_PTR;
    spi_XferContext_t * xferCtx   = SPI_NULL_PTR;
    gpdma_PeriphReqId_t txRequest = GPDMA_REQ_SPI1_TX; /* Overwritten by Spi_Get_PeriphDmaReq() */
    gpdma_PeriphReqId_t rxRequest = GPDMA_REQ_SPI1_RX; /* Overwritten by Spi_Get_PeriphDmaReq() */

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_XferContext( periphId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_PeriphDmaReq( periphId, &txRequest, &rxRequest );
    }
    else
    {
        /* Transfer context is not available */
    }

    if( ( SPI_REQUEST_OK  == retState ) &&
        ( SPI_DMA_DIR_CNT  > dmaDir   )    )
    {
        spi_DmaChannelState_t * const    chState    = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const spi_DmaIsrConfig_t * const isrConfig  = &spi_DmaIsrConfig[ periphId ];
        gpdma_ConfigStruct_t             dmaConfig  = { 0u };
        gpdma_TransferConfig_t           xferConfig = { 0u };
        spi_DmaPeriphId_t                dmaPeriph  = xferCtx->Config.TxDmaPeriphId;
        spi_DmaChannelId_t               dmaChannel = xferCtx->Config.TxDmaChannelId;
        spi_DmaPriority_t                dmaPrio    = xferCtx->Config.TxDmaPriority;
        gpdma_RequestState_t             dmaState   = GPDMA_REQUEST_ERROR;

        /* --- Common transfer parameters (width, address and block size are set by every start) --- */
        xferConfig.EventMode              = GPDMA_TRANSFER_EVENT_BLOCK;
        xferConfig.TriggerType            = GPDMA_TRG_NOT_USED;
        xferConfig.TriggerMode            = GPDMA_TRIGGER_BLOCK;
        xferConfig.RequestMode            = GPDMA_PERIPH_REQ_SINGLE;
        xferConfig.BlockRepetitionCount   = 0u;
        xferConfig.BlockSize              = 0u;
        xferConfig.SourceDataSize         = GPDMA_DATA_SIZE_8BITS;
        xferConfig.SourceBurstLength      = SPI_DMA_BURST_LEN;
        xferConfig.SourcePortId           = GPDMA_PORT_DEFAULT;
        xferConfig.SourceDataOp           = GPDMA_SRC_DATA_PRESERVE;
        xferConfig.DestinationDataSize    = GPDMA_DATA_SIZE_8BITS;
        xferConfig.DestinationBurstLength = SPI_DMA_BURST_LEN;
        xferConfig.DestinationPortId      = GPDMA_PORT_DEFAULT;
        xferConfig.DestinationDataOp      = GPDMA_DEST_DATA_PRESERVE;

        dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.TransferExecMode    = GPDMA_XFER_EXEC_CONTINUOUS;
        dmaConfig.TransferConfig      = &xferConfig;
        dmaConfig.TransfersCount      = SPI_DMA_TRANSFERS_CNT;
        dmaConfig.XferListAccessMode  = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
        dmaConfig.XferList            = &spi_DmaXferList[ periphId ][ dmaDir ];
        dmaConfig.TransferLockState   = GPDMA_TRANSFER_LIST_LOCKED;
        dmaConfig.ErrorMask           = SPI_DMA_ERROR_MASK;
        dmaConfig.TransferCompleteIsr = GPDMA_NULL_PTR;
        dmaConfig.HalfTransferIsr     = GPDMA_NULL_PTR;

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
            xferConfig.RequestSource       = txRequest;
            xferConfig.SourceAddr          = 0u;
            xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
            xferConfig.DestinationAddr     = (gpdma_DstAddr_t)LL_SPI_DMA_GetTxRegAddr( periphReg );
            xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;

            dmaConfig.ErrorIsr             = isrConfig->TxErrorIsr;
        }
        else
        {
            dmaPeriph  = xferCtx->Config.RxDmaPeriphId;
            dmaChannel = xferCtx->Config.RxDmaChannelId;
            dmaPrio    = xferCtx->Config.RxDmaPriority;

            xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
            xferConfig.RequestSource       = rxRequest;
            xferConfig.SourceAddr          = (gpdma_SrcAddr_t)LL_SPI_DMA_GetRxRegAddr( periphReg );
            xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
            xferConfig.DestinationAddr     = 0u;
            xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;

            dmaConfig.ErrorIsr             = isrConfig->RxErrorIsr;
        }

        dmaConfig.PeriphId    = (gpdma_PeriphId_t)dmaPeriph;
        dmaConfig.ChannelId   = (gpdma_ChannelId_t)dmaChannel;
        dmaConfig.ChannelPrio = (gpdma_Priority_t)dmaPrio;

        /* --- GPDMA channel initialization or reuse --- */
        if( ( SPI_FUNCTION_ACTIVE == chState->Initialized ) &&
            ( dmaPeriph           == chState->PeriphId    ) &&
            ( dmaChannel          == chState->ChannelId   )    )
        {
            /* Channel is already configured for this direction - priority is updated */
            dmaState = Gpdma_Set_Priority( dmaConfig.PeriphId, dmaConfig.ChannelId, dmaConfig.ChannelPrio );
        }
        else if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Init( &dmaConfig );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = SPI_FUNCTION_ACTIVE;
                chState->PeriphId    = dmaPeriph;
                chState->ChannelId   = dmaChannel;
            }
            else
            {
                /* GPDMA channel initialization failed */
            }
        }
        else
        {
            /* Default configuration is not available */
        }

        /* GPDMA module does not enable the channel interrupt in NVIC by itself */
        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
        }
        else
        {
            /* GPDMA channel configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Disables the GPDMA channel of one direction and its interrupt (channel configuration is
 *        kept, see Spi_Dma_Set_ChannelInit())
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (also if no channel was initialized). Otherwise
 *         returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelOff( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT  > periphId ) &&
        ( SPI_DMA_DIR_CNT > dmaDir   )    )
    {
        const spi_DmaChannelState_t * const chState = &spi_DmaChannelState[ periphId ][ dmaDir ];

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            const gpdma_PeriphId_t  dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
            const gpdma_ChannelId_t dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
            gpdma_RequestState_t    dmaState  = Gpdma_Set_ChannelInactive( dmaPeriph, dmaChan );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_InterruptInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Channel could not be disabled */
            }

            if( GPDMA_REQUEST_OK == dmaState )
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
            /* No GPDMA channel was initialized for the direction */
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
 * \brief Checks that the transfer fits into one GPDMA block and the used buffers are aligned to
 *        the frame size (GPDMA requires addresses aligned to the data width)
 *
 * \param xferCtx [in]: Pointer to transfer context of the starting transfer
 *
 * \return Returns \ref SPI_REQUEST_OK if the transfer can be moved by GPDMA. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Check_Buffers( const spi_XferContext_t * const xferCtx )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_NULL_PTR != xferCtx         ) &&
        ( 0u            < xferCtx->FrameBytes )    )
    {
        const spi_DmaByteCnt_t xferBytes = (spi_DmaByteCnt_t)xferCtx->Request.XferSize * xferCtx->FrameBytes;
        const spi_DmaByteCnt_t txAlign   = (spi_DmaByteCnt_t)(uintptr_t)xferCtx->Request.TxData % xferCtx->FrameBytes;
        const spi_DmaByteCnt_t rxAlign   = (spi_DmaByteCnt_t)(uintptr_t)xferCtx->Request.RxData % xferCtx->FrameBytes;

        if( ( SPI_DMA_BLOCK_BYTES_MAX >= xferBytes ) &&
            ( 0u                      == txAlign   ) &&
            ( 0u                      == rxAlign   )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* Transfer is too long or buffer is not aligned */
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
 * \brief Arms the GPDMA channel of one direction (data width, block size, memory address and its
 *        addressing mode, enable)
 *
 * - Transmission: XferSize frames from TxData (static zero word if TxData is not set)
 * - Reception:    XferSize frames into RxData (static dummy word if RxData is not set)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Transfer( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( ( SPI_REQUEST_OK      == retState                                          ) &&
        ( SPI_DMA_DIR_CNT      > dmaDir                                            ) &&
        ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ dmaDir ].Initialized ) )
    {
        const spi_DmaChannelState_t * const chState   = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const gpdma_PeriphId_t              dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
        const gpdma_ChannelId_t             dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
        const gpdma_BlockSize_t             blockSize = (gpdma_BlockSize_t)( (spi_DmaByteCnt_t)xferCtx->Request.XferSize * xferCtx->FrameBytes );
        gpdma_DataSize_t                    dataSize  = GPDMA_DATA_SIZE_32BITS;
        gpdma_AddrMode_t                    addrMode  = GPDMA_ADDR_INCREMENT;
        uint32_t                            memAddr   = 0u;
        gpdma_RequestState_t                dmaState  = GPDMA_REQUEST_ERROR;

        if( SPI_DMA_FRAME_BYTES_8BIT == xferCtx->FrameBytes )
        {
            dataSize = GPDMA_DATA_SIZE_8BITS;
        }
        else if( SPI_DMA_FRAME_BYTES_16BIT == xferCtx->FrameBytes )
        {
            dataSize = GPDMA_DATA_SIZE_16BITS;
        }
        else
        {
            /* Frame up to 32 bits */
        }

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            if( SPI_NULL_PTR != xferCtx->Request.TxData )
            {
                memAddr = (uint32_t)(uintptr_t)xferCtx->Request.TxData;
            }
            else
            {
                /* Zero frames are transmitted from a static word */
                memAddr  = (uint32_t)(uintptr_t)&spi_DmaDummyTx;
                addrMode = GPDMA_ADDR_STATIC;
            }
        }
        else
        {
            if( SPI_NULL_PTR != xferCtx->Request.RxData )
            {
                memAddr = (uint32_t)(uintptr_t)xferCtx->Request.RxData;
            }
            else
            {
                /* Received frames are discarded into a static word */
                memAddr  = (uint32_t)(uintptr_t)&spi_DmaDummyRx;
                addrMode = GPDMA_ADDR_STATIC;
            }
        }

        /* Source and destination width equal the frame size (TXDR / RXDR access width) */
        dmaState = Gpdma_Set_SourceDataSize( dmaPeriph, dmaChan, dataSize );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_DestinationDataSize( dmaPeriph, dmaChan, dataSize );
        }
        else
        {
            /* Source data width configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, blockSize );
        }
        else
        {
            /* Destination data width configuration failed */
        }

        if( ( GPDMA_REQUEST_OK == dmaState ) &&
            ( SPI_DMA_DIR_TX   == dmaDir   )    )
        {
            dmaState = Gpdma_Set_SourceAddrMode( dmaPeriph, dmaChan, addrMode );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_SourceAddr( dmaPeriph, dmaChan, (gpdma_SrcAddr_t)memAddr );
            }
            else
            {
                /* Source addressing mode configuration failed */
            }
        }
        else if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_DestinationAddrMode( dmaPeriph, dmaChan, addrMode );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_DestinationAddr( dmaPeriph, dmaChan, (gpdma_DstAddr_t)memAddr );
            }
            else
            {
                /* Destination addressing mode configuration failed */
            }
        }
        else
        {
            /* Block size configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_ChannelActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Enables / disables SPI DMA request of one direction (TXDMAEN / RXDMAEN)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
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

    if( ( SPI_REQUEST_OK  == retState ) &&
        ( SPI_DMA_DIR_CNT  > dmaDir   )    )
    {
        const spi_RegValue_t reqMask  = ( SPI_DMA_DIR_TX == dmaDir ) ? SPI_CFG1_TXDMAEN : SPI_CFG1_RXDMAEN;
        const spi_RegValue_t reqValue = ( SPI_FUNCTION_ACTIVE == reqState ) ? reqMask : 0u;

        retState = Spi_Set_RegField( &periphReg->CFG1, reqMask, reqValue );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops one direction - GPDMA channel (if initialized) and SPI DMA request are disabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Stop( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT  > periphId ) &&
        ( SPI_DMA_DIR_CNT > dmaDir   )    )
    {
        const spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        gpdma_RequestState_t                dmaState = GPDMA_REQUEST_OK;

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            dmaState = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId );
        }
        else
        {
            /* No GPDMA channel was initialized for the direction */
        }

        const spi_RequestState_t reqState = Spi_Dma_Set_Request( periphId, dmaDir, SPI_FUNCTION_INACTIVE );

        if( ( GPDMA_REQUEST_OK == dmaState ) &&
            ( SPI_REQUEST_OK   == reqState )    )
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
 * \brief Returns count of bytes not yet moved by the GPDMA channel of one direction
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir     [in]: Data transfer direction
 * \param remaining [out]: Pointer to store the remaining block size. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Get_Remaining( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, gpdma_BlockSize_t * const remaining )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT      > periphId                                          ) &&
        ( SPI_DMA_DIR_CNT     > dmaDir                                            ) &&
        ( SPI_NULL_PTR       != remaining                                         ) &&
        ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ dmaDir ].Initialized ) )
    {
        const spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const gpdma_RequestState_t          dmaState = Gpdma_Get_BlockSize( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId, remaining );

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief DMA error processing - the transfer is aborted and the first reported GPDMA error bit
 *        (order of \ref spi_DmaErrorConfig) is forwarded as data transfer error, one error
 *        callback per GPDMA error event
 *
 * \param periphId  [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param errorMask [in]: GPDMA error bit mask
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_XferError( spi_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask )
{
    spi_RequestState_t retState = SPI_REQUEST_OK;

    for( spi_DmaErrIdx_t errIdx = SPI_DMA_ERR_IDX_TRANSFER; SPI_DMA_ERR_IDX_CNT > errIdx; errIdx ++ )
    {
        if( 0u != ( (uint32_t)errorMask & (uint32_t)spi_DmaErrorConfig[ errIdx ].DmaError ) )
        {
            /* First error aborts the transfer and is reported, further bits of the event are not */
            retState = Spi_Set_XferError( periphId, spi_DmaErrorConfig[ errIdx ].ErrorId );
            break;
        }
        else
        {
            /* Error bit is not reported */
        }
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

#else

#include "Spi_Dma.h"                        /* Self include                   */
#include "Spi_Isr.h"                        /* SPI interrupt handler          */
#include "Spi.h"                            /* Module private interface       */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a DMA stream */
typedef enum
{
    SPI_DMA_DIR_TX = 0u, /**< Transmission (memory to TXDR) */
    SPI_DMA_DIR_RX,      /**< Reception (RXDR to memory)    */
    SPI_DMA_DIR_CNT      /**< Count of directions           */
}   spi_DmaDir_t;


/** \brief DMA handlers of one peripheral (registered in DMA module) */
typedef struct
{
    dma_IsrCallback TxErrorIsr; /**< Transmission transfer error handler */
    dma_IsrCallback RxErrorIsr; /**< Reception transfer error handler    */
}   spi_DmaIsrConfig_t;


/** \brief Initialized DMA stream of one direction */
typedef struct
{
    spi_FunctionState_t Initialized; /**< DMA stream was initialized for the direction */
    dma_PeriphId_t      DmaId;       /**< Initialized DMA peripheral                   */
    dma_ChannelId_t     ChannelId;   /**< Initialized DMA stream                       */
}   spi_DmaChannelState_t;


/** \brief Static word used as source of zero frames / destination of discarded frames */
typedef uint32_t spi_DmaDummy_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares DMA handlers of one SPI peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Spi1)
 */
#define SPI_DMA_DECLARE_HANDLERS( name )                                                \
    static void Spi_Dma_##name##_TxError ( void );                                      \
    static void Spi_Dma_##name##_RxError ( void )

/**
 * \brief Defines DMA handlers of one SPI peripheral - every handler forwards the event with the
 *        peripheral identification to the common processing function
 *
 * \param name     [in]: Peripheral name used in handler names (e.g. Spi1)
 * \param periphId [in]: Peripheral identification, value from \ref spi_PeriphId_t
 */
#define SPI_DMA_DEFINE_HANDLERS( name, periphId )                                                  \
    static void Spi_Dma_##name##_TxError( void ) { (void)Spi_Dma_XferError( periphId ); }          \
    static void Spi_Dma_##name##_RxError( void ) { (void)Spi_Dma_XferError( periphId ); }

/**
 * \brief Handler table entry of one SPI peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Spi1)
 */
#define SPI_DMA_ISR_CONFIG( name )                                                      \
    { .TxErrorIsr = Spi_Dma_##name##_TxError, .RxErrorIsr = Spi_Dma_##name##_RxError }

/* ======================== FORWARD DECLARATIONS ============================ */

static spi_RequestState_t Spi_Dma_Set_ChannelInit ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Set_ChannelOff  ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Check_Buffers   ( const spi_XferContext_t * const xferCtx );
static spi_RequestState_t Spi_Dma_Set_Transfer    ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Set_Request     ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, spi_FunctionState_t reqState );
static spi_RequestState_t Spi_Dma_Set_Stop        ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir );
static spi_RequestState_t Spi_Dma_Get_Remaining   ( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, dma_DataCount_t * const remaining );
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

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Buffer bytes of 8-bit frame */
#define SPI_DMA_FRAME_BYTES_8BIT     ( 1u )

/** Buffer bytes of 16-bit frame */
#define SPI_DMA_FRAME_BYTES_16BIT    ( 2u )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief spi_PeriphId_t -> DMA handlers (SPI6 has no DMA mode - BDMA / DMAMUX2) */
static const spi_DmaIsrConfig_t spi_DmaIsrConfig[ ] =
{
#ifdef SPI1
    SPI_DMA_ISR_CONFIG( Spi1 ),
#endif /* SPI1 */
#ifdef SPI2
    SPI_DMA_ISR_CONFIG( Spi2 ),
#endif /* SPI2 */
#ifdef SPI3
    SPI_DMA_ISR_CONFIG( Spi3 ),
#endif /* SPI3 */
#ifdef SPI4
    SPI_DMA_ISR_CONFIG( Spi4 ),
#endif /* SPI4 */
#ifdef SPI5
    SPI_DMA_ISR_CONFIG( Spi5 ),
#endif /* SPI5 */
#ifdef SPI6
    { .TxErrorIsr = DMA_NULL_PTR, .RxErrorIsr = DMA_NULL_PTR },
#endif /* SPI6 */
};

_Static_assert( SPI_PERIPH_CNT == ( sizeof(spi_DmaIsrConfig) / sizeof(spi_DmaIsrConfig_t) ), "Spi: spi_DmaIsrConfig has incorrect size." );


/** \brief Initialized DMA streams per peripheral and direction */
static spi_DmaChannelState_t spi_DmaChannelState[ SPI_PERIPH_CNT ][ SPI_DMA_DIR_CNT ];


/** \brief Source of zero frames (transmission without TxData) - never written */
static spi_DmaDummy_t spi_DmaDummyTx = 0u;


/** \brief Destination of discarded frames (reception without RxData) */
static spi_DmaDummy_t spi_DmaDummyRx = 0u;

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data handling configuration
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the peripheral has DMAMUX1 requests, DMA
 *         identifications and priorities are valid and the transmit stream differs from the
 *         receive stream. Otherwise (also for SPI6 - requests on BDMA / DMAMUX2) returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    dma_PeriphReqId_t  txRequest = DMA_REQ_MEM2MEM;
    dma_PeriphReqId_t  rxRequest = DMA_REQ_MEM2MEM;

    if( ( SPI_PERIPH_CNT > periphId   ) &&
        ( SPI_NULL_PTR  != dataConfig )    )
    {
        retState = Spi_Get_PeriphDmaReq( periphId, &txRequest, &rxRequest );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    if( SPI_REQUEST_OK == retState )
    {
        if( ( SPI_DMA_PERIPH_CNT          > dataConfig->TxDmaPeriphId           ) &&
            ( SPI_DMA_CHANNEL_CNT         > dataConfig->TxDmaChannelId          ) &&
            ( (uint32_t)DMA_PRIORITY_CNT  > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( SPI_DMA_PERIPH_CNT          > dataConfig->RxDmaPeriphId           ) &&
            ( SPI_DMA_CHANNEL_CNT         > dataConfig->RxDmaChannelId          ) &&
            ( (uint32_t)DMA_PRIORITY_CNT  > (uint32_t)dataConfig->RxDmaPriority ) &&
            ( ( dataConfig->TxDmaPeriphId  != dataConfig->RxDmaPeriphId  ) ||
              ( dataConfig->TxDmaChannelId != dataConfig->RxDmaChannelId )    )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* DMA configuration is invalid or both directions share one stream */
            retState = SPI_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameters or the peripheral has no DMAMUX1 request (SPI6) */
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA data transfer - DMA streams of both directions, DMA requests and SPI
 *        interrupt sources are disabled until the transfer start
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
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
        /* Receive stream initialization failed */
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer - DMA streams are released (disabled, interrupts and
 *        handlers removed), DMA requests and SPI interrupt sources are disabled. All steps are
 *        executed, any failure is reported.
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
 * \brief Starts DMA data transfer before the peripheral is enabled - receive DMA request is
 *        enabled, DMA streams of the used directions are armed and transmit DMA request is
 *        enabled (RM sequence)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if the buffers are not aligned to the
 *         frame size) returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferStart( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Dma_Check_Buffers( xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed )    )
    {
        retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_RX, SPI_FUNCTION_ACTIVE );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_RX );
        }
        else
        {
            /* Receive DMA request could not be enabled */
        }
    }
    else
    {
        /* Previous step failed or nothing is received */
    }

    if( ( SPI_REQUEST_OK      == retState        ) &&
        ( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed )    )
    {
        retState = Spi_Dma_Set_Transfer( periphId, SPI_DMA_DIR_TX );

        if( SPI_REQUEST_OK == retState )
        {
            retState = Spi_Dma_Set_Request( periphId, SPI_DMA_DIR_TX, SPI_FUNCTION_ACTIVE );
        }
        else
        {
            /* Transmit stream could not be armed */
        }
    }
    else
    {
        /* Previous step failed or nothing is transmitted */
    }

    return ( retState );
}


/**
 * \brief Runs DMA data transfer after the peripheral is enabled - end of transfer and error
 *        interrupts are enabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_XferRun( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItActive( periphId, SPI_ISR_IT_EVENTS );

    return ( retState );
}


/**
 * \brief Stops DMA data transfer - DMA streams, SPI DMA requests and SPI interrupt sources
 *        are disabled. All steps are executed, any failure is reported.
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
 * \brief Checks that DMA streams moved all frames of the transfer (remaining data count of the
 *        used directions is zero). The last received frames are moved after the end of transfer
 *        - the check is repeated up to SPI_TIMEOUT_RAW times.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Returns \ref SPI_REQUEST_OK if all frames were moved. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Dma_Check_Done( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    for( spi_DmaDir_t dmaDir = SPI_DMA_DIR_TX; SPI_DMA_DIR_CNT > dmaDir; dmaDir ++ )
    {
        if( SPI_REQUEST_OK != retState )
        {
            /* Context is not available or previous direction did not finish */
            break;
        }
        else
        {
            const spi_FunctionState_t dirUsed = ( SPI_DMA_DIR_TX == dmaDir ) ? xferCtx->TxUsed : xferCtx->RxUsed;

            if( SPI_FUNCTION_ACTIVE == dirUsed )
            {
                retState = SPI_REQUEST_ERROR;

                for( spi_TimeoutCnt_t iterationCnt = 0u; SPI_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    dma_DataCount_t          remaining = 0u;
                    const spi_RequestState_t remState  = Spi_Dma_Get_Remaining( periphId, dmaDir, &remaining );

                    if( SPI_REQUEST_OK != remState )
                    {
                        /* Remaining count is not available */
                        break;
                    }
                    else if( 0u == remaining )
                    {
                        retState = SPI_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Frames are still being moved, keep return state as error */
                    }
                }
            }
            else
            {
                /* Direction is not used */
            }
        }
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Initializes the DMA stream of one direction (normal mode, DMAMUX1 request of the SPI,
 *        priority, transfer error handler) and enables its error interrupt. Data width, memory
 *        address and data count are set by every transfer start.
 *
 * - Transmission: memory -> TXDR (static)
 * - Reception:    RXDR (static) -> memory
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelInit( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *       periphReg = SPI_NULL_PTR;
    spi_XferContext_t * xferCtx   = SPI_NULL_PTR;
    dma_PeriphReqId_t   txRequest = DMA_REQ_MEM2MEM; /* Overwritten by Spi_Get_PeriphDmaReq() */
    dma_PeriphReqId_t   rxRequest = DMA_REQ_MEM2MEM; /* Overwritten by Spi_Get_PeriphDmaReq() */

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_XferContext( periphId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Get_PeriphDmaReq( periphId, &txRequest, &rxRequest );
    }
    else
    {
        /* Transfer context is not available */
    }

    if( ( SPI_REQUEST_OK  == retState ) &&
        ( SPI_DMA_DIR_CNT  > dmaDir   )    )
    {
        spi_DmaChannelState_t * const    chState   = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const spi_DmaIsrConfig_t * const isrConfig = &spi_DmaIsrConfig[ periphId ];
        dma_ConfigStruct_t               dmaConfig;
        dma_RequestState_t               dmaState  = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.TransferMode             = DMA_TRANSFER_MODE_NORMAL;
        dmaConfig.PeriphAddrIncrement      = DMA_PERIPH_ADDR_STATIC;
        dmaConfig.MemoryAddrIncrement      = DMA_MEMORY_ADDR_INCREMENT;
        dmaConfig.PeriphTransferSize       = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.MemoryTransferSize       = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.DataCount                = 0u;
        dmaConfig.TransferCompleteCallback = DMA_NULL_PTR;
        dmaConfig.HalfTransferCallback     = DMA_NULL_PTR;

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            dmaConfig.DmaPeriphId           = (dma_PeriphId_t)xferCtx->Config.TxDmaPeriphId;
            dmaConfig.DmaChannel            = (dma_ChannelId_t)xferCtx->Config.TxDmaChannelId;
            dmaConfig.Priority              = (dma_Priority_t)xferCtx->Config.TxDmaPriority;
            dmaConfig.Direction             = DMA_DIR_MEMORY_TO_PERIPH;
            dmaConfig.PeripheralReqId       = txRequest;
            dmaConfig.PeriphAddress         = (dma_PeriphAddr_t)LL_SPI_DMA_GetTxRegAddr( periphReg );
            dmaConfig.MemoryAddress         = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyTx;
            dmaConfig.TransferErrorCallback = isrConfig->TxErrorIsr;
        }
        else
        {
            dmaConfig.DmaPeriphId           = (dma_PeriphId_t)xferCtx->Config.RxDmaPeriphId;
            dmaConfig.DmaChannel            = (dma_ChannelId_t)xferCtx->Config.RxDmaChannelId;
            dmaConfig.Priority              = (dma_Priority_t)xferCtx->Config.RxDmaPriority;
            dmaConfig.Direction             = DMA_DIR_PERIPH_TO_MEMORY;
            dmaConfig.PeripheralReqId       = rxRequest;
            dmaConfig.PeriphAddress         = (dma_PeriphAddr_t)LL_SPI_DMA_GetRxRegAddr( periphReg );
            dmaConfig.MemoryAddress         = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyRx;
            dmaConfig.TransferErrorCallback = isrConfig->RxErrorIsr;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Init( &dmaConfig );
        }
        else
        {
            /* Default configuration is not available */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Initialized = SPI_FUNCTION_ACTIVE;
            chState->DmaId       = dmaConfig.DmaPeriphId;
            chState->ChannelId   = dmaConfig.DmaChannel;

            dmaState = Dma_Set_TransferErrorIrqActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* DMA stream initialization failed */
        }

        /* DMA module does not enable the stream interrupt in NVIC by itself */
        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Transfer error interrupt could not be enabled */
        }

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief Releases the DMA stream of one direction - the stream is disabled, its error interrupt,
 *        NVIC line and error handler are released
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems (also if no stream was initialized). Otherwise
 *         returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_ChannelOff( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT  > periphId ) &&
        ( SPI_DMA_DIR_CNT > dmaDir   )    )
    {
        spi_DmaChannelState_t * const chState = &spi_DmaChannelState[ periphId ][ dmaDir ];

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_RequestState_t xferState = Dma_Set_TransferInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t teState   = Dma_Set_TransferErrorIrqInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t nvicState = Dma_Set_InterruptInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t teIsr     = Dma_Set_TransferErrorIsrHandler( chState->DmaId, chState->ChannelId, DMA_NULL_PTR );

            chState->Initialized = SPI_FUNCTION_INACTIVE;

            if( ( DMA_REQUEST_OK == xferState ) &&
                ( DMA_REQUEST_OK == teState   ) &&
                ( DMA_REQUEST_OK == nvicState ) &&
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
            /* No DMA stream was initialized for the direction */
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
 * \brief Checks that the used buffers are aligned to the frame size (DMA requires memory
 *        addresses aligned to the data width). The count of frames always fits into NDTR (16
 *        bits, XferSize is limited by TSIZE).
 *
 * \param xferCtx [in]: Pointer to transfer context of the starting transfer
 *
 * \return Returns \ref SPI_REQUEST_OK if the transfer can be moved by DMA. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Check_Buffers( const spi_XferContext_t * const xferCtx )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_NULL_PTR != xferCtx             ) &&
        ( 0u            < xferCtx->FrameBytes )    )
    {
        const uint32_t txAlign = (uint32_t)( (uintptr_t)xferCtx->Request.TxData % xferCtx->FrameBytes );
        const uint32_t rxAlign = (uint32_t)( (uintptr_t)xferCtx->Request.RxData % xferCtx->FrameBytes );

        if( ( 0u == txAlign ) &&
            ( 0u == rxAlign )    )
        {
            retState = SPI_REQUEST_OK;
        }
        else
        {
            /* Buffer is not aligned to the frame size */
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
 * \brief Arms the DMA stream of one direction (data width, memory address and its increment,
 *        count of frames, enable)
 *
 * - Transmission: XferSize frames from TxData (static zero word if TxData is not set)
 * - Reception:    XferSize frames into RxData (static dummy word if RxData is not set)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Transfer( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( ( SPI_REQUEST_OK      == retState                                              ) &&
        ( SPI_DMA_DIR_CNT      > dmaDir                                                ) &&
        ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ dmaDir ].Initialized )    )
    {
        const spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        dma_TransferSize_t                  dataSize = DMA_TRANSFER_SIZE_32BIT;
        dma_MemoryAddrInc_t                 addrInc  = DMA_MEMORY_ADDR_INCREMENT;
        dma_MemoryAddr_t                    memAddr  = 0u;
        dma_RequestState_t                  dmaState = DMA_REQUEST_ERROR;

        if( SPI_DMA_FRAME_BYTES_8BIT == xferCtx->FrameBytes )
        {
            dataSize = DMA_TRANSFER_SIZE_8BIT;
        }
        else if( SPI_DMA_FRAME_BYTES_16BIT == xferCtx->FrameBytes )
        {
            dataSize = DMA_TRANSFER_SIZE_16BIT;
        }
        else
        {
            /* Frame up to 32 bits */
        }

        if( SPI_DMA_DIR_TX == dmaDir )
        {
            if( SPI_NULL_PTR != xferCtx->Request.TxData )
            {
                memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Request.TxData;
            }
            else
            {
                /* Zero frames are transmitted from a static word */
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
                /* Received frames are discarded into a static word */
                memAddr = (dma_MemoryAddr_t)(uintptr_t)&spi_DmaDummyRx;
                addrInc = DMA_MEMORY_ADDR_STATIC;
            }
        }

        /* Peripheral and memory width equal the frame size (TXDR / RXDR access width, direct mode) */
        dmaState = Dma_Set_PeriphTransferSize( chState->DmaId, chState->ChannelId, dataSize );

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryTransferSize( chState->DmaId, chState->ChannelId, dataSize );
        }
        else
        {
            /* Peripheral data width configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddrIncrement( chState->DmaId, chState->ChannelId, addrInc );
        }
        else
        {
            /* Memory data width configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddr( chState->DmaId, chState->ChannelId, memAddr );
        }
        else
        {
            /* Memory increment configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( chState->DmaId, chState->ChannelId, (dma_DataCount_t)xferCtx->Request.XferSize );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Data count configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief Enables / disables SPI DMA request of one direction (TXDMAEN / RXDMAEN)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
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

    if( ( SPI_REQUEST_OK  == retState ) &&
        ( SPI_DMA_DIR_CNT  > dmaDir   )    )
    {
        const spi_RegValue_t reqMask  = ( SPI_DMA_DIR_TX == dmaDir ) ? SPI_CFG1_TXDMAEN : SPI_CFG1_RXDMAEN;
        const spi_RegValue_t reqValue = ( SPI_FUNCTION_ACTIVE == reqState ) ? reqMask : 0u;

        retState = Spi_Set_RegField( &periphReg->CFG1, reqMask, reqValue );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops one direction - DMA stream (if initialized) and SPI DMA request are disabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Set_Stop( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT  > periphId ) &&
        ( SPI_DMA_DIR_CNT > dmaDir   )    )
    {
        const spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        dma_RequestState_t                  dmaState = DMA_REQUEST_OK;

        if( SPI_FUNCTION_ACTIVE == chState->Initialized )
        {
            dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* No DMA stream was initialized for the direction */
        }

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
 * \brief Returns count of frames not yet moved by the DMA stream of one direction (NDTR)
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dmaDir     [in]: Data transfer direction
 * \param remaining [out]: Pointer to store the remaining data count. Must not be NULL.
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_Get_Remaining( spi_PeriphId_t periphId, spi_DmaDir_t dmaDir, dma_DataCount_t * const remaining )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    if( ( SPI_PERIPH_CNT       > periphId                                              ) &&
        ( SPI_DMA_DIR_CNT      > dmaDir                                                ) &&
        ( SPI_NULL_PTR        != remaining                                             ) &&
        ( SPI_FUNCTION_ACTIVE == spi_DmaChannelState[ periphId ][ dmaDir ].Initialized )    )
    {
        const spi_DmaChannelState_t * const chState  = &spi_DmaChannelState[ periphId ][ dmaDir ];
        const dma_RequestState_t            dmaState = Dma_Get_DataCount( chState->DmaId, chState->ChannelId, remaining );

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief DMA error processing - the transfer is aborted and the DMA transfer error is reported
 *        (STM32H7 DMA module reports only transfer errors)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Dma_XferError( spi_PeriphId_t periphId )
{
    return ( Spi_Set_XferError( periphId, SPI_XFER_ERROR_DMA_TRANSFER ) );
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

/* ================================ TASKS =================================== */

#endif /* STM32H7RS */
