/**
 * \author Mr.Nobody
 * \file Spi_Isr.c
 * \ingroup Spi
 * \brief Spi module interrupt data transfer handler
 *
 * SPI_XFER_MODE_ISR - data frames are moved by the SPI interrupt (TXE / RXNE), errors (OVR /
 * MODF / CRCERR / FRE) and the end of transfer are processed by Spi.c (Spi_Set_XferEvents).
 *
 * The transmit interrupt (TXE) is enabled only while a frame may be written - after the last
 * frame or while the in-flight limit is reached it would fire continuously.
 *
 * The handler is used in SPI_XFER_MODE_DMA too - data are moved by DMA there, the interrupt
 * processes errors and the last frame of master half-duplex reception (received by the CPU).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Spi_Isr.h"                        /* Self include                   */
#include "Spi.h"                            /* Module private interface       */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

static spi_RequestState_t Spi_Isr_Set_TxItUpdate( spi_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks interrupt related part of the data handling configuration
 *
 * \note  Interrupt mode has no mode specific resources - every peripheral is supported.
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
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
 * \brief Initializes interrupt data transfer - all SPI interrupt sources are disabled until
 *        the transfer start (NVIC is configured by Spi.c)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_XferInit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Deinitializes interrupt data transfer - all SPI interrupt sources are disabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_XferDeinit( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Prepares interrupt data transfer before the peripheral is enabled (interrupt sources
 *        stay disabled until Spi_Isr_XferRun())
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_XferStart( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Runs interrupt data transfer after the peripheral is enabled - data interrupts of the
 *        used directions and error interrupt are enabled (\ref Spi_Isr_Set_ItStart)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_XferRun( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState = SPI_REQUEST_ERROR;
    spi_XferContext_t * xferCtx  = SPI_NULL_PTR;

    retState = Spi_Get_XferContext( periphId, &xferCtx );

    if( SPI_REQUEST_OK == retState )
    {
        spi_IsrItMask_t itMask = SPI_ISR_IT_EVENTS;

        if( SPI_FUNCTION_ACTIVE == xferCtx->TxUsed )
        {
            itMask |= SPI_ISR_IT_TXE;
        }
        else
        {
            /* Nothing is transmitted */
        }

        if( SPI_FUNCTION_ACTIVE == xferCtx->RxUsed )
        {
            itMask |= SPI_ISR_IT_RXNE;
        }
        else
        {
            /* Nothing is received */
        }

        retState = Spi_Isr_Set_ItStart( periphId, itMask );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    return ( retState );
}


/**
 * \brief Stops interrupt data transfer - all SPI interrupt sources are disabled
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_XferStop( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState = SPI_REQUEST_ERROR;

    retState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Enables SPI interrupt sources (SPI_CR2 enable bits)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param itMask   [in]: Bit mask of \ref spi_IsrIt_t values
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_Set_ItActive( spi_PeriphId_t periphId, spi_IsrItMask_t itMask )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *      periphReg = SPI_NULL_PTR;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( ( SPI_REQUEST_OK == retState                                         ) &&
        ( 0u             == ( itMask & ~( (spi_IsrItMask_t)SPI_ISR_IT_ALL ) ) )    )
    {
        retState = Spi_Set_RegField( &periphReg->CR2, itMask, itMask );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables SPI interrupt sources of a running transfer (SPI_CR2 enable bits) without
 *        read-back verification
 *
 * \note  The enabled interrupt can be pending already (TXE is set while the data register is
 *        empty) - the interrupt service routine runs right after the write and modifies the
 *        enable bits itself (TXE disabled after the last frame, all sources disabled at the end
 *        of transfer). Read-back of the written value would report a false failure, the
 *        function has to be the last step of the transfer start.
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param itMask   [in]: Bit mask of \ref spi_IsrIt_t values
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_Set_ItStart( spi_PeriphId_t periphId, spi_IsrItMask_t itMask )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *      periphReg = SPI_NULL_PTR;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( ( SPI_REQUEST_OK == retState                                         ) &&
        ( 0u             == ( itMask & ~( (spi_IsrItMask_t)SPI_ISR_IT_ALL ) ) )    )
    {
        SET_BIT( periphReg->CR2, itMask );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables SPI interrupt sources (SPI_CR2 enable bits)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param itMask   [in]: Bit mask of \ref spi_IsrIt_t values
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_Set_ItInactive( spi_PeriphId_t periphId, spi_IsrItMask_t itMask )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *      periphReg = SPI_NULL_PTR;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( ( SPI_REQUEST_OK == retState                                         ) &&
        ( 0u             == ( itMask & ~( (spi_IsrItMask_t)SPI_ISR_IT_ALL ) ) )    )
    {
        retState = Spi_Set_RegField( &periphReg->CR2, itMask, 0u );
    }
    else
    {
        retState = SPI_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief SPI interrupt processing - data frames are moved in ISR mode (and the last frame of DMA
 *        master half-duplex reception), errors and the end of transfer are processed
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Isr_Handler( spi_PeriphId_t periphId )
{
    spi_RequestState_t  retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *       periphReg = SPI_NULL_PTR;
    spi_XferContext_t * xferCtx   = SPI_NULL_PTR;

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
        /* Flags before the data step - read of DR followed by read of SR clears overrun */
        const spi_StatusFlags_t stepFlags = LL_SPI_ReadReg( periphReg, SR );

        if( ( SPI_FUNCTION_ACTIVE == xferCtx->XferState       ) &&
            ( ( SPI_XFER_MODE_ISR   == xferCtx->Config.XferMode ) ||
              ( SPI_FUNCTION_ACTIVE == xferCtx->CpuRxTail       )    )    )
        {
            retState = Spi_Set_XferDataStep( periphId, stepFlags );

            if( ( SPI_REQUEST_OK    == retState                ) &&
                ( SPI_XFER_MODE_ISR == xferCtx->Config.XferMode )    )
            {
                retState = Spi_Isr_Set_TxItUpdate( periphId );
            }
            else
            {
                /* Data step failed or last frame of DMA reception (nothing is transmitted) */
            }
        }
        else
        {
            /* Data are moved by DMA or no transfer is running */
        }

        /* Flags are read after the data step too - errors of both snapshots are processed */
        const spi_StatusFlags_t  statusFlags = stepFlags | LL_SPI_ReadReg( periphReg, SR );
        const spi_RequestState_t eventState  = Spi_Set_XferEvents( periphId, statusFlags );

        if( ( SPI_REQUEST_OK == retState   ) &&
            ( SPI_REQUEST_OK == eventState )    )
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

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Enables the transmit interrupt (TXE) only while a frame may be written
 *        (\ref Spi_Get_TxPending)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
static spi_RequestState_t Spi_Isr_Set_TxItUpdate( spi_PeriphId_t periphId )
{
    spi_RequestState_t        retState  = SPI_REQUEST_ERROR;
    const spi_FunctionState_t txPending = Spi_Get_TxPending( periphId );

    if( SPI_FUNCTION_ACTIVE == txPending )
    {
        retState = Spi_Isr_Set_ItActive( periphId, SPI_ISR_IT_TXE );
    }
    else
    {
        retState = Spi_Isr_Set_ItInactive( periphId, SPI_ISR_IT_TXE );
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
