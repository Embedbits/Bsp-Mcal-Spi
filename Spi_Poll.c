/**
 * \author Mr.Nobody
 * \file Spi_Poll.c
 * \ingroup Spi
 * \brief Spi module polling data transfer handler
 *
 * SPI_XFER_MODE_POLL - no interrupt is used. Spi_Task() calls Spi_Poll_Task() which polls SPI
 * flags of the running transfer: frames are moved (TXP / RXP, one FIFO at most) and end of
 * transfer and errors are processed by Spi.c (Spi_Set_XferEvents) per call.
 *
 * \note  Master in full-duplex / transmitting directions clocks only when its transmit FIFO
 *        contains data - a slow task only slows the transfer down. Simplex RX master and slave
 *        are clocked independently of the task - overrun / underrun may be reported.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Spi_Poll.h"                       /* Self include                   */
#include "Spi.h"                            /* Module private interface       */
#include "Stm32_spi.h"                      /* SPI RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks polling related part of the data handling configuration
 *
 * \note  Polling mode has no mode specific resources - every peripheral is supported.
 *
 * \param periphId   [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref SPI_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_Check_Config( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig )
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
 * \brief Initializes polling data transfer (no HW resource is needed)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_XferInit( spi_PeriphId_t periphId )
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


/**
 * \brief Deinitializes polling data transfer (no HW resource is used)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_XferDeinit( spi_PeriphId_t periphId )
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


/**
 * \brief Starts polling data transfer (frames are moved by Spi_Task())
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_XferStart( spi_PeriphId_t periphId )
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


/**
 * \brief Stops polling data transfer (Spi_Task() stops polling when the transfer state in Spi.c
 *        is inactive, no HW resource is used)
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_XferStop( spi_PeriphId_t periphId )
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


/**
 * \brief Polls SPI flags of one peripheral and moves the running polling transfer (called from
 *        Spi_Task())
 *
 * \param periphId [in]: SPI peripheral identification, value from \ref spi_PeriphId_t
 *
 * \return Function processing state. Returns \ref SPI_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref SPI_REQUEST_ERROR.
 */
spi_RequestState_t Spi_Poll_Task( spi_PeriphId_t periphId )
{
    spi_RequestState_t retState  = SPI_REQUEST_ERROR;
    SPI_TypeDef *      periphReg = SPI_NULL_PTR;

    retState = Spi_Get_PeriphReg( periphId, &periphReg );

    if( SPI_REQUEST_OK == retState )
    {
        retState = Spi_Set_XferDataStep( periphId );

        /* Flags are read after the data step - end of transfer may have been reached meanwhile */
        const spi_StatusFlags_t  statusFlags = LL_SPI_ReadReg( periphReg, SR );
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

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
