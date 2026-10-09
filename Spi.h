/**
 * \author Mr.Nobody
 * \file Spi.h
 * \ingroup Spi
 * \brief Spi module private interface shared between Spi.c and data transfer handlers
 *
 * This file is private to the Spi library (not exported through public headers). It connects
 * the module root (Spi.c) with data transfer mode handlers (Spi_Dma.c, Spi_Isr.c, Spi_Poll.c).
 * The transfer sequencing (direction of the transfer, peripheral enable, CRC phase, end of
 * transfer and error handling) and user callbacks are implemented once in Spi.c, the mode
 * handlers only move the data and enable their resources.
 *
 */

#ifndef SPI_SPI_H
#define SPI_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Spi_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back) */
typedef uint32_t spi_TimeoutCnt_t;


/** \brief Type representing raw value of SPI SR register (flags snapshot) */
typedef uint32_t spi_StatusFlags_t;


/** \brief Type representing raw register value / bit field */
typedef uint32_t spi_RegValue_t;


/** \brief Count of buffer bytes occupied by one data frame (1 or 2) */
typedef uint8_t spi_FrameBytes_t;


/** \brief Runtime context of data handling (one per SPI peripheral) */
typedef struct
{
    spi_DataConfig_t              Config;      /**< Copy of user data handling configuration                 */
    spi_FunctionState_t           InitState;   /**< Data handling is initialized                             */
    spi_XferRequest_t             Request;     /**< Copy of the running transfer request                     */
    spi_FrameBytes_t              FrameBytes;  /**< Buffer bytes per frame of the running transfer           */
    spi_DataCnt_t                 InFlightMax; /**< Transmitted, not yet received frames (master 1, slave 2) */
    spi_FunctionState_t           TxUsed;      /**< Frames are transmitted (TxData or zero frames)           */
    spi_FunctionState_t           RxUsed;      /**< Frames are received (stored or discarded)                */
    spi_FunctionState_t           CrcUsed;     /**< CRC phase follows the data frames                        */
    spi_FunctionState_t           CrcCheck;    /**< Received CRC is checked (not in simplex transmission)    */
    spi_FunctionState_t           RxStopUsed;  /**< Master receive-only transfer (continuous clock) - the
                                                    peripheral is disabled during the last frame            */
    spi_TimeoutCnt_t              SckCycles;   /**< Core clock cycles of one SCK period (RxStopUsed)        */
    spi_FunctionState_t           CpuRxTail;   /**< Last frame of DMA transfer is received by the CPU (ISR)  */
    volatile spi_DataCnt_t        TxIdx;       /**< Index of the next frame to be transmitted (ISR / POLL)   */
    volatile spi_DataCnt_t        RxIdx;       /**< Index of the next frame to be received (ISR / POLL)      */
    volatile spi_FunctionState_t  CrcNext;     /**< CRC transmission was requested (CRCNEXT)                 */
    volatile spi_FunctionState_t  RxStopped;   /**< Peripheral was disabled for the last frame (RxStopUsed)  */
    volatile spi_FunctionState_t  XferState;   /**< Transfer is running                                      */
    volatile spi_XferErrorId_t    XferError;   /**< Error of the running / last transfer                     */
}   spi_XferContext_t;


/** \brief Data transfer mode handler interface (one per mode) */
typedef struct
{
    spi_RequestState_t ( *CheckConfig )( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig ); /**< Mode specific configuration check            */
    spi_RequestState_t ( *Init        )( spi_PeriphId_t periphId );                                            /**< Mode resources initialization                */
    spi_RequestState_t ( *Deinit      )( spi_PeriphId_t periphId );                                            /**< Mode resources deinitialization              */
    spi_RequestState_t ( *Start       )( spi_PeriphId_t periphId );                                            /**< Transfer start (before SPE is set)           */
    spi_RequestState_t ( *Run         )( spi_PeriphId_t periphId );                                            /**< Transfer run (after SPE is set)              */
    spi_RequestState_t ( *Stop        )( spi_PeriphId_t periphId );                                            /**< Transfer stop (resources released)           */
    spi_RequestState_t ( *CheckDone   )( spi_PeriphId_t periphId );                                            /**< All frames were moved at the end of transfer */
}   spi_XferModeIf_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used for register read-back (common for all Spi module files).
 *  Same order of magnitude and role as I2C_TIMEOUT_RAW / USART_TIMEOUT_RAW. */
#define SPI_TIMEOUT_RAW                     ( (spi_TimeoutCnt_t)0x84FCBu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Spi.c - common services for data transfer handlers */
spi_RequestState_t  Spi_Get_PeriphReg      ( spi_PeriphId_t periphId, SPI_TypeDef ** const periphReg );
spi_RequestState_t  Spi_Get_XferContext    ( spi_PeriphId_t periphId, spi_XferContext_t ** const xferContext );

spi_RequestState_t  Spi_Set_RegField       ( volatile uint32_t * const regAddr, spi_RegValue_t fieldMask, spi_RegValue_t fieldValue );

spi_FunctionState_t Spi_Get_TxPending      ( spi_PeriphId_t periphId );
spi_RequestState_t  Spi_Set_XferDataStep   ( spi_PeriphId_t periphId, spi_StatusFlags_t statusFlags );
spi_RequestState_t  Spi_Set_XferEvents     ( spi_PeriphId_t periphId, spi_StatusFlags_t statusFlags );
spi_RequestState_t  Spi_Set_XferFinish     ( spi_PeriphId_t periphId );
spi_RequestState_t  Spi_Set_XferRxStop     ( spi_PeriphId_t periphId );
spi_RequestState_t  Spi_Set_XferError      ( spi_PeriphId_t periphId, spi_XferErrorId_t errorId );

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_H */
