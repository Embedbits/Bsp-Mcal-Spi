/**
 * \author Mr.Nobody
 * \file Spi_Isr.h
 * \ingroup Spi
 * \brief Spi module interrupt data transfer handler (private to the Spi library)
 *
 */

#ifndef SPI_SPI_ISR_H
#define SPI_SPI_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Spi_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief SPI interrupt sources used by the data transfer handlers (SPI_IER enable bits) */
typedef enum
{
    SPI_ISR_IT_NONE   = 0u,                  /**< No interrupt source                                 */
    SPI_ISR_IT_RXP    = LL_SPI_IER_RXPIE,    /**< Receive FIFO contains a frame (RXP)                 */
    SPI_ISR_IT_TXP    = LL_SPI_IER_TXPIE,    /**< Transmit FIFO has space for a frame (TXP)           */
    SPI_ISR_IT_EOT    = LL_SPI_IER_EOTIE,    /**< End of transfer (EOT)                               */
    SPI_ISR_IT_UDR    = LL_SPI_IER_UDRIE,    /**< Underrun (UDR)                                      */
    SPI_ISR_IT_OVR    = LL_SPI_IER_OVRIE,    /**< Overrun (OVR)                                       */
    SPI_ISR_IT_TIFRE  = LL_SPI_IER_TIFREIE,  /**< TI frame format error (TIFRE)                       */
    SPI_ISR_IT_MODF   = LL_SPI_IER_MODFIE,   /**< Mode fault (MODF)                                   */
    SPI_ISR_IT_EVENTS = ( LL_SPI_IER_EOTIE  | LL_SPI_IER_UDRIE   |
                          LL_SPI_IER_OVRIE  | LL_SPI_IER_TIFREIE |
                          LL_SPI_IER_MODFIE ),                      /**< End of transfer and errors (DMA and ISR mode) */
    SPI_ISR_IT_ALL    = ( LL_SPI_IER_RXPIE  | LL_SPI_IER_TXPIE   |
                          LL_SPI_IER_EOTIE  | LL_SPI_IER_UDRIE   |
                          LL_SPI_IER_OVRIE  | LL_SPI_IER_TIFREIE |
                          LL_SPI_IER_MODFIE )                       /**< All interrupt sources used by the handlers    */
}   spi_IsrIt_t;


/** \brief Bit mask of \ref spi_IsrIt_t values */
typedef uint32_t spi_IsrItMask_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

spi_RequestState_t Spi_Isr_Check_Config ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
spi_RequestState_t Spi_Isr_XferInit     ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Isr_XferDeinit   ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Isr_XferStart    ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Isr_XferRun      ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Isr_XferStop     ( spi_PeriphId_t periphId );

spi_RequestState_t Spi_Isr_Set_ItActive   ( spi_PeriphId_t periphId, spi_IsrItMask_t itMask );
spi_RequestState_t Spi_Isr_Set_ItStart    ( spi_PeriphId_t periphId, spi_IsrItMask_t itMask );
spi_RequestState_t Spi_Isr_Set_ItInactive ( spi_PeriphId_t periphId, spi_IsrItMask_t itMask );

spi_RequestState_t Spi_Isr_Handler      ( spi_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_ISR_H */
