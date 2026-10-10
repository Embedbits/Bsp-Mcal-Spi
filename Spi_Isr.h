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

/** \brief SPI interrupt sources used by the data transfer handlers (SPI_CR2 enable bits) */
typedef enum
{
    SPI_ISR_IT_NONE   = 0u,                  /**< No interrupt source                                     */
    SPI_ISR_IT_RXNE   = LL_SPI_CR2_RXNEIE,   /**< Receive buffer contains a frame (RXNE)                  */
    SPI_ISR_IT_TXE    = LL_SPI_CR2_TXEIE,    /**< Transmit buffer is empty (TXE)                          */
    SPI_ISR_IT_ERR    = LL_SPI_CR2_ERRIE,    /**< Errors (OVR, MODF, CRCERR, FRE)                         */
    SPI_ISR_IT_EVENTS = LL_SPI_CR2_ERRIE,    /**< Events processed in DMA and ISR mode (errors)           */
    SPI_ISR_IT_ALL    = ( LL_SPI_CR2_RXNEIE | LL_SPI_CR2_TXEIE |
                          LL_SPI_CR2_ERRIE )  /**< All interrupt sources used by the handlers              */
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
