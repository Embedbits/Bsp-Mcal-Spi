/**
 * \author Mr.Nobody
 * \file Spi_Dma.h
 * \ingroup Spi
 * \brief Spi module DMA data transfer handler (private to the Spi library)
 *
 */

#ifndef SPI_SPI_DMA_H
#define SPI_SPI_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Spi_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

spi_RequestState_t Spi_Dma_Check_Config ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
spi_RequestState_t Spi_Dma_XferInit     ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Dma_XferDeinit   ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Dma_XferStart    ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Dma_XferRun      ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Dma_XferStop     ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Dma_Check_Done   ( spi_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_DMA_H */
