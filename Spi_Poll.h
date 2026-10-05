/**
 * \author Mr.Nobody
 * \file Spi_Poll.h
 * \ingroup Spi
 * \brief Spi module polling data transfer handler (private to the Spi library)
 *
 */

#ifndef SPI_SPI_POLL_H
#define SPI_SPI_POLL_H

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

spi_RequestState_t Spi_Poll_Check_Config ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
spi_RequestState_t Spi_Poll_XferInit     ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Poll_XferDeinit   ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Poll_XferStart    ( spi_PeriphId_t periphId );
spi_RequestState_t Spi_Poll_XferStop     ( spi_PeriphId_t periphId );

spi_RequestState_t Spi_Poll_Task         ( spi_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_POLL_H */
