/**
 * \author Mr.Nobody
 * \file Spi_Port.h
 * \ingroup Spi
 * \brief Serial Peripheral Interface (SPI) MCAL module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef SPI_SPI_PORT_H
#define SPI_SPI_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Spi_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

spi_ModuleVersion_t     Spi_Get_ModuleVersion           ( void );

spi_RequestState_t      Spi_Init                        ( const spi_Config_t * const spiConfig );
spi_RequestState_t      Spi_Deinit                      ( spi_PeriphId_t periphId );
void                    Spi_Task                        ( void );

spi_RequestState_t      Spi_Get_DefaultConfig           ( spi_Config_t * const spiConfig );

/*------------------------- Peripheral configuration -------------------------*/

spi_RequestState_t      Spi_Get_PeriphState             ( spi_PeriphId_t periphId, spi_FlagState_t * const periphState );

spi_RequestState_t      Spi_Set_Mode                    ( spi_PeriphId_t periphId, spi_Mode_t mode );
spi_RequestState_t      Spi_Get_Mode                    ( spi_PeriphId_t periphId, spi_Mode_t * const mode );

spi_RequestState_t      Spi_Set_BusFreq                 ( spi_PeriphId_t periphId, spi_FreqHz_t busFreq );
spi_RequestState_t      Spi_Get_BusFreq                 ( spi_PeriphId_t periphId, spi_FreqHz_t * const busFreq );

spi_RequestState_t      Spi_Set_ClockMode               ( spi_PeriphId_t periphId, spi_ClockMode_t clockMode );
spi_RequestState_t      Spi_Get_ClockMode               ( spi_PeriphId_t periphId, spi_ClockMode_t * const clockMode );

spi_RequestState_t      Spi_Set_DataSize                ( spi_PeriphId_t periphId, spi_DataSize_t dataSize );
spi_RequestState_t      Spi_Get_DataSize                ( spi_PeriphId_t periphId, spi_DataSize_t * const dataSize );

spi_RequestState_t      Spi_Set_BitOrder                ( spi_PeriphId_t periphId, spi_BitOrder_t bitOrder );
spi_RequestState_t      Spi_Get_BitOrder                ( spi_PeriphId_t periphId, spi_BitOrder_t * const bitOrder );

spi_RequestState_t      Spi_Set_Direction               ( spi_PeriphId_t periphId, spi_Direction_t direction );
spi_RequestState_t      Spi_Get_Direction               ( spi_PeriphId_t periphId, spi_Direction_t * const direction );

spi_RequestState_t      Spi_Set_FrameFormat             ( spi_PeriphId_t periphId, spi_FrameFormat_t frameFormat );
spi_RequestState_t      Spi_Get_FrameFormat             ( spi_PeriphId_t periphId, spi_FrameFormat_t * const frameFormat );

/*------------------------------ NSS management ------------------------------*/

spi_RequestState_t      Spi_Set_NssConfig               ( spi_PeriphId_t periphId, const spi_NssConfig_t * const nssConfig );
spi_RequestState_t      Spi_Get_NssConfig               ( spi_PeriphId_t periphId, spi_NssConfig_t * const nssConfig );

spi_RequestState_t      Spi_Set_MasterTiming            ( spi_PeriphId_t periphId, spi_IdleCycles_t interDataIdle, spi_IdleCycles_t ssIdle );
spi_RequestState_t      Spi_Get_MasterTiming            ( spi_PeriphId_t periphId, spi_IdleCycles_t * const interDataIdle, spi_IdleCycles_t * const ssIdle );

/*----------------------------------- CRC ------------------------------------*/

spi_RequestState_t      Spi_Set_CrcConfig               ( spi_PeriphId_t periphId, const spi_CrcConfig_t * const crcConfig );
spi_RequestState_t      Spi_Get_CrcConfig               ( spi_PeriphId_t periphId, spi_CrcConfig_t * const crcConfig );
spi_RequestState_t      Spi_Get_CrcValue                ( spi_PeriphId_t periphId, spi_CrcValue_t * const txCrc, spi_CrcValue_t * const rxCrc );

/*-------------------- Data handling (DMA / ISR / POLL) ----------------------*/

spi_RequestState_t      Spi_Set_DataConfig              ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );
spi_RequestState_t      Spi_Get_DataConfig              ( spi_PeriphId_t periphId, spi_DataConfig_t * const dataConfig );

spi_RequestState_t      Spi_Set_XferStart               ( spi_PeriphId_t periphId, const spi_XferRequest_t * const xferRequest );
spi_RequestState_t      Spi_Set_XferStop                ( spi_PeriphId_t periphId );
spi_RequestState_t      Spi_Get_XferState               ( spi_PeriphId_t periphId, spi_FunctionState_t * const xferState );
spi_RequestState_t      Spi_Get_XferError               ( spi_PeriphId_t periphId, spi_XferErrorId_t * const xferError );

/*------------------------------ Interrupts ----------------------------------*/

spi_RequestState_t      Spi_Set_IrqPriority             ( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio );
spi_RequestState_t      Spi_Get_IrqPriority             ( spi_PeriphId_t periphId, spi_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* SPI_SPI_PORT_H */
