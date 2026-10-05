# SPI Peripheral Driver

This module provides an abstraction layer for configuring and managing **SPI peripherals** on STM32H5 MCUs.  
It supports initialization, bus frequency configuration with automatic prescaler calculation, frame format setup, NSS management and data transfers in DMA, interrupt or polling mode.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of SPI peripheral
- Support for default configuration retrieval
- Master and slave mode
- Kernel clock source selection - bus frequency in Hz, baudrate prescaler (/2 … /256) is calculated from kernel clock
- Clock polarity and phase (SPI modes 0 - 3)
- Data size 4 - 32 bits (SPI1 - SPI3) / 4 - 16 bits (SPI4 - SPI6), MSB / LSB first
- Communication direction: full-duplex, simplex TX, simplex RX, half-duplex
- Motorola and TI frame format
- NSS management: hardware (output / input) or software, NSS polarity, NSS pulse between data frames
- Master inter-data idleness (MIDI) and SS-to-first-clock delay (MSSI)
- Hardware CRC calculation (configurable polynomial and CRC length)
- Transfers longer than TSIZE limit (65535 frames) via TSER reload
- Data transfer modes: DMA (GPDMA), ISR, POLL (`Spi_Task()`) - same request, same callbacks
- Error detection: overrun, underrun (slave), CRC error, mode fault, TI frame error
- SCK / MISO / MOSI / NSS pin lists per device family
- Standardized request state return values
- Versioning support for module management

Not supported: I2S mode (handled by a separate module), multi-master arbitration.

---

## Supported Hardware

| MCU family | Peripheral    | Max data size | FIFO size  |
|------------|---------------|---------------|------------|
| STM32H503  | SPI1 - SPI3   | 32-bit        | 16 × 8-bit |
| STM32H563  | SPI1 - SPI3   | 32-bit        | 16 × 8-bit |
| STM32H563  | SPI4 - SPI6   | 16-bit        | 8 × 8-bit  |

---

## Public API

### Module Management
- `spi_ModuleVersion_t     Spi_Get_ModuleVersion   ( void );`
- `spi_RequestState_t      Spi_Init                ( const spi_Config_t * const spiConfig );`
- `spi_RequestState_t      Spi_Deinit              ( spi_PeriphId_t periphId );`
- `void                    Spi_Task                ( void );`
- `spi_RequestState_t      Spi_Get_DefaultConfig   ( spi_Config_t * const spiConfig );`

### Peripheral Control
- `spi_RequestState_t      Spi_Set_PeriphActive    ( spi_PeriphId_t periphId );`
- `spi_RequestState_t      Spi_Set_PeriphInactive  ( spi_PeriphId_t periphId );`
- `spi_RequestState_t      Spi_Get_PeriphState     ( spi_PeriphId_t periphId, spi_FlagState_t * const reqState );`

### Bus Configuration
The following setters require the peripheral to be inactive (SPE = 0).
- `spi_RequestState_t      Spi_Set_Mode            ( spi_PeriphId_t periphId, spi_Mode_t mode );`
- `spi_RequestState_t      Spi_Get_Mode            ( spi_PeriphId_t periphId, spi_Mode_t * const mode );`
- `spi_RequestState_t      Spi_Set_BusFreq         ( spi_PeriphId_t periphId, spi_BusFreq_t busFreq );`
- `spi_RequestState_t      Spi_Get_BusFreq         ( spi_PeriphId_t periphId, spi_BusFreq_t * const busFreq );`
- `spi_RequestState_t      Spi_Set_ClockMode       ( spi_PeriphId_t periphId, spi_ClockMode_t clockMode );`
- `spi_RequestState_t      Spi_Get_ClockMode       ( spi_PeriphId_t periphId, spi_ClockMode_t * const clockMode );`
- `spi_RequestState_t      Spi_Set_DataSize        ( spi_PeriphId_t periphId, spi_DataSize_t dataSize );`
- `spi_RequestState_t      Spi_Get_DataSize        ( spi_PeriphId_t periphId, spi_DataSize_t * const dataSize );`
- `spi_RequestState_t      Spi_Set_BitOrder        ( spi_PeriphId_t periphId, spi_BitOrder_t bitOrder );`
- `spi_RequestState_t      Spi_Get_BitOrder        ( spi_PeriphId_t periphId, spi_BitOrder_t * const bitOrder );`
- `spi_RequestState_t      Spi_Set_Direction       ( spi_PeriphId_t periphId, spi_Direction_t direction );`
- `spi_RequestState_t      Spi_Get_Direction       ( spi_PeriphId_t periphId, spi_Direction_t * const direction );`
- `spi_RequestState_t      Spi_Set_FrameFormat     ( spi_PeriphId_t periphId, spi_FrameFormat_t frameFormat );`
- `spi_RequestState_t      Spi_Get_FrameFormat     ( spi_PeriphId_t periphId, spi_FrameFormat_t * const frameFormat );`

### NSS Management
- `spi_RequestState_t      Spi_Set_NssConfig       ( spi_PeriphId_t periphId, const spi_NssConfig_t * const nssConfig );`
- `spi_RequestState_t      Spi_Get_NssConfig       ( spi_PeriphId_t periphId, spi_NssConfig_t * const nssConfig );`
- `spi_RequestState_t      Spi_Set_MasterTiming    ( spi_PeriphId_t periphId, spi_InterDataIdle_t midi, spi_SsIdle_t mssi );`
- `spi_RequestState_t      Spi_Get_MasterTiming    ( spi_PeriphId_t periphId, spi_InterDataIdle_t * const midi, spi_SsIdle_t * const mssi );`

### CRC
- `spi_RequestState_t      Spi_Set_CrcConfig       ( spi_PeriphId_t periphId, const spi_CrcConfig_t * const crcConfig );`
- `spi_RequestState_t      Spi_Get_CrcConfig       ( spi_PeriphId_t periphId, spi_CrcConfig_t * const crcConfig );`
- `spi_RequestState_t      Spi_Get_CrcValue        ( spi_PeriphId_t periphId, spi_CrcValue_t * const txCrc, spi_CrcValue_t * const rxCrc );`

### Data Transfers
- `spi_RequestState_t      Spi_Set_DataConfig      ( spi_PeriphId_t periphId, const spi_DataConfig_t * const dataConfig );`
- `spi_RequestState_t      Spi_Get_DataConfig      ( spi_PeriphId_t periphId, spi_DataConfig_t * const dataConfig );`
- `spi_RequestState_t      Spi_Set_XferStart       ( spi_PeriphId_t periphId, const spi_XferRequest_t * const request );`
- `spi_RequestState_t      Spi_Set_XferStop        ( spi_PeriphId_t periphId );`
- `spi_RequestState_t      Spi_Get_XferState       ( spi_PeriphId_t periphId, spi_XferState_t * const xferState );`
- `spi_RequestState_t      Spi_Get_XferError       ( spi_PeriphId_t periphId, spi_XferError_t * const xferError );`
- `spi_RequestState_t      Spi_Get_TxRegisterAddr  ( spi_PeriphId_t periphId, spi_RegAddr_t * const regAddr );`
- `spi_RequestState_t      Spi_Get_RxRegisterAddr  ( spi_PeriphId_t periphId, spi_RegAddr_t * const regAddr );`

### Flags
- `spi_RequestState_t      Spi_Get_Flag            ( spi_PeriphId_t periphId, spi_FlagId_t flagId, spi_FlagState_t * const flagState );`
- `spi_RequestState_t      Spi_Clear_Flag          ( spi_PeriphId_t periphId, spi_FlagId_t flagId );`

---

## Transfer Modes

The transfer mode is selected in `spi_DataConfig_t` and handled by a dedicated sub-module:

| Mode    | Source         | Description                                                           |
|---------|----------------|-----------------------------------------------------------------------|
| Polling | `Spi_Poll.c`   | FIFO serviced by `Spi_Task()`, completion reported by callback        |
| ISR     | `Spi_Isr.c`    | FIFO serviced in SPI interrupt (TXP / RXP / EOT)                      |
| DMA     | `Spi_Dma.c`    | Data moved by GPDMA channels, EOT and errors handled in SPI interrupt |

All modes share the same `spi_XferRequest_t` and the same completion / error callbacks.

---

## Interrupts and Callbacks
- `spi_RequestState_t      Spi_Set_IrqPriority     ( spi_PeriphId_t periphId, spi_IrqPrio_t irqPrio );`
- `spi_RequestState_t      Spi_Get_IrqPriority     ( spi_PeriphId_t periphId, spi_IrqPrio_t * const irqPrio );`

Callbacks are registered in `spi_DataConfig_t`:
- `XferCompleteCallback` - called after end of transfer (EOT)
- `ErrorCallback` - called on overrun, underrun, CRC error, mode fault or TI frame error

---

## GPIO Configuration
- `spi_RequestState_t      Spi_InitSckGpio         ( spi_SckPin_t pinId );`
- `spi_RequestState_t      Spi_InitMisoGpio        ( spi_MisoPin_t pinId );`
- `spi_RequestState_t      Spi_InitMosiGpio        ( spi_MosiPin_t pinId );`
- `spi_RequestState_t      Spi_InitNssGpio         ( spi_NssPin_t pinId );`

Pins are selected from `spi_SckPin_t` / `spi_MisoPin_t` / `spi_MosiPin_t` / `spi_NssPin_t` - only pins available on the selected device are defined. The pin must belong to `PeriphId`, otherwise `Spi_Init()` returns error. Use `SPI_*_PIN_NONE` for unused signals (e.g. MISO in simplex TX, NSS in software mode).

---

## Usage

```c
static const spi_DataConfig_t spiData =
{
    .XferMode             = SPI_XFER_MODE_DMA,
    .IrqPriority          = 5u,
    .XferCompleteCallback = App_SpiDone,
    .ErrorCallback        = App_SpiError,
};

spi_Config_t spiConfig;

(void)Spi_Get_DefaultConfig( &spiConfig );

spiConfig.PeriphId   = SPI_PERIPH_1;
spiConfig.Mode       = SPI_MODE_MASTER;
spiConfig.BusFreq    = 8000000u;
spiConfig.ClockMode  = SPI_CLOCK_MODE_0;
spiConfig.DataSize   = SPI_DATA_SIZE_8BIT;
spiConfig.DataConfig = &spiData;
spiConfig.SckPin     = SPI_SCK_PIN_SPI1_PA5;
spiConfig.MisoPin    = SPI_MISO_PIN_SPI1_PA6;
spiConfig.MosiPin    = SPI_MOSI_PIN_SPI1_PA7;
spiConfig.NssPin     = SPI_NSS_PIN_SPI1_PA4;

(void)Spi_Init( &spiConfig );

/* Full-duplex transfer of 4 bytes */
static const spi_Data_t txData[ 4u ] = { 0x9Fu, 0x00u, 0x00u, 0x00u };
static spi_Data_t       rxData[ 4u ];

const spi_XferRequest_t request =
{
    .TxData   = txData,
    .RxData   = rxData,
    .XferSize = 4u,
};

(void)Spi_Set_XferStart( SPI_PERIPH_1, &request );
```

---

## Usage Notes

- The SPI module is hardware dependent and must be configured per STM32 family branch.
- The **default configuration API** helps to ensure safe initialization.
- Bus configuration (mode, frequency, clock mode, data size, CRC, NSS) can be changed only while the peripheral is inactive.
- The real bus frequency is the closest lower value reachable by the prescaler - read it back with `Spi_Get_BusFreq()`.
- The `Task` function should be periodically called if polling mode is used.
- In slave mode, the TX FIFO should be filled before the master starts the clock, otherwise an underrun is reported.
- SPI4 - SPI6 have a limited feature set (max. 16-bit data, smaller FIFO) - requesting larger data size returns error.

---

## 🛠 CMake Integration

```cmake
target_link_libraries(User_Lib PRIVATE Spi_Lib)
```

1. Link `Spi_Lib` to your CMake library.
2. Include `Spi_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
