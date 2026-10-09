# SPI Peripheral Driver

This module provides an abstraction layer for configuring and managing **SPI peripherals** on STM32H5 MCUs in **master or slave mode**.  
It supports kernel clock selection, SCK frequency with automatic prescaler calculation, frame format setup, NSS management, hardware CRC and data transfers in DMA, interrupt or polling mode.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Master and slave mode
- Kernel clock source selection - SCK frequency in Hz, baudrate prescaler (/2 … /256) is calculated from the kernel clock
- Clock polarity and phase (SPI modes 0 - 3)
- Data size 4 - 32 bits (SPI1 - SPI3) / 4 - 16 bits (SPI4 - SPI6), MSB / LSB first
- Communication direction: full-duplex, simplex TX, simplex RX, half-duplex
- Motorola and TI frame format
- NSS management: software or hardware (master output / slave input), NSS polarity, NSS pulse between frames
- Master inter-data idleness (MIDI) and SS-to-first-clock delay (MSSI)
- Hardware CRC (configurable length, polynomial and initialization pattern)
- Data transfer modes: DMA (GPDMA), ISR, POLL (`Spi_Task()`) - same request, same callbacks
- Error detection: overrun, underrun (slave), CRC error, mode fault, TI frame error
- Standardized request state return values
- Versioning support for module management

Not supported: I2S mode, multi-master (master with NSS input), transfers longer than one TSIZE (no TSER reload on STM32H5).

---

## Supported Hardware

| Peripheral  | Devices          | Max data size | CRC length     | FIFO size  | Max frames per transfer |
|-------------|------------------|---------------|----------------|------------|-------------------------|
| SPI1 - SPI3 | all STM32H5      | 32-bit        | 4 - 32 bits    | 16 × 8-bit | 65534                   |
| SPI4 - SPI6 | where available  | 16-bit        | 8 or 16 bits   | 8 × 8-bit  | 1022                    |

Kernel clock sources (`spi_ClkSrc_t`) - a list with one item per peripheral and source, named `SPI_CLK_SRC_SPI<n>_<source>`
(e.g. `SPI_CLK_SRC_SPI1_PLL2P`). The sources a peripheral does not offer are not in the list, the item shall belong to the
peripheral of the configuration (`spi_Config_t::PeriphId`):

| Peripheral  | Available sources                                         |
|-------------|-----------------------------------------------------------|
| SPI1 - SPI3 | PLL1Q, PLL2 (P), PLL3 (P, devices with PLL3)              |
| SPI4 - SPI6 | PCLK, PLL2 (Q), PLL3 (Q, devices with PLL3), HSI, CSI, HSE |

---

## Public API

### Module Management
- `spi_ModuleVersion_t Spi_Get_ModuleVersion( void );`
- `spi_RequestState_t  Spi_Init( const spi_Config_t * const spiConfig );`
- `spi_RequestState_t  Spi_Deinit( spi_PeriphId_t periphId );`
- `void                Spi_Task( void );`
- `spi_RequestState_t  Spi_Get_DefaultConfig( spi_Config_t * const spiConfig );`

### Peripheral Configuration
All setters require that no transfer is running.
- `Spi_Get_PeriphState` (SPE - set only while a transfer is running)
- `Spi_Set_Mode` / `Spi_Get_Mode`
- `Spi_Set_BusFreq` / `Spi_Get_BusFreq`
- `Spi_Set_ClockMode` / `Spi_Get_ClockMode`
- `Spi_Set_DataSize` / `Spi_Get_DataSize`
- `Spi_Set_BitOrder` / `Spi_Get_BitOrder`
- `Spi_Set_Direction` / `Spi_Get_Direction`
- `Spi_Set_FrameFormat` / `Spi_Get_FrameFormat`

### NSS Management
- `Spi_Set_NssConfig` / `Spi_Get_NssConfig`
- `Spi_Set_MasterTiming` / `Spi_Get_MasterTiming`

### CRC
- `Spi_Set_CrcConfig` / `Spi_Get_CrcConfig`
- `Spi_Get_CrcValue`

### Data Transfers
- `Spi_Set_DataConfig` / `Spi_Get_DataConfig`
- `Spi_Set_XferStart` / `Spi_Set_XferStop`
- `Spi_Get_XferState` / `Spi_Get_XferError`

### Interrupts
- `Spi_Set_IrqPriority` / `Spi_Get_IrqPriority`

---

## Transfer Modes

The transfer mode is selected in `spi_DataConfig_t` and handled by a dedicated sub-module:

| Mode    | Source         | Description                                                            |
|---------|----------------|------------------------------------------------------------------------|
| Polling | `Spi_Poll.c`   | FIFO serviced by `Spi_Task()`, completion reported by callback         |
| ISR     | `Spi_Isr.c`    | FIFO serviced in SPI interrupt (TXP / RXP), end of transfer by EOT     |
| DMA     | `Spi_Dma.c`    | Frames moved by GPDMA channels, EOT and errors handled in SPI interrupt |

All modes share the same `spi_XferRequest_t` and the same callbacks:
- `XferCompleteCallback` - all frames of the request were transferred
- `ErrorCallback` - transfer terminated by an error (`spi_XferErrorId_t`)

Every frame occupies 1 / 2 / 4 bytes of the buffer (data size up to 8 / 16 / 32 bits, little endian). Buffer use depends on the direction:

| Direction   | TxData                                  | RxData                                 |
|-------------|-----------------------------------------|----------------------------------------|
| Full-duplex | optional (NULL - zero frames are sent)  | optional (NULL - frames are discarded) |
| Simplex TX  | required                                | ignored                                |
| Simplex RX  | ignored                                 | required                               |
| Half-duplex | set for transmission (RxData = NULL)    | set for reception (TxData = NULL)      |

---

## GPIO Configuration

Pins are given in `spi_Config_t` (`SckPin`, `MisoPin`, `MosiPin`, `NssPin`) and configured by `Spi_Init()` as push-pull alternate function with `PinSpeed`. The pins are selected from the pin tables `spi_SckPin_t` / `spi_MisoPin_t` / `spi_MosiPin_t` / `spi_NssPin_t` (e.g. `SPI_SCK_PIN_SPI1_PA5`, `SPI_MISO_PIN_SPI1_PA6`, `SPI_NSS_PIN_SPI1_PA4`) - only pins available on the selected device line are defined. The pin tables were generated from the STM32CubeMX database, the alternate function of the item is part of its value. A pin missing in the tables can be encoded by `SPI_PIN_ENCODE( periph, port, pin, alternate function )` (alternate function number from the device datasheet). The pin must belong to `PeriphId`, otherwise `Spi_Init()` returns error. Use the `SPI_*_PIN_UNUSED` item of the table (equal to `SPI_PIN_UNUSED`) for signals not configured by the module (e.g. MISO in simplex TX, NSS with software NSS).

---

## Usage

```c
static const spi_DataConfig_t spiData =
{
    .XferMode             = SPI_XFER_MODE_DMA,
    .TxDmaPeriphId        = SPI_DMA_PERIPH_1,
    .TxDmaChannelId       = SPI_DMA_CHANNEL_0,
    .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
    .RxDmaPeriphId        = SPI_DMA_PERIPH_1,
    .RxDmaChannelId       = SPI_DMA_CHANNEL_1,
    .RxDmaPriority        = SPI_DMA_PRIORITY_HIGH,
    .IrqPriority          = 5u,
    .XferCompleteCallback = App_SpiDone,
    .ErrorCallback        = App_SpiError,
};

spi_Config_t spiConfig;

(void)Spi_Get_DefaultConfig( &spiConfig );

spiConfig.PeriphId   = SPI_PERIPH_1;
spiConfig.ClkSrc     = SPI_CLK_SRC_SPI1_PLL1Q;
spiConfig.BusFreq    = 8000000u;
spiConfig.DataConfig = &spiData;
spiConfig.SckPin     = SPI_SCK_PIN_SPI1_PA5;
spiConfig.MisoPin    = SPI_MISO_PIN_SPI1_PA6;
spiConfig.MosiPin    = SPI_MOSI_PIN_SPI1_PA7;

(void)Spi_Init( &spiConfig );

/* Full-duplex transfer of 4 bytes (slave select driven by application GPIO) */
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

- The peripheral is enabled (SPE) only for the duration of one transfer - configuration registers are write protected while it is enabled. Master keeps its pins driven between transfers (AFCNTR), so SCK stays at the idle level.
- With hardware NSS the master drives NSS active for the whole transfer (or pulses it between frames with `Pulse`). With software NSS the slave select has to be driven by the application.
- The real SCK frequency is the closest lower value reachable by the prescaler - read it back with `Spi_Get_BusFreq()`. After a change of the kernel clock the frequency has to be configured again.
- CRC and master idle timing are reset by `Spi_Init()` - configure them afterwards with `Spi_Set_CrcConfig()` / `Spi_Set_MasterTiming()`.
- DMA mode: buffers must be aligned to the frame size (2 / 4 bytes) and one transfer must not exceed 65535 bytes.
- POLL mode: `Spi_Task()` has to be called periodically. Master full-duplex / TX transfers only slow down with a slow task; simplex RX master and slave transfers can report overrun / underrun.
- SPI4 - SPI6 have a limited feature set (16-bit data, 8 / 16-bit CRC, 1022 frames per transfer) - larger values are refused.

---

## 🛠 CMake Integration

```cmake
target_link_libraries(User_Lib PRIVATE Spi_Lib)
```

1. Link `Spi_Lib` to your CMake library.
2. Include `Spi_Port.h` in your project.

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
