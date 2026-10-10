# SPI Peripheral Driver

This module provides an abstraction layer for configuring and managing **SPI peripherals** on STM32F7 MCUs in **master or slave mode**.  
It supports SCK frequency with automatic prescaler calculation, frame format setup, NSS management, hardware CRC and data transfers in DMA, interrupt or polling mode.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family. The public interface (`Spi_Port.h`, `Spi_Types.h`) is common for all families - features not available on STM32F7 are refused by the functions.

---

## Features

- Master and slave mode
- SCK frequency in Hz, baudrate prescaler (/2 … /256) is calculated from the APB clock
- Clock polarity and phase (SPI modes 0 - 3)
- Data size 4 - 16 bits, MSB / LSB first
- Communication direction: full-duplex, simplex TX, simplex RX, half-duplex
- Motorola and TI frame format
- NSS management: software or hardware (master output / slave input), NSS pulse between frames
- Hardware CRC (8 / 16 bits, configurable polynomial)
- Data transfer modes: DMA (DMA1 / DMA2 streams of the SPI requests), ISR, POLL (`Spi_Task()`) - same request, same callbacks
- Error detection: overrun, CRC error, mode fault, TI frame error
- Standardized request state return values
- Versioning support for module management

Not supported on STM32F7 (refused): data size above 16 bits, active high NSS, NSS pulse with software NSS, master idle timing (`Spi_Set_MasterTiming()` accepts zero cycles only), CRC length other than 8 / 16 bits or smaller than the data size, CRC initialized with ones, CRC in master half-duplex reception, I2S mode, multi-master (master with NSS input).

---

## Supported Hardware

| Peripheral | Devices                                   | Clock (`SPI_CLK_SRC_PCLK`) | Data size  | CRC length              | Max frames per transfer |
|------------|-------------------------------------------|----------------------------|------------|-------------------------|-------------------------|
| SPI1       | all                                       | PCLK2 (APB2)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |
| SPI2, SPI3 | all                                       | PCLK1 (APB1)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |
| SPI4, SPI5 | all                                       | PCLK2 (APB2)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |
| SPI6       | F745 / F746 / F750 / F756 / F765 / F767 / F769 / F777 / F779 | PCLK2 (APB2) | 4 - 16-bit | 8 / 16-bit (>= data) | 65535                |

The I2S clock (I2SSRC / PLLI2S) of SPI1 / SPI2 / SPI3 is used by I2S only - SPI mode is clocked by the APB clock.

DMA: `TxDma` / `RxDma` select the DMA stream from the lists `spi_TxDma_t` / `spi_RxDma_t` - one item per SPI peripheral, DMA peripheral and stream, named `SPI_TX_DMA_SPIx_DMAy_STREAMz` / `SPI_RX_DMA_SPIx_DMAy_STREAMz` (e.g. `SPI_TX_DMA_SPI1_DMA2_STREAM3`); the channel selection of the stream is part of the item. Items of another SPI peripheral, items of the other direction and `SPI_TX_DMA_UNUSED` / `SPI_RX_DMA_UNUSED` are refused in the DMA mode. Items of streams existing only on some device lines (see the table) are guarded by the CMSIS device line. The stream is disabled before every arming.

| Request | DMA | Streams / channel (all STM32F7) | STM32F76x / F77x only (channel 9) |
|---------|-----|---------------------------------|-----------------------------------|
| SPI1_RX | DMA2 | stream 0 / ch 3, stream 2 / ch 3 | |
| SPI1_TX | DMA2 | stream 3 / ch 3, stream 5 / ch 3 | |
| SPI2_RX | DMA1 | stream 3 / ch 0 | stream 1 / ch 9 |
| SPI2_TX | DMA1 | stream 4 / ch 0 | stream 6 / ch 9 |
| SPI3_RX | DMA1 | stream 0 / ch 0, stream 2 / ch 0 | |
| SPI3_TX | DMA1 | stream 5 / ch 0, stream 7 / ch 0 | |
| SPI4_RX | DMA2 | stream 0 / ch 4, stream 3 / ch 5 | |
| SPI4_TX | DMA2 | stream 1 / ch 4, stream 4 / ch 5 | stream 2 / ch 9 |
| SPI5_RX | DMA2 | stream 3 / ch 2, stream 5 / ch 7 | |
| SPI5_TX | DMA2 | stream 4 / ch 2, stream 6 / ch 7 | |
| SPI6_RX | DMA2 | stream 6 / ch 1 (SPI6 devices) | |
| SPI6_TX | DMA2 | stream 5 / ch 1 (SPI6 devices) | |

Source: STM32CubeMX database (DMA IP of STM32F722 / STM32F777). Channel 9 requires the 4-bit CHSEL of STM32F76x / F77x (`DMA_SxCR_CHSEL_3`).

### Device errata (ES0334)

ES0334 Rev 9 (STM32F76x / F77x), the same SPI IP is used on all STM32F7 lines.

| Erratum | Handling |
|---------|----------|
| BSY bit may stay high when SPI is disabled | master transmitter is disabled only with the transmit FIFO empty (FTLVL = 0) and BSY = 0; master receive-only transfer (half-duplex reception) ignores BSY |
| BSY bit may stay high at the end of data transfer in slave mode | slave transfers with reception end with the last received frame (BSY is not checked); slave transmitter without reception is disabled while its last frame is transmitted (transmit FIFO empty), BSY is checked afterwards |
| Wrong CRC in full-duplex mode handled by DMA with imbalanced setting of data counters | not affected - transmit and receive DMA counters are set to the count of data frames, the received CRC frame is read from the receive FIFO by the module |
| CRC error in SPI slave mode if internal NSS changes before CRC transfer (documentation erratum) | application note: a slave with CRC has to use software NSS when the master pulses NSS (NSS pulse mode can not be combined with CRC) |

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
- `Spi_Set_MasterTiming` / `Spi_Get_MasterTiming` (zero cycles only)

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

| Mode    | Source         | Description                                                                 |
|---------|----------------|-----------------------------------------------------------------------------|
| Polling | `Spi_Poll.c`   | Data register serviced by `Spi_Task()`, completion reported by callback     |
| ISR     | `Spi_Isr.c`    | Data register serviced in SPI interrupt (TXE / RXNE)                        |
| DMA     | `Spi_Dma.c`    | Frames moved by DMA streams, end of transfer by DMA transfer complete       |

All modes share the same `spi_XferRequest_t` and the same callbacks:
- `XferCompleteCallback` - all frames of the request were transferred
- `ErrorCallback` - transfer terminated by an error (`spi_XferErrorId_t`)

Every frame occupies 1 / 2 bytes of the buffer (data size 4 - 8 / 9 - 16 bits, little endian, right aligned). The data register is accessed by 8-bit accesses for frames up to 8 bits (RXNE threshold FRXTH = 8 bits) and by 16-bit accesses otherwise - the FIFO does not pack data. Buffer use depends on the direction:

| Direction   | TxData                                  | RxData                                 |
|-------------|-----------------------------------------|----------------------------------------|
| Full-duplex | optional (NULL - zero frames are sent)  | optional (NULL - frames are discarded) |
| Simplex TX  | required                                | ignored                                |
| Simplex RX  | ignored                                 | required                               |
| Half-duplex | set for transmission (RxData = NULL)    | set for reception (TxData = NULL)      |

STM32F7 SPI has 32-bit FIFOs, but no transfer counter and no end of transfer flag - the module counts the frames and finishes the transfer when all frames were moved, the CRC frame(s) were received and the transmission ended (transmit FIFO empty, BSY = 0). Frames left in the receive FIFO are read out at the end of every transfer, data left in the transmit FIFO of an aborted / failed transfer are removed by the peripheral reset (the configuration is restored). Hardware configuration of the directions:

| Direction   | Master                                               | Slave                              |
|-------------|------------------------------------------------------|------------------------------------|
| Full-duplex | 2 lines                                              | 2 lines                            |
| Simplex TX  | bidirectional output (MOSI)                          | 2 lines, received frames discarded |
| Simplex RX  | 2 lines, zero frames transmitted (leave MOSI unused) | receive only (RXONLY)              |
| Half-duplex | bidirectional MOSI, reception stopped in last frame  | bidirectional MISO                 |

---

## GPIO Configuration

Pins are given in `spi_Config_t` (`SckPin`, `MisoPin`, `MosiPin`, `NssPin`) and configured by `Spi_Init()` as push-pull alternate function with `PinSpeed`. SCK gets pull-down (CPOL = 0) or pull-up (CPOL = 1) so the line keeps its idle level while the peripheral is disabled between transfers (`Spi_Set_ClockMode()` updates the pull), NSS gets pull-up. The pins are selected from the pin tables `spi_SckPin_t` / `spi_MisoPin_t` / `spi_MosiPin_t` / `spi_NssPin_t` (e.g. `SPI_SCK_PIN_SPI1_PA5`, `SPI_MISO_PIN_SPI3_PC11`, `SPI_NSS_PIN_SPI5_PF6`) - only pins available on the selected device line are defined (SPI6 does not exist on STM32F72x / F73x, some pins exist only on STM32F76x / F77x, e.g. SPI1 SCK on PG11). The pin tables were generated from the STM32CubeMX database, the alternate function of the item is part of its value (SPI1, SPI2, SPI4, SPI5 AF5, SPI3 AF6, SPI6 AF5 / AF8). A pin missing in the tables can be encoded by `SPI_PIN_ENCODE( periph, port, pin, alternate function )` (alternate function number from the device datasheet). The pin must belong to `PeriphId`, otherwise `Spi_Init()` returns error. Use the `SPI_*_PIN_UNUSED` item of the table (equal to `SPI_PIN_UNUSED`) for signals not configured by the module (e.g. MISO in simplex TX, NSS with software NSS).

---

## Usage

```c
static const spi_DataConfig_t spiData =
{
    .XferMode             = SPI_XFER_MODE_DMA,
    .TxDma                = SPI_TX_DMA_SPI1_DMA2_STREAM3,   /* DMA2 stream 3 - SPI1_TX (channel 3) */
    .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
    .RxDma                = SPI_RX_DMA_SPI1_DMA2_STREAM0,   /* DMA2 stream 0 - SPI1_RX (channel 3) */
    .RxDmaPriority        = SPI_DMA_PRIORITY_HIGH,
    .IrqPriority          = 5u,
    .XferCompleteCallback = App_SpiDone,
    .ErrorCallback        = App_SpiError,
};

spi_Config_t spiConfig;

(void)Spi_Get_DefaultConfig( &spiConfig );

spiConfig.PeriphId   = SPI_PERIPH_1;
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

- The peripheral is enabled (SPE) only for the duration of one transfer. With hardware NSS the master drives NSS low for the whole transfer (with NSS pulse NSS is released for one SCK period between the frames - Motorola format, CPHA = 0). With software NSS the slave select has to be driven by the application.
- The real SCK frequency is the closest lower value reachable by the prescaler - read it back with `Spi_Get_BusFreq()`. After a change of the APB clock the frequency has to be configured again.
- CRC is reset by `Spi_Init()` - configure it afterwards with `Spi_Set_CrcConfig()`. The data size can not exceed the length of the enabled CRC. 16-bit CRC of frames up to 8 bits is transferred as two frames.
- The SPI CRC calculator supports odd polynomials only - even polynomials are refused.
- Master transfers are clocked frame by frame in ISR / POLL mode (next frame after the previous one was received) - the receiver can not overrun. End of transfer busy-waits for the CRC frame(s) and the frames left in the transmit FIFO in the context of the event (SPI / DMA interrupt, `Spi_Task()`).
- Master half-duplex reception clocks continuously: the peripheral is disabled one SCK period after the last but one frame was read (RM0440 "Disabling the SPI"). The SCK period is measured by the SysTick counter (read only) when it runs, otherwise estimated by a busy-wait loop. Additional frames clocked after the stop stay in the receive FIFO and are discarded at the end of the transfer.
- Slave transfers and master half-duplex reception are clocked independently of the module - overrun is reported if frames are not read in time.
- Slave half-duplex transmission ends when the last frame was moved to the shift register and BSY is cleared - the master has to clock the frames without gaps.
- CRC in ISR / POLL mode: the CRC frame follows the last data frame without gap - the last received data frame has to be read within one frame period.
- DMA mode: frames of 9 - 16 bits require buffers aligned to 2 bytes.

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
