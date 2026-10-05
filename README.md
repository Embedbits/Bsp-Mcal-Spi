# SPI Peripheral Driver

This module provides an abstraction layer for configuring and managing **SPI peripherals** on STM32F4 MCUs in **master or slave mode**.  
It supports SCK frequency with automatic prescaler calculation, frame format setup, NSS management, hardware CRC and data transfers in DMA, interrupt or polling mode.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family. The public interface (`Spi_Port.h`, `Spi_Types.h`) is common for all families - features not available on STM32F4 are refused by the functions.

---

## Features

- Master and slave mode
- SCK frequency in Hz, baudrate prescaler (/2 … /256) is calculated from the APB clock
- Clock polarity and phase (SPI modes 0 - 3)
- Data size 8 or 16 bits, MSB / LSB first
- Communication direction: full-duplex, simplex TX, simplex RX, half-duplex
- Motorola and TI frame format
- NSS management: software or hardware (master output / slave input)
- Hardware CRC (8 / 16 bits, configurable polynomial)
- Data transfer modes: DMA, ISR, POLL (`Spi_Task()`) - same request, same callbacks
- Error detection: overrun, CRC error, mode fault, TI frame error
- Standardized request state return values
- Versioning support for module management

Not supported on STM32F4 (refused): data size other than 8 / 16 bits, active high NSS, NSS pulse between frames, master idle timing (`Spi_Set_MasterTiming()` accepts zero cycles only), CRC initialized with ones, CRC in master half-duplex reception, I2S mode, multi-master (master with NSS input).

---

## Supported Hardware

| Peripheral        | Clock (`SPI_CLK_SRC_PCLK`) | Data size  | CRC length         | Max frames per transfer |
|-------------------|----------------------------|------------|--------------------|-------------------------|
| SPI1, SPI4 - SPI6 | PCLK2 (APB2)               | 8 / 16-bit | equal to data size | 65535                   |
| SPI2, SPI3        | PCLK1 (APB1)               | 8 / 16-bit | equal to data size | 65535                   |

DMA streams (`TxDmaChannelId` / `RxDmaChannelId` select the stream, channel selection is set by the module):

| Peripheral | Transmission                       | Reception                          |
|------------|------------------------------------|------------------------------------|
| SPI1       | DMA2 stream 3 or 5                 | DMA2 stream 0 or 2                 |
| SPI2       | DMA1 stream 4                      | DMA1 stream 3                      |
| SPI3       | DMA1 stream 5 or 7                 | DMA1 stream 0 or 2                 |
| SPI4       | DMA2 stream 1 or 4                 | DMA2 stream 0 or 3                 |
| SPI5       | DMA2 stream 4 or 6                 | DMA2 stream 3 or 5                 |
| SPI6       | DMA2 stream 5                      | DMA2 stream 6                      |

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

Every frame occupies 1 / 2 bytes of the buffer (data size 8 / 16 bits, little endian). Buffer use depends on the direction:

| Direction   | TxData                                  | RxData                                 |
|-------------|-----------------------------------------|----------------------------------------|
| Full-duplex | optional (NULL - zero frames are sent)  | optional (NULL - frames are discarded) |
| Simplex TX  | required                                | ignored                                |
| Simplex RX  | ignored                                 | required                               |
| Half-duplex | set for transmission (RxData = NULL)    | set for reception (TxData = NULL)      |

STM32F4 SPI has no transfer counter and no end of transfer flag - the module counts the frames and finishes the transfer when all frames were moved, the CRC frame was received and the transmission ended (TXE = 1, BSY = 0). Hardware configuration of the directions:

| Direction   | Master                                               | Slave                              |
|-------------|------------------------------------------------------|------------------------------------|
| Full-duplex | 2 lines                                              | 2 lines                            |
| Simplex TX  | bidirectional output (MOSI)                          | 2 lines, received frames discarded |
| Simplex RX  | 2 lines, zero frames transmitted (leave MOSI unused) | receive only (RXONLY)              |
| Half-duplex | bidirectional MOSI, reception stopped in last frame  | bidirectional MISO                 |

---

## GPIO Configuration

Pins are given in `spi_Config_t` (`SckPin`, `MisoPin`, `MosiPin`, `NssPin`) and configured by `Spi_Init()` as push-pull alternate function with `PinSpeed`. SCK gets pull-down (CPOL = 0) or pull-up (CPOL = 1) so the line keeps its idle level while the peripheral is disabled between transfers (`Spi_Set_ClockMode()` updates the pull), NSS gets pull-up. The pin is encoded by `SPI_PIN_ENCODE( periph, port, pin, alternate function )` - the alternate function number has to be taken from the device datasheet. The pin must belong to `PeriphId`, otherwise `Spi_Init()` returns error. Use `SPI_PIN_UNUSED` for signals not configured by the module (e.g. MISO in simplex TX, NSS with software NSS).

---

## Usage

```c
static const spi_DataConfig_t spiData =
{
    .XferMode             = SPI_XFER_MODE_DMA,
    .TxDmaPeriphId        = SPI_DMA_PERIPH_2,
    .TxDmaChannelId       = SPI_DMA_CHANNEL_3,   /* DMA2 stream 3 - SPI1_TX */
    .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
    .RxDmaPeriphId        = SPI_DMA_PERIPH_2,
    .RxDmaChannelId       = SPI_DMA_CHANNEL_0,   /* DMA2 stream 0 - SPI1_RX */
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
spiConfig.SckPin     = SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_5, GPIO_ALT_FUNC_5 );
spiConfig.MisoPin    = SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_6, GPIO_ALT_FUNC_5 );
spiConfig.MosiPin    = SPI_PIN_ENCODE( SPI_PERIPH_1, GPIO_PORT_A, GPIO_PIN_ID_7, GPIO_ALT_FUNC_5 );

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

- The peripheral is enabled (SPE) only for the duration of one transfer. With hardware NSS the master drives NSS low for the whole transfer. With software NSS the slave select has to be driven by the application.
- The real SCK frequency is the closest lower value reachable by the prescaler - read it back with `Spi_Get_BusFreq()`. After a change of the APB clock the frequency has to be configured again.
- CRC is reset by `Spi_Init()` - configure it afterwards with `Spi_Set_CrcConfig()`. The CRC length follows the data size.
- Master transfers are clocked frame by frame in ISR / POLL mode (next frame after the previous one was received) - the receiver can not overrun. End of transfer busy-waits for the CRC frame and the last transmitted frame (up to two frames) in the context of the event (SPI / DMA interrupt, `Spi_Task()`).
- Master half-duplex reception clocks continuously: the peripheral is disabled one SCK period after the last but one frame was read (RM0090 "Disabling the SPI"). The SCK period is measured by the SysTick counter (read only) when it runs, otherwise estimated by a busy-wait loop. The last but one frame has to be read within one frame period minus one SCK period (interrupt latency / `Spi_Task()` period), otherwise additional frames are clocked and discarded.
- Slave transfers and master half-duplex reception are clocked independently of the module - overrun is reported if frames are not read in time.
- Slave transfers with reception end with the last received frame (BSY is not checked - it may stay set in slave mode, device errata). Slave half-duplex transmission ends when the last frame was moved to the shift register and BSY is cleared - the master has to clock the frames without gaps.
- CRC in ISR / POLL mode: the CRC frame follows the last data frame without gap - the last received data frame has to be read within one frame period.
- DMA mode: 16-bit frames require buffers aligned to 2 bytes. The DMA streams have to be connected to the SPI requests (see table above).

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
