# SPI Peripheral Driver

This module provides an abstraction layer for configuring and managing **SPI peripherals** on STM32L4 / STM32L4+ MCUs in **master or slave mode**.
It supports SCK frequency with automatic prescaler calculation, frame format setup, NSS management, hardware CRC and data transfers in DMA, interrupt or polling mode.
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family. The public interface (`Spi_Port.h`, `Spi_Types.h`) is common for all families (STM32H5 interface, STM32G4 implementation as base) - features not available on STM32L4 are refused by the functions.

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
- Data transfer modes: DMA (STM32L4 request mapping, DMAMUX1 on STM32L4+), ISR, POLL (`Spi_Task()`) - same request, same callbacks
- Error detection: overrun, CRC error, mode fault, TI frame error
- Standardized request state return values
- Versioning support for module management

Not supported on STM32L4 (refused): data size above 16 bits, active high NSS, NSS pulse with software NSS, master idle timing (`Spi_Set_MasterTiming()` accepts zero cycles only), CRC length other than 8 / 16 bits or smaller than the data size, CRC initialized with ones, CRC in master half-duplex reception, multi-master (master with NSS input).

---

## Supported Hardware

| Peripheral | Devices                                           | Clock (`SPI_CLK_SRC_PCLK`) | Data size  | CRC length              | Max frames per transfer |
|------------|---------------------------------------------------|----------------------------|------------|-------------------------|-------------------------|
| SPI1       | all                                               | PCLK2 (APB2)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |
| SPI2       | all except STM32L432 / L442                       | PCLK1 (APB1)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |
| SPI3       | all except STM32L41x / L42x                       | PCLK1 (APB1)               | 4 - 16-bit | 8 / 16-bit (>= data)    | 65535                   |

SPI has no kernel clock multiplexer on STM32L4 - it is clocked by the APB clock.

DMA: `TxDma` / `RxDma` select the DMA channel from the lists `spi_TxDma_t` / `spi_RxDma_t` - one item per SPI peripheral, DMA peripheral and channel, named `SPI_TX_DMA_SPIx_DMAy_CHANNELz` / `SPI_RX_DMA_SPIx_DMAy_CHANNELz` (e.g. `SPI_TX_DMA_SPI1_DMA1_CHANNEL3`); transmit and receive channel must differ. STM32L4: fixed request mapping (DMA_CSELR, the request selection is part of the item) - SPI1 TX / RX DMA1 channel 3 / 2 or DMA2 channel 4 / 3, SPI2 TX / RX DMA1 channel 5 / 4, SPI3 TX / RX DMA2 channel 2 / 1; the lists contain these channels only. STM32L4+: any channel of DMA1 / DMA2, the SPI request (`DMA_REQ_SPIx_TX` / `DMA_REQ_SPIx_RX`) is routed by DMAMUX1, the lists contain all of them. Items of another SPI peripheral and `SPI_TX_DMA_UNUSED` / `SPI_RX_DMA_UNUSED` are refused in the DMA mode. The channel is disabled before every arming (the channel stays enabled after a normal mode transfer).

### Device errata

The workarounds of the STM32G4 module (same SPI IP, STM32G4 errata sheets ES0430 / ES0431 / ES0523) are kept - STM32L4 / STM32L4+ errata sheets are not reviewed yet:

| STM32G4 erratum | Handling |
|-----------------|----------|
| ES0430 2.18.1 - BSY bit may stay high when SPI is disabled | master transmitter is disabled only with the transmit FIFO empty (FTLVL = 0) and BSY = 0; master receive-only transfer (half-duplex reception) ignores BSY |
| ES0430 2.18.2 - BSY bit may stay high at the end of data transfer in slave mode | slave transfers with reception end with the last received frame (BSY is not checked); slave transmitter without reception is disabled while its last frame is transmitted (transmit FIFO empty), BSY is checked afterwards |
| Cortex-M4 r0p1 erratum 838869 - store immediate overlapping exception return might vector to incorrect interrupt | every SPI interrupt handler ends with `__DSB()` |

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
| DMA     | `Spi_Dma.c`    | Frames moved by DMA channels, end of transfer by DMA transfer complete      |

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

STM32L4 SPI has 32-bit FIFOs, but no transfer counter and no end of transfer flag - the module counts the frames and finishes the transfer when all frames were moved, the CRC frame(s) were received and the transmission ended (transmit FIFO empty, BSY = 0). Frames left in the receive FIFO are read out at the end of every transfer, data left in the transmit FIFO of an aborted / failed transfer are removed by the peripheral reset (the configuration is restored). Hardware configuration of the directions:

| Direction   | Master                                               | Slave                              |
|-------------|------------------------------------------------------|------------------------------------|
| Full-duplex | 2 lines                                              | 2 lines                            |
| Simplex TX  | bidirectional output (MOSI)                          | 2 lines, received frames discarded |
| Simplex RX  | 2 lines, zero frames transmitted (leave MOSI unused) | receive only (RXONLY)              |
| Half-duplex | bidirectional MOSI, reception stopped in last frame  | bidirectional MISO                 |

---

## GPIO Configuration

Pins are given in `spi_Config_t` (`SckPin`, `MisoPin`, `MosiPin`, `NssPin`) and configured by `Spi_Init()` as push-pull alternate function with `PinSpeed`. SCK gets pull-down (CPOL = 0) or pull-up (CPOL = 1) so the line keeps its idle level while the peripheral is disabled between transfers (`Spi_Set_ClockMode()` updates the pull), NSS gets pull-up. The pins are selected from the pin tables `spi_SckPin_t` / `spi_MisoPin_t` / `spi_MosiPin_t` / `spi_NssPin_t` (e.g. `SPI_SCK_PIN_SPI1_PA5`, `SPI_MISO_PIN_SPI1_PA6`, `SPI_NSS_PIN_SPI1_PA4`) - only pins available on the selected device line are defined. The pin tables were generated from the STM32CubeMX database, the alternate function of the item is part of its value. A pin missing in the tables can be encoded by `SPI_PIN_ENCODE( periph, port, pin, alternate function )` (alternate function number from the device datasheet). The pin must belong to `PeriphId`, otherwise `Spi_Init()` returns error. Use the `SPI_*_PIN_UNUSED` item of the table (equal to `SPI_PIN_UNUSED`) for signals not configured by the module (e.g. MISO in simplex TX, NSS with software NSS).

---

## Usage

```c
static const spi_DataConfig_t spiData =
{
    .XferMode             = SPI_XFER_MODE_DMA,
    .TxDma                = SPI_TX_DMA_SPI1_DMA1_CHANNEL3,   /* DMA1 channel 3 - SPI1_TX (STM32L4 mapping) */
    .TxDmaPriority        = SPI_DMA_PRIORITY_LOW,
    .RxDma                = SPI_RX_DMA_SPI1_DMA1_CHANNEL2,   /* DMA1 channel 2 - SPI1_RX (STM32L4 mapping) */
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

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Spi.c` - SPI registers emulated, RCC / NVIC / GPIO / DMA mocked; run on several presets (SPI2 / SPI3 availability differs).
- Integration tests (target, boards named by the MCU): `Tests/IntegrationTests/ItTest_Spi.c` - SPI1 master and SPI2 / SPI3 slave of the board wired together (pins free on the boards per STM32CubeMX board files, the slave uses software NSS):
  - Nucleo-64 / Nucleo-144 (NUCLEO-L476RG, L496ZG(-P), L4A6ZG, L4P5ZG, L4R5ZI(-P)): PA5 - PB13 (SCK), PA6 - PC2 (MISO), PA7 - PC3 (MOSI), master NSS PA4
  - NUCLEO-L432KC / L433RC-P, NUCLEO-L452RE / L452RE-P: PA1 - PB3, PA11 - PB4, PA12 - PB5 (slave SPI3), master NSS PB0
  - NUCLEO-L412RB-P: PA1 - PB10, PA11 - PC2, PA12 - PC3, master NSS PB0; NUCLEO-L412KB has no slave pins - tests with the slave are ignored (package read at run time)

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
