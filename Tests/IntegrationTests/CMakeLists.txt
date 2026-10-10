#===================== Integration tests configuration ========================#
# Integration tests of Spi module on target (Nucleo-144 / 32F746GDISCOVERY). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# SPI5 (master) and SPI2 (slave) are wired together: 32F746GDISCOVERY SCK PF7 (A4) -
# PI1 (D13), MISO PF8 (A3) - PB14 (D12), MOSI PF9 (A2) - PB15 (D11); Nucleo-144 SCK
# PF7 - PB10, MISO PF8 - PC2, MOSI PF9 - PC3.
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Spi.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Spi.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()


if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

    # STM32H7R / STM32H7S (NUCLEO-H7S3L8)
    # The firmware needs 116 % of the 64 KB flash at -O0 and does not link, optimization is required.
    if(CMAKE_C_FLAGS_INTEGRATIONTEST MATCHES "-O0")
        message(WARNING "Integration test IT_Spi of STM32H7R / STM32H7S does not fit into 64 KB flash at -O0 - configure with -DINTEGRATION_TEST_OPTIMIZATION=-Os")
    endif()

    IntegrationTesting_Add_Test(
        NAME            Spi
        TEST_SOURCE     ItTest_Spi_H7RS.c
        SOURCES         ItTarget_Spi.c      # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        Spi_Lib             # Module under test
                        Rcc_Lib             # SPI clock state verification
                        Gpdma_Lib           # GPDMA channels of the SPI
        TIMEOUT         60
    )

else()

    # Classic STM32H7 (NUCLEO-H723ZG, NUCLEO-H7A3ZI-Q)
    IntegrationTesting_Add_Test(
        NAME            Spi
        TEST_SOURCE     ItTest_Spi.c
        SOURCES         ItTarget_Spi.c      # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        Spi_Lib             # Module under test
                        Rcc_Lib             # SPI clock state verification
                        Dma_Lib             # DMA channels of the SPI
        TIMEOUT         60
    )

endif()
