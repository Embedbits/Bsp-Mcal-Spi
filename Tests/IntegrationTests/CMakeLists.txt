#===================== Integration tests configuration ========================#
# Integration tests of Spi module on target (NUCLEO-G474RE, NUCLEO-G431RB). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# SPI1 (master) and SPI2 (slave) are wired together on CN10: SCK PA5 - PB13,
# MISO PA6 - PB14, MOSI PA7 - PB15.
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
