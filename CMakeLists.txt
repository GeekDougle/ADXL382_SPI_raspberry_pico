# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(adxl382_spi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Output stack usage detail files in build dir.
add_compile_options("-fstack-usage")

# Add executable. Default name is the project name, version 0.1
add_executable(adxl382_spi
        adxl382.c
        main.c     
        )

        # pull in common dependencies and additional spi hardware support
target_link_libraries(adxl382_spi
    pico_stdlib        # provides stdio.h, stdlib.h, string.h, errno.h, pico/stdlib.h
    pico_binary_info   # pico/binary_info.h (optional but commonly used)
    hardware_pll       # hardware/pll.h
    hardware_clocks    # hardware/clocks.h
    hardware_spi       # hardware/spi.h
    hardware_uart      # hardware/uart.h
    hardware_gpio      #hardware/gpio.h
    pico_time         #pico/time.h
)

# Make sure all files can be found
target_include_directories(adxl382_spi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

#execute all code in RAM for speed.
#pico_set_binary_type(adxl382_spi no_flash)

#enable uart and usb port
pico_enable_stdio_uart(adxl382_spi 0)
pico_enable_stdio_usb(adxl382_spi 1)

#increase USB TX buffer sizes to support faster transfer rates.
target_compile_definitions(adxl382_spi PUBLIC CFG_TUD_CDC_RX_BUFSIZE=1024)
target_compile_definitions(adxl382_spi PUBLIC CFG_TUD_CDC_TX_BUFSIZE=1024)
target_compile_definitions(adxl382_spi PUBLIC CFG_TUD_CDC_EP_BUFSIZE=1024)
#don't let data get thrown out 
target_compile_definitions(adxl382_spi PUBLIC PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)

# create map/bin/hex file etc.
pico_add_extra_outputs(adxl382_spi)

string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")


