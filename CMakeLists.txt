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

# It looks like either of the following methods work (add_compile_option or set(...), but they must be immediately after pico_sdk_init())
 # Append the -fstack-usage flag to C compiler flags
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-usage")
# If you have C++ code, also add it to CXX flags:
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-usage")
add_compile_options("-fstack-usage")

# Add executable. Default name is the project name, version 0.1
add_executable(adxl382_spi
        adxl382.c
        main.c     
        )

        #enable uart and usb port
pico_enable_stdio_uart(adxl382_spi 0)
pico_enable_stdio_usb(adxl382_spi 1)
# pull in common dependencies and additional spi hardware support
target_link_libraries(adxl382_spi pico_stdlib hardware_spi)

#execute all code in RAM for speed.
pico_set_binary_type(adxl382_spi no_flash)

#increase USB TX buffer sizes to support faster transfer rates.
target_compile_definitions(adxl382_spi PUBLIC CFG_TUD_CDC_TX_BUFSIZE=1024)
target_compile_definitions(adxl382_spi PUBLIC CFG_TUD_CDC_EP_BUFSIZE=1024)

# create map/bin/hex file etc.
pico_add_extra_outputs(adxl382_spi)

string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")


