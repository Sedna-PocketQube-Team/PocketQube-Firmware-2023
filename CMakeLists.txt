cmake_minimum_required(VERSION 3.12)

# Build functions from Pico SDK
include(/Users/cmd/pico/pico-sdk/external/pico_sdk_import.cmake)

# Set project name and C/C++ Standards
project(Sedna C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD arduino_nano_rp2040_connect)

# creates pico-sdk subdir in our project for the libs
pico_sdk_init()

# add library
add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

# Executable source file
add_executable(${PROJECT_NAME}
    main.c
    hw_config.c
    bmp280.c
)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(${PROJECT_NAME}  
        ${CMAKE_CURRENT_LIST_DIR}/buzzer.pio
)

# create output files (uf2, map, bin, hex)
pico_add_extra_outputs(${PROJECT_NAME})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_time
    hardware_clocks
    hardware_uart
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_pio
    hardware_clocks
    hardware_i2c
    hardware_dma
    hardware_irq
    FatFs_SPI
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)