cmake_minimum_required(VERSION 3.12)

# Build functions from Pico SDK
include(${CMAKE_CURRENT_LIST_DIR}/../pico-sdk/external/pico_sdk_import.cmake)

# Set project name and C/C++ Standards
project(Sedna C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD arduino_nano_rp2040_connect)

# creates pico-sdk subdir in our project for the libs
pico_sdk_init()

# add library
add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)
add_subdirectory(lib/pico-lora/src)
add_subdirectory(lib/pico-tflmicro/)
add_subdirectory(lib/minmea/)

# Executable source file
add_executable(${PROJECT_NAME}
    main.cpp
    hw_config.c
    bmp280.c
    main_functions.cpp
    model.cpp
)

add_library(pico_ADXL345 lib/ADXL345_RP2040/ADXL345.h lib/ADXL345_RP2040/ADXL345.cpp)
target_link_libraries(pico_ADXL345 pico_stdlib hardware_i2c)

target_include_directories(${PROJECT_NAME} PUBLIC lib/pico-lora/src)
target_include_directories(${PROJECT_NAME} PUBLIC lib/pico-tflmicro/)
target_include_directories(${PROJECT_NAME} PUBLIC lib/minmea/)

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
    pico_sync
    pico_multicore
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
    LoRa_pico_lib
    pico-tflmicro
    pico_ADXL345
    minmea
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)