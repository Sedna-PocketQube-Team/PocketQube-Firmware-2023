#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "string.h"

#include "bmp280.h"
#include "sd_card.h"
#include "ff.h"
#include "buzzer.pio.h"
#include "LoRa-RP2040.h"

// I2C
#define I2C_HARDWARE_UNIT			(&i2c0_inst)
#define I2C_PORT i2c0
#define I2C_DEVICE_ADDRESS 0x76

// UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0 
#define UART_RX_PIN 1
#define BUFFER_SIZE 6

extern "C" void bmp280_init();
extern "C" void bmp280_get_calib_params(bmp280_calib_param*);
extern "C" int32_t bmp280_convert_temp(long, bmp280_calib_param*);
extern "C" int32_t bmp280_convert_pressure(long, long, bmp280_calib_param*);


int8_t state = -1;
uint32_t statemillis = 0;

int main() {
    const int csPin = 5;          // LoRa radio chip select
    const int resetPin = -1;        // LoRa radio reset
    const int irqPin = 6;          // change for your board; must be a hardware interrupt pin

    stdio_init_all();
    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(915E6)) {         // initialize ratio at 915 MHz
        
        if (true) {
            sleep_ms(3000);
            printf("LoRa init failed. Check your connections.");                   // if failed, do nothing
        }
        ;
    } else {
        sleep_ms(5000);
        LoRa.dumpRegisters();
        while (true) ;
    }

    // I2C Setup
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Set up our UART with the required speed.
    uart_init(UART_ID, 9600);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // claim DMA channels
    uint dma_tx 	= dma_claim_unused_channel(true);
    uint dma_rx 	= dma_claim_unused_channel(true);    


    static uint16_t txbuf[32];
    static uint8_t rxbuf[BUFFER_SIZE];

    static uint8_t buf_idx = 0;
    static char mybuf[256];


    // Start BMP280

    // configure BMP280
    bmp280_init();

    // retrieve fixed compensation params
    struct bmp280_calib_param params;
    bmp280_get_calib_params(&params);

    int32_t raw_temperature;
    int32_t raw_pressure;
    uint16_t raw_uv_adc;
    

    // UV Sensor - ADC init
    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);


    static char cmd3[] = "$PMTK251,115200*1F\r\n"; // 115200 Baud Rate

    sleep_ms(250); // sleep so that data polling and register update don't collide

    uart_write_blocking(UART_ID, (uint8_t*)cmd3, strlen(cmd3));



    sleep_ms(250);

    uart_set_baudrate(UART_ID, 115200);


    static char cmd[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // RMCGGA

    sleep_ms(1000); // sleep so that data polling and register update don't collide

    uart_write_blocking(UART_ID, (uint8_t*)cmd, strlen(cmd));


    static char cmd2[] = "$PMTK220,100*2F\r\n"; // 10 Hz Updates

    sleep_ms(250); // sleep so that data polling and register update don't collide

    uart_write_blocking(UART_ID, (uint8_t*)cmd2, strlen(cmd2));

   
    static char startlogging[] = "$PMTK185,0*22\r\n"; // 10 Hz Updates

    sleep_ms(250); // sleep so that data polling and register update don't collide

    uart_write_blocking(UART_ID, (uint8_t*)startlogging, strlen(startlogging));
    /*static char verify[] = "$PMTK605*31\r\n";

    uart_write_blocking(UART_ID, verify, strlen(verify));*/

    sleep_ms(250);

    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    char buf[100];
     // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
    }

    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio, &buzzer_program);

    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / ((float)(1000)*200.0);
    //float div = (float)clock_get_hz(clk_sys) / 500;

    // Initialize the program using the helper function in our .pio file
    buzzer_program_init(pio, sm, offset, 15, div);

    // Start running our PIO program in the state machine
    pio_sm_set_enabled(pio, sm, true);



    // buzzer test

    /*
    while (1) {

        for(int buzHz=100; buzHz <= 2000; buzHz+=100) {
            div = (float)clock_get_hz(clk_sys) / ((float)(buzHz)*200.0);
            pio_sm_set_clkdiv(pio, sm, div);
            printf("Freq: %.2f Hz\n", (float)buzHz);
            pio_sm_set_enabled(pio, sm, true);
            sleep_ms(500);
            pio_sm_set_enabled(pio, sm, false);
            sleep_ms(100);
            pio_sm_set_enabled(pio, sm, true);
            sleep_ms(500);
            pio_sm_set_enabled(pio, sm, false);
            sleep_ms(500);
        }

        // alarm: 800 Hz, 1550 Hz

        int buzHz = 1550;

        div = (float)clock_get_hz(clk_sys) / ((float)(buzHz)*200.0);

        pio_sm_set_clkdiv(pio, sm, div);
        pio_sm_set_enabled(pio, sm, true);
        sleep_ms(500);
        pio_sm_set_enabled(pio, sm, false);
        sleep_ms(100);
        
        buzHz = 800;

        div = (float)clock_get_hz(clk_sys) / ((float)(buzHz)*200.0);

        pio_sm_set_clkdiv(pio, sm, div);
        pio_sm_set_enabled(pio, sm, true);
        sleep_ms(500);
        pio_sm_set_enabled(pio, sm, false);
        sleep_ms(1000);


        // buz = d/200
        for(int buzHz=100; buzHz <= 10000; buzHz+=1) {
            div = (float)clock_get_hz(clk_sys) / ((float)(buzHz)*200.0);
            pio_sm_set_clkdiv(pio, sm, div);
            printf("Freq: %.2f Hz\n", (float)buzHz);
            sleep_ms(10);
        }
    }
    */

    
    while(1) {
        if (uart_is_readable(UART_ID)) {
            uart_read_blocking(UART_ID, (uint8_t*)(mybuf+buf_idx), 1);
            buf_idx += 1;
            if(buf_idx == 256 || mybuf[buf_idx-1] == '\n') {
                mybuf[buf_idx-1] = '\0';
                //printf("%d\n", buf_idx);
                //puts(mybuf);
                buf_idx = 0;

                // Open file for writing ()
                fr = f_open(&fil, "GPS.txt", FA_WRITE | FA_OPEN_APPEND);
                if (fr != FR_OK) {
                    printf("ERROR: Could not open file (%d)\r\n", fr);
                }

                // Write something to file
                ret = f_printf(&fil, mybuf);
                if (ret < 0) {
                    printf("ERROR: Could not write to file (%d)\r\n", ret);
                    f_close(&fil);
                }

                // Close file
                fr = f_close(&fil);
                if (fr != FR_OK) {
                    printf("ERROR: Could not close file (%d)\r\n", fr);
                }
            }
        }

        if (state==-1 &&  to_ms_since_boot((get_absolute_time())) - statemillis > 1000) {
            state = 0;

            rxbuf[0]=0;

            // BME680 registers: https://cdn-shop.adafruit.com/product-files/3660/BME680.pdf

            txbuf[0] = REG_PRESSURE_MSB; //0xD0;
            txbuf[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
            //txbuf[1] |= I2C_IC_DATA_CMD_RESTART_BITS;
            

            for (size_t i = 1; i < BUFFER_SIZE+1; ++i) {
                txbuf[i] = I2C_IC_DATA_CMD_CMD_BITS;
            }
            txbuf[BUFFER_SIZE] |= I2C_IC_DATA_CMD_STOP_BITS;

            i2c_get_hw(i2c0)->enable = 0;
            i2c_get_hw(i2c0)->tar = 0x76;
            i2c_get_hw(i2c0)->enable = 1;
            

            dma_channel_config c1 = dma_channel_get_default_config(dma_tx);

            // channel_config_set_transfer_data_size(&_DMA_Transmit_Channel, DMA_SIZE_8);
            channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);
            channel_config_set_dreq(&c1, i2c_get_dreq(i2c0, true));
            channel_config_set_read_increment(&c1, true);
            channel_config_set_write_increment(&c1, false);
            dma_channel_configure(	dma_tx,
                                    &c1,
                                    &i2c_get_hw(i2c0)->data_cmd,	// Write Address
                                    txbuf, 										// Read Address
                                    BUFFER_SIZE+2,								// Element Count (Each element is of size transfer_data_size)
                                    true);	 									// DO start directly
            //dma_channel_set_irq1_enabled(dma_tx, true);

            dma_channel_config rx_config = dma_channel_get_default_config(dma_rx);
            channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
            channel_config_set_dreq(&rx_config, i2c_get_dreq(i2c0, false));
            channel_config_set_read_increment(&rx_config, false);
            channel_config_set_write_increment(&rx_config, true);
            dma_channel_configure(  dma_rx,
                                    &rx_config,
                                    rxbuf,
                                    &i2c_get_hw(i2c0)->data_cmd,
                                    BUFFER_SIZE,
                                    true); // START
            
            raw_uv_adc = adc_read();
        } else if (state == 0 && !dma_channel_is_busy(dma_rx)) {
            

            int32_t raw_pressure = (rxbuf[0] << 12) | (rxbuf[1] << 4) | (rxbuf[2] >> 4);
            int32_t raw_temperature = (rxbuf[3] << 12) | (rxbuf[4] << 4) | (rxbuf[5] >> 4);

            // BME 280 Extra humidity:
            //int32_t raw_humidity = (rxbuf[6] << 8) | (rxbuf[7]);
            

            
            int32_t temperature = bmp280_convert_temp(raw_temperature, &params);
            int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);

            state = 1;
            dma_channel_unclaim(dma_tx);
            dma_channel_unclaim(dma_rx);
            
            // Open file for writing ()
            fr = f_open(&fil, "sensordata.csv", FA_WRITE | FA_OPEN_APPEND);
            if (fr != FR_OK) {
                printf("ERROR: Could not open file (%d)\r\n", fr);
            }

            // Write something to file
            printf("%.3f, %.2f, %d\n", pressure / 1000.f, temperature / 100.f, raw_uv_adc);
            ret = f_printf(&fil, "%.3f, %.2f, %d\n", pressure / 1000.f, temperature / 100.f, raw_uv_adc);
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
            }

            // Close file
            fr = f_close(&fil);
            if (fr != FR_OK) {
                printf("ERROR: Could not close file (%d)\r\n", fr);
            }

            state = -1;
            dma_tx 	= dma_claim_unused_channel(true);
            dma_rx 	= dma_claim_unused_channel(true);   
            statemillis = to_ms_since_boot((get_absolute_time())); 
        }
    }

    return 0;
}



