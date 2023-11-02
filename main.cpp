#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/mutex.h"

#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"

#include "string.h"

#include "qubesettings.h"
#include "bmp280.h"
#include "lib/ADXL345_RP2040/ADXL345.h"
#include "sd_card.h"
#include "ff.h"
#include "buzzer.pio.h"
#include "LoRa-RP2040.h"
#include "minmea.h"
#include "main_functions.h"
#include "hw_config.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// I2C
#define I2C_HARDWARE_UNIT			(&i2c0_inst)
#define I2C_PORT i2c0
#define I2C_DEVICE_ADDRESS 0x76

// UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0 
#define UART_RX_PIN 1

// DMA
#define BMP_BUFFER_SIZE 6
#define IMU_BUFFER_SIZE 6

extern "C" {
    void bmp280_init();
    void bmp280_get_calib_params(bmp280_calib_param*);
    int32_t bmp280_convert_temp(long, bmp280_calib_param*);
    int32_t bmp280_convert_pressure(long, long, bmp280_calib_param*);

    #ifdef BM_HUMIDITY
    int32_t bme280_convert_humidity(int32_t, int32_t, struct bmp280_calib_param* params) ;
    #endif
}

// sensor data:
int32_t raw_temperature;
int32_t raw_pressure;
uint16_t raw_uv_adc;
int32_t temperature;
int32_t pressure;
uint32_t humidity;
int32_t pressure_tfpredicted;
float altitude_tfpredicted;
auto_init_mutex(pressureTfMtx); // Create a mutex that protects the "pressure" variable from being accessed by 2 threads (tensorflow and main code) at the same time
auto_init_mutex(predictionTfMtx); // Create a mutex that protects the inference results from being accessed by 2 threads (tensorflow and main code) at the same time


int8_t state = -1;
int32_t filesystemerrcnt = 0;
uint32_t statemillis = 0;
uint32_t lora_location_broadcast_millis = 0;

void core1_loop() {
    setup(); // TFMicro setup

    while (true) {
        // make a copy of the current pressure (so that the inference doesn't block saving data)
        mutex_enter_blocking(&pressureTfMtx);
        float pressure_tfcopy = (float)pressure;
        mutex_exit(&pressureTfMtx);

        // run tensorflow
        float altitude_tfcopy = loop(pressure_tfcopy);

        if (altitude_tfcopy != -1) {
            mutex_enter_blocking(&predictionTfMtx);
            pressure_tfpredicted = pressure_tfcopy;
            altitude_tfpredicted = altitude_tfcopy;
            mutex_exit(&predictionTfMtx);
        }

        // wait 800ms
        sleep_ms(800);
    }
}

int main() {
    const int csPin = 5;          // LoRa radio chip select
    const int resetPin = -1;      // LoRa radio reset (disconnected)
    const int irqPin = 6;

    stdio_init_all();
    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(915E6)) {         // initialize ratio at 915 MHz
        if (true) {
            sleep_ms(3000);
            printf("LoRa init failed. Check your connections.\n");
        }
    } else {
        sleep_ms(3000);
        printf("LoRa init successful!\n");
        LoRa.dumpRegisters();
    }

    multicore_launch_core1(core1_loop); // start Tensorflow on second core

    // I2C Setup
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);


    // GPS Setup
    // Set up our UART with the required speed.
    uart_init(UART_ID, 9600);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // GPS Buffer
    static uint8_t buf_idx = 0;
    static char gpsbuf[258];


    // I2C DMA
    // claim DMA channels
    uint dma_tx 	= dma_claim_unused_channel(true);
    uint dma_rx 	= dma_claim_unused_channel(true);    

    // DMA buffers
    static uint16_t txbuf[32];
    static uint8_t rxbuf[BMP_BUFFER_SIZE];





    // Start BMP280
    // configure BMP280
    bmp280_init();

    // retrieve fixed compensation params
    struct bmp280_calib_param params;
    bmp280_get_calib_params(&params);
    

    #ifdef SEDNA_ACCELEROMETER
    // Start ADXL345
    ADXL345 accelerometer = ADXL345();
    accelerometer.begin();
    accelerometer.setRange(ADXL345_RANGE_16_G);
    #endif

    // UV Sensor - ADC init
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0); // Select ADC input 0 (GPIO26)


    // GPS Init
    const char cmd3[] = "$PMTK251,115200*1F\r\n"; // Set 115200 Baud Rate for GPS
    sleep_ms(250);
    uart_write_blocking(UART_ID, (uint8_t*)cmd3, strlen(cmd3));
    sleep_ms(250);
    uart_set_baudrate(UART_ID, 115200); // Change GPS Baud rate once complete

    const char cmd[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29";
    //const char cmd[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    sleep_ms(1000);
    uart_write_blocking(UART_ID, (uint8_t*)cmd, strlen(cmd));

    const char cmd2[] = "$PMTK220,100*2F\r\n"; // 10 Hz GPS Updates
    sleep_ms(250); 
    uart_write_blocking(UART_ID, (uint8_t*)cmd2, strlen(cmd2));

    const char startlogging[] = "$PMTK185,0*22\r\n"; // Start logging data to internal GPS memory, just in case we need it.
    sleep_ms(250);
    uart_write_blocking(UART_ID, (uint8_t*)startlogging, strlen(startlogging));

    /*static char verify[] = "$PMTK605*31\r\n";
    uart_write_blocking(UART_ID, verify, strlen(verify));*/

    sleep_ms(250);


    // SD Card
    FRESULT fr;
    FATFS fs;
    FIL fil;
    FILINFO fno;


    int ret;
    char buf[100];

    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
    }


    // Buzzer PIO
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

    /*  Main loop

        The I2C/DMA part of the code includes a simple finite state machine with the following states: 
             -1) Configure and start I2C DMA & read ADC sensor; go to state 1 when data transmission finished
              0) When DMA finished, save data & go to state -1)
    */

    while(1) {
        /*if(to_ms_since_boot((get_absolute_time())) - lora_location_broadcast_millis > 2000)
        {
            mutex_enter_blocking(&sd_get_by_num(0)->mutex);
            sleep_ms(100);
            mutex_exit(&sd_get_by_num(0)->mutex);
            lora_location_broadcast_millis = to_ms_since_boot((get_absolute_time()));
        }*/

        if (filesystemerrcnt > 10) {
            printf("Resetting SD Card\r\n");

            //card_initialized = false;
            f_unmount("0:");

            // Initialize SD card
            if (!sd_init_driver()) {
                printf("ERROR: Could not initialize SD card\r\n");
            }

            // Mount drive
            fr = f_mount(&fs, "0:", 1);
            if (fr != FR_OK) {
                printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
            }

            filesystemerrcnt = 0;
            sleep_ms(3000);
        }

        if (uart_is_readable(UART_ID)) {
            uart_read_blocking(UART_ID, (uint8_t*)(gpsbuf+buf_idx), 1);
            buf_idx += 1;
            if(buf_idx == 256 || gpsbuf[buf_idx-1] == '\n' || gpsbuf[buf_idx-1] == '\r') {
                gpsbuf[buf_idx] = '\0';
                
                #ifdef VERBOSE_GPS_LOG
                puts(gpsbuf);
                #endif
                buf_idx = 0;

                // Using minmea library for GPS parsing: https://github.com/kosma/minmea

                switch (minmea_sentence_id(gpsbuf, false)) {
                    case MINMEA_SENTENCE_RMC: {
                        struct minmea_sentence_rmc frame;
                        if (minmea_parse_rmc(&frame, gpsbuf)) {
                            printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                                    frame.latitude.value, frame.latitude.scale,
                                    frame.longitude.value, frame.longitude.scale,
                                    frame.speed.value, frame.speed.scale);
                            printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                                    minmea_rescale(&frame.latitude, 1000),
                                    minmea_rescale(&frame.longitude, 1000),
                                    minmea_rescale(&frame.speed, 1000));
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));
                        }
                    } break;

                    case MINMEA_SENTENCE_GGA: {
                        struct minmea_sentence_gga frame;
                        if (minmea_parse_gga(&frame, gpsbuf)) {
                            // Open file for writing ()
                            #ifdef GPS_PARSED_LOG
                            printf("[%d:%d:%d] $GGA: fix quality: %d, altitude: %d%c, Coordinates: %f %f \n", frame.time.hours, frame.time.minutes, frame.time.seconds, frame.fix_quality, frame.altitude, frame.altitude_units, minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));
                            #endif

                            fr = f_open(&fil, "GPS.txt", FA_WRITE | FA_OPEN_APPEND);
                            if (fr != FR_OK) {
                                printf("ERROR: Could not open file (%d)\r\n", fr);
                                filesystemerrcnt ++;

                            }

                            // Write something to file
                            ret = f_printf(&fil, gpsbuf);
                            if (ret < 0) {
                                printf("ERROR: Could not write to file (%d)\r\n", ret);
                                f_close(&fil);

                                filesystemerrcnt++;
                            }

                            // Close file
                            fr = f_close(&fil);
                            if (fr != FR_OK) {
                                printf("ERROR: Could not close file (%d)\r\n", fr);

                                filesystemerrcnt++;
                            }
                        }
                    } break;

                    case MINMEA_SENTENCE_GSV: {
                        struct minmea_sentence_gsv frame;
                        if (minmea_parse_gsv(&frame, gpsbuf)) {
                            printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                            printf("$GSV: satellites in view: %d\n", frame.total_sats);
                            for (int i = 0; i < 4; i++)
                                printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                                    frame.sats[i].nr,
                                    frame.sats[i].elevation,
                                    frame.sats[i].azimuth,
                                    frame.sats[i].snr);
                        }
                    } break;
                }

                /* Update the access time and modification time */
                
                /*fno.fdate = (WORD)(((2007 - 1980) * 512U) | 1 * 32U | 9);
                fno.ftime = (WORD)(9 * 2048U | 41 * 32U | 00 / 2U);
                fr = f_utime("GPS.txt", &fno);
                if (fr != FR_OK) {
                    printf("ERROR: Could not change modification time (%d)\r\n", fr);
                    filesystemerrcnt++;
                }*/
            }
        }

        if (state==-1 &&  to_ms_since_boot((get_absolute_time())) - statemillis > 10) {
            state = 0;

            rxbuf[0]=0;

            txbuf[0] = REG_PRESSURE_MSB; //0xD0;
            txbuf[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
            //txbuf[1] |= I2C_IC_DATA_CMD_RESTART_BITS;

            for (size_t i = 1; i < BMP_BUFFER_SIZE+1; ++i) {
                txbuf[i] = I2C_IC_DATA_CMD_CMD_BITS;
            }
            txbuf[BMP_BUFFER_SIZE] |= I2C_IC_DATA_CMD_STOP_BITS;

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
                                    BMP_BUFFER_SIZE+2,								// Element Count (Each element is of size transfer_data_size)
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
                                    BMP_BUFFER_SIZE,
                                    true); // START
            
            raw_uv_adc = adc_read();
        } else if (state==0 && !dma_channel_is_busy(dma_rx)) {
            // DMA Transfer finished
            raw_pressure = (rxbuf[0] << 12) | (rxbuf[1] << 4) | (rxbuf[2] >> 4);
            raw_temperature = (rxbuf[3] << 12) | (rxbuf[4] << 4) | (rxbuf[5] >> 4);

            // BME 280 Extra humidity:
            #ifdef BM_HUMIDITY
            int32_t raw_humidity = (rxbuf[6] << 8) | (rxbuf[7]);
            humidity = bme280_convert_humidity(raw_humidity, raw_temperature, &params);
            #endif
            

            
            temperature = bmp280_convert_temp(raw_temperature, &params);
            mutex_enter_blocking(&pressureTfMtx);
            pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
            mutex_exit(&pressureTfMtx);

            dma_channel_unclaim(dma_tx);
            dma_channel_unclaim(dma_rx);

            dma_tx 	= dma_claim_unused_channel(true);
            dma_rx 	= dma_claim_unused_channel(true); 

            state = 1;

            #ifdef SEDNA_ACCELEROMETER

            rxbuf[0]=0;

            txbuf[0] = 0x32; //0xD0;
            txbuf[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
            //txbuf[1] |= I2C_IC_DATA_CMD_RESTART_BITS;
            

            for (size_t i = 1; i < IMU_BUFFER_SIZE+1; ++i) {
                txbuf[i] = I2C_IC_DATA_CMD_CMD_BITS;
            }
            txbuf[IMU_BUFFER_SIZE] |= I2C_IC_DATA_CMD_STOP_BITS;

            i2c_get_hw(i2c0)->enable = 0;
            i2c_get_hw(i2c0)->tar = 0x53;
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
                                    IMU_BUFFER_SIZE+2,								// Element Count (Each element is of size transfer_data_size)
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
                                    IMU_BUFFER_SIZE,
                                    true); // START
            #endif
            
        } else if (state == 1 && !dma_channel_is_busy(dma_rx)) {
            
            #ifdef SEDNA_ACCELEROMETER
            int16_t accelX = uint16_t(rxbuf[1]) << 8 | uint16_t(rxbuf[0]);
            int16_t accelY = uint16_t(rxbuf[3]) << 8 | uint16_t(rxbuf[2]);
            int16_t accelZ = uint16_t(rxbuf[5]) << 8 | uint16_t(rxbuf[4]);
            #else
            int16_t accelX = 0;
            int16_t accelY = 0;
            int16_t accelZ = 0;
            #endif

            dma_channel_unclaim(dma_tx);
            dma_channel_unclaim(dma_rx);

            // Open file for writing ()
            fr = f_open(&fil, "sensordata.csv", FA_WRITE | FA_OPEN_APPEND);
            if (fr != FR_OK) {
                printf("ERROR: Could not open file (%d)\r\n", fr);

                filesystemerrcnt ++;
            }

            mutex_enter_blocking(&predictionTfMtx);
            int32_t usedpressure = pressure_tfpredicted;
            float predicted_altitude = altitude_tfpredicted;
            mutex_exit(&predictionTfMtx);

            // Write something to file
            #ifdef VERBOSE_SENSOR_LOG
            printf("%.3f, %.2f, %d, %d, %.2f\n", pressure / 1000.f, temperature / 100.f, raw_uv_adc, usedpressure, predicted_altitude);
            #endif
            ret = f_printf(&fil, "%.3f, %.2f, %.2f, %d, %d, %d, %d, %d, %.2f\n", pressure / 1000.f, temperature / 100.f, humidity/1024.f, raw_uv_adc, accelX, accelY, accelZ, predicted_altitude);
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);

                filesystemerrcnt ++;
                f_close(&fil);
            }

            // Close file
            fr = f_close(&fil);
            if (fr != FR_OK) {
                printf("ERROR: Could not close file (%d)\r\n", fr);

                filesystemerrcnt++;
            }

            /*
            // set time
            fno.fdate = (WORD)(((2007 - 1980) * 512U) | 1 * 32U | 9);
            fno.ftime = (WORD)(9 * 2048U | 41 * 32U | 00 / 2U);
            fr = f_utime("sensordata.csv", &fno);
            if (fr != FR_OK) {
                printf("ERROR: Could not change modification time (%d)\r\n", fr);
                filesystemerrcnt++;
            }*/

            state = -1;
            dma_tx 	= dma_claim_unused_channel(true);
            dma_rx 	= dma_claim_unused_channel(true);   
            #ifdef HZ_OUTPUT
            printf("%.2f Hz\n", 1000.0/(to_ms_since_boot((get_absolute_time())) - statemillis));//, filesystemerrcnt);
            #endif
            statemillis = to_ms_since_boot((get_absolute_time()));    
        }
    }

    return 0;
}



