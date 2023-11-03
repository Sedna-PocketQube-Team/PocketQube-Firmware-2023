#ifndef SATTELITE_DATA_H
#define SATTELITE_DATA_H

#include <stdio.h>

struct sattelite_sensor_data {
    uint32_t ms_since_boot;
    uint16_t raw_uv_adc;
    int32_t temperature;
    int32_t pressure;
    uint32_t humidity;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int32_t usedpressure;
    float predicted_altitude;
} ;

struct gps_data {
    //frame.time.hours, frame.time.minutes, frame.time.seconds, frame.fix_quality, frame.altitude, frame.altitude_units, minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));
    int hours;
    int minutes;
    int seconds;
    int fix_quality;
    float altitude;
    char altitude_units;
    float latitude, longitude;
} ;

#endif