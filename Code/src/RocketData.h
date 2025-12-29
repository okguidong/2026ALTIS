#ifndef ROCKET_DATA_H
#define ROCKET_DATA_H

#include <Arduino.h>

struct __attribute__((packed)) SensorData {
    uint32_t timestamp = 0;   // 4 bytes
    float ax = 0.0f, ay = 0.0f, az = 0.0f;// 4 * 3 = 12 bytes
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;// 4 * 3 = 12 bytes
    float pressure = 0.0f;       // 4 bytes
    float raw_altitude = 0.0f;   // 4 bytes
    float filt_altitude = 0.0f;  // 4 bytes
    float velocity = 0.0f;       // 4 bytes
    uint8_t flight_state = 0; // 1 byte
    uint8_t ej1_state = 0; // 1 byte
    uint8_t ej2_state = 0; // 1 byte
    uint8_t sep_state = 0; // 1 byte
}; 
// 총 크기: 48 bytes

#endif