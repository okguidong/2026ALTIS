#ifndef ROCKET_DATA_H
#define ROCKET_DATA_H

#include <Arduino.h>

struct __attribute__((packed)) SensorData {
    uint32_t timestamp;   // 4 bytes
    float ax, ay, az;     // 4 * 3 = 12 bytes
    float gx, gy, gz;     // 4 * 3 = 12 bytes
    float pressure;       // 4 bytes
    float raw_altitude;   // 4 bytes
    float filt_altitude;  // 4 bytes
    float velocity;       // 4 bytes
    uint8_t flight_state; // 1 byte
    uint8_t ej1_state; // 1 byte
    uint8_t ej2_state; // 1 byte
    uint8_t sep_state; // 1 byte
}; 
// 총 크기: 48 bytes

#endif