#ifndef ROCKET_DATA_H
#define ROCKET_DATA_H

#include <Arduino.h>

struct __attribute__((packed)) SensorData {
    uint32_t timestamp = 0;   // 4 bytes
    uint8_t sensor_update = 0; // 1 byte
    
    float ax = 0.0f, ay = 0.0f, az = 0.0f;// 4 * 3 = 12 bytes
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;// 4 * 3 = 12 bytes
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;// 4 * 4 = 16 bytes
    
    float raw_p = 0.0f;       // 4 bytes
    float filt_p = 0.0f;   // 4 bytes
    
    float filt_alt = 0.0f;  // 4 bytes
    float alt_baro = 0.0f;  // 4 bytes
    float alt_imu = 0.0f;  // 4 bytes
    
    float filt_velocity = 0.0f;       // 4 bytes
    float vel_z_imu = 0.0f;
    float vel_z_baro = 0.0f;

    uint8_t flight_state = 0; // 1 byte
    uint8_t ej1_state = 0; // 1 byte
    uint8_t ej2_state = 0; // 1 byte
    uint8_t sep_state = 0; // 1 byte
}; 
// 총 크기: 81 bytes

#endif