#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "SparkFunBMP384.h"
#include "BMI088.h"
#include "Config.h"

class Sensor {
public:
    Sensor();

    bool begin();
    
    // --- IMU (Accel & Gyro) ---
    bool isAccelReady();
    bool isGyroReady();
    void readAccel(float &ax, float &ay, float &az);
    void readGyro(float &gx, float &gy, float &gz);

    // --- Barometer ---
    bool isBaroReady();
    float getPressure();

private:
    // 센서 객체
    Bmi088Accel devAccel;
    Bmi088Gyro  devGyro;
    BMP384      devBaro;
    
    // 데이터 준비 플래그
    static volatile bool _accelReady;
    static volatile bool _gyroReady;
    static volatile bool _baroReady;

    // 인터럽트 핸들러 (ISR)
    static void IRAM_ATTR isrAccel();
    static void IRAM_ATTR isrGyro();
    static void IRAM_ATTR isrBaro();
};

#endif // SENSOR_H