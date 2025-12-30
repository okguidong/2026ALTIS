#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "BMI088.h"
#include "Config.h"

class SensorIMU {
public:
    bool begin();
    bool isAccelReady();
    bool isGyroReady();
    void readAccel(float &ax, float &ay, float &az);
    void readGyro(float &gx, float &gy, float &gz);

private:
    static void IRAM_ATTR isrAccel();
    static void IRAM_ATTR isrGyro();
    static volatile bool _accelReady;
    static volatile bool _gyroReady;
};
#endif // SENSOR_IMU_H