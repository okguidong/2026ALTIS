#include "SensorIMU.h"

Bmi088Accel devAccel(SPI, ACCEL_CS_PIN);
Bmi088Gyro  devGyro(SPI, GYRO_CS_PIN);

volatile bool SensorIMU::_accelReady = false;
volatile bool SensorIMU::_gyroReady = false;

void IRAM_ATTR SensorIMU::isrAccel() { _accelReady = true; }
void IRAM_ATTR SensorIMU::isrGyro()  { _gyroReady = true; }

bool SensorIMU::begin() {
    if (devAccel.begin() < 0) return false;
    devAccel.setRange(Bmi088Accel::RANGE_24G);
    devAccel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
    devAccel.pinModeInt1(Bmi088Accel::PUSH_PULL, Bmi088Accel::ACTIVE_HIGH);
    devAccel.mapDrdyInt1(true);

    if (devGyro.begin() < 0) return false;
    devGyro.setRange(Bmi088Gyro::RANGE_2000DPS);
    devGyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    devGyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    devGyro.mapDrdyInt3(true);

    pinMode(ACCEL_INT_PIN, INPUT);
    attachInterrupt(ACCEL_INT_PIN, isrAccel, RISING);
    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, isrGyro, RISING);
    return true;
}

bool SensorIMU::isAccelReady() {
    if (_accelReady) { _accelReady = false; return true; }
    return false;
}
bool SensorIMU::isGyroReady() {
    if (_gyroReady) { _gyroReady = false; return true; }
    return false;
}
void SensorIMU::readAccel(float &ax, float &ay, float &az) {
    devAccel.readSensor();
    ax = devAccel.getAccelX_mss();
    ay = devAccel.getAccelY_mss();
    az = devAccel.getAccelZ_mss();
}
void SensorIMU::readGyro(float &gx, float &gy, float &gz) {
    devGyro.readSensor();
    gx = devGyro.getGyroX_rads();
    gy = devGyro.getGyroY_rads();
    gz = devGyro.getGyroZ_rads();
}