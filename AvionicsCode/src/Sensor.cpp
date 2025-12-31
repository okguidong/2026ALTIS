#include "Sensor.h"

// Static 멤버 변수 정의 (메모리 할당)
volatile bool Sensor::_accelReady = false;
volatile bool Sensor::_gyroReady = false;
volatile bool Sensor::_baroReady = false;

// 생성자: BMI088 객체는 생성 시 핀 설정이 필요함
Sensor::Sensor() 
    : devAccel(SPI, ACCEL_CS_PIN), devGyro(SPI, GYRO_CS_PIN) {
}

// 인터럽트 핸들러 구현
void IRAM_ATTR Sensor::isrAccel() { _accelReady = true; }
void IRAM_ATTR Sensor::isrGyro()  { _gyroReady = true; }
void IRAM_ATTR Sensor::isrBaro()  { _baroReady = true; }

bool Sensor::begin() {
    bool status = true;

    // 1. Barometer 초기화
    if (devBaro.beginSPI(BARO_CS_PIN, 4000000) != BMP3_OK) status = false;
    else {
        devBaro.setODRFrequency(BMP3_ODR_200_HZ);
        devBaro.setFilterCoefficient(BMP3_IIR_FILTER_COEFF_3);
        bmp3_int_ctrl_settings s = {BMP3_INT_PIN_PUSH_PULL, BMP3_INT_PIN_ACTIVE_HIGH, BMP3_INT_PIN_NON_LATCH, BMP3_ENABLE};
        devBaro.setInterruptSettings(s);
        
        pinMode(BARO_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(BARO_INT_PIN), isrBaro, RISING);
    }

    // 2. Accel 초기화
    if (devAccel.begin() < 0) status = false;
    else {
        devAccel.setRange(Bmi088Accel::RANGE_24G);
        devAccel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
        devAccel.pinModeInt1(Bmi088Accel::PUSH_PULL, Bmi088Accel::ACTIVE_HIGH);
        devAccel.mapDrdyInt1(true);

        pinMode(ACCEL_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(ACCEL_INT_PIN), isrAccel, RISING);
    }

    // 3. Gyro 초기화
    if (devGyro.begin() < 0) status = false;
    else {
        devGyro.setRange(Bmi088Gyro::RANGE_2000DPS);
        devGyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
        devGyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
        devGyro.mapDrdyInt3(true);

        pinMode(GYRO_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), isrGyro, RISING);
    }

    return status;
}

// --- IMU Methods ---
bool Sensor::isAccelReady() {
    if (_accelReady) { _accelReady = false; return true; }
    return false;
}

bool Sensor::isGyroReady() {
    if (_gyroReady) { _gyroReady = false; return true; }
    return false;
}

void Sensor::readAccel(float &ax, float &ay, float &az) {
    devAccel.readSensor();
    ax = devAccel.getAccelX_mss();
    ay = devAccel.getAccelY_mss();
    az = devAccel.getAccelZ_mss();
}

void Sensor::readGyro(float &gx, float &gy, float &gz) {
    devGyro.readSensor();
    gx = devGyro.getGyroX_rads();
    gy = devGyro.getGyroY_rads();
    gz = devGyro.getGyroZ_rads();
}

// --- Baro Methods ---
bool Sensor::isBaroReady() {
    if (_baroReady) { _baroReady = false; return true; }
    return false;
}

float Sensor::getPressure() {
    bmp3_data d;
    devBaro.getSensorData(&d);
    return d.pressure;
}