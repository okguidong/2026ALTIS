#include "SensorBaro.h"

BMP384 devBaro;
volatile bool SensorBaro::_baroReady = false;

void IRAM_ATTR SensorBaro::isrBaro() { _baroReady = true; }

bool SensorBaro::begin() {
    if (devBaro.beginSPI(BARO_CS_PIN, 4000000) != BMP3_OK) return false;
    devBaro.setODRFrequency(BMP3_ODR_200_HZ);
    devBaro.setFilterCoefficient(BMP3_IIR_FILTER_COEFF_3);
    
    bmp3_int_ctrl_settings s = {BMP3_INT_PIN_PUSH_PULL, BMP3_INT_PIN_ACTIVE_HIGH, BMP3_INT_PIN_NON_LATCH, BMP3_ENABLE};
    devBaro.setInterruptSettings(s);

    pinMode(BARO_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BARO_INT_PIN), isrBaro, RISING);
    return true;
}

bool SensorBaro::isReady() {
    if (_baroReady) { _baroReady = false; return true; }
    return false;
}
float SensorBaro::getPressure() {
    bmp3_data d;
    devBaro.getSensorData(&d);
    return d.pressure;
}
float SensorBaro::getAltitude() {
    float p = getPressure();
    if (p > 0) return 44330.0 * (1.0 - pow(p / seaLevelPa, 0.1903));
    return 0.0f;
}
void SensorBaro::setSeaLevelPressure(float hpa) {
    seaLevelPa = hpa * 100.0f;
}