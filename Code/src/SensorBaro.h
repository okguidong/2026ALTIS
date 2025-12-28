#ifndef SENSOR_BARO_H
#define SENSOR_BARO_H

#include "SparkFunBMP384.h"
#include "Config.h"

class SensorBaro {
public:
    bool begin();
    bool isReady();
    float getPressure();
    float getAltitude(); // 단순 변환만 수행
    void setSeaLevelPressure(float hpa);

private:
    static void IRAM_ATTR isrBaro();
    static volatile bool _baroReady;
    float seaLevelPa = 101325.0f;
};
#endif // SENSOR_BARO_H