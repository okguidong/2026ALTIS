#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include "RocketData.h"
#include <math.h>

class Navigation {
public:
    Navigation();
    void reset();
    void update(SensorData &data, uint8_t sensor_update);
    void setSeaLevelPressure(float hpa);

private:
    uint32_t _launchTime;
    float _prevPressure;
    float _prevAltitude;
    bool _launched;
    bool _EJ1;
    bool _EJ2;
    bool _separated;
    unsigned long _velocity_prevtime;
    float _seaLevelPa = 101325.0f;

    void applyFilter(SensorData &data, uint8_t sensor_update);
    void applyFilter_baro(SensorData &data);
    void applyFilter_gyro(SensorData &data);
    void applyFilter_accel(SensorData &data);
    float calculateAltitude(float pa);

};
#endif