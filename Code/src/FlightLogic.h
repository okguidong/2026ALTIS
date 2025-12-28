#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include "Config.h"
#include "RocketData.h"
#include <math.h>

class FlightLogic {
public:
    FlightLogic();
    void reset();
    
    void update(SensorData &data, uint8_t sensor_update);

    // Main에서 상태를 확인하기 위한 함수들
    bool isLaunchDetected();
    bool isEJect1();
    bool isEJect2();
    bool isSeparation();

private:
    uint32_t _launchTime;
    float _prevAlt;
    bool _launched;
    bool _EJ1;
    bool _EJ2;
    bool _separated;
    unsigned long _velocity_prevtime;

    // 내부 로직 함수
    void applyFilter(SensorData &data, uint8_t sensor_update);
    void applyFilter_baro(SensorData &data);
    void applyFilter_gyro(SensorData &data);
    void applyFilter_accel(SensorData &data);
    void checkLaunch(SensorData &data);
    void checkEJ1(SensorData &data);
    void checkEJ2(SensorData &data);
    void checkSeparation(SensorData &data);
};
#endif