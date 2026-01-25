#include "FlightLogic.h"

FlightLogic::FlightLogic()
{
    reset();
}

void FlightLogic::reset()
{
    _launchTime = 0;
    _prevAlt = 0.0f;
    _launched = false;
    _EJ1 = false;
    _EJ2 = false;
    _separated = false;
}

void FlightLogic::update(SensorData &data, uint8_t sensor_update)
{
/*
0: 발사전
1: 대기
2: 발사감지후 비행중 (단분리전)
3: 단분리 & 비행중
4: 사출 &  비행중
*/
    
    if (!_launched)// 발사전
    {
        data.flight_state = 1;
        if (sensor_update & UPDATE_ACCEL)
        {
            checkLaunch(data);// 발사 감지 flight_state->2
        }
    }
    
    else if (!_separated)//분리전
    {//분리중
        data.flight_state = 2; // Boost / Coast
        if (sensor_update & UPDATE_BARO)
        {
            checkSeparation(data); // 단분리 조건 확인
        }
    }

    // 분리 이후
    else
    {//사출중
        data.flight_state = 3; // Apogee Check
        if (sensor_update & UPDATE_BARO)
        {
            checkEJ1(data);// 1단부 사출 조건 확인
            checkEJ2(data);// 2단부 사출 조건 확인
        }
    }
    
}

void FlightLogic::checkLaunch(SensorData &data)
{
    // 가속도 벡터 크기
    float accNorm = sqrt(pow(data.ax, 2) + pow(data.ay, 2) + pow(data.az, 2));

    if (accNorm >= (LAUNCH_THRESHOLD_G * 9.81f))
    {
        _launched = true;
        _launchTime = millis();
    }
}

void FlightLogic::checkEJ1(SensorData &data)
{ // 사출 조건
    bool timeOut = (millis() - _launchTime > EJECT1_TIMEOUT_MS);
    bool altReached = (data.alt_baro >= EJECT_ALTITUDE);

    if (timeOut)
    {
        data.ej1_state |= 0x02;
    }

    if (altReached)
    {
        data.ej1_state |= 0x01;
    }

    if (timeOut && altReached)
    {
        _EJ1 = true;
        data.flight_state = 4;
    }
}

void FlightLogic::checkEJ2(SensorData &data)
{ // 사출 조건
    static bool timeOut = false;
    static bool altReached = false;

    if (millis() - _launchTime > EJECT2_TIMEOUT_MS)
    {
        timeOut = 1;
        data.ej2_state |= 0x02;
    }

    if (data.alt_baro >= EJECT_ALTITUDE)
    {
        altReached = 1;
        data.ej2_state |= 0x01;
    }

    if (timeOut && altReached)
    {
        _EJ2 = true;
        data.flight_state = 5;
    }
}

void FlightLogic::checkSeparation(SensorData &data)
{ // 단분리 조건
    static bool timeOut = false;
    static bool altReached = false;

    if (millis() - _launchTime > SEPARATION_TIMEOUT_MS)
    {
        timeOut = 1;
        data.sep_state |= 0x02;
    }

    if (data.alt_baro >= SEPARATION_ALTITUDE)
    {
        altReached = 1;
        data.sep_state |= 0x01;
    }

    if (timeOut && altReached)
    {
        _separated = true;
    }
}

bool FlightLogic::isLaunchDetected() { return _launched; }
bool FlightLogic::isEJect1() { return _EJ1; }
bool FlightLogic::isEJect2() { return _EJ2; }
bool FlightLogic::isSeparation() { return _separated; }