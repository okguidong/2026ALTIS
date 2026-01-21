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
    // 발사전
    if (!_launched)
    {
        //비행중 (아직 분리전)
        data.flight_state = 0; // Ready
        if (sensor_update & UPDATE_ACCEL)
        {
            checkLaunch(data);
        }
    }
    //분리전
    else if (!_separated)
    {//분리중
        data.flight_state = 1; // Boost / Coast
        if (sensor_update & UPDATE_BARO)
        {
            checkSeparation(data);
        }
    }

    // 분리 이후
    else
    {//사출중
        data.flight_state = 2; // Apogee Check
        if (sensor_update & UPDATE_BARO)
        {
            checkEJ1(data);
            checkEJ2(data);
        }
    }
    //하강
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