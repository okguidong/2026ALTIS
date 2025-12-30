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
    applyFilter(data, sensor_update);

    // 1단계: 아직 발사 안 됨 (대기 상태)
    if (!_launched)
    {
        data.flight_state = 0; // Ready
        if (sensor_update & UPDATE_ACCEL)
        {
            checkLaunch(data);
        }
    }
    // 2단계: 발사는 됐는데, 아직 분리는 안 됨 (상승 1단계)
    else if (!_separated)
    {
        data.flight_state = 1; // Boost / Coast
        if (sensor_update & UPDATE_BARO)
        {
            checkSeparation(data);
        }
    }
    // 3단계: 분리는 됐는데, 아직 사출은 안 됨 (상승 2단계 또는 활공)
    else if (!_EJ1)
    {
        data.flight_state = 2; // Apogee Check
        if (sensor_update & UPDATE_BARO)
        {
            checkEJ1(data);
        }
    }
    // 4단계: 1단부 사출까지 됨 (하강 상태)
    else if (!_EJ2)
    {
        data.flight_state = 3; // Descent
        if (sensor_update & UPDATE_BARO)
        {
            checkEJ2(data);
        }
    }
    // 4단계: 사출까지 됨 (하강 상태)
    else
    {
        data.flight_state = 4; // Descent
    }
}

void FlightLogic::applyFilter(SensorData &data, uint8_t sensor_update)
{
    if (sensor_update & UPDATE_BARO)
    {
        applyFilter_baro(data);
    }
    if (sensor_update & UPDATE_GYRO)
    {
        applyFilter_gyro(data);
    }
    if (sensor_update & UPDATE_ACCEL)
    {
        applyFilter_accel(data);
    }
}
void FlightLogic::applyFilter_gyro(SensorData &data)
{
}

void FlightLogic::applyFilter_accel(SensorData &data)
{
}

void FlightLogic::applyFilter_baro(SensorData &data)
{
    if (_prevAlt == 0.0f && data.raw_altitude != 0.0f)
    {
        _prevAlt = data.raw_altitude;
    }

    data.filt_altitude = (ALT_LPF_ALPHA * data.raw_altitude) + ((1.0f - ALT_LPF_ALPHA) * _prevAlt);

    float dt = (micros() - _velocity_prevtime) / 1000000.0f;
    if (_velocity_prevtime == 0.0f || dt <= 0.0f)
    {
        _velocity_prevtime = micros();
    }

    else
    {
        _velocity_prevtime = micros();
        data.velocity = (data.filt_altitude - _prevAlt) / dt;
    }
    _prevAlt = data.filt_altitude;
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
    static bool timeOut = false;
    static bool altReached = false;

    if (millis() - _launchTime > EJECT1_TIMEOUT_MS)
    {
        timeOut = 1;
        data.ej1_state |= 0x02;
    }

    if (data.filt_altitude >= EJECT_ALTITUDE)
    {
        altReached = 1;
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

    if (data.filt_altitude >= EJECT_ALTITUDE)
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

    if (data.filt_altitude >= SEPARATION_ALTITUDE)
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