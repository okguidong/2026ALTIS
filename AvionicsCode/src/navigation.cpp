#include "navigation.h"

Navigation::Navigation()
{
    reset();
}

void Navigation::reset()
{
    _launchTime = 0;
    _prevAlt = 0.0f;
}

void Navigation::update(SensorData &data, uint8_t sensor_update)
{
    applyFilter(data, sensor_update);

    // 1단계: 아직 발사 안 됨 (대기 상태)
    if (data.flight_state == 0)
    {

    }
    // 2단계: 발사는 됐는데, 아직 분리는 안 됨 (상승 1단계)
    else if (data.flight_state == 1)
    {
        data.flight_state = 1; // Boost / Coast
        if (sensor_update & UPDATE_BARO)
        {

        }
    }
    // 3단계: 분리는 됐는데, 아직 사출은 안 됨 (상승 2단계 또는 활공)
    else if (data.flight_state == 2)
    {

    }
    // 4단계: 1단부 사출까지 됨 (하강 상태)
    else if (data.flight_state == 3)
    {
        data.flight_state = 3; // Descent
        if (sensor_update & UPDATE_BARO)
        {

        }
    }
    // 4단계: 사출까지 됨 (하강 상태)
    else
    {
        data.flight_state = 4; // Descent
    }
}

void Navigation::applyFilter(SensorData &data, uint8_t sensor_update)
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
void Navigation::applyFilter_gyro(SensorData &data)
{
}

void Navigation::applyFilter_accel(SensorData &data)
{
}

void Navigation::applyFilter_baro(SensorData &data)
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