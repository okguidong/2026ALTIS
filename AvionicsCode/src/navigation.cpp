#include "navigation.h"

Navigation::Navigation()
{
    reset();
}

void Navigation::reset()
{
    _launchTime = 0;
    _prevAltitude = 0.0f;
    _prevPressure = 0.0f;
    _launched = false;
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
void Navigation::accel_imu_to_body(SensorData &data)
{
    float imu_ax = data.ax;
    float imu_ay = data.ay;
    float imu_az = data.az;

    data.ax = -imu_ay;
    data.ay = imu_ax;
    data.az = -imu_az;
}

void Navigation::gyro_imu_to_body(SensorData &data)
{
    float imu_gx = data.gx;
    float imu_gy = data.gy;
    float imu_gz = data.gz;

    data.gx = -imu_gy;
    data.gy = imu_gx;
    data.gz = -imu_gz;
}
void Navigation::gyro_to_quaternion(SensorData &data)
{
    static unsigned long prev_time = 0;
    unsigned long now = micros();

    if (prev_time == 0)
    {
        prev_time = now;
        return;
    }

    double dt = (now - prev_time) / 1000000.0;
    if (dt <= 0.0 || dt > 0.01)
    { // 0 < dt < 10ms 만 사용 센서 데이터 200Hz
        prev_time = now;
        return;
    }
    prev_time = now;

    float gx = data.gx; // rad/s, body frame
    float gy = data.gy;
    float gz = data.gz;
    // 미소 각
    float dth_x = gx * dt;
    float dth_y = gy * dt;
    float dth_z = gz * dt;
    // 미소 각 벡터 크기
    float total_angle = sqrtf(dth_x * dth_x + dth_y * dth_y + dth_z * dth_z);

    // 미소 쿼터니언각
    Quat dq;
    if (total_angle < 1e-6f)
    {
        // 작은 각도 근사: dq ≈ [1, 0.5*dθ]
        dq.w = 1.0f;
        dq.x = 0.5f * dth_x;
        dq.y = 0.5f * dth_y;
        dq.z = 0.5f * dth_z;
    }
    else
    {
        // dq = cos(th/2), unit_x*sin(th/2), unit_y*sin(th/2), unit_z*sin(th/2)
        // unit axis = [dth_x,dth_y,dth_z]/total_angle
        float half = 0.5f * total_angle;
        float s = sinf(half) / total_angle; // unit axis * sin(θ/2)
        dq.w = cosf(half);
        dq.x = dth_x * s;
        dq.y = dth_y * s;
        dq.z = dth_z * s;
    }

    // 여기서부터는 기존 쿼터니언(q)과 곱해주는 부분이 필요
    // q_new = q_old ⊗ dq
    Quat old;
    old.w = data.w;
    old.x = data.x;
    old.y = data.y;
    old.z = data.z;

    Quat r = quat_mul(old, dq);

    float n = sqrtf(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z); // 수치 오차 누적을 줄이고자 정규화를 한번 더
    if (n > 0.0f)
    {
        float inv = 1.0f / n;
        r.w *= inv;
        r.x *= inv;
        r.y *= inv;
        r.z *= inv;
    }
    data.w = r.w;
    data.x = r.x;
    data.y = r.y;
    data.z = r.z;
}

Quat Navigation::quat_mul(const Quat &a, const Quat &b)
{ // 쿼터니언 곱 Q x dq a= old , b= dq
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

EulerRPY Navigation::quat_to_euler_zyx_deg(const Quat &q)
{
    const float rad2deg = 180.0f / M_PI;
    EulerRPY e;
    float ww = q.w, xx = q.x, yy = q.y, zz = q.z;

    // roll (X axis)
    float sinr_cosp = 2.0f * (ww * xx + yy * zz);
    float cosr_cosp = 1.0f - 2.0f * (xx * xx + yy * yy);
    e.roll = atan2f(sinr_cosp, cosr_cosp)* rad2deg;

    // pitch (Y axis)
    float sinp = 2.0f * (ww * yy - zz * xx);
    if (fabsf(sinp) >= 1.0f)
        e.pitch = copysignf(M_PI / 2.0f, sinp)* rad2deg; // clamp
    else
        e.pitch = asinf(sinp) * rad2deg;

    // yaw (Z axis)
    float siny_cosp = 2.0f * (ww * zz + xx * yy);
    float cosy_cosp = 1.0f - 2.0f * (yy * yy + zz * zz);
    e.yaw = atan2f(siny_cosp, cosy_cosp) * rad2deg;

    return e; // deg
}

void Navigation::printEulerDebug(const SensorData &data) {
#ifdef NAV_DEBUG
    Quat q { data.w, data.x, data.y, data.z };
    EulerRPY e = quat_to_euler_zyx_deg(q);

    Serial.printf("RPY[deg] = (%.2f, %.2f, %.2f)\n",
                  e.roll, e.pitch, e.yaw);
#endif
}

void Navigation::applyFilter_gyro(SensorData &data)
{
    gyro_imu_to_body(data);
    gyro_to_quaternion(data);
    printEulerDebug(data);
}

void Navigation::applyFilter_accel(SensorData &data)
{
    accel_imu_to_body(data);
}

// baro
void Navigation::applyFilter_baro(SensorData &data)
{
    if (data.filt_p == 0.0f)
    {
        if (data.raw_p > 0.0f)
        {
            data.filt_p = data.raw_p;
            data.alt_baro = calculateAltitude(data.filt_p);
            _velocity_prevtime = micros();
        }
        return;
    }
    _prevAltitude = data.alt_baro;
    data.filt_p = (PRESSURE_LPF_ALPHA * data.raw_p) + ((1.0f - PRESSURE_LPF_ALPHA) * data.filt_p);
    data.alt_baro = calculateAltitude(data.filt_p);

    unsigned long now = micros();
    float dt = (now - _velocity_prevtime) / 1000000.0f;

    if (dt > 0.0f && dt < 1.0f)
    {
        _velocity_prevtime = now;
        data.vel_z_baro = (data.alt_baro - _prevAltitude) / dt;
    }
}

float Navigation::calculateAltitude(float pa)
{
    float Altitude = 44330.0 * (1.0 - pow(pa / _seaLevelPa, 0.1903));
    return Altitude;
}

void Navigation::setSeaLevelPressure(float hpa)
{
    _seaLevelPa = hpa * 100.0f;
}
// interface
