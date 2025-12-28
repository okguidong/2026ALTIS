#ifndef CONFIG_H
#define CONFIG_H

// Pin Map
//  IMU
#define ACCEL_CS_PIN 25
#define GYRO_CS_PIN 33
#define ACCEL_INT_PIN 34
#define GYRO_INT_PIN 35
// Baro
#define BARO_CS_PIN 32
#define BARO_INT_PIN 39
// interface
#define VAT_PIN 36
#define BUZZER_PIN 16
// Pyro
#define Pyro1 21
#define Pyro2 5
#define Pyro3 17
#define Pyro1_available true
#define Pyro2_available true
#define Pyro3_available true
// servo
#define Servo1 26
#define Servo2 27
#define Servo3 22
#define SERVO1_available true
#define SERVO2_available true
#define SERVO3_available true
#define Servo1_start 90
#define Servo2_start 90
#define Servo3_start 90
#define Servo1_end 0
#define Servo2_end 0
#define Servo3_end 0
#define servo_time 1000 // 서보 작동 시간 (ms)
#define pyro_time 500   // 파이로 작동 시간 (ms)
//1~3 서보 4~6 파이로
#define EJECT_1 1 //1단부 사출 사용할 채널 
#define EJECT_2 2 //2단부 사출 사용할 채널
#define SEPARATION 3 //단분리 사출 사용할 채널


// sdcard
#define SD_DATA0_PIN 2
#define SD_DATA1_PIN 4
#define SD_DATA2_PIN 12
#define SD_DATA3_PIN 13
#define SD_CLK_PIN 14
#define SD_CMD_PIN 15
// bit flag
#define UPDATE_ACCEL 0x01
#define UPDATE_GYRO 0x02
#define UPDATE_BARO 0x04

// --- Parameters ---
#define ALT_LPF_ALPHA 0.1f         // 저주파 통과 필터 계수
#define LAUNCH_THRESHOLD_G 3.0f    // 발사 감지 임계값 (3G)
#define EJECT1_TIMEOUT_MS 6000      // 발사 후 1단부 강제 사출 시간 (ms)
#define EJECT2_TIMEOUT_MS 10000      // 발사 후 2단부 강제 사출 시간 (ms)
#define EJECT_ALTITUDE 25.0f      // 사출 최소 고도 (m)
#define SEPARATION_TIMEOUT_MS 3000 // 발사 후 강제 분리 시간 (ms)
#define SEPARATION_ALTITUDE 50.0f // 단분리 최소 고도 (m)

#endif // CONFIG_H