#include "BluetoothSerial.h"
#include "Config.h"
#include "RocketData.h"
#include "Sensor.h"
#include "FlightLogic.h"
#include "DataLogger.h"
#include "Recovery.h"
#include "navigation.h"

// --- 객체 생성 ---
BluetoothSerial SerialBT;
Sensor sensor;
FlightLogic logic;
Navigation navigation;
DataLogger logger;
SensorData data;
Recovery recovery;

// 테스크 코어0
void flightTask(void *pvParam)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    bool parachute1Fired = false;
    bool parachute2Fired = false;
    bool separationFired = false;

    float t_ax, t_ay, t_az;
    float t_gx, t_gy, t_gz;

    while (true)
    {
        uint8_t sensor_update = 0;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        data.timestamp = micros();

        // 센서 데이터 수집
        if (sensor.isAccelReady())
        {
            sensor_update |= UPDATE_ACCEL;
            sensor.readAccel(t_ax, t_ay, t_az);
            data.ax = t_ax;
            data.ay = t_ay;
            data.az = t_az;
        }

        if (sensor.isGyroReady())
        {
            sensor_update |= UPDATE_GYRO;
            sensor.readGyro(t_gx, t_gy, t_gz);
            data.gx = t_gx;
            data.gy = t_gy;
            data.gz = t_gz;
        }

        if (sensor.isBaroReady())
        {
            sensor_update |= UPDATE_BARO;
            data.raw_pressure = sensor.getPressure();
        }
        // 비행 로직 업데이트
        navigation.update(data, sensor_update);
        logic.update(data, sensor_update);

        // 비행 상태에 따른 액션
        if (data.flight_state == 0)
        { // 대기 모드
        }
        else if (data.flight_state == 1)
        { // 상승 중
        }
        else if (data.flight_state == 2 && !separationFired)
        { // 분리
            recovery.trigger(SEPARATION);
            separationFired = true;
        }
        else if (data.flight_state == 3 && !parachute1Fired)
        { // EJ1
            recovery.trigger(EJECT_1);
            parachute1Fired = true;
        }
        else if (data.flight_state == 4 && !parachute2Fired)
        { // EJ2
            recovery.trigger(EJECT_2);
            parachute2Fired = true;
        }
        // 데이터 저장
        if (sensor_update != 0)
        {
            logger.push(data);
            recovery.update();
        }
    }
}

void buzzerTask(void *pvParam)
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    while (true)
    {
        switch (data.flight_state)
        {

        // [상태 0] 대기 모드 (Ready)
        //(느리게 깜빡임)
        case 0:
            digitalWrite(BUZZER_PIN, HIGH);
            vTaskDelay(100 / portTICK_PERIOD_MS); // 0.1초 켜짐
            digitalWrite(BUZZER_PIN, LOW);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 2초 꺼짐
            break;

        // [상태 1] 상승 중 (Boost/Coast)
        case 1:
            digitalWrite(BUZZER_PIN, LOW);
            break;

        // [상태 2] 분리/사출 대기 (Apogee Check)
        // 특징: "삐비빅, 삐비빅" (경고음)
        case 2:
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(BUZZER_PIN, HIGH);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                digitalWrite(BUZZER_PIN, LOW);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;
        case 3:
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(BUZZER_PIN, HIGH);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                digitalWrite(BUZZER_PIN, LOW);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;

        // [상태 4] 하강 및 착륙 (Descent / Landed)
        // 1초마다 울림
        case 4:
            digitalWrite(BUZZER_PIN, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            digitalWrite(BUZZER_PIN, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;

        default:
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    SerialBT.begin("ALTIS2026");
    SPI.begin();
    pinMode(VAT_PIN, INPUT);

    bool sysOK = true;
    if (!sensor.begin())
    {
        Serial.println("Sensor Fail");
        sysOK = false;
        SerialBT.println("ERR: Sensor Init Failed!");
    }
    if (!logger.begin())
    {
        Serial.println("SD Fail");
        sysOK = false;
        SerialBT.println("ERR: SD Card Init Failed!");
    }
    if (!recovery.begin())
    {
        Serial.println("Recovery Fail");
        sysOK = false;
        SerialBT.println("ERR: Recovery System Init Failed!");
    }
    if (!sysOK)
    {
        while (1)
        {
            SerialBT.println("ERR: Check Failed!");
            digitalWrite(BUZZER_PIN, HIGH);
            delay(1000);
            ESP.restart();
        }
    }
    SerialBT.println("System OK. Waiting for Config...");

    // 대기 모드
    bool isArmed = false;
    while (!isArmed)
    {
        if (millis() % 1000 == 0)
        {
            long sum = 0;
            for (int i = 0; i < 10; i++)
            {
                sum += analogRead(VAT_PIN);
            }
            SerialBT.printf("Vattery Voltage: %.2f V\n", sum * 0.0006667);
            SerialBT.println("Enter Sea Level Pressure (hPa) or Commands:\n- READY: Arm System\n- SERVO1~3: Test Servo\n- PYRO1~3: Test PYRO");
        }

        if (SerialBT.available())
        {
            String s = SerialBT.readStringUntil('\n');
            s.trim();

            if (isdigit(s.charAt(0)))
            {
                float p = s.toFloat();
                navigation.setSeaLevelPressure(p);
                SerialBT.printf("Pressure Set: %.2f hpa\n", p);
            }
            else if (s.equalsIgnoreCase("READY"))
            {
                SerialBT.println("ARMED!");
                isArmed = true;
            }
            else if (s.equalsIgnoreCase("SERVO1"))
            {
                SerialBT.println("test SERVO1");
                recovery.trigger(1);
            }
            else if (s.equalsIgnoreCase("SERVO2"))
            {
                SerialBT.println("test SERVO2");
                recovery.trigger(2);
            }
            else if (s.equalsIgnoreCase("SERVO3"))
            {
                SerialBT.println("test SERVO3");
                recovery.trigger(3);
            }
            else if (s.equalsIgnoreCase("PYRO1"))
            {
                SerialBT.println("test PYRO1");
                recovery.trigger(4);
            }
            else if (s.equalsIgnoreCase("PYRO2"))
            {
                SerialBT.println("test PYRO2");
                recovery.trigger(5);
            }
            else if (s.equalsIgnoreCase("PYRO3"))
            {
                SerialBT.println("test PYRO3");
                recovery.trigger(6);
            }
        }
    }

    // 비행 태스크 시작 (Core 0, High Priority)
    xTaskCreatePinnedToCore(flightTask, "Flight", 10000, NULL, 10, NULL, 0);
    // 버저 태스크 (Core 1, 우선순위 낮음 1)
    xTaskCreatePinnedToCore(buzzerTask, "Buzzer", 2048, NULL, 1, NULL, 1);
}

void loop()
{
    if (SerialBT.available())
    {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();

        if (cmd.equalsIgnoreCase("REBOOT"))
        {
            SerialBT.println("SYSTEM REBOOTING...");
            delay(100);
            ESP.restart();
        }
    }
    delay(100);
}