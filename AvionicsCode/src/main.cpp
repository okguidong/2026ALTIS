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
        data.sensor_update = 0;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        data.timestamp = micros();

        // 센서 데이터 수집
        if (sensor.isAccelReady())
        {
            sensor_update |= UPDATE_ACCEL;
            sensor.readAccel(t_ax, t_ay, t_az);//m/s/s
            data.ax = t_ax;
            data.ay = t_ay;
            data.az = t_az;
        }

        if (sensor.isGyroReady())
        {
            sensor_update |= UPDATE_GYRO;
            sensor.readGyro(t_gx, t_gy, t_gz);//rad/s
            data.gx = t_gx;
            data.gy = t_gy;
            data.gz = t_gz;
        }

        if (sensor.isBaroReady())
        {
            sensor_update |= UPDATE_BARO;
            data.raw_p = sensor.getPressure();
        }
        // 비행 로직 업데이트
        navigation.update(data, sensor_update);
        recovery.update();
        logic.update(data, sensor_update);

        if(logic.isSeparation()){
            recovery.Separate();
            Serial.println("sep");
        }  
        if(logic.isEJect1()){
            recovery.EJ1();
        }
        if(logic.isEJect2()){
            recovery.EJ2();
        }
        // 데이터 저장
        if (sensor_update != 0)
        {
            data.sensor_update = sensor_update;
            logger.push(data);
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
        // [상태 2] 단분리 이후
        case 2:
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
    SerialBT.begin("ALTIS2026_1");
    SPI.begin();
    pinMode(VAT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    uint8_t sysOK = 0;
    if (!sensor.begin())
    {
        sysOK |= 0x01;
        Serial.println("Sensor Fail");
        SerialBT.println("ERR: Sensor Init Failed!");
    }
    if (!logger.begin())
    {
        Serial.println("SD Fail");
        sysOK |= 0x02;
        SerialBT.println("ERR: SD Card Init Failed!");
    }
    if (!recovery.begin())
    {
        Serial.println("Recovery Fail");
        sysOK |= 0x04;
        SerialBT.println("ERR: Recovery System Init Failed!");
    }
    if (sysOK != 0x00)
    {
        while (1)
        {
            SerialBT.printf("ERR: Check Failed! err: %b 0x01, 0x02, 0x04 == sensor, sdcard, recovery",sysOK);
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
        static unsigned long time = 0;
        if (millis() - time > 1000)
        {
            time = millis();
            long sum = 0;
            for (int i = 0; i < 10; i++)
            {
                sum += analogRead(VAT_PIN);
            }
            SerialBT.printf("Vattery Voltage: %.2f V , SERVO1 = %d , SERVO2 = %d ,SERVO3 = %d ,\n", sum * 0.0006667,Servo1_start,Servo2_start,Servo3_start);
            SerialBT.println("Enter \n- Sea Level Pressure (hPa)\n- READY: Arm System\n- SERVO1~3: Test Servo\n- PYRO1~3: Test PYRO\n -REBOOT: System Rebooting");
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
                SerialBT.println(Servo1_end);
                recovery.trigger(1);
                delay(servo_time);
                recovery.begin();
            }
            else if (s.equalsIgnoreCase("SERVO2"))
            {
                SerialBT.println("test SERVO2");
                recovery.trigger(2);
                SerialBT.println(Servo2_end);
                delay(servo_time);
                recovery.begin();
            }
            else if (s.equalsIgnoreCase("SERVO3"))
            {
                SerialBT.println("test SERVO3");
                recovery.trigger(3);
                SerialBT.println(Servo3_end);
                delay(servo_time);
                recovery.begin();
            }
            else if (s.equalsIgnoreCase("PYRO1"))
            {
                SerialBT.println("test PYRO1");
                recovery.trigger(4);
                delay(pyro_time);
                recovery.begin();
            }
            else if (s.equalsIgnoreCase("PYRO2"))
            {
                SerialBT.println("test PYRO2");
                recovery.trigger(5);
                delay(pyro_time);
                recovery.begin();
            }
            else if (s.equalsIgnoreCase("PYRO3"))
            {
                SerialBT.println("test PYRO3");
                recovery.trigger(6);
                delay(pyro_time);
                recovery.begin();
            }
            else if(s.equalsIgnoreCase("REBOOT"))
            {
                SerialBT.println("SYSTEM REBOOTING");
                ESP.restart();
            }
        }
    }

    xTaskCreatePinnedToCore(flightTask, "Flight", 10000, NULL, 10, NULL, 0);
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
            SerialBT.println("SYSTEM REBOOTING");
            delay(100);
            ESP.restart();
        }
        else if(cmd.equalsIgnoreCase("END")){
            delay(1000);
            while(1);
        }
    }
    delay(100);
}