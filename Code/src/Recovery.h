#ifndef RECOVERY_H
#define RECOVERY_H

#include <Arduino.h>
#include <ESP32_Servo.h>
#include "Config.h" // 핀 번호 및 각도 설정 포함

class Recovery {
public:
    Recovery();
    bool begin();

    void trigger(int id);
    void update();

private:
    Servo _servo1, _servo2, _servo3;

    unsigned long _triggerTime[6]; 
};

#endif