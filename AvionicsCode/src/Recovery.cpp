#include "Recovery.h"

Recovery::Recovery()
{
    // 시간 배열 0으로 초기화
    for (int i = 0; i < 7; i++)
        _triggerTime[i] = 0;
}

bool Recovery::begin()
{
    // [Config 설정에 따라 핀 초기화]
    if (Pyro1_available) { pinMode(Pyro1, OUTPUT); digitalWrite(Pyro1, LOW); }
    if (Pyro2_available) { pinMode(Pyro2, OUTPUT); digitalWrite(Pyro2, LOW); }
    if (Pyro3_available) { pinMode(Pyro3, OUTPUT); digitalWrite(Pyro3, LOW); }

    if (SERVO1_available) { _servo1.attach(Servo1); _servo1.write(Servo1_start); }
    if (SERVO2_available) { _servo2.attach(Servo2); _servo2.write(Servo2_start); }
    if (SERVO3_available) { _servo3.attach(Servo3); _servo3.write(Servo3_start); }

    // [연결 검사]
    if (SERVO1_available && !_servo1.attached()) return false;
    if (SERVO2_available && !_servo2.attached()) return false;
    if (SERVO3_available && !_servo3.attached()) return false;

    return true;
}

void Recovery::trigger(int id)
{
    if (id < 1 || id > 6) return;
    
    // Config에서 안 쓴다고 했으면 무시
    switch (id) {
        case 1: if (!SERVO1_available) return; break;
        case 2: if (!SERVO2_available) return; break;
        case 3: if (!SERVO3_available) return; break;
        case 4: if (!Pyro1_available) return; break;
        case 5: if (!Pyro2_available) return; break;
        case 6: if (!Pyro3_available) return; break;
    }

    if (_triggerTime[id] > 0) return; // 중복 방지

    _triggerTime[id] = millis(); // 타이머 시작

    switch (id)
    {
    case 1: // 서보 1
        if (!_servo1.attached()) _servo1.attach(Servo1);
        _servo1.write(Servo1_end);
        break;
    case 2: // 서보 2
        if (!_servo2.attached()) _servo2.attach(Servo2);
        _servo2.write(Servo2_end);
        break;
    case 3: // 서보 3
        if (!_servo3.attached()) _servo3.attach(Servo3);
        _servo3.write(Servo3_end);
        break;
    case 4: // 파이로 1
        digitalWrite(Pyro1, HIGH);
        break;
    case 5: // 파이로 2
        digitalWrite(Pyro2, HIGH);
        break;
    case 6: // 파이로 3
        digitalWrite(Pyro3, HIGH);
        break;
    }
}

void Recovery::update()
{
    for (int id = 0; id <= 2; id++) 
    {
        if (_triggerTime[id] == 0) continue;

        unsigned long elapsed = millis() - _triggerTime[id];

        if (elapsed > servo_time)
        {
            switch (id)
            {
            case 0:
                if (SERVO1_available && _servo1.attached()) _servo1.detach();
                break;
            case 1:
                if (SERVO2_available && _servo2.attached()) _servo2.detach();
                break;
            case 2:
                if (SERVO3_available && _servo3.attached()) _servo3.detach();
                break;
            }
        }
    }

    for (int id = 3; id <= 5; id++)
    {
        if (_triggerTime[id] == 0) continue;

        unsigned long elapsed = millis() - _triggerTime[id];

        if (elapsed > pyro_time)
        {
            switch (id)
            {
            case 3:
                if (Pyro1_available) digitalWrite(Pyro1, LOW);
                break;
            case 4:
                if (Pyro2_available) digitalWrite(Pyro2, LOW);
                break;
            case 5:
                if (Pyro3_available) digitalWrite(Pyro3, LOW);
                break;
            }
        }
    }
}