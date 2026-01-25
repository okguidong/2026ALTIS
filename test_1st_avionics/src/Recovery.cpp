#include "Recovery.h"

bool Recovery::begin()
{
    pinMode(Pyro1, OUTPUT);
    digitalWrite(Pyro1, LOW);
    pinMode(Pyro2, OUTPUT);
    digitalWrite(Pyro2, LOW);
    pinMode(Pyro3, OUTPUT);
    digitalWrite(Pyro3, LOW);

    _servo1.attach(Servo1);
    _servo1.write(Servo1_start);
    _servo2.attach(Servo2);
    _servo2.write(Servo2_start);
    _servo3.attach(Servo3);
    _servo3.write(Servo3_start);

    for (int i = 0; i < 7; i++)
    _triggerTime[i] = 0;

    return true;
}

void Recovery::trigger(int id)
{
    if (id < 1 || id > 6)
        return;

    _triggerTime[id] = millis();

    switch (id)
    {
    case 1: // 서보 1
        _servo1.attach(Servo1);
        _servo1.write(Servo1_end);
        break;
    case 2: // 서보 2
        _servo2.attach(Servo2);
        _servo2.write(Servo2_end);
        break;
    case 3: // 서보 3
        _servo3.attach(Servo3);
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
    unsigned long returnDelay = 500;
    if(separated){
        unsigned long dt = millis() - separated_time;
        if(dt > REFIRE_TIMER){
            trigger(RE_FIRE);
        }
    }
    for (int id = 1; id <= 3; id++)
    {
        if (_triggerTime[id] == 0)
            continue;

        unsigned long elapsed = millis() - _triggerTime[id];

        if (elapsed <= servo_time)
        {
        }

        else if (elapsed <= (servo_time + returnDelay))
        {
            switch (id)
            {
            case 1:
                if (!_servo1.attached())
                    _servo1.attach(Servo1);
                _servo1.write(Servo1_start);
                break;
            case 2:
                if (!_servo2.attached())
                    _servo2.attach(Servo2);
                _servo2.write(Servo2_start);
                break;
            case 3:
                if (!_servo3.attached())
                    _servo3.attach(Servo3);
                _servo3.write(Servo3_start);
                break;
            }
        }
        else
        {
            switch (id)
            {
            case 1:
                if (_servo1.attached())
                    _servo1.detach();
                break;
            case 2:
                if (_servo2.attached())
                    _servo2.detach();
                break;
            case 3:
                if (_servo3.attached())
                    _servo3.detach();
                break;
            }
        }
    }

    for (int id = 4; id <= 6; id++)
    {
        if (_triggerTime[id] == 0)
            continue;
        unsigned long elapsed = millis() - _triggerTime[id];

        if (elapsed > pyro_time)
        {
            switch (id)
            {
            case 4:
                digitalWrite(Pyro1, LOW);
                break;
            case 5:
                digitalWrite(Pyro2, LOW);
                break;
            case 6:
                digitalWrite(Pyro3, LOW);
                break;
            }
        }
    }
}
void Recovery::EJ1()
{
    if (EJ1ed == true)
        return;
    trigger(EJECT_1);
    EJ1ed = true;
}

void Recovery::EJ2()
{
    if (EJ2ed == true)
        return;
    trigger(EJECT_2);
    EJ2ed = true;
}

void Recovery::Separate()
{
    if (separated == true)
        return;
    separated = true;
    separated_time = millis();
    trigger(SEPARATION);
}