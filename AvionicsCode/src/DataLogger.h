#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include "RocketData.h"
#include "Config.h"

class DataLogger {
public:
    bool begin();
    void push(SensorData data);

private:
    static void task(void *param);
    static QueueHandle_t _queue;
    static File _file; 
};

#endif