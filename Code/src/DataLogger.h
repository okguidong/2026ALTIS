#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "FS.h"
#include "SD_MMC.h"
#include "RocketData.h"

class DataLogger {
public:
    bool begin();
    void push(SensorData data);

private:
    static void task(void *param);
    static QueueHandle_t _queue;
};
#endif // DATA_LOGGER_H