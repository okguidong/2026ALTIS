#include "DataLogger.h"

QueueHandle_t DataLogger::_queue = NULL;

bool DataLogger::begin() {
    _queue = xQueueCreate(500, sizeof(SensorData));
    if (!_queue) return false;
    if (!SD_MMC.begin()) return false;

    // Core 1에서 로깅 작업 수행
    xTaskCreatePinnedToCore(task, "Logger", 8192, NULL, 1, NULL, 1);
    return true;
}

void DataLogger::push(SensorData data) {
    if (_queue) xQueueSend(_queue, &data, 0);
}

void DataLogger::task(void *param) {
    File f = SD_MMC.open("/flight_log.bin", "w");
    if (!f) vTaskDelete(NULL);

    SensorData d;
    int count = 0;

    while (true) {
        if (xQueueReceive(_queue, &d, portMAX_DELAY)) {
            
            f.write((uint8_t*)&d, sizeof(SensorData));
            
            // 50개 데이터마다 Flush
            if (++count >= 50) { 
                f.flush(); 
                count = 0; 
            }
        }
    }
}