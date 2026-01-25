#include "DataLogger.h"

// 정적 멤버 변수 초기화
QueueHandle_t DataLogger::_queue = NULL;

bool DataLogger::begin() {
    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_DATA0_PIN, SD_DATA1_PIN, SD_DATA2_PIN, SD_DATA3_PIN);

    if (!SD_MMC.begin("/sdcard", false, true)) {
        Serial.println("SD_MMC Init Failed! Check Pull-up Resistors.");
        return false;
    }

    _queue = xQueueCreate(500, sizeof(SensorData));
    if (!_queue) return false;
    xTaskCreatePinnedToCore(task, "Logger", 8192, NULL, 1, NULL, 1);
    return true;
}

void DataLogger::push(SensorData data) {
    if (_queue) {
        xQueueSend(_queue, &data, 0);
    }
}

void DataLogger::task(void *param) {
    char fileName[32];
    int fileNum = 0;
    while (true) {
        sprintf(fileName, "/flight_log_%03d.bin", fileNum);
        if (!SD_MMC.exists(fileName)) {
            break; // 파일이 없으면 이 이름을 사용
        }
        fileNum++;
    }
    
    Serial.printf("Logging to: %s\n", fileName);

    File f = SD_MMC.open(fileName, FILE_WRITE);
    if (!f) {
        Serial.println("Failed to open file for writing");
        vTaskDelete(NULL);
    }

    SensorData d;
    int flushCount = 0;

    while (true) {
        if (xQueueReceive(_queue, &d, portMAX_DELAY)) {
            f.write((uint8_t*)&d, sizeof(SensorData));
            if (++flushCount >= 50) { 
                f.flush(); 
                flushCount = 0; 
            }
        }
    }
}