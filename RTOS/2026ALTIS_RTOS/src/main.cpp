#include <Arduino.h>

// [정의] 비트 플래그 설정
#define NOTE_IMU_READY  (1 << 0)  // 0x01
#define NOTE_BARO_READY (1 << 1)  // 0x02

// [전역] Flight Task 핸들
TaskHandle_t hTaskFlight = NULL;

// ----------------------------------------------------
// 1. 인터럽트 서비스 루틴 (ISR) - 최대한 짧게!
// ----------------------------------------------------

// BMI088 Gyro INT 핀이 뛸 때 실행
void IRAM_ATTR isr_imu() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Task에게 "0번 비트(IMU)"를 OR 연산으로 셋팅하며 깨움
    xTaskNotifyFromISR(hTaskFlight, NOTE_IMU_READY, eSetBits, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// BMP388 INT 핀이 뛸 때 실행
void IRAM_ATTR isr_baro() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Task에게 "1번 비트(Baro)"를 OR 연산으로 셋팅 (깨우진 않아도 됨, 혹은 깨워도 됨)
    xTaskNotifyFromISR(hTaskFlight, NOTE_BARO_READY, eSetBits, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// ----------------------------------------------------
// 2. 메인 비행 태스크 (Core 1)
// ----------------------------------------------------
void TaskFlight(void *pvParameters) {
    hTaskFlight = xTaskGetCurrentTaskHandle();
    
    // 인터럽트 연결
    attachInterrupt(IMU_PIN, isr_imu, RISING);
    attachInterrupt(BARO_PIN, isr_baro, RISING);

    uint32_t notifiedValue = 0;

    for(;;) {
        // ★ 여기서 대기 (인터럽트가 올 때까지 Sleep)
        // ulTaskNotifyTake 대신 xTaskNotifyWait를 써서 어떤 비트가 왔는지 확인
        // pdTRUE: 읽고 나서 비트를 지움 (Clear)
        // portMAX_DELAY: 신호 올 때까지 무한 대기
        xResult = xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY);

        if (xResult == pdPASS) {
            
            // [순서 1] Baro 데이터가 준비되었는지 확인 (비트 1번 체크)
            if ((notifiedValue & NOTE_BARO_READY) != 0) {
                // SPI 버스 사용 (Baro 읽기)
                readBMP388(); 
                
                // 퓨전 업데이트 (Update Step)
                updateFusionBaro();
            }

            // [순서 2] IMU 데이터가 준비되었는지 확인 (비트 0번 체크)
            // 비행 제어의 메인 트리거
            if ((notifiedValue & NOTE_IMU_READY) != 0) {
                // SPI 버스 사용 (IMU 읽기)
                // ★ 위에서 Baro를 읽고 내려왔더라도, 순차 실행되므로 충돌 안 남!
                readBMI088(); 
                
                // 자세 제어 및 퓨전 예측 (Predict Step)
                runAttitudeAndControl();
                
                // 모터 출력
                writeMotors();
                
                // 로깅 큐 전송
                sendToLogQueue();
            }
        }
    }
}