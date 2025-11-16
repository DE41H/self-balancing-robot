#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>

static constexpr int SAMPLE_FREQ_HZ = 100;
static constexpr int TASK_STACK_SIZE = 4096;
static constexpr int TASK_PRIORITY = 0;
static constexpr int TASK_CORE_ID = 0;
static TaskHandle_t TASK_HANDLE;

Gyro gyro = Gyro();
PIDController balance = PIDController(gyro.getPitchQueue());
PIDController turn = PIDController(gyro.getYawQueue());

void setup() {
    if (!gyro.begin() || !Motor::begin()) {
        return;
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        taskTrampoline,
        "Main Task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        &TASK_HANDLE,
        TASK_CORE_ID
    );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create motor task!");
        return;
    }
}

void update() {
    double balanceInput;
    double turnInput;
    xQueueReceive(gyro.getPitchQueue(), &balanceInput, portMAX_DELAY);
    xQueueReceive(gyro.getYawQueue(), &turnInput, portMAX_DELAY);

    double balanceOutput;
    double turnOutput;
    balanceOutput = balance.compute(balanceInput);
    turnOutput = balance.compute(turnInput);

    A.setRPM(balanceOutput + turnOutput);
    B.setRPM(balanceOutput - turnOutput);
}

void taskLoop() {
    static const TickType_t xLoopPeriod = (1000 / SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void taskTrampoline(void *pvParameters) {
    taskLoop();

    vTaskDelete(NULL);
}
