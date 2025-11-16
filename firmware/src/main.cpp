#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>
#include <config.hpp>

static TaskHandle_t taskHandle;

Gyro gyro = Gyro();
PIDController balance = PIDController(Config::BALANCE_KP, Config::BALANCE_KI, Config::BALANCE_KD, Config::BALANCE_MIN, Config::BALANCE_MAX);
PIDController turn = PIDController(Config::TURN_KP, Config::TURN_KI, Config::TURN_KP, Config::TURN_MIN, Config::TURN_MAX);

void update() {
    double balanceInput;
    double turnInput;
    xQueueReceive(gyro.getPitchQueue(), &balanceInput, portMAX_DELAY);
    xQueueReceive(gyro.getYawQueue(), &turnInput, portMAX_DELAY);

    double balanceOutput;
    double turnOutput;
    balanceOutput = balance.compute(balanceInput, 0);
    turnOutput = turn.compute(turnInput, 0);

    A.setRPM(balanceOutput + turnOutput);
    B.setRPM(balanceOutput - turnOutput);
}

void taskLoop() {
    static const TickType_t xLoopPeriod = (1000 / Config::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;
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

void setup() {
    if (!gyro.begin() || !Motor::begin()) {
        return;
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        taskTrampoline,
        "Main Task",
        Config::MAIN_TASK_STACK_SIZE,
        NULL,
        Config::MAIN_TASK_PRIORITY,
        &taskHandle,
        Config::MAIN_TASK_CORE_ID
    );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create motor task!");
        return;
    }
}
