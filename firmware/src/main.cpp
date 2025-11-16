#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>
#include <config.hpp>

static TaskHandle_t taskHandle;
float filteredBalanceInput = 0.0f;

Gyro gyro = Gyro();
PIDController balance = PIDController(Config::BALANCE_KP, Config::BALANCE_KI, Config::BALANCE_KD, Config::BALANCE_MIN, Config::BALANCE_MAX);
PIDController turn = PIDController(Config::TURN_KP, Config::TURN_KI, Config::TURN_KD, Config::TURN_MIN, Config::TURN_MAX);

void update() {
    float balanceInput;
    float turnInput;
    xQueueReceive(gyro.getPitchQueue(), &balanceInput, portMAX_DELAY);
    xQueueReceive(gyro.getYawQueue(), &turnInput, portMAX_DELAY);

    filteredBalanceInput = (Config::EMA_ALPHA * balanceInput) + (1.0f - Config::EMA_ALPHA) * filteredBalanceInput;

    float balanceOutput = balance.compute(filteredBalanceInput, 0.0f);
    float turnOutput = turn.compute(turnInput, 0.0f);

    A.setRPM(balanceOutput + turnOutput);
    B.setRPM(balanceOutput - turnOutput);
}

void taskLoop() {
    while (true) {
        update();
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
