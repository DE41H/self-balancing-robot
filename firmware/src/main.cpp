#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>
#include <config.hpp>

static TaskHandle_t taskHandle;

Gyro gyro = Gyro();
PIDController balance = PIDController(Config::BALANCE_KP, Config::BALANCE_KI, Config::BALANCE_KD, Config::BALANCE_MIN, Config::BALANCE_MAX);
PIDController turn = PIDController(Config::TURN_KP, Config::TURN_KI, Config::TURN_KD, Config::TURN_MIN, Config::TURN_MAX);

void update() {
    struct Gyro::Data input;
    xQueueReceive(gyro.getDataQueue(), &input, portMAX_DELAY);

    if (abs(input.pitch) > 70.0f) {
        A.setPWM(0);
        B.setPWM(0);
        Motor::stby(true);
        return;
    }

    struct Gyro::Data output = {balance.compute(input.pitch, 0.0f), turn.compute(input.yaw, 0.0f)};

    A.setPWM((int) (output.pitch + output.yaw));
    B.setPWM((int) (output.pitch - output.yaw));
}

void loop() {
    
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
