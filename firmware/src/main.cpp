#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>
#include <config.hpp>

static TaskHandle_t taskHandle;

Gyro gyro = Gyro();
PIDController balance = PIDController(Config::BALANCE_KP, Config::BALANCE_KI, Config::BALANCE_KD, Config::BALANCE_MIN, Config::BALANCE_MAX);
PIDController turn = PIDController(Config::TURN_KP, Config::TURN_KI, Config::TURN_KD, Config::TURN_MIN, Config::TURN_MAX);
PIDController speed = PIDController(Config::SPEED_KP, Config::SPEED_KI, Config::SPEED_KD, Config::SPEED_MIN, Config::SPEED_MAX);

unsigned int lastEncoderCountA = 0;
unsigned int lastEncoderCountB = 0;
float filteredSpeed = 0.0f;
int lastPwmA = 0;

void update() {
    struct Gyro::Data input;
    xQueueReceive(gyro.getDataQueue(), &input, portMAX_DELAY);

    if (abs(input.pitch) > Config::DEADMAN_ANGLE) {
        A.setPWM(0);
        B.setPWM(0);
        Motor::stby(true);
        return;
    }

    unsigned int currentEncoderCountA = A.getEncoderCount();
    unsigned int currentEncoderCountB = B.getEncoderCount();

    int speedA = currentEncoderCountA - lastEncoderCountA;
    int speedB = currentEncoderCountB - lastEncoderCountB;

    lastEncoderCountA = currentEncoderCountA;
    lastEncoderCountB = currentEncoderCountB;

    if (lastPwmA < 0) speedA = -speedA;
    if (lastPwmA < 0) speedB = -speedB;

    float rawSpeed = (speedA + speedB) / 2.0f;
    filteredSpeed = (Config::SPEED_ALPHA * filteredSpeed) + ((1.0f - Config::SPEED_ALPHA) * rawSpeed);

    float targetPitch = speed.compute(-filteredSpeed, 0.0f);

    struct Gyro::Data output = {balance.compute(input.pitch, targetPitch), turn.compute(input.yaw, 0.0f)};

    int pwmA = (int) (output.pitch + output.yaw);
    int pwmB = (int) (output.pitch - output.yaw);

    A.setPWM(pwmA);
    B.setPWM(pwmB);

    lastPwmA = pwmA;
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
