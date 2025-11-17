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
int lastPwmB = 0;

void update() {
    struct Gyro::Data input;

    if (xQueueReceive(gyro.getDataQueue(), &input, pdMS_TO_TICKS(Config::MAX_WAIT_MS)) != pdTRUE || abs(input.pitch) > Config::DEADMAN_ANGLE) {
        A.setPWM(0);
        B.setPWM(0);
        Motor::stby(true);

        balance.reset();
        speed.reset();
        turn.reset();
        
        return;
    }

    unsigned int currentEncoderCountA = A.getEncoderCount();
    unsigned int currentEncoderCountB = B.getEncoderCount();

    int ticksA = currentEncoderCountA - lastEncoderCountA;
    int ticksB = currentEncoderCountB - lastEncoderCountB;

    lastEncoderCountA = currentEncoderCountA;
    lastEncoderCountB = currentEncoderCountB;

    int directionA = (lastPwmA >= 0) ? 1 : -1;
    int directionB = (lastPwmB >= 0) ? 1 : -1;

    float rawSpeedA = ticksA * directionA;
    float rawSpeedB = ticksB * directionB;
    float rawSpeed = (rawSpeedA + rawSpeedB) / 2.0f;
    
    filteredSpeed = (Config::SPEED_ALPHA * filteredSpeed) + ((1.0f - Config::SPEED_ALPHA) * rawSpeed);
    float targetPitch = speed.compute(-filteredSpeed, 0.0f);

    struct Gyro::Data output = {balance.compute(input.pitch, targetPitch), turn.compute(input.yaw, 0.0f)};

    int pwmA = (int) (output.pitch + output.yaw);
    int pwmB = (int) (output.pitch - output.yaw);

    A.setPWM(pwmA);
    B.setPWM(pwmB);

    lastPwmA = pwmA;
    lastPwmB = pwmB;
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
