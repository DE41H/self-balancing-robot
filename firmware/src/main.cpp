#include <Arduino.h>
#include <gyro.hpp>
#include <motor.hpp>
#include <pid_controller.hpp>

static constexpr int SAMPLE_FREQ_HZ = 100;
static constexpr int TASK_STACK_SIZE = 4096;
static constexpr int TASK_PRIORITY = 0;
static constexpr int TASK_CORE_ID = 0;
static TaskHandle_t TASK_HANDLE;

static constexpr double BALANCE_KP = 0.0;
static constexpr double BALANCE_KI = 0.0;
static constexpr double BALANCE_KD = 0.0;
static constexpr double BALANCE_MIN = 0.0;
static constexpr double BALANCE_MAX = 0.0;

static constexpr double TURN_KP = 0.0;
static constexpr double TURN_KI = 0.0;
static constexpr double TURN_KD = 0.0;
static constexpr double TURN_MIN = 0.0;
static constexpr double TURN_MAX = 0.0;

Gyro gyro = Gyro();
PIDController balance = PIDController(BALANCE_KP, BALANCE_KI, BALANCE_KP, BALANCE_MIN, BALANCE_MAX);
PIDController turn = PIDController(TURN_KP, TURN_KI, TURN_KP, TURN_MIN, TURN_MAX);

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
