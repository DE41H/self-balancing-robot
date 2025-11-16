#include <pid_controller.hpp>

static const TickType_t xLoopPeriod = (1000 / PIDController::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

PIDController::PIDController(const double kp, const double ki, const double kd, const double min, const double max, const String name, const byte priority, QueueHandle_t inputQueue):
_input(0),
_output(0),
_setpoint(0),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT),
_name(name),
_priority(priority),
_inputQueue(inputQueue)
{
    init(min, max);
}

void PIDController::init(const double min, const double max) {
    _outputQueue = xQueueCreate(1, sizeof(double));
    if (_outputQueue == NULL) {
        Serial.println("Failed to create yaw queue!");
        return;
    }

    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        taskTrampoline,
        "Gyro Task",
        TASK_STACK_SIZE,
        this,
        _priority,
        &_taskHandle,
        TASK_CORE_ID
    );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create " + _name + " task!");
        vQueueDelete(_outputQueue);
        return;
    }

    _pid.SetSampleTime(SAMPLE_TIME);
    _pid.SetOutputLimits(min, max);
    _pid.SetMode(AUTOMATIC);
}

void PIDController::update() {
    double buffer;
    _input = xQueuePeek(_inputQueue, &buffer, NULL);
    _pid.Compute();
    xQueueOverwrite(_outputQueue, &_output);
}

void PIDController::taskLoop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void PIDController::taskTrampoline(void* pvParameters) {
    PIDController* instance = static_cast<PIDController*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}