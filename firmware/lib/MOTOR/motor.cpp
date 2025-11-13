#include <motor.hpp>

static const TickType_t xLoopPeriod = (1000 / Gyro::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

Motor::Motor(byte pwm, byte channel, byte in1, byte in2, byte encoderA, byte encoderB, pcnt_unit_t pcntUnit):
_pwmPin(pwm),
_in1Pin(in1),
_in2Pin(in2),
_pwmChannel(channel),
_encoderPinA(encoderA),
_encoderPinB(encoderB),
_pcntUnit(pcntUnit),
_lastPcntCount(0),
_speedController(KP, KI, KD, 0, -PWM_LIMIT, PWM_LIMIT)
{
    init();
}

void Motor::begin() {
    setup();

    xTaskCreatePinnedToCore(
        taskTrampoline,
        "Motor Task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        &_taskHandle,
        TASK_CORE_ID
    );
}
void Motor::init() {
    _rpmQueue = xQueueCreate(1, sizeof(float));
    if (_rpmQueue == NULL) {
        Serial.println("ERROR: Could not create rpm queue");
    }

    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    if (!setupPCNT()) {
        Serial.printf("Failed to setup PCNT for unit %d\n", _pcntUnit);
    }
}

bool Motor::setupPCNT() {
    pcnt_config_t pcntConfig = {};
    pcntConfig.pulse_gpio_num = _encoderPinA;
    pcntConfig.ctrl_gpio_num = _encoderPinB;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = _pcntUnit;
    
    pcntConfig.pos_mode = PCNT_COUNT_INC;
    pcntConfig.neg_mode = PCNT_COUNT_DEC;
    pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
    pcntConfig.hctrl_mode = PCNT_MODE_KEEP;

    pcnt_unit_config(&pcntConfig);
    pcnt_set_filter_value(_pcntUnit, 1023);
    pcnt_filter_enable(_pcntUnit);
    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_get_counter_value(_pcntUnit, &_lastPcntCount);
    pcnt_counter_resume(_pcntUnit);
    return true;
}

void Motor::setup() {
    pinMode(STBY_PIN, OUTPUT);
    stby(false);
    
    #ifdef ESP32
    ledcSetup(A._pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(B._pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(A._pwmPin, A._pwmChannel);
    ledcAttachPin(B._pwmPin, B._pwmChannel);
    #endif
}

void Motor::setRPM(float RPM) {
    _speedController.setpoint((double) RPM);
}

void Motor::stby(bool enable) {
    static float bufferA;
    static float bufferB;
    xQueuePeek(A._rpmQueue, &bufferA, NULL);
    xQueuePeek(B._rpmQueue, &bufferB, NULL);
    if (enable && !bufferA && !bufferB) {
        digitalWrite(STBY_PIN, LOW);
    }
    else {
        digitalWrite(STBY_PIN, HIGH);
    }
}

void Motor::drive(double pwm) {
    if (pwm > 0) {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
        ledcWrite(_pwmChannel, (uint32_t)pwm);
    } else if (pwm < 0) {
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
        ledcWrite(_pwmChannel, (uint32_t)(-pwm));
    } else {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, HIGH);
        ledcWrite(_pwmChannel, 0);
    }
}

void Motor::update() {
    int16_t currentCount = 0;
    pcnt_get_counter_value(_pcntUnit, &currentCount);
    int16_t delta = currentCount - _lastPcntCount;
    _lastPcntCount = currentCount;
    float rpm = (float) delta * RPM_FACTOR;
    xQueueOverwrite(_rpmQueue, &rpm);
    _speedController.compute(rpm);
    float buffer;
    xQueuePeek(_speedController.getOutput(), &buffer, NULL);
    drive(buffer);
}

void Motor::taskLoop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        A.update();
        B.update();
        xTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

void Motor::taskTrampoline(void* pvParameters) {
    taskLoop();

    vTaskDelete(NULL);
}
