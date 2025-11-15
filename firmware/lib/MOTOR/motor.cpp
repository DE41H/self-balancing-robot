#include <motor.hpp>

static const TickType_t xLoopPeriod = (1000 / Motor::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

#define MOTOR_A_PWM_PIN 25
#define MOTOR_A_IN1_PIN 26
#define MOTOR_A_IN2_PIN 27
#define MOTOR_A_ENCA_PIN 34
#define MOTOR_A_ENCB_PIN 35
#define MOTOR_A_PWM_CHAN 0
#define MOTOR_A_PCNT_UNIT PCNT_UNIT_0

#define MOTOR_B_PWM_PIN 16
#define MOTOR_B_IN1_PIN 17
#define MOTOR_B_IN2_PIN 18
#define MOTOR_B_ENCA_PIN 32
#define MOTOR_B_ENCB_PIN 33
#define MOTOR_B_PWM_CHAN 1
#define MOTOR_B_PCNT_UNIT PCNT_UNIT_1

Motor motorA(
    MOTOR_A_PWM_PIN,
    MOTOR_A_PWM_CHAN,
    MOTOR_A_IN1_PIN,
    MOTOR_A_IN2_PIN,
    MOTOR_A_ENCA_PIN,
    MOTOR_A_ENCB_PIN,
    MOTOR_A_PCNT_UNIT 
);

Motor motorB(
    MOTOR_B_PWM_PIN,
    MOTOR_B_PWM_CHAN,
    MOTOR_B_IN1_PIN,
    MOTOR_B_IN2_PIN,
    MOTOR_B_ENCA_PIN,
    MOTOR_B_ENCB_PIN,
    MOTOR_B_PCNT_UNIT
);

Motor::Motor(byte pwm, byte channel, byte in1, byte in2, byte encoderA, byte encoderB, pcnt_unit_t pcntUnit):
_pwmPin(pwm),
_in1Pin(in1),
_in2Pin(in2),
_pwmChannel(channel),
_encoderPinA(encoderA),
_encoderPinB(encoderB),
_pcntUnit(pcntUnit),
_lastPcntCount(0),
_input(0),
_output(0),
_setpoint(0),
_rpm(&_input, &_output, &_setpoint, KP, KI, KD, DIRECT)
{

}

void Motor::begin() {
    A.init();
    B.init();

    pinMode(STBY_PIN, OUTPUT);
    stby(false);

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
    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    ledcSetup(_pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(_pwmPin, _pwmChannel);

    _rpm.SetOutputLimits(-PWM_LIMIT, PWM_LIMIT);
    _rpm.SetSampleTime(1000 / SAMPLE_FREQ_HZ);
    _rpm.SetMode(AUTOMATIC);

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
    pcnt_set_filter_value(_pcntUnit, PCNT_FILTER_VALUE);
    pcnt_filter_enable(_pcntUnit);
    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_get_counter_value(_pcntUnit, &_lastPcntCount);
    pcnt_counter_resume(_pcntUnit);
    return true;
}

void Motor::setRPM(double RPM) {
    _setpoint = RPM;
}

void Motor::stby(bool enable) {
    if (enable && !A._setpoint && !B._setpoint) {
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
    _input = delta * RPM_FACTOR;
    _rpm.Compute();
    drive(_output);
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
    Motor::taskLoop();

    vTaskDelete(NULL);
}
