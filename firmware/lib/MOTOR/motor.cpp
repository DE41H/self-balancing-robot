#include <motor.hpp>

Motor A(
    Motor::MOTOR_A_PWM_PIN,
    Motor::MOTOR_A_PWM_CHAN,
    Motor::MOTOR_A_IN1_PIN,
    Motor::MOTOR_A_IN2_PIN,
    Motor::MOTOR_A_ENCA_PIN,
    Motor::MOTOR_A_ENCB_PIN,
    Motor::MOTOR_A_PCNT_UNIT 
);

Motor B(
    Motor::MOTOR_B_PWM_PIN,
    Motor::MOTOR_B_PWM_CHAN,
    Motor::MOTOR_B_IN1_PIN,
    Motor::MOTOR_B_IN2_PIN,
    Motor::MOTOR_B_ENCA_PIN,
    Motor::MOTOR_B_ENCB_PIN,
    Motor::MOTOR_B_PCNT_UNIT
);

Motor::Motor(const byte pwm, const byte channel, const byte in1, const byte in2, const byte encoderA, const byte encoderB, const pcnt_unit_t pcntUnit):
_pwmPin(pwm),
_in1Pin(in1),
_in2Pin(in2),
_pwmChannel(channel),
_encoderPinA(encoderA),
_encoderPinB(encoderB),
_pcntUnit(pcntUnit),
_lastPcntCount(0),
_rpm(KP, KI, KD, -PWM_LIMIT, +PWM_LIMIT)
{

}

bool Motor::begin() {
    A.init();
    B.init();

    pinMode(STBY_PIN, OUTPUT);
    stby(false);
}

void Motor::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    ledcSetup(_pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(_pwmPin, _pwmChannel);

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
    _targetRPM = RPM;
    update();
}

void Motor::stby(bool enable) {
    if (enable && !A._currentRPM && !B._currentRPM) {
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
    _currentRPM = delta * RPM_FACTOR;
    drive(_rpm.compute(_currentRPM, _targetRPM));
}
