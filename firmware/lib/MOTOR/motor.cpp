#include <motor.hpp>

void IRAM_ATTR encoderPingA() {
    A.ping();
}

void IRAM_ATTR encoderPingB() {
    B.ping();
}

Motor A(
    Config::MOTOR_A_PWM_PIN,
    Config::MOTOR_A_PWM_CHAN,
    Config::MOTOR_A_IN1_PIN,
    Config::MOTOR_A_IN2_PIN,
    Config::MOTOR_A_ENC_PIN
);

Motor B(
    Config::MOTOR_B_PWM_PIN,
    Config::MOTOR_B_PWM_CHAN,
    Config::MOTOR_B_IN1_PIN,
    Config::MOTOR_B_IN2_PIN,
    Config::MOTOR_B_ENC_PIN
);

Motor::Motor(const unsigned int pwm, const unsigned int channel, const unsigned int in1, const unsigned int in2, const unsigned int encoderPin):
_pwmPin(pwm),
_in1Pin(in1),
_in2Pin(in2),
_pwmChannel(channel),
_encoderPin(encoderPin),
_encoderCount(0)
{

}

bool Motor::begin() {
    A.init();
    B.init();

    attachInterrupt(digitalPinToInterrupt(Config::MOTOR_A_ENC_PIN), encoderPingA, RISING);
    attachInterrupt(digitalPinToInterrupt(Config::MOTOR_B_ENC_PIN), encoderPingB, RISING);

    pinMode(Config::STBY_PIN, OUTPUT);
    stby(false);

    return true;
}

void Motor::init() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    pinMode(_encoderPin, INPUT_PULLUP);
    ledcSetup(_pwmChannel, Config::PWM_FREQUENCY, Config::PWM_RESOLUTION);
    ledcAttachPin(_pwmPin, _pwmChannel);
}

void IRAM_ATTR Motor::setPWM(int pwm) {
    if (pwm > Config::PWM_LIMIT) {
        pwm = Config::PWM_LIMIT; 
    } else if (pwm < -Config::PWM_LIMIT) {
        pwm = -Config::PWM_LIMIT;
    }
    drive(pwm);
}

void IRAM_ATTR Motor::stby(bool enable) {
    digitalWrite(Config::STBY_PIN, enable ? HIGH : LOW);
}

void IRAM_ATTR Motor::drive(int pwm) {
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

void IRAM_ATTR Motor::ping() {
    _encoderCount++;
}
