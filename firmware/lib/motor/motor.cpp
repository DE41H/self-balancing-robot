#include <motor.hpp>

Motor A(LEFT_PWM, LEFT_IN1, LEFT_IN2, LEFT_ENCODER);
Motor B(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, RIGHT_ENCODER);

const float circumference = PI * WHEEL_DIAMETER;

Motor::Motor(byte pwm, byte in1, byte in2, byte encoder) {
    init(pwm, in1, in2, encoder);
}

void Motor::init(byte pwm, byte in1, byte in2, byte encoder) {
    pwmPin = pwm;
    in1Pin = in1;
    in2Pin = in2;
    encoderPin = encoder;
    currentSpeed = 0;
    encoderCount = 0;
    rpm = 0.0;
    velocity = 0.0;

    pinMode(pwmPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);
}

void Motor::setup() {
    pinMode(STBY, OUTPUT);
    stby(true);
    attachInterrupt(digitalPinToInterrupt(A.encoderPin), LEFT_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(B.encoderPin), RIGHT_ISR, RISING);

    #ifdef ESP32
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(A.pwmPin, 0);
    ledcAttachPin(B.pwmPin, 1);
    #endif

    A.free();
    B.free();
}

void Motor::setSpeed(byte speed) {
    speed = constrain(speed, 0, MAX_SPEED);
    currentSpeed = speed;

    stby(false);
    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, speed);
    }
    else if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, speed);
    }
    else {
        free();
    }
}

void Motor::free() {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
    currentSpeed = 0;
    stby(true);
}

void Motor::brake() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 0);
    currentSpeed = 0;
    stby(true);
}

void Motor::stby(bool enable) {
    if (enable) {
        digitalWrite(STBY, LOW);
        A.currentSpeed = 0;
        B.currentSpeed = 0;
    }
    else {
        digitalWrite(STBY, HIGH);
    }
}

void Motor::update() {
    float dt = xLoopPeriod;
    rpm = (encoderCount / (float) ENCODER_PPR) * (60.0/dt);
    velocity = rpm * (circumference/60.0);
    resetEncoder();
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
    Motor* instance = static_cast<Motor*>(pvParameters);
    
    instance->taskLoop();

    vTaskDelete(NULL);
}

void IRAM_ATTR LEFT_ISR() {
    A.ping();
}

void IRAM_ATTR RIGHT_ISR() {
    B.ping();
}

void Motor::ping() {
    encoderCount++;
}

void Motor::resetEncoder() {
    noInterrupts();
    encoderCount = 0;
    interrupts();
}
