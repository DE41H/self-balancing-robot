#include <motor.hpp>

Motor A(Motor::LEFT_PWM, Motor::LEFT_IN1, Motor::LEFT_IN2, Motor::LEFT_ENCODER, Motor::LEFT_PWM_CHANNEL);
Motor B(Motor::RIGHT_PWM, Motor::RIGHT_IN1, Motor::RIGHT_IN2, Motor::RIGHT_ENCODER, Motor::RIGHT_PWM_CHANNEL);

Motor::Motor(byte pwm, byte in1, byte in2, byte encoder, byte channel) {
    init(pwm, in1, in2, encoder, channel);
}

void Motor::begin() {
    setup();

    xTaskCreatePinnedToCore(
        taskTrampoline,
        "Motor Task",
        4096,
        NULL,
        2,
        &taskHandle,
        0
    );
}

void IRAM_ATTR LEFT_ISR() {
    A.ping();
}

void IRAM_ATTR RIGHT_ISR() {
    B.ping();
}

void IRAM_ATTR Motor::ping() {
    static byte tick = 1;
    xQueueSendToBackFromISR(encoderQueue, &tick, NULL);
}

void Motor::init(byte pwm, byte in1, byte in2, byte encoder, byte channel) {
    pwmPin = pwm;
    pwmChannel = channel;
    in1Pin = in1;
    in2Pin = in2;
    encoderPin = encoder;
    currentSpeed = 0;
    rpm = 0.0;
    velocity = 0.0;

    encoderQueue = xQueueCreate(ENCODER_QUEUE_LENGTH, sizeof(byte));
    if (encoderQueue == NULL) {
        Serial.println("ERROR: Could not create encoder queue");
    }

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
    ledcAttachPin(A.pwmPin, A.pwmChannel);
    ledcAttachPin(B.pwmPin, B.pwmChannel);
    #endif

    A.free();   
    B.free();
}

void Motor::setSpeed(short int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    currentSpeed = speed;

    stby(false);
    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        ledcWrite(pwmChannel, speed);
    }
    else if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        ledcWrite(pwmChannel, -speed);
    }
    else {
        free();
    }
}

void Motor::free() {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmChannel, 0);
    currentSpeed = 0;
    stby(true);
}

void Motor::brake() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmChannel, 0);
    currentSpeed = 0;
    stby(true);
}

void Motor::stby(bool enable) {
    if (enable) {
        digitalWrite(STBY, LOW);
    }
    else {
        digitalWrite(STBY, HIGH);
    }
}

void Motor::update() {
    uint8_t tick;
    unsigned long encoderCount = 0;
    while (xQueueReceive(encoderQueue, &tick, 0) == pdTRUE) {
        encoderCount++;
    }

    rpm = ((float) encoderCount /(float) ENCODER_PPR) * (60.0/dt);
    velocity = rpm * (circumference/60.0);
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
