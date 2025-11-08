#pragma once
#include <Arduino.h>

#define LEFT_PWM 0
#define LEFT_IN1 0
#define LEFT_IN2 0
#define LEFT_ENCODER 0
#define RIGHT_PWM 0
#define RIGHT_IN1 0
#define RIGHT_IN2 0
#define RIGHT_ENCODER 0
#define STBY 0

#define ENCODER_PPR 20
#define WHEEL_DIAMETER 65.0
#define MAX_SPEED 255

#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

#define SAMPLE_FREQ_HZ 100
const TickType_t xLoopPeriod = (1000 / SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;

class Motor {
    public:
        Motor(byte pwm, byte in1, byte in2, byte encoder);

        static void begin();
        void ping();
        void setSpeed(byte speed);
        static void setSpeed(byte speed);

    private:
        byte pwmPin;
        byte in1Pin;
        byte in2Pin;
        byte encoderPin;
        byte currentSpeed;
        volatile unsigned long encoderCount;
        float rpm;
        float velocity;

        void init(byte pwm, byte in1, byte in2, byte encoder);
        void setup();
        void free();
        void brake();
        void stby(bool enable);
        void update();
        void resetEncoder();
        void taskLoop();
        void taskTrampoline(void *pvParameters);
};