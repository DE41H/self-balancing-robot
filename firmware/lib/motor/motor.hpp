#pragma once
#include <Arduino.h>
#include <cmath>


class Motor {
    public:
        Motor(byte pwm, byte in1, byte in2, byte encoder, byte channel);

        static constexpr byte LEFT_PWM = 0;
        static constexpr byte LEFT_PWM_CHANNEL = 0;
        static constexpr byte LEFT_IN1 = 0;
        static constexpr byte LEFT_IN2 =  0;
        static constexpr byte LEFT_ENCODER =  0;
        static constexpr byte RIGHT_PWM = 0;
        static constexpr byte RIGHT_PWM_CHANNEL = 1;
        static constexpr byte RIGHT_IN1 = 0;
        static constexpr byte RIGHT_IN2 = 0;
        static constexpr byte RIGHT_ENCODER = 0;
        static constexpr byte STBY = 0;
        static constexpr byte SAMPLE_FREQ_HZ = 100;
        static constexpr TickType_t xLoopPeriod = (1000 / Motor::SAMPLE_FREQ_HZ) / portTICK_PERIOD_MS;
        
        void IRAM_ATTR ping();
        void setSpeed(short int speed);

        float getVelocity() { return velocity; }
        float getRPM() { return rpm; }

        static void begin();

    private:
        static constexpr byte ENCODER_PPR = 20;
        static constexpr float WHEEL_DIAMETER = 65.0;
        static constexpr byte MAX_SPEED = 255;
        static constexpr float circumference = M_PI * WHEEL_DIAMETER;
        
        static constexpr int PWM_FREQUENCY = 20000;
        static constexpr byte PWM_RESOLUTION = 8;

        static constexpr float dt = 1.0 / SAMPLE_FREQ_HZ;

        static constexpr byte ENCODER_QUEUE_LENGTH = 32;

        static TaskHandle_t taskHandle;
        QueueHandle_t encoderQueue;

        byte pwmPin;
        byte pwmChannel;
        byte in1Pin;
        byte in2Pin;
        byte encoderPin;
        short int currentSpeed;
        float rpm;
        float velocity;

        void init(byte pwm, byte in1, byte in2, byte encoder, byte channel);
        void free();
        void brake();
        void update();

        static void setup();
        static void stby(bool enable);

        static void taskLoop();
        static void taskTrampoline(void *pvParameters);
};


extern Motor A;
extern Motor B;