#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "driver/ledc.h"
#include <pid_controller.hpp>


class Motor {
    public:
        Motor(const byte pwm, const byte channel, const byte in1, const byte in2, const byte encoderA, const byte encoderB, const pcnt_unit_t pcntUnit);

        static constexpr int SAMPLE_FREQ_HZ = 100;
        static constexpr byte MOTOR_A_PWM_PIN = 0;
        static constexpr byte MOTOR_A_IN1_PIN = 0;
        static constexpr byte MOTOR_A_IN2_PIN = 0;
        static constexpr byte MOTOR_A_ENCA_PIN = 0;
        static constexpr byte MOTOR_A_ENCB_PIN = 0;
        static constexpr byte MOTOR_A_PWM_CHAN = 0;
        static constexpr pcnt_unit_t MOTOR_A_PCNT_UNIT = PCNT_UNIT_0;
        static constexpr byte MOTOR_B_PWM_PIN = 0;
        static constexpr byte MOTOR_B_IN1_PIN = 0;
        static constexpr byte MOTOR_B_IN2_PIN = 0;
        static constexpr byte MOTOR_B_ENCA_PIN = 0;
        static constexpr byte MOTOR_B_ENCB_PIN = 0;
        static constexpr byte MOTOR_B_PWM_CHAN = 0;
        static constexpr pcnt_unit_t MOTOR_B_PCNT_UNIT = PCNT_UNIT_1;
        
        void setRPM(double RPM);

        static bool begin();

    private:
        static constexpr int TASK_STACK_SIZE = 4096;
        static constexpr int TASK_PRIORITY = 10;
        static constexpr int TASK_CORE_ID = 0;
        static constexpr int PWM_FREQUENCY = 20000;
        static constexpr byte PWM_RESOLUTION = 8;
        static constexpr int PWM_LIMIT = (1 << PWM_RESOLUTION) - 1;
        static constexpr byte STBY_PIN = 0;
        static constexpr double ENCODER_PPR = 20.0;
        static constexpr double RPM_FACTOR = (60.0 * SAMPLE_FREQ_HZ) / ENCODER_PPR;
        static constexpr double KP = 0.0;
        static constexpr double KI = 0.0;
        static constexpr double KD = 0.0;
        static constexpr int PCNT_FILTER_VALUE = 100;

        byte _pwmPin, _pwmChannel, _in1Pin, _in2Pin, _encoderPinA, _encoderPinB;
        pcnt_unit_t _pcntUnit;
        int16_t _lastPcntCount;

        static TaskHandle_t _taskHandle;

        QueueHandle_t _currentRpmQueue;
        QueueHandle_t _targetRpmQueue;
        PIDController _rpm;

        void init();
        void update();
        bool setupPCNT();
        void drive(double pwm);

        static void stby(bool enable);

        static void taskLoop();
        static void taskTrampoline(void *pvParameters);
};


extern Motor A;
extern Motor B;