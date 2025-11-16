#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "driver/ledc.h"
#include <pid_controller.hpp>
#include <config.hpp>


class Motor {
    public:
        Motor(const byte pwm, const byte channel, const byte in1, const byte in2, const byte encoderA, const byte encoderB, const pcnt_unit_t pcntUnit);
        
        void setRPM(float RPM);

        static bool begin();

    private:
        byte _pwmPin, _pwmChannel, _in1Pin, _in2Pin, _encoderPinA, _encoderPinB;
        pcnt_unit_t _pcntUnit;
        int16_t _lastPcntCount;

        float _currentRPM;
        float _targetRPM;
        PIDController _rpm;

        void init();
        void update();
        bool setupPCNT();
        void drive(float pwm);

        static void stby(bool enable);
};


extern Motor A;
extern Motor B;