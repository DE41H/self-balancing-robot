#pragma once
#include <Arduino.h>
#include "driver/ledc.h"
#include <config.hpp>


class Motor {
    public:
        Motor(const unsigned int pwm, const unsigned int channel, const unsigned int in1, const unsigned int in2, const unsigned int encoderPin);
        
        void setPWM(int pwm);
        void ping();
        unsigned int getEncoderCount() const { return _encoderCount; }
        static bool begin();

    private:
        unsigned int _pwmPin, _pwmChannel, _in1Pin, _in2Pin, _encoderPin;
        volatile unsigned int _encoderCount;

        void init();
        void drive(int pwm);

        static void stby(bool enable);
};


extern Motor A;
extern Motor B;