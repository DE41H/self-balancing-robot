#pragma once
#include <Arduino.h>
#include <QuickPID.h>
#include <config.hpp>


class PIDController {
    public:
        PIDController(const float kp, const float ki, const float kd, const float min, const float max);

        float compute(float input, float setpoint);
    
    private:
        float _input;
        float _output;
        float _setpoint;
        QuickPID _pid;

        void init(const float min, const float max);
};
