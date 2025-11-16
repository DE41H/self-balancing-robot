#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include <config.hpp>


class PIDController {
    public:
        PIDController(const double kp, const double ki, const double kd, const double min, const double max);

        double compute(double input, double setpoint);
    
    private:
        double _input;
        double _output;
        double _setpoint;
        PID _pid;

        void init(const double min, const double max);
};
