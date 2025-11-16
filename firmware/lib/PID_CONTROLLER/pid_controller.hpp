#pragma once
#include <Arduino.h>
#include <PID_v1.h>


class PIDController {
    public:
        PIDController(const double kp, const double ki, const double kd, const double min, const double max);

        double compute(double input, double setpoint);
    
    private:
        static constexpr int SAMPLE_FREQ_HZ = 100;
        static constexpr int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;

        double _input;
        double _output;
        double _setpoint;
        PID _pid;

        void init(const double min, const double max);
};
