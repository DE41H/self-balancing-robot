#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include <gyro.hpp>


class PIDController {
    public:
        PIDController(double ki, double kp, double kd, double setpoint, double min, double max);

        void setpoint(double target);
        void compute(float input);
        QueueHandle_t getOutput() const { return _outputQueue; }

    private:
        double _setpoint;
        double _input;
        double _output;
        PID _pid;
        QueueHandle_t _outputQueue;

        void init(double min, double max);
};