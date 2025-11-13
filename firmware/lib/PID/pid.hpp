#pragma once
#include <Arduino.h>
#include <PID_v1.h>

class CascadedPID {
    public:
        CascadedPID(double ki, double kp, double kd, double setpoint, double min, double max);

        void compute(float input);
        QueueHandle_t getOutput() { return _outputQueue; }

    private:
        double _setpoint;
        double _input;
        double _output;
        PID _pid;
        QueueHandle_t _outputQueue;

        void init(double min, double max);
};