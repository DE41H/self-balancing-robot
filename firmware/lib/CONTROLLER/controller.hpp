#pragma once
#include <Arduino.h>
#include <PID_v1.h>

class PIDControl {
    public:
        PIDControl();

        void begin(double ki, double kp, double kd, double setpoint);
        void compute(float pitch);
        QueueHandle_t getOutput() { return _outputQueue; }

    private:
        static constexpr double KI = 0.0;
        static constexpr double KP = 0.0;
        static constexpr double KD = 0.0;
        static constexpr double SET_POINT = 0.0;
        static constexpr double MINIMUM_OUTPUT = 0.0;
        static constexpr double MAXIMUM_OUTPUT = 0.0;

        double _setpoint;
        double _input;
        double _output;
        PID _controls;
        QueueHandle_t _outputQueue;
};