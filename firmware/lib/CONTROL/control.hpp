#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include <gyro.hpp>


class Control {
    public:
        Control();

    private:
        static constexpr int SAMPLE_TIME = 1000 / Gyro::SAMPLE_FREQ_HZ;
        static constexpr double BALANCE_MIN = 0.0;
        static constexpr double BALANCE_MAX = 0.0;
        static constexpr double TURN_MIN = 0.0;
        static constexpr double TURN_MAX = 0.0;
        static constexpr double DRIFT_MIN = 0.0;
        static constexpr double DRIFT_MAX = 0.0;

        double _balanceInput;
        double _balanceOutput;
        double _balanceSetpoint;
        PID _balance;
        double _turnInput;
        double _turnOutput;
        double _turnSetpoint;
        PID _turn;
        double _driftInput;
        double _driftOutput;
        double _driftSetpoint;
        PID _drift;

        void init();
};