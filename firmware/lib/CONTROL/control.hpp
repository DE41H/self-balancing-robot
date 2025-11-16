#pragma once
#include <Arduino.h>
#include <PID_v1.h>


class Control {
    public:
        Control();
        
        static constexpr int SAMPLE_FREQ_HZ = 100;

        void begin();
        QueueHandle_t getBalanceRPMQueue() const { return _balanceQueue; }
        QueueHandle_t getTurnRPMQueue() const { return _turnQueue; }
        
    private:
        static constexpr int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;
        static constexpr double BALANCE_MIN = 0.0;
        static constexpr double BALANCE_MAX = 0.0;
        static constexpr double TURN_MIN = 0.0;
        static constexpr double TURN_MAX = 0.0;
    
        QueueHandle_t _balanceQueue;
        double _balanceInput;
        double _balanceOutput;
        double _balanceSetpoint;
        PID _balance;

        QueueHandle_t _turnQueue;
        double _turnInput;
        double _turnOutput;
        double _turnSetpoint;
        PID _turn;

        void init();
};

class PIDController {
    public:
        PIDController();

        double _setpoint;

        QueueHandle_t getQueue() const { return _queue; }
    
    private:
        static constexpr int SAMPLE_TIME = 1000 / Control::SAMPLE_FREQ_HZ;

        QueueHandle_t _queue;
        double _input;
        double _output;
        PID _pid;

        void init();
};