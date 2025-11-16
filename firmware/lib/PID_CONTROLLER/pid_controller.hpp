#include <Arduino.h>
#include <PID_v1.h>


class PIDController {
    public:
        PIDController(const double kp, const double ki, const double kd, const double min, const double max, QueueHandle_t inputQueue);

        static constexpr int SAMPLE_FREQ_HZ = 100;

        double _setpoint;
        double compute();
        double getOutput() const { return _output; }
    
    private:
        static constexpr int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;

        QueueHandle_t _inputQueue;
        double _input;
        double _output;
        PID _pid;

        void init(const double min, const double max);
};
