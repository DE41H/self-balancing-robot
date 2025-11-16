#include <Arduino.h>
#include <PID_v1.h>


class PIDController {
    public:
        PIDController(const double kp, const double ki, const double kd, const double min, const double max, const String name, const byte priority, QueueHandle_t inputQueue);

        static constexpr int SAMPLE_FREQ_HZ = 100;

        double _setpoint;

        QueueHandle_t getOutputQueue() const { return _outputQueue; }
    
    private:
        static constexpr int SAMPLE_TIME = 1000 / SAMPLE_FREQ_HZ;
        static constexpr int TASK_STACK_SIZE = 4096;
        static constexpr int TASK_CORE_ID = 0;

        String _name;
        byte _priority;
        TaskHandle_t _taskHandle;

        QueueHandle_t _inputQueue;
        QueueHandle_t _outputQueue;
        double _input;
        double _output;
        PID _pid;

        void getInput(QueueHandle_t queue);
        void init(const double min, const double max);
        void update();
        void taskLoop();
        static void taskTrampoline(void *pvParameters);
};
