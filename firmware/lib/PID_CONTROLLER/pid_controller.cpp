#include <pid_controller.hpp>

PIDController::PIDController(const double kp, const double ki, const double kd, const double min, const double max, QueueHandle_t inputQueue):
_input(0),
_output(0),
_setpoint(0),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT),
_inputQueue(inputQueue)
{
    init(min, max);
}

void PIDController::init(const double min, const double max) {
    _pid.SetSampleTime(SAMPLE_TIME);
    _pid.SetOutputLimits(min, max);
    _pid.SetMode(AUTOMATIC);
}

double PIDController::compute() {
    double buffer;
    _input = xQueuePeek(_inputQueue, &buffer, NULL);
    _pid.Compute();
    return _output;
}
