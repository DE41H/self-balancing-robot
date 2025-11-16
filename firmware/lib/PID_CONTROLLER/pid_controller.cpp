#include <pid_controller.hpp>

PIDController::PIDController(const double kp, const double ki, const double kd, const double min, const double max):
_input(0),
_output(0),
_setpoint(0),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT)
{
    init(min, max);
}

void PIDController::init(const double min, const double max) {
    _pid.SetSampleTime(Config::SAMPLE_TIME);
    _pid.SetOutputLimits(min, max);
    _pid.SetMode(AUTOMATIC);
}

double PIDController::compute(double input, double setpoint) {
    _input = input;
    _setpoint = setpoint;
    _pid.Compute();
    return _output;
}
