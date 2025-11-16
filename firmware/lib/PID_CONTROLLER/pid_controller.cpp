#include <pid_controller.hpp>

PIDController::PIDController(const float kp, const float ki, const float kd, const float min, const float max):
_input(0.0f),
_output(0.0f),
_setpoint(0.0f),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct)
{
    init(min, max);
}

void PIDController::init(const float min, const float max) {
    _pid.SetSampleTimeUs(Config::SAMPLE_TIME);
    _pid.SetOutputLimits(min, max);
    _pid.SetMode(QuickPID::Control::automatic);
}

float PIDController::compute(float input, float setpoint) {
    _input = input;
    _setpoint = setpoint;
    _pid.Compute();
    return _output;
}
