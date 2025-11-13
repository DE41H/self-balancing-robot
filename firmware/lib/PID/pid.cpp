#include <pid.hpp>

PIDController::PIDController(double kp, double ki, double kd, double setpoint, double min, double max):
_input(0),
_output(0),
_setpoint(setpoint),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT)
{
    init(min, max);
}

void PIDController::init(double min, double max) {
    _outputQueue = xQueueCreate(1, sizeof(double));
    if (_outputQueue == NULL) {
        Serial.println("Failed to create pitch queue!");
        return;
    }
    _pid.SetOutputLimits(min, max);
    _pid.SetSampleTime(1000 / Gyro::SAMPLE_FREQ_HZ);
    _pid.SetMode(AUTOMATIC);
}

void PIDController::setpoint(double point) {
    _setpoint = point;
}

void PIDController::compute(float input) {
    _input = (double) input;
    if (_pid.Compute()) {
        xQueueOverwrite(_outputQueue, &_output);
    }
}