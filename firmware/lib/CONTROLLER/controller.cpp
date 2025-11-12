#include <controller.hpp>
#include <gyro.hpp>

PIDControl::PIDControl():
_input(0),
_output(0),
_setpoint(0),
_controls(&_input, &_output, &_setpoint, 0, 0, 0, DIRECT)
{
    begin(KI, KP, KD, SET_POINT);
}

void PIDControl::begin(double ki, double kp, double kd, double setpoint) {
    _outputQueue = xQueueCreate(1, sizeof(double));
    if (_outputQueue == NULL) {
        Serial.println("Failed to create pitch queue!");
        return;
    }
    _setpoint = setpoint;
    _controls.SetTunings(ki, kp, kd);
    _controls.SetOutputLimits(MINIMUM_OUTPUT, MAXIMUM_OUTPUT);
    _controls.SetSampleTime(1000 / Gyro::SAMPLE_FREQ_HZ);
    _controls.SetMode(AUTOMATIC);
}

void PIDControl::compute(float pitch) {
    _input = (double) pitch;
    if (_controls.Compute()) {
        xQueueOverwrite(_outputQueue, &_output);
    }
}