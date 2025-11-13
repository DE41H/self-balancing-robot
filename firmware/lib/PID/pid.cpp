#include <pid.hpp>
#include <gyro.hpp>


// Control::Control():
// _input(0),
// _output(0),
// _setpoint(0),
// _controls(&_input, &_output, &_setpoint, 0, 0, 0, DIRECT)
// {
//     begin(KI, KP, KD, SET_POINT);
// }

// void Control::begin(double ki, double kp, double kd, double setpoint) {
//     _outputQueue = xQueueCreate(1, sizeof(double));
//     if (_outputQueue == NULL) {
//         Serial.println("Failed to create pitch queue!");
//         return;
//     }
//     _setpoint = setpoint;
//     _controls.SetTunings(ki, kp, kd);
//     _controls.SetOutputLimits(MINIMUM_OUTPUT, MAXIMUM_OUTPUT);
//     _controls.SetSampleTime(1000 / Gyro::SAMPLE_FREQ_HZ);
//     _controls.SetMode(AUTOMATIC);
// }

// void PIDControl::compute(float pitch) {
//     _input = (double) pitch;
//     if (_controls.Compute()) {
//         xQueueOverwrite(_outputQueue, &_output);
//     }
// }

CascadedPID::CascadedPID(double kp, double ki, double kd, double setpoint, double min, double max):
_input(0),
_output(0),
_setpoint(setpoint),
_pid(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT)
{
    init(min, max);
}

void CascadedPID::init(double min, double max) {
    _outputQueue = xQueueCreate(1, sizeof(double));
    if (_outputQueue == NULL) {
        Serial.println("Failed to create pitch queue!");
        return;
    }
    _pid.SetOutputLimits(min, max);
    _pid.SetSampleTime(1000 / Gyro::SAMPLE_FREQ_HZ);
    _pid.SetMode(AUTOMATIC);
}

void CascadedPID::compute(float input) {
    _input = (double) input;
    if (_pid.Compute()) {
        xQueueOverwrite(_outputQueue, &_output);
    }
}