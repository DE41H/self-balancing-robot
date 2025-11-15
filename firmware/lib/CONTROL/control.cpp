#include <control.hpp>

Control::Control():
_balanceInput(0),
_balanceOutput(0),
_balanceSetpoint(0),
_driftInput(0),
_driftOutput(0),
_driftSetpoint(0),
_turnInput(0),
_turnOutput(0),
_turnSetpoint(0),
_balance(&_balanceInput, &_balanceOutput, &_balanceSetpoint, 0, 0, 0, DIRECT),
_turn(&_turnInput, &_turnOutput, &_turnSetpoint, 0, 0, 0, DIRECT),
_drift(&_driftInput, &_driftOutput, &_driftSetpoint, 0, 0, 0, DIRECT)
{
    init();
}

void Control::init() {
    _balance.SetOutputLimits(BALANCE_MIN, BALANCE_MAX);
    _balance.SetSampleTime(SAMPLE_TIME);
    _balance.SetMode(AUTOMATIC);
    _drift.SetOutputLimits(DRIFT_MIN, DRIFT_MAX);
    _drift.SetSampleTime(SAMPLE_TIME);
    _drift.SetMode(AUTOMATIC);
    _turn.SetOutputLimits(TURN_MIN, TURN_MAX);
    _turn.SetSampleTime(SAMPLE_TIME);
    _turn.SetMode(AUTOMATIC);
}