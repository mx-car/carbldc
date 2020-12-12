//
// Created by firat on 20.01.20.
//

#include <car/bldc/motor.h>

uint8_t car::bldc::Motor::getFeedForwardPIDTerm() {
    return FFPIDparams.RPStoSpeedCmdMap[int( (targetRPS - FFPIDparams.rps_min) * 100.0f / (FFPIDparams.rps_max - FFPIDparams.rps_min))];
}

void car::bldc::Motor::updatePreviousRPSMeasurement() {
    previousRPS = currentRPS;
}

