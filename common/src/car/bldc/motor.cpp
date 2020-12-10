//
// Created by firat on 20.01.20.
//

#include <car/bldc/motor.h>

uint8_t car::bldc::Motor::getFeedForwardPIDTerm() {
    return feedForwardCmdList[int( (targetRPS - min_rps) * 100.0f / (max_rps - min_rps))];
}

void car::bldc::Motor::updatePreviousRPSMeasurement() {
    speedRPSprevious = speedRPS;
}

