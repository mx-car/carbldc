//
// Created by firat on 20.01.20.
//

#include <car/bldc/motor.h>

uint8_t car::bldc::Motor::getFeedForwardPIDTerm() {
    return feedForwardCmdList[int( (targetRPS - min_rps) * 100.0f / (max_rps - min_rps))];
}

const float car::bldc::Motor::get_min_rps() {
    //do an eeprom read;
    return 1;
}

const float car::bldc::Motor::get_max_rps() {
    return 0;
}

const std::array<uint8_t , 100> car::bldc::Motor::populateFeedForwardCmdListFromEEPROM() {
    return std::array<uint8_t , 100>();
}

void car::bldc::Motor::updatePreviousRPSMeasurement() {
    speedRPSprevious = speedRPS;
}

