//
// Created by firat on 10.12.20.
//
#include <EEPROM.h>
#include "car/bldc/PIDController.h"
using namespace car::bldc;

uint8_t PIDController::calculate_speed_commmand(Motor &m)  {
    uint8_t feedForwardTerm = m.getFeedForwardPIDTerm();
    float error = m.currentRPS - m.targetRPS;
    float rps_measurement_diff = m.currentRPS - m.previousRPS;
    if(m.cumulativePIDError <= I_term_max){
        m.cumulativePIDError += error;
    }
    float PIDTerm = error * P + m.cumulativePIDError * I - rps_measurement_diff * D;

    return static_cast<uint8_t>(feedForwardTerm + PIDTerm);

}

void PIDController::getFFIDparametersFromEEProm() {
    uint8_t idx = 4;
    EEPROM.get(idx,motorParameters[0]);
     idx += sizeof(MotorFFPIDParameters);
    EEPROM.get(idx,motorParameters[1]);


}

