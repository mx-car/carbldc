//
// Created by firat on 10.12.20.
//
#include "car/bldc/PIDController.h"
using namespace car::bldc;

uint8_t PIDController::calculate_speed_commmand(Motor &m) const {
    uint8_t feedForwardTerm = m.getFeedForwardPIDTerm();
    float error = m.speedRPS - m.targetRPS;
    float rps_measurement_diff = m.speedRPS - m.speedRPSprevious;
    if(m.cumulativePIDError <= I_term_max){
        m.cumulativePIDError += error;
    }
    float PIDTerm = error * P + m.cumulativePIDError * I - rps_measurement_diff * D;

    return static_cast<uint8_t>(feedForwardTerm + PIDTerm);

}
