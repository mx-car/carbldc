//
// Created by firat on 21.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_SVPWM_H
#define INC_1MOTOR_REFACTOR_SVPWM_H


#include "car/bldc/Motor.h"
#include "car/bldc/SVPWM_LUT.h"


struct SPWMDutyCycles{
    uint16_t inDutyCycleW;
    uint16_t inDutyCycleV;
    uint16_t inDutyCycleU;

};

/***
 * Space Vector Pulse Width Modulation(SVPWM) class -
 * All attributes are initialized at compile time.
 * Responsible for fetching correct value from the LUT
 */
class SVPWM {
private:
    static constexpr uint16_t LUTSize = LUTGenerator::LUTSize;
    static constexpr uint16_t direction_offset = LUTSize / 4; // @TODO calculate -90 + 90, plot for variations
    static constexpr auto LUT = LUTGenerator::generate();
    static constexpr ModulationIndexScalingParams modulationIndexParams = LUTGenerator::calculateModulationIndexScalingOffsetParameters();
    static uint16_t scaleDutyCyclesToModulationIndex(float scalar);
public:

    static SPWMDutyCycles calculateDutyCycles(Motor &x);
    static const auto & getLUT(){ return LUT; }
    static const uint16_t getLutSize() {return LUTSize;}

};
#endif //INC_1MOTOR_REFACTOR_SVPWM_H
