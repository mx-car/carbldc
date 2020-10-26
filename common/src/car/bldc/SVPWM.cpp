//
// Created by firat on 21.01.20.
//

#include "car/bldc/SVPWM.h"

//ModulationIndexScalingParams SVPWM::modulationIndexParams = LUTGenerator::calculateModulationIndexScalingOffsetParameters();
/***
 * This function does the actual work and consists of 4 steps:
 * 1- Calculates modulationIndexOffset : There is an offset for different modulation indices. This is calculate by
 *  the speedScalar (0 - 100)
 * 2 - Calculate field weakening : To achieve higher speeds field weakening - a small shift on the LUT index is required.
 *  This can be skipped entirely and is just an optimisation. For the old motors, instead of a fix field weakening,
 *  a speed scalar based index shift produces better results. Field weakening seems to be slightly different for different
 *  directions, hence * x.direction - 20
 *
 * 3- Base index calculation : By combining
 *      - scaledRotaryEncoderPosition - Corresponds to rotor flux angle @TODO : automate calculating this
 *      - fieldWeakening : Explained above, an optimisation
 *      - angleOffset : +90 -90 degrees (1/4 of the entire LUT range) to produces torque
 *  the LUT index that gives the optimal PWM Duty cycle value is calculated.  "+ LUTSize) % LUTSize " is required since
 *  adding the angleOffset can produce a negative index.
 *
 * 4 - Calculation other phase's indices : By simply adding 120 ° and 240° (LUTSize/3 and LUTSize*2/3) other phase
 * indices are acquired
 *
 * 5 - Scaling down the duty cycles for lower modulation indices : This is done by multiplying full duty cycle first
 * and then adding the modulationIndexOffset.
 *
 * @param x - motor object
 * @return SVPWM duty cycles for each phase of a motor
 */
SPWMDutyCycles SVPWM::calculateDutyCycles(Motor &x){
    SPWMDutyCycles temp;

    uint16_t modulationIndexOffset =  scaleDutyCyclesToModulationIndex(x.speedScalar);
    //int8_t fieldWeakening = -x.speedScalar; // gives the best results
    //uint16_t base = (x.scaledRotaryEncoderPosition + angleOffset * x.direction + LUTSize) % LUTSize;
    //x.setAngleOffset(-60 + x.speedScalar * x.direction)
    int32_t localOffset = x.calculateAngleOffsetFromSpeedCommand(x.speedScalar);
    uint16_t base = (x.scaledRotaryEncoderPosition + (angleOffset + x.angleOffset ) * x.direction +  LUTSize) % LUTSize;
    //uint16_t base = (x.scaledRotaryEncoderPosition + ((fieldWeakening + 600+ angleOffset) * x.direction - 20) + LUTSize) % LUTSize;
    /*
     * This part is tricky; there is a field-weakening and the best results has been found at -120 and + 80
     * To avoid an else-if check the fieldWeakening is set to 100 and with this -20 it is set to -120 or 80 according to the direction of the motor
     * It is very peculiar that the implicit Sensor Offet in the scaledRotaryEncoderPosition is different in each direction, one direction has a 40 degrees offset compared to the other
     * This issue clearly shows itself with the max speed difference if left unattended. With the current fieldWeakening parameter the max speed is around 13.50 in each direction
     *
     *
     * */
    uint16_t LUTIndexW = base;
    uint16_t LUTIndexV = (base + (LUTSize / 3) ) % LUTSize;
    uint16_t LUTIndexU = (base + (2 * (LUTSize / 3)) ) % LUTSize;
    float intermediateMultiplier = x.speedScalar * 0.01f;
    temp.inDutyCycleW = static_cast<uint16_t >(LUT[LUTIndexW]* intermediateMultiplier) + modulationIndexOffset;
    temp.inDutyCycleU = static_cast<uint16_t >(LUT[LUTIndexV]* intermediateMultiplier) + modulationIndexOffset;
    temp.inDutyCycleV = static_cast<uint16_t >(LUT[LUTIndexU]* intermediateMultiplier) + modulationIndexOffset;

    return temp;

};
uint16_t SVPWM::scaleDutyCyclesToModulationIndex(float scalar){
    return static_cast<uint16_t >(modulationIndexParams.offsetParam_m*scalar + modulationIndexParams.offsetParam_c);
}

