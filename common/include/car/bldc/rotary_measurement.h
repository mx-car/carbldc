//
// Created by firat on 21.01.20.
//

#ifndef CAR_BLDC_MEASUREMENT_H
#define CAR_BLDC_MEASUREMENT_H

#include "car/bldc/motor.h"

namespace car::bldc{
/**
 * RotaryMeasurement class - The rotary encoder readings are taken a fixed time interval - every duty cycle -
 * This class encapsulates attributes and functions to calculate RPS, RPM and meters per sec
 */

class RotaryMeasurement {
    static constexpr float wheelDiameter{4.12f};
    static constexpr float wheelCircumference = wheelDiameter * M_PI * 2;
    static constexpr float measurementsPerSecond = PID_FREQUENCY; // same as PID, compile flag
    static constexpr float rotaryEncoderMaxValue = ENCODER_RESOLUTION;


public:

    static float getRotationsPerMinute(Motor &motor) {
        return getRotationsPerSecond3(motor) * 60;
    }

    static float getRotationsPerSecond3(Motor &m) {
        int32_t diff = m.rotaryEncoderPosition - m.previousRotaryEncoderValue;

        if (m.previousRotaryEncoderValue < 5000 && m.rotaryEncoderPosition > 10000) { // OVERFLOW ROT POS INCREASING
            diff -= ENCODER_RESOLUTION;

        }


        if (m.rotaryEncoderPosition < 5000 && m.previousRotaryEncoderValue > 10000) { // OVERFLOW ROT POS DECREASING
            diff += ENCODER_RESOLUTION;


        }
        float_t retVal = static_cast<float>(diff) / ENCODER_RESOLUTION * PID_FREQUENCY;
        m.previousRotaryEncoderValue = m.rotaryEncoderPosition;

        return retVal * m.leftWheel;
    }


    static float getRotationsPerSecond4(Motor &motor) {
        int32_t timeDiffInMicroSeconds = motor.getTimeDifference();
        int32_t diff = (motor.rotaryEncoderPosition - motor.previousRotaryEncoderValue)  *  static_cast<int>(motor.direction) * motor.leftWheel ;
        int16_t and_val = 16383; // binary magic to get rid of overflow
        diff &= and_val;
        float_t rps = ((static_cast<float>(diff) / timeDiffInMicroSeconds) / ENCODER_RESOLUTION) * 1000000;
        motor.updatePrevRotaryEncoderPosition();
        motor.startCounting();
        return rps;
    }

    static float getMetersPerSecond(Motor &motor) {
        return getRotationsPerSecond3(motor) * wheelCircumference;
    }

};
}
#endif //CAR_BLDC_MEASUREMENT_H
