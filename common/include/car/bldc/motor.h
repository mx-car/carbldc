//
// Created by firat on 20.01.20.
//

#ifndef CAR_BLDC_MOTOR_H
#define CAR_BLDC_MOTOR_H

#include <Arduino.h>
#include <array>

namespace car::bldc{

struct INHPins {
    const uint8_t InhibitPinU;
    const uint8_t InhibitPinV;
    const uint8_t InhibitPinW;

    constexpr INHPins(uint8_t PinU_, uint8_t PinV_, uint8_t PinW_) : InhibitPinU(PinU_), InhibitPinV(PinV_),
                                                                     InhibitPinW(PinW_) {}

};

struct PWMPins {
    const uint8_t InitPinU;
    const uint8_t InitPinV;
    const uint8_t InitPinW;

    constexpr PWMPins(uint8_t PinU_, uint8_t PinV_, uint8_t PinW_) : InitPinU(PinU_), InitPinV(PinV_),
                                                                     InitPinW(PinW_) {}

};

struct ServoPins {
    const uint8_t servoPin1;
    const uint8_t servoPin2;

    constexpr ServoPins(uint8_t servoPin1_, uint8_t servoPin2_) : servoPin1(servoPin1_), servoPin2(servoPin2_) {}
};


struct ISPins {
    const uint8_t ISPinU;
    const uint8_t ISPinV;
    const uint8_t ISPinW;

    constexpr ISPins(uint8_t ISPinU_, uint8_t ISPinV_, uint8_t ISPinW_) : ISPinU(ISPinU_), ISPinV(ISPinV_),
                                                                          ISPinW(ISPinW_) {}

};


enum Direction {
    Forward = -1, STOP, Backward
};


/**
 * Motor class - Encapsulates all motor related attributes and related functionality.
 * @TODO : improve naming
 */

class Motor {
private:
    const float min_rps;
    const float max_rps;
    const float get_min_rps();
    const float get_max_rps();
    const std::array<uint8_t ,100> feedForwardCmdList;
    const std::array<uint8_t ,100> populateFeedForwardCmdListFromEEPROM();
public:
    Motor(INHPins inhibitPins_, PWMPins initPins_, uint8_t CSPin_, ISPins IsPins_)
            : min_rps(get_min_rps()),max_rps(get_max_rps()), feedForwardCmdList(populateFeedForwardCmdListFromEEPROM()),inhibitPins(inhibitPins_), initPins(initPins_), CSPin(CSPin_), IsPins(IsPins_) {}

    const INHPins inhibitPins;
    const PWMPins initPins;
    const uint8_t CSPin;
    const ISPins IsPins;
    Direction direction = Direction::Forward;
    float speedRPS = 0;
    float speedRPSprevious = 0;
    float cumulativePIDError = 0;
    float targetRPS = 0;
    float torque = 0;
    float speedScalar = 0; // actual speed command 0.. 100
    uint16_t rotaryEncoderPosition = 0;
    uint16_t previousRotaryEncoderValue = 0; // hold the previous rotaryEncoderValue
    int16_t scaledRotaryEncoderPosition = 0; // accounts for the fieldWeakening
    int32_t encoderCumulativeValue = 0;
    int16_t angleOffset = 0;
    uint16_t PIDCounter = 0;
    int32_t leftWheel = 1;
    elapsedMicros start;
    uint8_t getFeedForwardPIDTerm();
    void updatePreviousRPSMeasurement();
    void setAsLeftWheel() {
        leftWheel = -1;
    }

    bool isTimeForPIDControl() {
        if (++PIDCounter == (SVPWM_FREQUENCY / PID_FREQUENCY)) {
            PIDCounter = 0;
            return true;
        } else {
            return false;
        }


    };


    void setAngleOffset(int16_t _angleOffset) {
        angleOffset = _angleOffset;

    }

    int32_t calculateAngleOffsetFromSpeedCommand(uint32_t speed_command) {
        if (leftWheel) {
            if (direction == Backward) {
                int32_t res = angleOffset - (100 - speed_command);
                return res;
            } else {
                return angleOffset + (speed_command);

            }
        } else {
            if (direction == Forward) {
                return angleOffset + (100 - speed_command);
            } else {
                return angleOffset - (speed_command);
            }


        }
    }


    void startCounting() {
        elapsedMicros temp;
        start = temp;
    }

    uint32_t getTimeDifference() const {
        return start;
    }

    /**
     * Raw rotary encoder position can not be used to determine the LUT index since it gives the physical position of the
     * rotor, rather then the electrical position (rotor flux position) . First step to get the index is to convert
     * physical degrees to electrical degrees: Which is done the following way :
     * ScaledRotaryEncoderPosition = RawRotaryEncoderPosition % (MAX_ROTARY_ENCODER_VALUE / POLE COUNT)
     * (MAX_ROTARY_ENCODER_VALUE / POLE COUNT) -> 16384 / 11 = 1489
     * @TODO : change hardcoded values and improve naming
     * @TODO : 1489 - (....) part way a way to get the actual rotor flux position. This works
     * with new motor but shouldnt be done here
     *
     * @param rotPos Raw 14 bit Encoder Value reading
     */
    void updateRotaryEncoderPosition(uint16_t rotPos) {
        scaledRotaryEncoderPosition = (rotPos % 1489);
        rotaryEncoderPosition = rotPos;

    }

    void updatePrevRotaryEncoderPosition() {
        previousRotaryEncoderValue = rotaryEncoderPosition;

    }

    void updateSpeedRPS(float_t rps) {
        updatePreviousRPSMeasurement();
        speedRPS = rps;
    }

    /**
     * Motor knows its desired speed value, ranging from 0 to 100.
     * This value, referred as scalar determines the duty cycle
     * @param speed - a value between 0 and 100
     */
    void updateSpeedScalar(float_t speed) {
        if(speed > 0){
            direction = Direction::Forward;
        }
        else{
            direction = Direction::Backward;
            speed *= -1.0f;
        }
        speedScalar = speed;
    };


};
}

#endif //CAR_BLDC_MOTOR_H
