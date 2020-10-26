//
// Created by firat on 20.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_MOTOR_H
#define INC_1MOTOR_REFACTOR_MOTOR_H

#include <Arduino.h>
#include <array>


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
    CW = -1, STOP, CCW
};


/**
 * Motor class - Encapsulates all motor related attributes and related functionality.
 * @TODO : improve naming
 */

class Motor {
public:
    Motor(INHPins inhibitPins_, PWMPins initPins_, uint8_t CSPin_, ISPins IsPins_)
            : inhibitPins(inhibitPins_), initPins(initPins_), CSPin(CSPin_), IsPins(IsPins_) {}

    const INHPins inhibitPins;
    const PWMPins initPins;
    const uint8_t CSPin;
    const ISPins IsPins;
    Direction direction = Direction::CW;
    float speedRPS = 0;
    float torque = 0;
    float speedScalar = 0; // actual speed command 0.. 100
    uint16_t rotaryEncoderPosition = 0;
    uint16_t previousRotaryEncoderValue = 0; // hold the previous rotaryEncoderValue
    int16_t scaledRotaryEncoderPosition = 0; // accounts for the fieldWeakening
    int32_t encoderCumulativeValue = 0;
    int16_t angleOffset = 0;
    uint16_t PIDCounter = 0;
    bool rightWheel = false;

    void setAsRightWheel() {
        rightWheel = true;
    }

    bool isTimeForPIDControl(){
        if(++PIDCounter == (SVPWM_FREQUENCY/PID_FREQUENCY)){
            PIDCounter = 0;
            return true;
        }
        else{
            return false;
        }


    };
    /**
     * This function is supposed to set the sensor offset at initialization after calculating the sensor offset.
     * @param sensorOffset_ - is the -supposedly- fixed offset between raw rotary encoder readings and
     * the rotor flux angle.
     * @TODO: Not being used. Find an automated way to calculate the offset and use this at start up.
     * @TODO: change naming, field weakening is something else
     */

    void setEncoderCumulativeValueToZero() {
        //Serial.println(encoderCumulativeValue);
        encoderCumulativeValue = 0;
    }

    void setAngleOffset(int16_t _angleOffset) {
        angleOffset = _angleOffset;

    }

    int32_t calculateAngleOffsetFromSpeedCommand(uint32_t speed_command) {
        if (rightWheel) {
            if (direction == CCW) {
                int32_t res = angleOffset - (100 - speed_command);
                return res;
            } else {
                return angleOffset + (speed_command);

            }
        } else {
            if (direction == CW) {
                return angleOffset + (100 - speed_command);
            } else {
                return angleOffset - (speed_command);
            }


        }
    }

    /**
     * This function cumulatively adds the difference between rotary encoder position readings to calculate RPM later on.
     * The encoder values overflow after 16384 so this is taken care of.
     * @TODO : might behave weirdly at abrupt direction changes, needs more testing
     * @param rotPos : raw 14 bit rotary encoder reading from SPI
     */
    void cumulativeAdd(uint16_t rotPos) {

        uint16_t diff = abs(rotPos - previousRotaryEncoderValue);
        if (diff > 16200) {
            diff = ENCODER_RESOLUTION - diff;
        }

        previousRotaryEncoderValue = rotPos;
        encoderCumulativeValue += diff;


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
        scaledRotaryEncoderPosition = 1489 - (rotPos % 1489);
        rotaryEncoderPosition = rotPos;

    }

    void updatePrevRotaryEncoderPosition(uint16_t rotPos) {
        previousRotaryEncoderValue = rotPos;

    }

    void updateSpeedRPS(float_t rps) {
        speedRPS = rps;
    }

    /**
     * Motor knows its desired speed value, ranging from 0 to 100.
     * This value, referred as scalar determines the duty cycle
     * @param speed - a value between 0 and 100
     */
    void updateSpeedScalar(float_t speed) {
        speedScalar = speed;
    };


};


#endif //INC_1MOTOR_REFACTOR_MOTOR_H
