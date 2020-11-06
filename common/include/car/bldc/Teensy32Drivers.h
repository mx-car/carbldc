//
// Created by firat on 29.08.20.
//

#ifndef INC_2MOTORNEWBOARD_TEENSY32DRIVERS_H
#define INC_2MOTORNEWBOARD_TEENSY32DRIVERS_H

#include "Arduino.h"
#include "Motor.h"
#include "SVPWM.h"
#include <ADC.h>


class Teensy32Drivers {
public:
    Teensy32Drivers() {};

    static void activateInhibitPins(Motor &x);

    static void initInhibitPins(Motor &x);

    static void updatePWMPinsDutyCycle(const SPWMDutyCycles &x, Motor &motor);

    static void initPWMPins();

    static void initADCconversions();

    static float setSpeedFromADC();

private:
    static constexpr uint8_t ADC_PIN = A1;
    static ADC adc;
    static constexpr uint32_t PWMFrequency = SVPWM_FREQUENCY; // compile flag
    static constexpr uint8_t numberOfMotors = 2; // @TODO take it as a define or discard here, you can init those pwm pins by default

};

#endif //INC_2MOTORNEWBOARD_TEENSY32DRIVERS_H
