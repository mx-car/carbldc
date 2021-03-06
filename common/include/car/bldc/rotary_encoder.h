//
// Created by firat on 21.01.20.
//

#ifndef CAR_BLDC_ROTARYENCODER_H
#define CAR_BLDC_ROTARYENCODER_H

#include <Arduino.h>
#include <SPI.h>
#include "car/bldc/motor.h"


namespace car::bldc{

class RotaryEncoder{
private:

    static const SPISettings spiSettings; //i


public:
    static uint16_t SPITransfer(const Motor &x);
    static void initSPI(uint8_t SPI_CLK);
    static void initMotorCSPins(const Motor &x);

};
}
#endif //CAR_BLDC_ROTARYENCODER_H
