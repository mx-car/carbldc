//
// Created by firat on 10.12.20.
//

#ifndef CAR_FIRMWARE_PIDCONTROLLER_H
#define CAR_FIRMWARE_PIDCONTROLLER_H
#include "car/bldc/motor.h"
namespace car::bldc{

class PIDController{
private:
    float P = 0.5;
    float I = 0.2;
    float D = 0.1;
    float const I_term_max = 100; // anti wind up
public:
    uint8_t calculate_speed_commmand(Motor &m) const;
};



}
#endif //CAR_FIRMWARE_PIDCONTROLLER_H
