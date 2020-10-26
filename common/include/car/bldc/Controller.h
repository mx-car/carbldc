//
// Created by firat on 21.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_FOC_H
#define INC_1MOTOR_REFACTOR_FOC_H

#include "car/bldc/Motor.h"
#include "car/bldc/SpeedCalculation.h"
#include "car/bldc/RotaryEncoderCommnunication.h"
#include "car/bldc/SVPWM.h"
#include "car/bldc/PID.h"
#include "car/bldc/utils.h"
#include "car/bldc/Teensy32Drivers.h"

#include <car/com/mc/interface.h>

class Controller {

public:

    car::com::objects::State target;
    car::com::objects::State state;
    
    inline static  uint8_t numberOfMotors = 0;
    Motor * motors[2];
    void run();

    void registerMotors(Motor *m_ptr);
    void initHardware(uint8_t SPI_CLK);
    static Controller &getInstance();  // Singleton handler


};

#endif //INC_1MOTOR_REFACTOR_FOC_H
