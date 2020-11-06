//
// Created by firat on 21.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_FOC_H
#define INC_1MOTOR_REFACTOR_FOC_H

#include "Motor.h"
#include "SpeedCalculation.h"
#include "RotaryEncoderCommnunication.h"
#include "SVPWM.h"
#include "PID.h"
#include "utils.h"
#include "Teensy32Drivers.h"

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
