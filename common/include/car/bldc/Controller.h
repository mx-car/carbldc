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

class Controller {

public:
    
    inline static  uint8_t numberOfMotors = 0;
    Motor * motors[2];
    void run();

    void registerMotors(Motor *m_ptr);
    void initHardware(uint8_t SPI_CLK);
    static Controller &getInstance();  // Singleton handler

    float command[2];                   // command power between 0 and 1
    float speed[2];                     // speed [rps]
    float torque[2];                    // current power consumption 
    int64_t tstamp_state[2];            // timestamp  micros();   
    int64_t tstamp_command[2];          // timestamp  micros();   
};

#endif //INC_1MOTOR_REFACTOR_FOC_H
