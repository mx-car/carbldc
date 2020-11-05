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

    /**
     * Sets a new motor power value
     * @param value 0 .. 1
     * @param motor motor id
     **/
    void setCommand(float value, uint8_t motor);
    /**
     * Returns the current motor power value command
     * @param motor motor id
     * @return motor power value
     **/
    const float& getCommand(uint8_t motor) const;
    /**
     * Returns the current motor speed measurement
     * @param motor motor id
     * @return motor speed
     **/
    const float& getSpeed(uint8_t motor) const;
    /**
     * Returns the current motor torque measurement
     * @param motor motor id
     * @return motor torque
     **/
    const float& getTorque(uint8_t motor) const;
    /**
     * Returns the timestamp to the measurement;
     * @return timestamp
     **/
    const int64_t& getTStampMeasurement() const;
    /**
     * Returns the timestamp to last command;
     * @return timestamp
     **/
    const int64_t& getTStampCommand() const;

    
private:

    float command[2];                   // command power between 0 and 1
    float speed[2];                     // speed [rps]
    float torque[2];                    // current power consumption 
    int64_t tstamp_state_update;        // timestamp  micros();   
    int64_t tstamp_command_update;      // timestamp  micros();   
};

#endif //INC_1MOTOR_REFACTOR_FOC_H
