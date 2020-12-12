//
// Created by firat on 10.12.20.
//

#ifndef CAR_FIRMWARE_PIDCONTROLLER_H
#define CAR_FIRMWARE_PIDCONTROLLER_H

#include "car/bldc/motor.h"

namespace car::bldc {

    struct MotorFFPIDParameters {
        float rps_min;
        float rps_max;
        std::array<uint8_t,100> RPStoSpeedCmdMap;
    };

    class PIDController {
    private:
        static inline float P = 0.5;
        static inline float I = 0.2;
        static inline float D = 0.1;
        static inline float const I_term_max = 100; // anti wind up
        static inline std::array<MotorFFPIDParameters,2> motorParameters;
    public:
        static uint8_t calculate_speed_commmand(Motor &m);
        static void getFFIDparametersFromEEProm();
        static uint8_t getFeedForwardPIDTerm();
    };


}
#endif //CAR_FIRMWARE_PIDCONTROLLER_H
