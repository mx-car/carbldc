//
// Created by firat on 25.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_UTILS_H
#define INC_1MOTOR_REFACTOR_UTILS_H


#include <cstdint>
#include "car/bldc/svpwm.h"
#include "car/bldc/teensy32.h"
#include "car/bldc/rotary_encoder.h"
#include "car/bldc/rotary_measurement.h"
#include "driver.h"

namespace car::bldc{

struct CommandParameters {
    int32_t angle;
    float rps;
    Direction direction;
};

class DerivateFilter {
private:
    static float s;
    static float s_dot;
    static constexpr uint32_t delta_T = 1; // measurements are done in constant intervals
    static constexpr float  T_f = 5; // only thing to tune
public:

    static uint32_t getFilteredMeasurement(uint32_t measurement);

};

class SerialHelper {
    static uint16_t plot_counter;
    static CommandParameters cmd_parameters;

public:
    static void plot_16t(SPWMDutyCycles &x) {
        uint8_t arrayToSend[6];
        memcpy(arrayToSend + plot_counter, &x.inDutyCycleW, 2);
        memcpy(arrayToSend + plot_counter + 4, &x.inDutyCycleU, 2);
        memcpy(arrayToSend + plot_counter + 8, &x.inDutyCycleV, 2);
        plot_counter += 6;
        Serial.write(arrayToSend, 6);
        //Serial.println(target_rps);
        //Serial.println(my_speed);
    }

    static uint16_t parse3DigitIntFromString(uint8_t *string) {
        return (string[0] - '0') * 100 + (string[1] - '0') * 10 + (string[2] - '0');


    }

    static const CommandParameters &setVelandSteeringAngleFromSerial();


};

class Diagnostics {
public:
    struct FluxAngleOffsetCalibrationParams{
        uint32_t angle_offset;
        float variance;
        float average_rps;

    };
    static int16_t calculateSensorOffset(Motor &motor, const uint16_t LUTindex);

    static void testMotors(Motor &x);

    static void primitiveSpin(uint16_t LUTindex, Motor &motor);

    static void primitiveSpinMotor(Motor &motor,uint32_t delayMicroSecs = 12);

    static void speedSweep(Motor &motor);

    static FluxAngleOffsetCalibrationParams calculateParams(uint32_t angle_offset, float *rps, uint32_t rps_list_size);

    static float calculate_variance(float mean, float* rps_list, uint32_t rps_list_size);

    static void calculateAndPrintOptimalFluxAngle(Motor &m);
};

}

#endif //INC_1MOTOR_REFACTOR_UTILS_H
