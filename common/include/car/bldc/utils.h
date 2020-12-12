//
// Created by firat on 25.01.20.
//

#ifndef CAR_BLDC_UTILS_H
#define CAR_BLDC_UTILS_H


#include <cstdint>
#include "car/bldc/svpwm.h"
#include "car/bldc/teensy32.h"
#include "car/bldc/rotary_encoder.h"
#include "car/bldc/rotary_measurement.h"
#include "driver.h"
#include <EEPROM.h>

namespace car::bldc{

struct CommandParameters {
    int32_t angle;
    float rps;
    Direction direction;
};
class FFPIDParameterIdentification{
    static std::array<float,100> rps_list;
    static boolean populate_rps_list(Motor &m);
    static inline uint32_t EEPROM_stack_index =4 ;
    static float getMinRps(std::array<float,100> & rps_list);
    static float getMaxRps(std::array<float,100> & rps_list);
public:
    static uint8_t getFFSpeedCommand(float rps); //this should be implemented on motor basis
    static boolean constructRPStoSpeedCmdHelperList(std::array<float,100> & rps_list);
    static std::array<uint8_t ,100> rps_to_speed_cmd_helper_list;
    static void WriteFFPIDParamsToEEPROM(MotorFFPIDParameters & param);





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

    struct OptimalFluxAngles{
        uint16_t motor1FluxAngle;
        uint16_t motor2FluxAngle;
    };

    static constexpr size_t paramsListSize = 50;
    static constexpr size_t rps_list_size = 30;

private:
    static constexpr float motor_speed_scalar = -25;
    static bool firstMotorDone;
    static bool secondMotorDone;
    static uint32_t motor1FluxAnlge;
    static uint32_t  motor2FluxAnlge;
    static uint32_t angleOffsetIncrement;
    static uint32_t angle_offset;
    static std::array<FluxAngleOffsetCalibrationParams,paramsListSize> params_list;
    static std::array<float,rps_list_size> rps_list;
    static uint32_t findOptimalFluxAngle();
    static bool correctSpinningDirection(FluxAngleOffsetCalibrationParams &param);



    static void printParamsList(std::array<FluxAngleOffsetCalibrationParams,paramsListSize> & paramsList);


    static bool gatherDataForOptimalFluxAngleCalculation(Motor &m);

    static bool myComperator(FluxAngleOffsetCalibrationParams &a,FluxAngleOffsetCalibrationParams &b);
public:
    static int16_t calculateSensorOffset(Motor &motor, const uint16_t LUTindex);

    static void testMotors(Motor &x);
    static OptimalFluxAngles
    calculateOptimalFluxAngleForBothMotor(Motor &motor1, Motor &motor2, uint32_t motor1InitialFluxAngle = 0, uint32_t motor2InitialFluxAngle = 0);


    static void primitiveSpin(uint16_t LUTindex, Motor &motor);

    static void primitiveSpinMotor(Motor &motor,uint32_t delayMicroSecs = 12);

    static void speedSweep(Motor &motor);

    static FluxAngleOffsetCalibrationParams calculateParams(uint32_t angle_offset, std::array<float,rps_list_size> rps_list);

    static float calculate_variance(float mean,std::array<float,rps_list_size> rps_list );

    static void calculateAndPrintOptimalFluxAngle(Motor &m);

    static void calculateAngleFiner(Motor &motor1, Motor &motor2);

};


}

#endif //CAR_BLDC_UTILS_H
