//
// Created by firat on 25.01.20.
//

#include <algorithm>
#include "car/bldc/utils.h"


using namespace car::bldc;

uint16_t SerialHelper::plot_counter = 0;
float DerivateFilter::s = 0;
float DerivateFilter::s_dot = 0;
uint32_t Diagnostics::motor1FluxAnlge = 0;
uint32_t Diagnostics::motor2FluxAnlge = 0;
uint32_t Diagnostics::angleOffsetIncrement = 30;
uint32_t Diagnostics::angle_offset = 0;
bool Diagnostics::firstMotorDone = false;
bool Diagnostics::secondMotorDone = false;
std::array<Diagnostics::FluxAngleOffsetCalibrationParams, Diagnostics::paramsListSize> Diagnostics::params_list;
std::array<float, Diagnostics::rps_list_size> Diagnostics::rps_list;
std::array<float, 100> FFPIDParameterIdentification::rps_list;
std::array<uint8_t , 100> FFPIDParameterIdentification::rps_to_speed_cmd_helper_list;


uint32_t DerivateFilter::getFilteredMeasurement(uint32_t measurement) {
    float s_temp = s + s_dot * delta_T; // new value, old value + difference (s_dot * delta_T)
    float s_dot_temp = s_dot + (-s / (T_f * T_f) - 2 * s_dot / T_f + measurement) * delta_T;
    s = s_temp;
    s_dot = s_dot_temp;
    return static_cast<uint32_t>(s / (T_f * T_f));


}

const CommandParameters &SerialHelper::setVelandSteeringAngleFromSerial() {
    int index = 0;
    byte transmit_buffer[14];
    while (Serial.available()) {
        transmit_buffer[index] = Serial.read();  // will not be -1
        index++;

    }
    if (index > 6) { // 7 bytes are transmitted per command
        if (transmit_buffer[0] == 'B') {
            cmd_parameters.direction = Direction::Backward;
        } else if (transmit_buffer[0] == 'F') {
            cmd_parameters.direction = Direction::Forward;

        } else {
            cmd_parameters.direction = Direction::STOP;
        }
        uint16_t parsedInt = parse3DigitIntFromString(transmit_buffer + 1);
        Serial.println(parsedInt);
        cmd_parameters.rps = static_cast<float>(parsedInt) / 100.f;
        parsedInt = parse3DigitIntFromString(transmit_buffer + 4);
        cmd_parameters.angle = parsedInt;

    }
    return cmd_parameters;
}

/*
 * @TODO : add a serial interface to send data for debugging
 * @TODO : -> plotting
 * @TODO : -> simple send -> wrap em up in a class
 *
 *
 *
 *
 *
 * */

/**
 * Initial step of testing any motor.
 * Simply creates a wave form for the motor to follow. No sensor involved. If the Pins and motor pole count is correct
 * the motor should spin.
 * - This function should be called in a loop with incrementing LUT indexes.
 * - A delay between function calls is necessary (10 micro seconds seems to be okay for any motor)
 * @param LUTindex
 * @param motor
 */
void Diagnostics::primitiveSpin(uint16_t LUTindex, Motor &motor) {
    // 10 microsec delays seems like ideal
    uint16_t LUTSize = SVPWM::getLutSize();
    uint16_t dutyCycleW = SVPWM::getLUT()[LUTindex];
    uint16_t dutyCycleV = SVPWM::getLUT()[(LUTindex + (LUTSize / 3)) % LUTSize];
    uint16_t dutyCycleU = SVPWM::getLUT()[(LUTindex + (LUTSize / 3) * 2) % LUTSize];
    SPWMDutyCycles x{dutyCycleW, dutyCycleV, dutyCycleU};
    Teensy32::updatePWMPinsDutyCycle(x, motor);


}

int16_t Diagnostics::calculateSensorOffset(Motor &motor,
                                           const uint16_t LUTindex) { //the index is used as a parameter to maybe plot the sensor offset all over the motor range
    uint16_t LUTSize = SVPWM::getLutSize();
    uint16_t dutyCycleW = SVPWM::getLUT()[LUTindex];
    uint16_t dutyCycleU = SVPWM::getLUT()[(LUTindex + (LUTSize / 3)) % LUTSize];
    uint16_t dutyCycleV = SVPWM::getLUT()[(LUTindex + (LUTSize / 3) * 2) % LUTSize];
    SPWMDutyCycles x{dutyCycleW, dutyCycleV, dutyCycleU};
    Teensy32::updatePWMPinsDutyCycle(x, motor);

    delay(500);

    Serial.print("\nCurrent stator flux index: ");
    Serial.println(LUTindex);

    uint16_t encoderVal = RotaryEncoder::SPITransfer(motor);
    uint16_t encoderValScaled = encoderVal % LUTSize;
    Serial.print(" Modulo scaled encoder reading: ");
    Serial.println(encoderValScaled);

    uint16_t expectedRotorFlux = (LUTindex + LUTSize / 2) % LUTSize; // + 180
    Serial.print("Expected Rotor Flux position: ");
    Serial.println(expectedRotorFlux);

    int16_t sensorOffset = expectedRotorFlux - encoderValScaled;
    Serial.print("Difference between calculated rotor flux and expected rotor flux ");
    Serial.println((1489 - encoderValScaled) - expectedRotorFlux);
    return sensorOffset;
    /**
     *We want to find the SensorOffset because:
     * 1- We want to measure/acquire the flux vector position of permanent magnets (a.k.a. Rotor Flux) because:
     *      - We want to create a flux vector (a.k.a Stator flux) perpendicular to it so we can generate the max. torque.
     *      - In order to do so, we need to know the Rotor flux position which is constant in magnitude and rotates with the motor.
     *      - For example, if we know Rotor flux is at 100°, we can create our Stator flux at 10° or at 190° degrees depending on the desired direction of rotation
     *      - When we get a rotary encoder reading, this tells us the physical position of the rotor and has no info about the position of the rotor flux vector
     *      - Which means there is an offset between the physical position of the motor and the Rotor Flux. We want to calculate this offset.
     * 2- We calculate it in the following way:
     *      - We energize the motor (create a stator flux at an arbitrary position) - for example 100°
     *      - Motor locks itself to a position depending on the stator flux vector.
     *      - Since the motor has locked itself  we know the rotor flux has aligned itself at the opposite direction, i.e. stator flux position + 180°
     *      - Stator flux position was taken 100° so the rotor flux position is at 280°
     *      - We take a rotary encoder value which tells us something between 0 and 360. This is just a physical reading with no reference to anything.
     *      - But we know the expected rotor flux position which is 280°
     *      - We calculate the difference between measured physical position and expected rotor flux position.
     *      - We have our offset which should be added to /substracted from our actual rotary encoder reading to get the rotor flux vector position at any given time.
     *
     * CONCLUSION: At the end, substracting what we have mesaured from the the max sensor reading gave us the rotor flux position ( 360 - measured_rot_pos). The method described above doesnt give us a constant offset
     * because the stator flux position increases as the rotor flux position decreases.
     * stator flux = 10 -> rotor flux = 350 (360-10)
     * stator flux = 11 -> rotor flux = 349 (360-11)
     * stator flux = 12 -> rotor flux = 348 (360-12)
     * stator flux = 13 -> rotor flux = 347 (360-13)
     * stator flux = 14 -> rotor flux = 346 (360-14)
     *
     *
    **/


}

void Diagnostics::testMotors(Motor( &x)) {

    for (int j = 0; j < 1489; j += 20) { calculateSensorOffset(x, j); }


}

void Diagnostics::speedSweep(Motor &motor) {
    static uint32_t speed_increase_counter = 0;
    static float speed_command = 12.f;
    static float speed_cumulative_value = 0.f;
    static const uint32_t values_to_add_up = 40;


        uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(motor);

        motor.updateRotaryEncoderPosition(rotaryEncoderValue0);

        if (motor.isTimeForPIDControl()) { //20 times a sec
            float_t rps = RotaryMeasurement::getRotationsPerSecondWithTimeDifference(motor);
            motor.updateSpeedRPS(rps);
            motor.updateSpeedScalar(speed_command);
            if (speed_increase_counter < values_to_add_up) {
                speed_cumulative_value += rps;
            } else if (speed_increase_counter == values_to_add_up) {
                float average = speed_cumulative_value / values_to_add_up;
                Serial.print("Command : ");
                Serial.print(speed_command);
                Serial.print("  RPS : ");
                Serial.println(average);
                speed_command += 1.0f;
                motor.updateSpeedScalar(speed_command);
                speed_cumulative_value = 0.0f;


            } else if (speed_increase_counter == (values_to_add_up + 5)) {
                speed_increase_counter = 0;
            }
            if (speed_command > 100) {
                speed_command = 10.0f;
                motor.updateSpeedScalar(speed_command);
            }
            speed_increase_counter++;


        }

        SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(motor);
        Teensy32::updatePWMPinsDutyCycle(dutyCycles, motor);



}

void Diagnostics::primitiveSpinMotor(Motor &motor, uint32_t delayMicroSecs) {
    while (1) {
        for (int i = 0; i < 1489; ++i) {
            Diagnostics::primitiveSpin(i, motor);
            delayMicroseconds(delayMicroSecs);

        }
    }


}

Diagnostics::FluxAngleOffsetCalibrationParams
Diagnostics::calculateParams(uint32_t angle_offset, std::array<float, rps_list_size> rps_list) {
    Diagnostics::FluxAngleOffsetCalibrationParams params{angle_offset, 0, 0};
    float sum = 0;
    for (float rps: rps_list) {
        sum += rps;
    }
    params.average_rps = sum / rps_list_size;
    params.variance = calculate_variance(params.average_rps, rps_list);
    return params;
}

float Diagnostics::calculate_variance(float mean, std::array<float, rps_list_size> rps_list) {
    float sum_of_squared_diffs = 0;
    for (uint32_t i = 0; i < rps_list_size; ++i) {
        sum_of_squared_diffs += sq(rps_list[i] - mean);
    }
    return sum_of_squared_diffs / rps_list_size;
}

void Diagnostics::calculateAndPrintOptimalFluxAngle(Motor &m) {
    static uint32_t rps_ctr = 0;
    static uint32_t param_ctr = 0;
    static uint32_t angle_offset = 1200;
    static bool finito = false;
    uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(m);
    m.updateRotaryEncoderPosition(rotaryEncoderValue0);
    if (!finito) {
        m.updateSpeedScalar(motor_speed_scalar);
    }

    if (m.isTimeForPIDControl()) { //every 0.25 sec
        float rps = RotaryMeasurement::getRotationsPerSecondPeriodic(m);
        rps_list[rps_ctr++] = rps;
    }
    if (rps_ctr == rps_list_size) {
        rps_ctr = 0;
        angle_offset += 2;
        angle_offset = (angle_offset % 1489);
        m.setAngleOffset(angle_offset);
        params_list[param_ctr++] = calculateParams(angle_offset, rps_list);
        if (param_ctr == params_list.size()) {
            printParamsList(params_list);
            finito = true;
            m.updateSpeedScalar(10);
        }
    }

    SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(m);
    Teensy32::updatePWMPinsDutyCycle(dutyCycles, m);

}

bool Diagnostics::myComperator(Diagnostics::FluxAngleOffsetCalibrationParams &a,
                               Diagnostics::FluxAngleOffsetCalibrationParams &b) {
    if(a.variance > 0.003){
        return false;
    }
    if (fabsf(a.average_rps) < 0.5) {
        return false;
    }
    if (!correctSpinningDirection(a) && correctSpinningDirection(b)) {
        return false;
    } else if (correctSpinningDirection(a) && !correctSpinningDirection(b)) {
        return true;
    }
    float a_rps = fabsf(a.average_rps);
    float b_rps = fabsf(b.average_rps);
    if(a.variance < b.variance){ a_rps += 0.1;}
    else { b_rps += 0.3;}
    return a_rps  > b_rps;

}

uint32_t
Diagnostics::findOptimalFluxAngle() {
    std::sort(params_list.begin(), params_list.end(), Diagnostics::myComperator);
    return params_list.front().angle_offset;
}

void Diagnostics::printParamsList(std::array<FluxAngleOffsetCalibrationParams, paramsListSize> &paramsList) {

    for (const auto &params: paramsList) {
        Serial.print(" Speed scalar: ");
        Serial.print(motor_speed_scalar, 5);
        Serial.print(" average rps: ");
        Serial.print(params.average_rps, 5);
        Serial.print(" - variance: ");
        Serial.print(params.variance, 5);
        Serial.print(" - angle offet: ");
        Serial.println(params.angle_offset);

    }
}

Diagnostics::OptimalFluxAngles
Diagnostics::calculateOptimalFluxAngleForBothMotor(Motor &motor1, Motor &motor2, uint32_t motor1InitialFluxAngle,
                                                   uint32_t motor2InitialFluxAngle) {
    OptimalFluxAngles crude{10000, 10000};
    static bool initial = true;
    if (initial) {
        angle_offset = motor1InitialFluxAngle;
        initial = false;
    }
    if (!firstMotorDone) {
        firstMotorDone = gatherDataForOptimalFluxAngleCalculation(motor1);
        if (firstMotorDone) {
            motor1FluxAnlge = findOptimalFluxAngle();
            printParamsList(params_list);
            params_list.fill(FluxAngleOffsetCalibrationParams{0, 0, 0});
            rps_list.fill(0);
            Serial.println(motor1FluxAnlge);
            angle_offset = motor2InitialFluxAngle;
            crude.motor1FluxAngle = motor1FluxAnlge;
            firstMotorDone = true;

        }
    } else if (firstMotorDone && !secondMotorDone) {
        secondMotorDone = gatherDataForOptimalFluxAngleCalculation(motor2);
        if (secondMotorDone) {
            motor2FluxAnlge = findOptimalFluxAngle();
            printParamsList(params_list);
            params_list.fill(FluxAngleOffsetCalibrationParams{0, 0, 0});
            rps_list.fill(0);
            Serial.println(motor2FluxAnlge);
            crude.motor2FluxAngle = motor2FluxAnlge;
            crude.motor1FluxAngle = motor1FluxAnlge;
            initial = true;
            firstMotorDone = false;
            secondMotorDone = false;

        }

    }
    return crude;
}

bool Diagnostics::gatherDataForOptimalFluxAngleCalculation(Motor &m) {
    static uint32_t rps_ctr = 0;
    static uint32_t param_ctr = 0;
    static bool finito = false;
    finito = false;
    uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(m);
    m.updateRotaryEncoderPosition(rotaryEncoderValue0);
    if (!finito) {
        m.updateSpeedScalar(motor_speed_scalar);
    }

    if (m.isTimeForPIDControl()) { //every 0.25 sec
        float rps = RotaryMeasurement::getRotationsPerSecondWithTimeDifference(m);
        rps_list[rps_ctr++] = rps;
    }
    if (rps_ctr == rps_list_size) {
        rps_ctr = 0;
        angle_offset += angleOffsetIncrement;
        angle_offset = (angle_offset % 1489);
        m.setAngleOffset(angle_offset);
        params_list[param_ctr++] = calculateParams(angle_offset, rps_list);
        if (param_ctr == params_list.size()) {
            finito = true;
            m.updateSpeedScalar(10);
            rps_ctr = 0;
            param_ctr = 0;
            angle_offset = 0;

        }
    }

    SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(m);
    Teensy32::updatePWMPinsDutyCycle(dutyCycles, m);
    return finito;
}

bool Diagnostics::correctSpinningDirection(Diagnostics::FluxAngleOffsetCalibrationParams &param) {
    return param.average_rps * motor_speed_scalar >= 0;
}

void Diagnostics::calculateAngleFiner(Motor &motor1, Motor &motor2) {
    static bool finito = false;
    if (finito) {
        return;
    }
    static OptimalFluxAngles crude{10000, 10000};
    static OptimalFluxAngles fineTuned{10000, 10000};

    if (crude.motor2FluxAngle == 10000) {
        crude = calculateOptimalFluxAngleForBothMotor(motor1, motor2);
    } else {
            angleOffsetIncrement = 2;
            uint32_t newMotor1FluxAngleStartingValue =
                    crude.motor1FluxAngle - angleOffsetIncrement * (paramsListSize / 2);
            uint32_t newMotor2FluxAngleStartingValue =
                    crude.motor2FluxAngle - angleOffsetIncrement * (paramsListSize / 2);
            fineTuned = calculateOptimalFluxAngleForBothMotor(motor1, motor2,
                                                              newMotor1FluxAngleStartingValue,
                                                              newMotor2FluxAngleStartingValue);
            if(fineTuned.motor2FluxAngle != 10000){
                finito = true;
                EEPROM.put(0,fineTuned); // EEPROM +4, next starts from 4

            }

    }


    }



boolean FFPIDParameterIdentification::populate_rps_list(Motor &motor) {
    static uint32_t speed_increase_counter = 0;
    static float speed_command = 12.f;
    static float speed_cumulative_value = 0.f;
    static const uint32_t values_to_add_up = 40;


    uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(motor);
    motor.updateRotaryEncoderPosition(rotaryEncoderValue0);

    if (motor.isTimeForPIDControl()) { //20 times a sec
        float_t rps = RotaryMeasurement::getRotationsPerSecondWithTimeDifference(motor);
        if (speed_increase_counter < values_to_add_up) { // continue adding up
            speed_cumulative_value += rps;
        } else if (speed_increase_counter == values_to_add_up) { // collected enough samples, increase speed command
            float average = speed_cumulative_value / values_to_add_up;
            rps_list[static_cast<size_t>(speed_command)] = average;
            speed_command += 1.0f;
            motor.updateSpeedScalar(speed_command);
            speed_cumulative_value = 0.0f;


        } else if (speed_increase_counter == (values_to_add_up + 5)) { // This to ensure cleaner operation, wait 5 more cycles before starting adding values up
            speed_increase_counter = 0;
        }
        if (speed_command > 100) { // done , sweep complete
            speed_command = 10.0f;
            motor.updateSpeedScalar(speed_command);
            return true;
        }
        speed_increase_counter++;


    }

    SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(motor);
    Teensy32::updatePWMPinsDutyCycle(dutyCycles, motor);
    return false; // not done yet



}

boolean FFPIDParameterIdentification::constructRPStoSpeedCmdHelperList(std::array<float,100> rps_list) {
    float max_rps_value = 9.51;
    float min_rps_value = 1.1; // get from the list
    float granularity = (max_rps_value - min_rps_value) / rps_list.size(); // size
    float starting_rps = min_rps_value;
    std::array<float,100>::iterator lower;
    for(auto & idx: rps_to_speed_cmd_helper_list){
        lower = std::lower_bound(rps_list.begin(), rps_list.end(), starting_rps);
        idx = (lower - rps_list.begin() + 1);
        starting_rps += granularity;
        Serial.println(idx);

    }
    return true;
}

uint8_t FFPIDParameterIdentification::getFFSpeedCommand(float rps) {
    return rps_to_speed_cmd_helper_list[int( (rps -1.1) * 100.0 / (9.51 - 1.1))]; // get max min from rps_list
}

void FFPIDParameterIdentification::writeRPStoCMDListToEEPROM() {
    EEPROM.put(EEPROM_stack_index,rps_to_speed_cmd_helper_list);
    EEPROM_stack_index+=rps_to_speed_cmd_helper_list.size();
}

