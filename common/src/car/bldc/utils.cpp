//
// Created by firat on 25.01.20.
//

#include <algorithm>
#include "car/bldc/utils.h"


using namespace car::bldc;

uint16_t SerialHelper::plot_counter=0;
float DerivateFilter::s = 0;
float DerivateFilter::s_dot = 0;



uint32_t DerivateFilter::getFilteredMeasurement(uint32_t measurement){
    float s_temp = s + s_dot * delta_T; // new value, old value + difference (s_dot * delta_T)
    float s_dot_temp = s_dot + (-s / (T_f * T_f) - 2 * s_dot / T_f + measurement) * delta_T;
    s = s_temp;
    s_dot = s_dot_temp;
    return static_cast<uint32_t>(s / (T_f * T_f));


}

const CommandParameters & SerialHelper::setVelandSteeringAngleFromSerial() {
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

void Diagnostics::speedSweep(Motor & motor) {
    static uint16_t prev = 0;
    static uint32_t speed_increase_counter = 0;
    static float speed_command = 10.f;
    static float speed_cumulative_value = 0.f;
    static const uint32_t values_to_add_up = 40;

    for (int i = 1; i < 2 /* numberOfMotors */ ; ++i) {

        uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(motor);
        uint16_t rotaryEncoderValue = RotaryEncoder::SPITransfer(motor);

        uint16_t diff = abs(prev - rotaryEncoderValue);
        if (diff > 30 && diff < 16365) {
            Serial.print("d : ");
            Serial.println(diff);
            rotaryEncoderValue = rotaryEncoderValue0;
        }
        prev = rotaryEncoderValue;

        motor.updateRotaryEncoderPosition(rotaryEncoderValue);

        if (motor.isTimeForPIDControl()) { //20 times a sec
            float_t rps = RotaryMeasurement::getRotationsPerSecond3(motor);
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
                speed_cumulative_value = 0.0f;


            } else if (speed_increase_counter == (values_to_add_up + 5)) {
                speed_increase_counter = 0;
            }
            if (speed_command > 100) {
                speed_command = 10.0f;
            }
            speed_increase_counter++;


        }

        SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(motor);
        Teensy32::updatePWMPinsDutyCycle(dutyCycles, motor);


    }
}

void Diagnostics::primitiveSpinMotor(Motor &motor, uint32_t delayMicroSecs) {
      while(1){
      for (int i = 0; i < 1489; ++i) {
          Diagnostics::primitiveSpin(i,motor);
          delayMicroseconds(delayMicroSecs);

      }
  }


}

Diagnostics::FluxAngleOffsetCalibrationParams
Diagnostics::calculateParams(uint32_t angle_offset,std::array<float,rps_list_size> rps_list ) {
    Diagnostics::FluxAngleOffsetCalibrationParams params{angle_offset,0,0};
    float sum = 0;
    for(float rps: rps_list){
        sum+=rps;
    }
    params.average_rps = sum/rps_list_size;
    params.variance = calculate_variance(params.average_rps,rps_list);
    return params;
}

float Diagnostics::calculate_variance(float mean,std::array<float,rps_list_size> rps_list ) {
    float sum_of_squared_diffs = 0;
    for (uint32_t i = 0; i < rps_list_size; ++i) {
        sum_of_squared_diffs += sq(rps_list[i] - mean);
    }
    return sum_of_squared_diffs/rps_list_size;
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
        float rps = RotaryMeasurement::getRotationsPerSecond3(m);
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
    return a.average_rps > b.average_rps;
}

uint32_t
Diagnostics::findOptimalFluxAngle() {
    std::sort(params_list.begin(),params_list.end(),Diagnostics::myComperator);
    return params_list.front().angle_offset;
}

void Diagnostics::printParamsList(std::array<FluxAngleOffsetCalibrationParams, paramsListSize> &paramsList) {

    for (const auto &params: paramsList) {
        Serial.print(" average rps: ");
        Serial.print(params.average_rps, 5);
        Serial.print(" - variance: ");
        Serial.print(params.variance, 5);
        Serial.print(" - angle offet: ");
        Serial.println(params.angle_offset);

    }
}

void Diagnostics::calculateOptimalFluxAngleForBothMotor(Motor &motor1, Motor &motor2) {
    static bool firstMotorDone = false;
    static bool secondMotorDone = false;
    if(!firstMotorDone){
        firstMotorDone = gatherDataForOptimalFluxAngleCalculation(motor1);
        if(firstMotorDone){
            uint32_t fluxAngle = findOptimalFluxAngle();
            params_list.fill(FluxAngleOffsetCalibrationParams{0,0,0});
            rps_list.fill(0);
            Serial.println(fluxAngle);
        }
    }
    else{
        secondMotorDone = gatherDataForOptimalFluxAngleCalculation(motor1);
        if(secondMotorDone){
            uint32_t fluxAngle = findOptimalFluxAngle();
            params_list.fill(FluxAngleOffsetCalibrationParams{0,0,0});
            rps_list.fill(0);
            Serial.println(fluxAngle);
    }

}

bool Diagnostics::gatherDataForOptimalFluxAngleCalculation(Motor &m) {
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
        float rps = RotaryMeasurement::getRotationsPerSecond3(m);
        rps_list[rps_ctr++] = rps;
    }
    if (rps_ctr == rps_list_size) {
        rps_ctr = 0;
        angle_offset += 2;
        angle_offset = (angle_offset % 1489);
        m.setAngleOffset(angle_offset);
        params_list[param_ctr++] = calculateParams(angle_offset, rps_list);
        if (param_ctr == params_list.size()) {
            finito = true;
            m.updateSpeedScalar(10);
        }
    }

    SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(m);
    Teensy32::updatePWMPinsDutyCycle(dutyCycles, m);
    return finito;
}







