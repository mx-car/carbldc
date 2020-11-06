//
// Created by firat on 25.01.20.
//

#include "utils.h"
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
    Teensy32Drivers::updatePWMPinsDutyCycle(x, motor);


}
int16_t Diagnostics::calculateSensorOffset(Motor &motor,
                                   const uint16_t LUTindex) { //the index is used as a parameter to maybe plot the sensor offset all over the motor range
    uint16_t LUTSize = SVPWM::getLutSize();
    uint16_t dutyCycleW = SVPWM::getLUT()[LUTindex];
    uint16_t dutyCycleU = SVPWM::getLUT()[(LUTindex + (LUTSize / 3)) % LUTSize];
    uint16_t dutyCycleV = SVPWM::getLUT()[(LUTindex + (LUTSize / 3) * 2) % LUTSize];
    SPWMDutyCycles x{dutyCycleW, dutyCycleV, dutyCycleU};
    Teensy32Drivers::updatePWMPinsDutyCycle(x, motor);

    delay(500);

    Serial.print("\nCurrent stator flux index: ");
    Serial.println(LUTindex);

    uint16_t encoderVal = RotaryEncoderCommunication::SPITransfer(motor);
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

        uint16_t rotaryEncoderValue0 = RotaryEncoderCommunication::SPITransfer(motor);
        uint16_t rotaryEncoderValue = RotaryEncoderCommunication::SPITransfer(motor);

        uint16_t diff = abs(prev - rotaryEncoderValue);
        if (diff > 30 && diff < 16365) {
            Serial.print("d : ");
            Serial.println(diff);
            rotaryEncoderValue = rotaryEncoderValue0;
        }
        prev = rotaryEncoderValue;

        motor.updateRotaryEncoderPosition(rotaryEncoderValue);

        if (motor.isTimeForPIDControl()) { //20 times a sec
            float_t rps = VelocityCalculation::getRotationsPerSecond3(motor);
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
        Teensy32Drivers::updatePWMPinsDutyCycle(dutyCycles, motor);


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
