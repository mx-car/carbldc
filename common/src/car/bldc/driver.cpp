//
// Created by firat on 21.01.20.
//

#include "car/bldc/driver.h"

using namespace car::bldc;
/**
 * Singleton
 * @TODO : check Meyer's Singleton
 * @return
 */
Driver &Driver::getInstance() {
    static Driver instance;
    return instance;


}

/**
 * Initializes all low lvl hardware related stuff.
 *  1- PWM Pins
 *  2- SPI
 *  3- ADC
 *  4- InitPins
 *  5- CS Pins for SPI
 *
 *  Called only once, at the begining, after registering the Motors
 * @param SPI_CLK
 */
void Driver::initHardware(uint8_t SPI_CLK) {

    Teensy32::initPWMPins();
    RotaryEncoder::initSPI(SPI_CLK);
    //initADCconversions();
    while(!getRotorFluxAngleOffsetFromEEPROM());

    for (int i = 0; i < numberOfMotors; ++i) {
        Teensy32::initInhibitPins(*motors[i]);
        Teensy32::activateInhibitPins(*motors[i]);
        RotaryEncoder::initMotorCSPins(*motors[i]);
    }
}


    void Driver::setCommand(float value, uint8_t motor){
        command[motor] = value;
        tstamp_command_update = micros();
    }
    const float& Driver::getCommand(uint8_t motor) const{
        return command[motor];
    }
    const float& Driver::getSpeed(uint8_t motor) const{
        return speed[motor];

    }
    const float& Driver::getTorque(uint8_t motor) const{
        return torque[motor];
    }
    const int64_t& Driver::getTStampMeasurement() const{
        return tstamp_state_update;
    }
    const int64_t& Driver::getTStampCommand() const{
        return tstamp_state_update;
    }


/***
 * The main function that compiles all the functionality. It is executed every interrupt cycle (20 kHz).
 * The procedure is as follows:
 * 1- Read the rotary encoder value
 * 2- Update the motors related attributes
 * 3- Every 0.1 sec:
 *      - Calculate the velocity
 *      - get a target_speed form somewhere ( 0..100)
 *      - feed that to PID and get the speed_command
 * 4- Calculate duty cycles (based on rotary encoder position and speed command)
 * 5- Feed that PWM registers
 * @return
 */
FASTRUN void Driver::run() {


    for (int i = 0; i < 2 /* numberOfMotors */ ; ++i) {

        uint16_t rotaryEncoderValue0 = RotaryEncoder::SPITransfer(*motors[i]);
        motors[i]->updateRotaryEncoderPosition(rotaryEncoderValue0);

        if (motors[i]->isTimeForPIDControl()) { //every 0.25 sec
            speed[i] = RotaryMeasurement::getRotationsPerSecondWithTimeDifference(*motors[i]);
            motors[i]->updateSpeedRPS(speed[i]);
            motors[i]->updateSpeedScalar(command[i]);

            tstamp_state_update = micros();
        }

        SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(*motors[i]);
        Teensy32::updatePWMPinsDutyCycle(dutyCycles, *motors[i]);
    }
}


/**
 * Driver class has a list of motor pointers, this function adds motor objects to this array.
 * and increases motor count.
 * @param m_ptr
 */

void Driver::registerMotors(Motor *m_ptr) {
    motors[numberOfMotors++] = m_ptr;


}

bool Driver::getRotorFluxAngleOffsetFromEEPROM() {
    if(EEPROM.read(0) == 255 && EEPROM.read(3) ){
        Serial.println("EEPROM Values not set!!! Check read.me and run the Diagnostics configuration....");
        return false;
    }
    else{
        Diagnostics::OptimalFluxAngles x;
        EEPROM.get(0,x);
        motors[0]->setAngleOffset(x.motor1FluxAngle);
        motors[1]->setAngleOffset(x.motor2FluxAngle);
        Serial.print("Motor0 Flux Angle: ");
        Serial.println(x.motor1FluxAngle);
        Serial.print("Motor1 Flux Angle: ");
        Serial.println(x.motor2FluxAngle);
        return true;

    }
}




