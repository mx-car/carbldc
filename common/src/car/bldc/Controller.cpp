//
// Created by firat on 21.01.20.
//

#include "car/bldc/Controller.h"

/**
 * Singleton
 * @TODO : check Meyer's Singleton
 * @return
 */
Controller &Controller::getInstance() {
    static Controller instance;
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
void Controller::initHardware(uint8_t SPI_CLK) {

    Teensy32Drivers::initPWMPins();
    RotaryEncoderCommunication::initSPI(SPI_CLK);
    //initADCconversions();


    for (int i = 0; i < numberOfMotors; ++i) {
        Teensy32Drivers::initInhibitPins(*motors[i]);
        Teensy32Drivers::activateInhibitPins(*motors[i]);
        RotaryEncoderCommunication::initMotorCSPins(*motors[i]);
    }


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
FASTRUN void Controller::run() {


    for (int i = 0; i < 2 /* numberOfMotors */ ; ++i) {

        uint16_t rotaryEncoderValue0 = RotaryEncoderCommunication::SPITransfer(*motors[i]);
        motors[i]->updateRotaryEncoderPosition(rotaryEncoderValue0);

        if (motors[i]->isTimeForPIDControl()) { //every 0.25 sec

            if(i == 0) {
                if(command[i] > 0) motors[i]->direction = CCW;
                else  motors[i]->direction = CW;
            } else {
                if(command[i] > 0) motors[i]->direction = CW;
                else  motors[i]->direction = CCW;
            }
            speed[i] = VelocityCalculation::getRotationsPerSecond3(*motors[i]);
            motors[i]->updateSpeedRPS(speed[i]);

            //float speed_command = SpeedPIDController::getSpeedCommand(*motors[i], 30);
            motors[i]->updateSpeedScalar(fabs(command[i]));
            tstamp_state[i] = micros();   

            if(fabs(command[i]) < 0.001){
                Teensy32Drivers::deactivateInhibitPins(*motors[i]);
            } else {
                Teensy32Drivers::activateInhibitPins(*motors[i]);
            }
        }

        SPWMDutyCycles dutyCycles = SVPWM::calculateDutyCycles(*motors[i]);
        Teensy32Drivers::updatePWMPinsDutyCycle(dutyCycles, *motors[i]);
    }


}


/**
 * Controller class has a list of motor pointers, this function adds motor objects to this array.
 * and increases motor count.
 * @param m_ptr
 */

void Controller::registerMotors(Motor *m_ptr) {
    motors[numberOfMotors++] = m_ptr;


}




