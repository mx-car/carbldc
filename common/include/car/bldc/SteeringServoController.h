//
// Created by firat on 21.01.20.
//

#ifndef INC_1MOTOR_REFACTOR_STEERINGSERVOCONTROLLER_H
#define INC_1MOTOR_REFACTOR_STEERINGSERVOCONTROLLER_H

#include "car/bldc/Motor.h"
#include <Servo.h>
class SteeringServoController {

    static Servo myservo;  // create servo object to control a servo
public:
    static void controlSteeringServo(int32_t angle){
        myservo.write(angle);

    }
    static void attachServoPins(ServoPins servoPins){
        myservo.attach(servoPins.servoPin1);
        //myservo.attach(servoPins.servoPin2);
    }



};


#endif //INC_1MOTOR_REFACTOR_STEERINGSERVOCONTROLLER_H
