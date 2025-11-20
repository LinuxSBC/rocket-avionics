#ifndef ROCKET_AVIONICS_ROLL_CONTROLLER_H
#define ROCKET_AVIONICS_ROLL_CONTROLLER_H

#include "sensors.h"

// Constants
double MaxAngle = 100; // max rotation of servo
double MinAngle = 0; // min rotation of servo
double Kp = 0.4; // proportional constant
double Ki = 1; // integral constant
double Kd = 2; // derivative constant
double targetAngleRocket = 0;

#endif