#include "roll-controller.h"
#include "sensors.h"
#include "flags.h"


class CalculateControlOutput
{
    public: 
        // init vars
        double control = 0.0f; // target servo angle
        float now = millis();
        double integralSum = 0;
        double lastError = 0;
        double lastUpdateTime = 0;
        double lastControlOutputValue = 0; // current servo angle


    float calculator(double airSpeed, double currentAngleRocket){
    // call in main loop
        if (abs(currentAngleRocket - targetAngleRocket) > 0.001) {// while setpoint isn't reached
            // time delta (how long since last calculation)
            double dt = now - lastUpdateTime;
            // if no time has passed, don't make any changes.
            if (dt <= 0) return lastControlOutputValue;

            // calculate the error (reference-actual)
            auto error = targetAngleRocket - currentAngleRocket;

            // calculate the integral
            integralSum += error * dt; // add to the integral history

            // calculate the derivative (rate of change, slope of line) term
            auto derivative = (error - lastError) / dt;

            // add the appropriate corrections
            control = Kp * (error + Ki*integralSum + Kd*derivative) / airSpeed;

            // clamp
            if (control > MaxAngle) control = MaxAngle;
            if (control < MinAngle) control = MinAngle;

            #if PID_TUNING
            {
                Serial.print("SP+PV+PID+O," + Serial.println(angle) + "," + Serial.println(currentAngle) + "," +
                    Serial.println(control) + "," + Serial.println(integralSum) + "," +
                    Serial.println(derivative));
            }
            #endif // PID_TUNING

            // persist our state variables
            lastControlOutputValue = control;
            lastError = error;
            lastUpdateTime = now;

        }
    }
};
