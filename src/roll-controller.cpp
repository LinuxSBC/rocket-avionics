#include "roll-controller.h"
#include "sensors.h"

float CalculateAngle(){
    double targetAngle; // desired angle
    double currentAngle; // servo position

    // formula!! 
    // angle = where we wanna go!

    if (abs(currentAngle - targetAngle) < .01){
        targetAngle = currentAngle;
        // if where we're at and where we want to go are too similar, do nothing!
    }

}
// shoudl these methods be separate?

float CalculateControlOutput(){
    // control = K_p(error + K_i*error + K_d*error)

    // init vars
    double control = 0.0f;
    auto now = millis();
    double integralSum = 0;
    double lastError = 0;
    double lastUpdateTime = 0;
    double lastControlOutputValue = 0;

while (abs(currentAngle - targetAngle) > 0.01) // while setpoint isn't reached
    // time delta (how long since last calculation)
    double dt = now - lastUpdateTime;
    // if no time has passed, don't make any changes.
    if (dt <= 0) return lastControlOutputValue;

    // calculate the error (reference-actual)
    auto error = targetAngle - currentAngle;

    // calculate the integral
    integralSum += error * dt; // add to the integral history

    // calculate the derivative (rate of change, slope of line) term
    auto derivative = (error - lastError) / dt;

    // add the appropriate corrections
    control = Kp * (error + Ki*integralSum + Kd*derivative);

    // clamp
    if (control > MaxAngle) control = MaxAngle;
    if (control < MinAngle) control = MinAngle;

    if (OutputTuningInformation)
    {
        Debug.Print("SP+PV+PID+O," + Serial.println(angle) + "," + Serial.println(currentAngle) + "," +
            Serial.println(control) + "," + Serial.println(integralSum) + "," +
            Serial.println(derivative));
    }

    // persist our state variables
    lastControlOutputValue = control;
    lastError = error;
    lastUpdateTime = now;
}

