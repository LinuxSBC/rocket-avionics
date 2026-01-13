#include "roll-controller.h"
#include "sensors.h"
#include "flags.h"
#include "utils.h"

// init vars
double integral_sum = 0;
double last_error = 0;
double last_update_time = 0;
double current_servo_angle = (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0f; // start in middle

double calculate_servo_angle(const double airspeed,
                             const double current_rocket_angle,
                             const double target_rocket_angle) {
  // call in main loop
  if (abs(current_rocket_angle - target_rocket_angle) > 0.001) {
    // while setpoint isn't reached
    unsigned long now = millis(); // TODO: Should probably add in micros somehow for more precision.
    double target_servo_angle = 0.0f;

    // time delta (how long since last calculation)
    const double dt = now - last_update_time;
    // if no time has passed, don't make any changes.
    if (dt <= 0)
      return current_servo_angle;

    // calculate the error (reference-actual)
    auto error = target_rocket_angle - current_rocket_angle;

    // calculate the integral
    integral_sum += error * dt; // add to the integral history

    // calculate the derivative (rate of change, slope of line) term
    auto derivative = (error - last_error) / dt;

    // add the appropriate corrections
    target_servo_angle = Kp * (error + Ki * integral_sum + Kd * derivative) / airspeed;

    // clamp
    target_servo_angle = clamp(target_servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    // TODO: Should probably log that it can't make the changes it wants

#if PID_TUNING
    Serial.print("SP+PV+PID+O," +
                 String(target_servo_angle) + "," +
                 String(current_servo_angle) + "," +
                 String(target_servo_angle) + "," +
                 String(integral_sum) + "," +
                 String(derivative));
#endif // PID_TUNING

    // persist our state variables
    current_servo_angle = target_servo_angle;
    last_error = error;
    last_update_time = now;

    return target_servo_angle;
  }
  return current_servo_angle;
}
