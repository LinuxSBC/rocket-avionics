#ifndef ROCKET_AVIONICS_STATES_H
#define ROCKET_AVIONICS_STATES_H

enum System_State {
  STATE_STARTING,
  READY_TO_LAUNCH,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED
};

void setHasGPSFix(bool hasFix);
bool hasGPSFix();

#endif
