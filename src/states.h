#ifndef ROCKET_AVIONICS_STATES_H
#define ROCKET_AVIONICS_STATES_H

#include <Arduino.h>

#include "flags.h"
#include "utils.h"
#if USE_GPS
#include "orientation/gps.h"
#endif

void handleState();
void setState(SystemState state);
void runBuzzer(float secondsDuration, float secondsBetween);
void initIndicators();
void error(const String& message, bool fatal = true);
void wait(int milliseconds);

#endif
