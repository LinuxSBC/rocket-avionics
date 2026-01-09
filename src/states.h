#ifndef ROCKET_AVIONICS_STATES_H
#define ROCKET_AVIONICS_STATES_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "flags.h"
#if USE_GPS
#include "gps.h"
#endif
#include "sdcard.h"
#include "sensors.h"

enum System_State {
  STATE_STARTING,
  STATE_READY_TO_LAUNCH,
  STATE_ASCENT,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED
};

void handleState();
void setState(System_State state);
void runBuzzer(float secondsDuration, float secondsBetween);
void initIndicators();
void error(const String& message, bool fatal = true);

#endif
