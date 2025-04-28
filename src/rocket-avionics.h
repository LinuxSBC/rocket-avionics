#include <Arduino.h>

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_NeoPixel.h>

#ifndef ROCKET_AVIONICS_H
#define ROCKET_AVIONICS_H

#include "sensors.h"
#include "gps.h"

enum System_State {
  STATE_STARTING,
  STATE_READY,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED,
  STATE_NO_GPS
};

extern File32 dataFile;

void notifyState();
void setState(System_State state);
void error(String message, bool fatal = true);
void runBuzzer(float secondsDuration, float secondsBetween);
void runBuzzer(float sequence[], int length);
void wait(int milliseconds);

void initSDCard();
#endif