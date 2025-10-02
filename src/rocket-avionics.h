#ifndef ROCKET_AVIONICS_H
#define ROCKET_AVIONICS_H

#include <Arduino.h>

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_NeoPixel.h>

#include "sensors.h"
#include "flags.h"
#if USE_GPS
#include "gps.h"
#endif
#include "states.h"

extern File32 dataFile;

void handleState();
void setState(System_State state);
void error(const String& message, bool fatal = true);
void runBuzzer(float secondsDuration, float secondsBetween);
void runBuzzer(float sequence[], int length);
void wait(int milliseconds);

void initSDCard();
#endif