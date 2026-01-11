#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <Adafruit_GPS.h>

#include "states.h"
#include "sensors.h"
#include "sdcard.h"
#include "flags.h"
#include "utils.h"

void initGPS();
void readGPS();
void printGPSData();
void setHasGPSFix(bool hasFix);
bool hasGPSFix();

#endif