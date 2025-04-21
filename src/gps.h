#include <Arduino.h>
#include <Adafruit_GPS.h>

#ifndef GPS_H
#define GPS_H

#include "rocket-avionics.h"
#include "sensors.h"

void initGPS();
void readGPS();
void printGPSHeader();
void printGPSData();

#endif