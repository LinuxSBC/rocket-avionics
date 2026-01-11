#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
#include <BMP3XX.h>

// #include "rocket-avionics.h"
#include "flags.h"
#if USE_GPS
#include "gps.h"
#endif
#include "sdcard.h"
#include "states.h"

void initSensors();

void initLowGAccelerometer();
void initMagnetometer();
void initHighGAccelerometer();
void initBarometer();

void readLSM();
void readLIS3();
void readADXL();
void readBMP();
void readSensors();

bool hasLaunched();
#endif