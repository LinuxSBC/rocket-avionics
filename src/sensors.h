#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>

#ifndef SENSORS_H
#define SENSORS_H

#include "rocket-avionics.h"
#include "gps.h"

void initSensors();
void printSensorsToFile();

void initLowGAccelerometer();
void initMagnetometer();
void initHighGAccelerometer();
void initBarometer();

void readLSM();
void readLIS3();
void readADXL();
void readBMP();
void readSensors();
#endif