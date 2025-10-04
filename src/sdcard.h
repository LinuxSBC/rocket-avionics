#ifndef ROCKET_AVIONICS_SD_CARD_H
#define ROCKET_AVIONICS_SD_CARD_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#include "flags.h"

extern File32 dataFile; // TODO: Figure out an interface for this

void initSDCard();
void ejectSDCard();

enum Sensor {
  HIGH_G_ACCELEROMETER = 0x00,
  IMU = 0x01,
  MAGNETOMETER = 0x02,
  BAROMETER = 0x03,
  GPS_SENSOR = 0x04,
  PROCESSED_DATA = 0x05,
  OTHER_SENSOR = 0x06,
};

enum SensorData { // Intended to be generic, so "ACCELEROMETER_X" could be high-g or low-g
  // Raw movement
  ACCELEROMETER_X = 0x00,
  ACCELEROMETER_Y = 0x01,
  ACCELEROMETER_Z = 0x02,
  MAGNETOMETER_X = 0x03,
  MAGNETOMETER_Y = 0x04,
  MAGNETOMETER_Z = 0x05,
  GYROSCOPE_X = 0x06,
  GYROSCOPE_Y = 0x07,
  GYROSCOPE_Z = 0x08,

  // Air
  PRESSURE = 0x10,
  TEMPERATURE = 0x11,

  // Time
  HOUR = 0x20,
  MINUTE = 0x21,
  SECOND = 0x22,
  MILLISECOND = 0x23,

  // GPS
  GPS_FIX = 0x30,
  GPS_QUALITY = 0x31,
  SATELLITES = 0x32,

  // Rocket state
  ROCKET_STATE = 0x40,
  BATTERY_VOLTAGE = 0x41,
  SENSORS_DETECTED = 0x42,
  SENSOR_DATA_AVAILABLE = 0x43,

  // Angle
  HEADING = 0x50,
  QUATERNION_1 = 0x51,
  QUATERNION_I = 0x52,
  QUATERNION_J = 0x53,
  QUATERNION_K = 0x54,

  // Location and ground-frame data
  LATITUDE = 0x60,
  LONGITUDE = 0x61,
  SPEED = 0x62,
  ACCELERATION_X = 0x63,
  ACCELERATION_Y = 0x64,
  ACCELERATION_Z = 0x65,
  VELOCITY_X = 0x66,
  VELOCITY_Y = 0x67,
  VELOCITY_Z = 0x68,
  POSITION_X = 0x69,
  POSITION_Y = 0x6A,
  POSITION_Z = 0x6B,
};

struct DataLine {
  unsigned long timestamp_micros;
  Sensor sensor;
  SensorData data;
  float value;
};

#endif