#include <Arduino.h>

#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_ICM20649 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345);

Adafruit_BMP3XX bmp;

bool error = false;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 chip; no gyro data");
    error = true;
  }
  if (!adxl_accel.begin()) {
    Serial.println("Failed to find ADXL375 chip; no high-g accelerometer data");
    error = true;
  }
  if (!bmp.begin_I2C()) {
    Serial.println("Failed to find BMP390 chip; no barometric altitude data");
    error = true;
  }

  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);

  Serial.println("ICM20649 details:");
  icm_temp = icm.getTemperatureSensor();
  icm_temp->printSensorDetails();

  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();

  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // TODO: Maybe change to 200Hz?
}

void loop() {
  // Get a new normalized sensor event
  sensors_event_t lowg_accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm_temp->getEvent(&temp);
  icm_accel->getEvent(&lowg_accel);
  icm_gyro->getEvent(&gyro);

  sensors_event_t highg_accel;
  adxl_accel.getEvent(&highg_accel);

  // TODO: Add timestamps. What time range should I set it as? 1ms seems too long.

  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); // TODO: Maybe set "sea level" to be the current pressure when powered on?
  Serial.print(",");

  Serial.print(bmp.temperature);
  Serial.print(",");

  Serial.print(temp.temperature);
  Serial.print(",");

  Serial.print(lowg_accel.acceleration.x);
  Serial.print(","); Serial.print(lowg_accel.acceleration.y);
  Serial.print(","); Serial.print(lowg_accel.acceleration.z);
  Serial.print(",");

  Serial.print(highg_accel.acceleration.x);
  Serial.print(","); Serial.print(highg_accel.acceleration.y);
  Serial.print(","); Serial.print(highg_accel.acceleration.z);
  Serial.print(",");

  Serial.print(gyro.gyro.x);
  Serial.print(","); Serial.print(gyro.gyro.y);
  Serial.print(","); Serial.print(gyro.gyro.z);
  Serial.println();


  delay(10);
}
