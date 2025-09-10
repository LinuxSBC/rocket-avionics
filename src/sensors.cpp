#include "sensors.h"
#include "debug.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer and gyro 
Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_BMP3XX bmp; // barometer

sensors_event_t lowg_accel;
sensors_event_t gyro;
sensors_event_t lsm6ds_temp;
sensors_event_t magnetometer;
sensors_event_t highg_accel;
float bmp_altitude;

void initLowGAccelerometer() {
  if (!lsm6dsox.begin_I2C())
  {
    error("Failed to find LSM6DS chip; no low-g accelerometer data", false);
  }

  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
}

void initMagnetometer() {
  if (!lis3mdl.begin_I2C())
  {
    error("Failed to find LIS3MDL chip; no magnetometer data", false);
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
}

void initHighGAccelerometer() {
  if (!adxl_accel.begin())
  {
    error("Failed to find ADXL375 chip; no high-g accelerometer data", false);
  }

  adxl_accel.setDataRate(ADXL343_DATARATE_800_HZ);
}

void initBarometer() {
  if (!bmp.begin_I2C())
  {
    error("Failed to find BMP390 chip; no barometric altitude data", false);
  }
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}

void initSensors() {
  dataFile.print("millis,");
  printGPSHeader(); dataFile.print(",");
  dataFile.print("low-G accelerometer X,low-G accelerometer Y,low-G accelerometer Z,");
  dataFile.print("gyroscope X,gyroscope Y,gyroscope Z,");
  dataFile.print("gyro temp,");
  dataFile.print("magnetometer X,magnetometer Y,magnetometer Z,");
  dataFile.print("high-G accelerometer X,high-G accelerometer Y,high-G accelerometer Z,");
  dataFile.println("barometric pressure,barometric altitude,barometer temperature");

  initLowGAccelerometer();
  initMagnetometer();
  initHighGAccelerometer();
  initBarometer();

  #if DEBUG
  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
  #endif
}

void readLSM() {
  lsm6dsox.getEvent(&lowg_accel, &gyro, &lsm6ds_temp);
}

void readLIS3() {
  lis3mdl.getEvent(&magnetometer);
}

void readADXL() {
  adxl_accel.getEvent(&highg_accel);
}

void readBMP() {
  bmp.performReading();
  float atmospheric = bmp.pressure / 100.0F;
  bmp_altitude = 44330.0 * (1.0 - pow((bmp.pressure / 100.0F) / SEALEVELPRESSURE_HPA, 0.1903));
}

void readSensors() {
  readLSM();
  readLIS3();
  readADXL();
  readBMP();
}

void printSensorsToFile() {
  #if DEBUG
  Serial.printf("Low-G Accel: %.2f X, %.2f Y, %.2f Z\n", lowg_accel.acceleration.x, lowg_accel.acceleration.y, lowg_accel.acceleration.z);
  Serial.printf("Gyro: %.2f X, %.2f Y, %.2f Z, temp: %.2f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, lsm6ds_temp.temperature);
  Serial.printf("Mag: %.2f X, %.2f Y, %.2f Z\n", magnetometer.magnetic.x, magnetometer.magnetic.y, magnetometer.magnetic.z);
  Serial.printf("High-G Accel: %.2f X, %.2f Y, %.2f Z\n", highg_accel.acceleration.x, highg_accel.acceleration.y, highg_accel.acceleration.z);
  Serial.printf("BMP: %.2f Pa, %.2f m, %.2f C\n", bmp.pressure, bmp_altitude, bmp.temperature);
  #endif

  dataFile.print(millis());
  dataFile.print(",");

  printGPSData();
  dataFile.print(",");

  dataFile.print(lowg_accel.acceleration.x);
  dataFile.print(",");
  dataFile.print(lowg_accel.acceleration.y);
  dataFile.print(",");
  dataFile.print(lowg_accel.acceleration.z);
  dataFile.print(",");
  dataFile.print(gyro.gyro.x);
  dataFile.print(",");
  dataFile.print(gyro.gyro.y);
  dataFile.print(",");
  dataFile.print(gyro.gyro.z);
  dataFile.print(",");
  dataFile.print(lsm6ds_temp.temperature);
  dataFile.print(",");

  dataFile.print(magnetometer.magnetic.x);
  dataFile.print(",");
  dataFile.print(magnetometer.magnetic.y);
  dataFile.print(",");
  dataFile.print(magnetometer.magnetic.z);
  dataFile.print(",");

  dataFile.print(highg_accel.acceleration.x);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.y);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.z);
  dataFile.print(",");
  
  dataFile.print(bmp.pressure);
  dataFile.print(",");
  dataFile.print(bmp_altitude);
  dataFile.print(",");
  dataFile.print(bmp.temperature);

  dataFile.println();
}