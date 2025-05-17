#include "sensors.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer and gyro 
Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_BMP3XX bmp; // barometer

sensors_event_t lowg_accel;
sensors_event_t lsm6ds_temp;
sensors_event_t magnetometer;
sensors_event_t highg_accel;
float bmp_altitude;
float gyro_x, gyro_y, gyro_z;

void initGyroscope() {
  if (!lsm6dsox.begin_I2C())
  {
    error("Failed to find LSM6DS chip; no gyro data", false);
  }

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
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400kHz (fast mode)

  dataFile.print("millis,");
  printGPSHeader(); dataFile.print(",");
  dataFile.print("gyroscope X,gyroscope Y,gyroscope Z,");
  dataFile.print("magnetometer X,magnetometer Y,magnetometer Z,");
  dataFile.print("high-G accelerometer X,high-G accelerometer Y,high-G accelerometer Z,");
  dataFile.println("barometric pressure,barometric altitude,barometer temperature");

  initGyroscope();
  initMagnetometer();
  initHighGAccelerometer();
  initBarometer();

  #if DEBUG
  Serial.println("BMP390 details:");
  bmp.printSensorDetails();

  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
  #endif
}

void readLSM() {
  const uint8_t GYRO_START = LSM6DS_OUTX_L_G | 0x80;  // set read bit high and read at gyro registers
  const uint8_t GYRO_BYTES = 6;                       // X/Y/Z each 2 bytes

  uint8_t buf[GYRO_BYTES];

  // tell the chip which register to start at, but keep the bus alive
  Wire.beginTransmission(LSM6DS_I2CADDR_DEFAULT);
  Wire.write(GYRO_START);
  Wire.endTransmission(false); // false = send a REPEATED START, not a STOP

  // request and read 6 bytes in one go
  Wire.requestFrom(LSM6DS_I2CADDR_DEFAULT, GYRO_BYTES);
  for (uint8_t i = 0; i < GYRO_BYTES; i++) {
    buf[i] = Wire.read();
  }

  // Convert little-endian pairs into signed 16-bit values:
  int16_t raw_gyro_x = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t raw_gyro_y = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t raw_gyro_z = (int16_t)(buf[4] | (buf[5] << 8));

  float gyro_scale = 1; // range is in milli-dps per bit!
  gyro_scale = 70.0;

  gyro_x = raw_gyro_x * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyro_y = raw_gyro_y * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyro_z = raw_gyro_z * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
}

void readLIS3() {
  lis3mdl.getEvent(&magnetometer);
}

void readADXL() {
  adxl_accel.getEvent(&highg_accel);
}

void readBMP() {
  bmp.performReading();
  bmp_altitude = 44330.0 * (1.0 - pow((bmp.pressure / 100.0F) / SEALEVELPRESSURE_HPA, 0.1903));
}

void readSensors() {
  readLSM();
  readLIS3();
  readADXL();
  readBMP();
}

void printSensorsToFile() {
  dataFile.print(millis());
  dataFile.print(",");

  printGPSData();
  dataFile.print(",");

  dataFile.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    gyro_x, gyro_y, gyro_z,
    magnetometer.magnetic.x, magnetometer.magnetic.y, magnetometer.magnetic.z,
    highg_accel.acceleration.x, highg_accel.acceleration.y, highg_accel.acceleration.z,
    bmp.pressure, bmp_altitude, bmp.temperature);
  dataFile.println();
}