#include "sensors.h"

#define GPSSerial Serial1

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer
Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_BMP3XX bmp; // barometer
Adafruit_GPS GPS(&GPSSerial);

void initLowGAccelerometer()
{
  if (!lsm6dsox.begin_I2C())
  {
    error("Failed to find LSM6DS chip; no low-g accelerometer data", false);
  }

  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
}

void initMagnetometer()
{
  if (!lis3mdl.begin_I2C())
  {
    error("Failed to find LIS3MDL chip; no magnetometer data", false);
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
}

void initHighGAccelerometer()
{
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
  dataFile.print("GPS hour,GPS minute,GPS seconds,GPS milliseconds,GPS fix,GPS fix quality,GPS latitude,GPS longitude,GPS speed (knots),GPS angle,GPS altitude,GPS satellites,GPS antenna,");
  
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
  Serial.println("BMP390 details:");
  bmp.printSensorDetails();

  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
  #endif
}

void initGPS() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSSerial.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  wait(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void readGPS() {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #if DEBUG
    if (c) Serial.print(c);
  #endif
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    if (GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      setState(STATE_READY);
    } else {
      setState(STATE_NO_GPS);
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  if (GPS.fix) {
    setState(STATE_READY);
  } else {
    setState(STATE_NO_GPS);
  }
}

void printSensorsToFile() {
  dataFile.print(millis());
  dataFile.print(",");

  readGPS();

  dataFile.print(GPS.hour);
  dataFile.print(",");
  dataFile.print(GPS.minute);
  dataFile.print(",");
  dataFile.print(GPS.seconds);
  dataFile.print(",");
  dataFile.print(GPS.milliseconds);
  dataFile.print(",");
  dataFile.print(GPS.fix);
  dataFile.print(",");
  dataFile.print(GPS.fixquality);
  dataFile.print(",");
  dataFile.print(GPS.latitude, 4);
  dataFile.print(GPS.lat);
  dataFile.print(",");
  dataFile.print(GPS.longitude, 4);
  dataFile.print(GPS.lon);
  dataFile.print(",");
  dataFile.print(GPS.speed);
  dataFile.print(",");
  dataFile.print(GPS.angle);
  dataFile.print(",");
  dataFile.print(GPS.altitude);
  dataFile.print(",");
  dataFile.print(GPS.satellites);
  dataFile.print(",");
  dataFile.print(GPS.antenna);
  dataFile.print(",");
  
  sensors_event_t lowg_accel;
  sensors_event_t gyro;
  sensors_event_t lsm6ds_temp;
  lsm6dsox.getEvent(&lowg_accel, &gyro, &lsm6ds_temp);

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

  sensors_event_t magnetometer;
  lis3mdl.getEvent(&magnetometer);
  dataFile.print(magnetometer.magnetic.x);
  dataFile.print(",");
  dataFile.print(magnetometer.magnetic.y);
  dataFile.print(",");
  dataFile.print(magnetometer.magnetic.z);
  dataFile.print(",");

  sensors_event_t highg_accel;
  adxl_accel.getEvent(&highg_accel);
  dataFile.print(highg_accel.acceleration.x);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.y);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.z);
  dataFile.print(",");
  
  dataFile.print(bmp.readPressure());
  dataFile.print(",");
  dataFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  dataFile.print(",");
  dataFile.print(bmp.readTemperature());

  dataFile.println();
}