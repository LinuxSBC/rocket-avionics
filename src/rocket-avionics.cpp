#include <Arduino.h>

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_LIS3MDL.h"
#include "Adafruit_LSM6DSOX.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GPS.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_CS_PIN 23
#define GPSSerial Serial1

#define DEBUG 0

void initSensors();
void initSDCard();
void printSensorsToFile(sensors_event_t &temp, sensors_event_t &lowg_accel, sensors_event_t &highg_accel, sensors_event_t &gyro, sensors_event_t &mag);
void error(String message, bool fatal = true);
void initGPS();

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_BMP3XX bmp; // barometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer
Adafruit_LIS3MDL lis3mdl; // magnetometer

Adafruit_GPS GPS(&GPSSerial);

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

void setup(void) {
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.setBrightness(50);
  pixel.show();

  Serial.begin(115200);
  #if DEBUG
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  delay(1000);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
  initGPS();

  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
}

void loop()
{
  // Get a new normalized sensor event
  sensors_event_t lowg_accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sensors_event_t highg_accel;
  adxl_accel.getEvent(&highg_accel);

  // TODO: Add timestamps. What time range should I set it as? 1ms seems too long.

  if (dataFile) {
    printSensorsToFile(temp, lowg_accel, highg_accel, gyro);
  }

  delay(10);

  // TODO: Find a way to close the file
  if (digitalRead(PIN_BUTTON) == LOW && dataFile) {
    Serial.println("Closing file");
    dataFile.close();
    pixel.setPixelColor(0, pixel.Color(0, 255, 255));
    pixel.show();
    while (1) {
      yield();
    }
  }
}

void setupBarometer() {
  if (!bmp.begin_I2C())
  {
    error("Failed to find BMP390 chip; no barometric altitude data", false);
  }
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ); // TODO: Maybe change to 200Hz?
}

void setupLowGAccelerometer()
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

void initSensors()
{
  dataFile.println("barometer altitude,barometer temperature,low-G accelerometer X,low-G accelerometer Y,low-G accelerometer Z,high-G accelerometer X,high-G accelerometer Y,high-G accelerometer Z,gyroscope X, gyroscope Y, gyroscope Z, magnetometer X, magnetometer Y, magnetometer Z");

  setupBarometer();

  if (!adxl_accel.begin())
  {
    error("Failed to find ADXL375 chip; no high-g accelerometer data", false);
  }

  #if DEBUG
  Serial.println("BMP390 details:");
  bmp.printSensorDetails();

  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
  #endif
}

void initSDCard()
{
  #if DEBUG
  Serial.print("Initializing SD card...");
  #endif

  // Retry mechanism for SD card initialization
  while (!SD.begin(config))
  {
    error("Initialization failed! Retrying...", false);
    delay(1000); // Wait for a second before retrying
  }
  #if DEBUG
  Serial.println("Initialization successful.");
  #endif

  dataFile = SD.open("data.csv", FILE_WRITE);
  if (!dataFile)
  {
    error("Failed to open data.csv on SD card");
  }

  if (!dataFile.seek(0))
  {
    error("Failed to seek to start of file");
  }
}

void initGPS()
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSSerial.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // Read from the GPS every 1ms
  repeating_timer_callback_t timer_callback
  add_repeating_timer_ms(1, timer_callback);
}

void printSensorsToFile(sensors_event_t &temp, sensors_event_t &lowg_accel, sensors_event_t &highg_accel, sensors_event_t &gyro, sensors_event_t &mag)
{
  dataFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); // TODO: Maybe set "sea level" to be the current pressure when powered on?
  dataFile.print(",");
  dataFile.print(bmp.readPressure());

  dataFile.print(",");
  dataFile.print(bmp.temperature);

  dataFile.print(",");
  dataFile.print(temp.temperature);

  dataFile.print(",");
  dataFile.print(lowg_accel.acceleration.x);
  dataFile.print(",");
  dataFile.print(lowg_accel.acceleration.y);
  dataFile.print(",");
  dataFile.print(lowg_accel.acceleration.z);

  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.x);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.y);
  dataFile.print(",");
  dataFile.print(highg_accel.acceleration.z);

  dataFile.print(",");
  dataFile.print(gyro.gyro.x);
  dataFile.print(",");
  dataFile.print(gyro.gyro.y);
  dataFile.print(",");
  dataFile.print(gyro.gyro.z);

  dataFile.print(",");
  dataFile.print(mag.magnetic.x);
  dataFile.print(",");
  dataFile.print(mag.magnetic.y);
  dataFile.print(",");
  dataFile.print(mag.magnetic.z);

  dataFile.println();
}

void error(String message, bool fatal) {
  Serial.println(message);
  if (fatal) {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    dataFile.close();
    while (1) {
      yield();
    }
  }
  else {
    pixel.setPixelColor(0, pixel.Color(255, 120, 0));
    pixel.show();
  }
}










/*
// GPS code

Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();


void setup()
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (DEBUG)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  Serial.print("\nTime: ");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
  }
}
*/