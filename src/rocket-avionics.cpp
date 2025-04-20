#include <Arduino.h>

#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_LIS3MDL.h"
#include "Adafruit_LSM6DSOX.h"
#include <Adafruit_ADXL375.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define SD_CS_PIN 23
#define GPSSerial Serial1
#define BUZZER_PIN 24

#define DEBUG 0

void initSensors();
void initSDCard();
void initGPS();
void readGPS();
void printSensorsToFile();

void notifyState();
void error(String message, bool fatal = true);
void runBuzzer(float secondsDuration, float secondsBetween);
void runBuzzer(float sequence[], int length);
void wait(int milliseconds);

void setupLowGAccelerometer();
void setupMagnetometer();
void setupHighGAccelerometer();

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer
Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_GPS GPS(&GPSSerial);

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

unsigned long lastTimeBuzzerChanged = 0;
bool buzzerOn = false;

enum System_State {
  STATE_STARTING,
  STATE_READY,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED,
  STATE_NO_GPS
};
System_State systemState = STATE_STARTING;

void setup() {
  pixel.begin();
  pixel.setBrightness(50);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  systemState = STATE_STARTING;
  notifyState();
  
  Serial.begin(115200);
  #if DEBUG
  while (!Serial)
    notifyState();
  #endif

  wait(1000);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
  initGPS();

  systemState = STATE_READY;
  notifyState();
}

void loop() {
  sensors_event_t lowg_accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sensors_event_t highg_accel;
  adxl_accel.getEvent(&highg_accel);

  if (dataFile) {
    printSensorsToFile();
  }
  
  notifyState();

  // TODO: Also close file on full SD card and low battery
  if (digitalRead(PIN_BUTTON) == LOW && dataFile) {
    dataFile.close();
    systemState = STATE_FILE_CLOSED;
    while (1) {
      notifyState();
    }
  }
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

void setupMagnetometer()
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

void setupHighGAccelerometer()
{
  if (!adxl_accel.begin())
  {
    error("Failed to find ADXL375 chip; no high-g accelerometer data", false);
  }

  adxl_accel.setDataRate(ADXL343_DATARATE_800_HZ);
}

void initSensors() {
  dataFile.print("millis,");
  dataFile.print("GPS hour,GPS minute,GPS seconds,GPS milliseconds,GPS fix,GPS fix quality,GPS latitude,GPS longitude,GPS speed (knots),GPS angle,GPS altitude,GPS satellites,GPS antenna,");
  
  dataFile.print("low-G accelerometer X,low-G accelerometer Y,low-G accelerometer Z,");
  dataFile.print("gyroscope X,gyroscope Y,gyroscope Z,");
  dataFile.print("gyro temp,");
  dataFile.print("magnetometer X,magnetometer Y,magnetometer Z,");
  dataFile.println("high-G accelerometer X,high-G accelerometer Y,high-G accelerometer Z");

  setupLowGAccelerometer();
  setupMagnetometer();
  setupHighGAccelerometer();

  #if DEBUG
  // Serial.println("BMP390 details:");
  // bmp.printSensorDetails();

  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
  #endif
}

void initSDCard() {
  #if DEBUG
  Serial.print("Initializing SD card...");
  #endif

  // Retry mechanism for SD card initialization
  while (!SD.begin(config))
  {
    error("Initialization failed! Retrying...", false);
    wait(1000); // Wait for a second before retrying
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
      systemState = STATE_READY;
    } else {
      systemState = STATE_NO_GPS;
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  if (GPS.fix) {
    systemState = STATE_READY;
  } else {
    systemState = STATE_NO_GPS;
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

  dataFile.println();
}

void error(String message, bool fatal) {
  Serial.println(message);
  if (fatal) {
    systemState = STATE_ERROR;
    notifyState();
    dataFile.close();
    while (1) {
      yield();
    }
  }
  else {
    systemState = STATE_WARNING;
  }
}

void notifyState() {
  switch (systemState) {
    case STATE_READY: {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      runBuzzer(0.2, 16);
      break;
    }
    case STATE_NO_GPS: {
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      runBuzzer(0.2, 8);
      break;
    }
    case STATE_FILE_CLOSED: {
      pixel.setPixelColor(0, pixel.Color(0, 255, 255));
      runBuzzer(0.2, 4);
      break;
    }
    case STATE_STARTING: {
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      runBuzzer(0.2, 2);
      break;
    }
    case STATE_WARNING: {
      pixel.setPixelColor(0, pixel.Color(255, 120, 0));
      runBuzzer(0.1, 0.5);
      break;
    }
    case STATE_ERROR: {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      runBuzzer(0.1, 0.1);
      break;
    }
  }
  pixel.show();
}

/*
 * Function to run the buzzer for a specified duration, every specified
 * number of seconds.
 * @param duration Duration in seconds
 * @param secondsBetween Seconds between each beep (between the last beep ended and this one starts)
 */
void runBuzzer(float secondsDuration, float secondsBetween) {
  if (!buzzerOn && millis() - lastTimeBuzzerChanged > secondsBetween * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
  } else if (buzzerOn && millis() - lastTimeBuzzerChanged > secondsDuration * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
  }
}

void wait(int milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    notifyState();
  }
}