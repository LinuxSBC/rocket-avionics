#include <Arduino.h>

#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_NeoPixel.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_CS_PIN 23

#define DEBUG 0

void initSensors();
void initSDCard();
void printSensorsToFile(sensors_event_t &temp, sensors_event_t &lowg_accel, sensors_event_t &highg_accel, sensors_event_t &gyro);
void error(String message, bool fatal = true);

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

Adafruit_ICM20649 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345);

Adafruit_BMP3XX bmp;

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

void setup(void) {
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
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

  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
}

void loop()
{
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

void initSensors()
{
  dataFile.println("barometer altitude,barometer temperature,icm temperature,low-G accelerometer X,low-G accelerometer Y,low-G accelerometer Z,high-G accelerometer X,high-G accelerometer Y,high-G accelerometer Z,gyroscope X, gyroscope Y, gyroscope Z");

  if (!bmp.begin_I2C())
  {
    error("Failed to find BMP390 chip; no barometric altitude data", false);
  }
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ); // TODO: Maybe change to 200Hz?

  if (!icm.begin_I2C())
  {
    error("Failed to find ICM20649 chip; no gyro data", false);
  }

  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

  icm_temp = icm.getTemperatureSensor();
  icm_accel = icm.getAccelerometerSensor();
  icm_gyro = icm.getGyroSensor();

  if (!adxl_accel.begin())
  {
    error("Failed to find ADXL375 chip; no high-g accelerometer data", false);
  }

  #if DEBUG
  Serial.println("BMP390 details:");
  bmp.printSensorDetails();

  Serial.println("ICM20649 details:");
  icm_temp->printSensorDetails();
  icm_accel->printSensorDetails();
  icm_gyro->printSensorDetails();

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

void printSensorsToFile(sensors_event_t &temp, sensors_event_t &lowg_accel, sensors_event_t &highg_accel, sensors_event_t &gyro)
{
  dataFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); // TODO: Maybe set "sea level" to be the current pressure when powered on?

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
