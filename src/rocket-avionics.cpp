#include <Arduino.h>

#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GPS.h>

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

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

Adafruit_GPS GPS(&GPSSerial);

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

unsigned long lastTimeBuzzerChanged = millis();
bool buzzerOn = false;

enum System_State {
  STATE_STARTING,
  STATE_READY,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED
};
System_State systemState = STATE_STARTING;

void setup() {
  pixel.begin();
  pixel.setBrightness(50);
  systemState = STATE_STARTING;
  notifyState();
  
  Serial.begin(115200);
  #if DEBUG
  while (!Serial)
    wait(10); // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  wait(1000);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
  initGPS();

  systemState = STATE_READY;
  notifyState();
}

void loop() {
  if (dataFile) {
    printSensorsToFile();
  }
  
  notifyState();

  // TODO: Find a way to close the file
  if (digitalRead(PIN_BUTTON) == LOW && dataFile) {
    Serial.println("Closing file");
    dataFile.close();
    systemState = STATE_FILE_CLOSED;
    notifyState();
    while (1) {
      yield();
    }
  }
}

void initSensors() {
  dataFile.println("GPS hour,GPS minute,GPS seconds,GPS milliseconds,GPS fix,GPS fix quality,GPS latitude,GPS longitude,GPS speed (knots),GPS angle,GPS altitude,GPS satellites,GPS antenna");
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
      systemState = STATE_WARNING;
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  if (GPS.fix) {
    systemState = STATE_READY;
  } else {
    systemState = STATE_WARNING;
  }
}

void printSensorsToFile() {
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
    case STATE_READY:
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      runBuzzer(0.2, 10);
      break;
    case STATE_FILE_CLOSED:
      pixel.setPixelColor(0, pixel.Color(0, 255, 255));
      const float buzzerSequence[4] = {0.2, 0.1, 0.2, 10};
      runBuzzer(buzzerSequence, 4);
      break;
    case STATE_STARTING:
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      const float buzzerSequence[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 10};
      runBuzzer(buzzerSequence, 6);
      break;
    case STATE_WARNING:
      pixel.setPixelColor(0, pixel.Color(255, 120, 0));
      const float buzzerSequence[8] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 3};
      runBuzzer(buzzerSequence, 8);
      break;
    case STATE_ERROR:
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      runBuzzer(0.1, 0.1);
      break;
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
  } else if (buzzerOn && millis() - lastTimeBuzzerChanged > secondsDuration * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWrite(BUZZER_PIN, LOW);
  }
}

/*
 * Function to run the buzzer for a specified sequence of durations and
 * intervals.
 * @param sequence Even-length array of durations and intervals
 * @param length Length of the array
 */
void runBuzzer(const float sequence[], int length) {
  if (length % 2 != 0) {
    error("Warning: sequence length must be even", false);
    return;
  }
  for (int i = 0; i < length; i += 2) {
    runBuzzer(sequence[i], sequence[i + 1]);
  }
}

void wait(int milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    notifyState();
  }
}