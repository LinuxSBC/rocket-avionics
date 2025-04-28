#include "rocket-avionics.h"

#define SD_CS_PIN 23
#define BUZZER_PIN 24

#define DEBUG 0

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

unsigned long lastTimeBuzzerChanged = 0;
bool buzzerOn = false;

System_State systemState = STATE_STARTING;

void setup() {
  pixel.begin();
  pixel.setBrightness(50);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  setState(STATE_STARTING);
  
  #if DEBUG
  Serial.begin(115200);
  while (!Serial)
    notifyState();
  #endif

  wait(1000);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
  initGPS();

  setState(STATE_READY);
}

void loop() {
  if (dataFile) {
    readGPS();
    readSensors();
    printSensorsToFile();
  }
  
  notifyState();

  // TODO: Also close file on full SD card and low battery
  if (digitalRead(PIN_BUTTON) == LOW && dataFile) {
    dataFile.close();
    setState(STATE_FILE_CLOSED);
    while (1) {
      notifyState();
    }
  }
}

void initSDCard() {
  #if DEBUG
  Serial.println("Initializing SD card...");
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

void error(String message, bool fatal) {
  #if DEBUG
  Serial.println(message);
  #endif
  if (fatal) {
    setState(STATE_ERROR);
    dataFile.close();
    while (1) {
      yield();
    }
  }
  else {
    setState(STATE_WARNING);
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

void setState(System_State state) {
  systemState = state;
  notifyState();
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