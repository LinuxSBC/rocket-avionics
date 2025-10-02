#include "rocket-avionics.h"

#define SD_CS_PIN 23
#define BUZZER_PIN 24

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
    handleState();
  #endif

  wait(500);
  
  Wire.setClock(400000); // set i2c clock to 400kHz (fast mode)

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
  #if USE_GPS
  initGPS();
  #endif

  setState(READY_TO_LAUNCH);
}

void loop() {
  if (dataFile) {
    #if USE_GPS
    readGPS();
    #endif
    readSensors();
    printSensorsToFile();

    // TODO: Also close file on full SD card and low battery
    if (digitalRead(PIN_BUTTON) == LOW && dataFile) {
      dataFile.close();
      setState(STATE_FILE_CLOSED);
      while (true) {
        handleState();
      }
    }
  } else {
    error("Data file closed unexpectedly", false);
    setState(STATE_FILE_CLOSED);
  }
  
  handleState();
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

  SD.remove("data.csv");
  dataFile = SD.open("data.csv", FILE_WRITE);
  if (!dataFile)
  {
    error("Failed to open data.csv on SD card");
  }

  if (!dataFile.seek(0))
  {
    error("Failed to seek to start of file");
  }

  // Pre-allocate 200MB for better write performance
  // dataFile.preAllocate(1024 * 1024 * 200);
}

void error(const String& message, const bool fatal) {
  #if DEBUG
  Serial.println(message);
  #endif
  if (fatal) {
    setState(STATE_ERROR);
    dataFile.close();
    while (true) {
      yield();
    }
  }
  setState(STATE_WARNING);
}

void handleState() {
  switch (systemState) {
    case READY_TO_LAUNCH: {
      #if GPS_ENABLED
      if (hasGPSFix()) {
        pixel.setPixelColor(0, pixel.Color(0, 255, 0));
        runBuzzer(0.2, 16);
      } else {
        pixel.setPixelColor(0, pixel.Color(255, 255, 0));
        runBuzzer(0.2, 8);
      }
      #else
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      runBuzzer(0.2, 16);
      #endif
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
  handleState();
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

void wait(const int milliseconds) {
  const unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    #if USE_GPS
    readGPS();
    #endif
    handleState();
  }
}