#include "states.h"

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN);

unsigned long lastTimeBuzzerChanged = 0;
bool buzzerOn = false;

System_State systemState;

void initIndicators() {
  pixel.begin();
  pixel.setBrightness(50);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void logData() {
#if USE_GPS
  readGPS();
#endif
  readSensors();
  printSensorsToFile();
}

void indicateState(System_State state) {
  switch (systemState) {
    case STATE_READY_TO_LAUNCH: {
#if USE_GPS
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
    case STATE_ASCENT: {
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

void handleState() { // operations and transition functions
  indicateState(systemState);

  // global operations and transition functions
  // TODO: Also close file on full SD card and low battery
  if (digitalRead(EJECT_BUTTON) == LOW && dataFile) {
    ejectSDCard();
    setState(STATE_FILE_CLOSED);
  }

  switch (systemState) {
    case STATE_READY_TO_LAUNCH: {
      if (dataFile) {
        logData();
      } else {
        error("Data file closed unexpectedly", false);
        setState(STATE_FILE_CLOSED);
      }

      // TODO: Add transition function to ascent
      // TODO: Clear interrupts on ADXL
      break;
    }
    case STATE_ASCENT: {
      // Transition function should probably be some threshold for chute deploy
      // bar+gyro+acc all crazy within 0.1s of each other?
      break;
    }
    case STATE_FILE_CLOSED: {
      break;
    }
    case STATE_STARTING: {
      break;
    }
    case STATE_WARNING: { // TODO: Warning should be a flag in Ready to Launch, not a state
      break;
    }
    case STATE_ERROR: {
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

void error(const String& message, const bool fatal) {
#if DEBUG
  Serial.println(message);
#endif
  if (fatal) {
    ejectSDCard();
    setState(STATE_ERROR);
    while (true) {
      yield();
    }
  }
  setState(STATE_WARNING);
}
