#include "states.h"

#include "sdcard.h"

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
