#include "rocket-avionics.h"

void setup() {
  setState(STATE_STARTING);
  initIndicators();

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
      ejectSDCard();
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
