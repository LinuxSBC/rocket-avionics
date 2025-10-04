#include "sdcard.h"

#include "utils.h"

SdFat SD;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

void initSDCard() {
#if DEBUG
  Serial.println("Initializing SD card...");
#endif

  // Retry mechanism for SD card initialization
  while (!SD.begin(config)) {
    error("Initialization failed! Retrying...", false);
    wait(1000); // Wait for a second before retrying
  }
#if DEBUG
  Serial.println("Initialization successful.");
#endif

  SD.remove("data.csv");
  dataFile = SD.open("data.csv", FILE_WRITE);
  if (!dataFile) {
    error("Failed to open data.csv on SD card");
  }

  if (!dataFile.seek(0)) {
    error("Failed to seek to start of file");
  }

  // Pre-allocate 200MB for better write performance
  // dataFile.preAllocate(1024 * 1024 * 200);
}

void ejectSDCard() {
  dataFile.close();
  setState(STATE_FILE_CLOSED);
}

// void write(DataLine line) {
//
// }