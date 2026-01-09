#include "sdcard.h"

#include "utils.h"

SdFat SD;
SdFile dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &SPI1); // 50MHz is fast, good for RP2040

// Buffer control
constexpr uint32_t SYNC_INTERVAL_MS = 1000; // Sync to SD card every 1 second
uint32_t lastSyncTime = 0;

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

  // create a new file name, incrementing if it's already used
  char filename[15];
  strcpy(filename, "log00.bin"); // TODO: Use datetime
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = i / 10 + '0';
    filename[4] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
  }

  if (!dataFile.open(filename, O_RDWR | O_CREAT | O_TRUNC)) {
    error("Failed to open log file on SD card");
    return;
  }

  // Write the Header once at the start
  FileHeader header;
  dataFile.write((uint8_t*)&header, sizeof(header));

#if DEBUG
  Serial.print("Logging to: ");
  Serial.println(filename);
#endif
}

void logPacket(const Sensor sensorId, const SensorData dataId, const float val) {
  DataLine packet{};
  packet.timestamp_micros = micros();
  packet.timestamp_millis = millis();
  packet.sensor = sensorId;
  packet.data = dataId;
  packet.value = val;

  // Calculate checksum
  packet.checksum = calculateChecksum(packet);

  // WRITE BINARY DATA
  // This copies the struct bytes into the SdFat software buffer.
  // It effectively writes 15 bytes (size of DataLine).
  dataFile.write((uint8_t*)&packet, sizeof(packet));

  // --- SYNC STRATEGY ---
  // Don't sync() after every write! It halts the processor to talk to the card.
  // Sync periodically to save data in case of power loss (crash).
  if (millis() - lastSyncTime > SYNC_INTERVAL_MS) {
    dataFile.sync();
    lastSyncTime = millis();
  }
}

void ejectSDCard() {
  dataFile.close();
  lastSyncTime = millis(); // not necessary, but just in case of further changes that might expect it
  setState(STATE_FILE_CLOSED);
}

bool fileOpen() {
  return dataFile && dataFile.isOpen();
}