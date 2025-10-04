#include "utils.h"

void wait(const int milliseconds) {
  const unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
#if USE_GPS
    readGPS();
#endif
    handleState();
  }
}