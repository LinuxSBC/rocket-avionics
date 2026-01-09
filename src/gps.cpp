#include "gps.h"

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

bool GPSFix = false;

void setHasGPSFix(const bool hasFix) {
  GPSFix = hasFix;
}

bool hasGPSFix() {
  return GPSFix;
}

void initGPS() {
  #if DEBUG
  Serial.println("Starting GPS initialization...");
  #endif

  GPS.begin(9600);
  #if DEBUG
  Serial.println("Connected to GPS at 9600 baud");
  #endif
  wait(200);
  
  // Enable RMC + GGA - RMC for basic data, GGA for altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  wait(200);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  wait(200);
  
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // Update GPS position at 5Hz (the maximum)
  wait(200);
  
  GPS.sendCommand(PGCMD_NOANTENNA); // Disable antenna status messages
  wait(1000);

  #if DEBUG
  Serial.println("GPS initialization complete");
  #endif
}

void readGPS() {
  // read data from the GPS in the 'main loop'
  while (GPSSerial.available()) {
    GPS.read();
    // char c = GPS.read();
    // // if you want to debug, this is a good time to do it!
    // #if DEBUG
    //   if (c) Serial.print(c);
    // #endif
  }
  
  // Process all available NMEA sentences
  while (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      setHasGPSFix(false);
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  if (GPS.fix) {
    setHasGPSFix(true);
  } else {
    setHasGPSFix(false);
  }
}

void printGPSData() {
  #if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("GPS time: %02d:%02d:%02d\n", GPS.hour, GPS.minute, GPS.seconds);
  Serial.printf("Fix: %d, quality: %d, satellites: %d\n", GPS.fix, GPS.fixquality, GPS.satellites);
  Serial.printf("Location: %.4f %c, %.4f %c\n", GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
  Serial.printf("Speed (knots): %.2f, angle: %.2f, altitude: %.2f\n", GPS.speed, GPS.angle, GPS.altitude);
  #endif

// TODO: Only log when new data is available
  logPacket(GPS_MODULE, HOUR, GPS.hour);
  logPacket(GPS_MODULE, MINUTE, GPS.minute);
  logPacket(GPS_MODULE, SECOND, GPS.seconds);
  logPacket(GPS_MODULE, MILLISECOND, GPS.milliseconds);
  logPacket(GPS_MODULE, GPS_FIX, GPS.fix);
  if (GPS.fix) {
    logPacket(GPS_MODULE, GPS_QUALITY, GPS.fixquality);
    logPacket(GPS_MODULE, LATITUDE, GPS.latitude);
    logPacket(GPS_MODULE, LONGITUDE, GPS.longitude);
    logPacket(GPS_MODULE, SPEED, GPS.speed);
    logPacket(GPS_MODULE, ANGLE, GPS.angle);
    logPacket(GPS_MODULE, ALTITUDE, GPS.altitude);
    logPacket(GPS_MODULE, SATELLITES, GPS.satellites);
  }
}
