#include "gps.h"

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

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
      setState(STATE_READY);
    } else {
      setState(STATE_NO_GPS);
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  if (GPS.fix) {
    setState(STATE_READY);
  } else {
    setState(STATE_NO_GPS);
  }
}

void printGPSHeader() {
  dataFile.print("GPS hour,"
    "GPS minute,"
    "GPS seconds,"
    "GPS milliseconds,"
    "GPS fix,"
    "GPS fix quality,"
    "GPS latitude,"
    "GPS longitude,"
    "GPS speed (knots),"
    "GPS angle,"
    "GPS altitude,"
    "GPS satellites,"
    "GPS antenna");
}

void printGPSData() {
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
}