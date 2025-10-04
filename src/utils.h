#ifndef ROCKET_AVIONICS_UTILS_H
#define ROCKET_AVIONICS_UTILS_H

#include "states.h"
#if USE_GPS
#include "gps.h"
#endif

void wait(int milliseconds);

#endif