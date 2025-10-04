#ifndef ROCKET_AVIONICS_SD_CARD_H
#define ROCKET_AVIONICS_SD_CARD_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#include "flags.h"

extern File32 dataFile; // TODO: Figure out an interface for this

void initSDCard();
void ejectSDCard();

#endif