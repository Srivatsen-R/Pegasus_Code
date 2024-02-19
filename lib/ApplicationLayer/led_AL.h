#ifndef __LED_AL_H
#define __LED_AL_H

#include "main.h"
#include "gpio.h"
#include "macros.h"
#include "dr_devices.h"

void writeColor(uint16_t color);
void toggleColor(uint16_t color);
void run_disco (void);


#endif