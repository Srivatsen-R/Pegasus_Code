#ifndef __ADC_AL_H
#define __ADC_AL_H

#include "main.h"
#include "adc.h"

#include "macros.h"
#include "structures.h"
#include "enums.h"
#include "dr_devices.h"


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
double Get_ADC_Reading(uint8_t channel);



#endif