#ifndef __EEPROM_AL_H
#define __EEPROM_AL_H


#include <string.h>

#include "main.h"
#include "i2c.h"
#include "enums.h"
#include "dr_devices.h"
#include "structures.h"


EEPROM_STATUS EEPROM_Write (I2C_HandleTypeDef* hi2c, uint16_t devAddress, uint16_t memAddress, uint8_t  *pData);
EEPROM_STATUS EEPROM_Read (I2C_HandleTypeDef* hi2c,  uint16_t devAddress, uint16_t memAddress, uint8_t  *pData, uint16_t size);

void EEPROM_Read_Data(void);
void EEPROM_Write_Data(uint32_t);

#endif