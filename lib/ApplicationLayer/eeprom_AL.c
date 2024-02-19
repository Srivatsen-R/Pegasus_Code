/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with EEPROM/I2C read/write.

*/

#include <math.h>
#include <stdlib.h>

#include "eeprom_AL.h"
#include "math_func.h"
#include "vehicle.h"
#include "usart.h"

extern vehicle_t vehicle;
extern struct uart_t  serial;


eeprom_t e24lc02 = {
         .size       = 8,
         .eepromI2C  = &hi2c2,
         .devAddress = 0xA0,
         .memAddress = 0x00,
         .read       = EEPROM_Read,
         .write      = EEPROM_Write,    
};

EEPROM_STATUS EEPROM_Write (I2C_HandleTypeDef* hi2c,  
                            uint16_t devAddress, 
                            uint16_t memAddress,
                            uint8_t  *pData) 
{
   EEPROM_STATUS eepromStatus = HAL_I2C_Mem_Write(hi2c, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, pData, strlen((char*)pData), 500);
   HAL_Delay(5);
   return eepromStatus;
}


EEPROM_STATUS EEPROM_Read (I2C_HandleTypeDef* hi2c,  
                            uint16_t devAddress, 
                            uint16_t memAddress,
                            uint8_t  *pData,
                            uint16_t size) 
{
   EEPROM_STATUS eepromStatus = HAL_I2C_Mem_Read(hi2c, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &e24lc02.pData, size, 500);
   return eepromStatus;
}

void EEPROM_Read_Data()
{
      // Reading odometer data from EEPROM 
      e24lc02.status =  e24lc02.read(e24lc02.eepromI2C, e24lc02.devAddress, e24lc02.memAddress, (uint8_t*) &e24lc02.pData, e24lc02.size);
      if(e24lc02.status != EEPROM_OK) 
      serial.print(serial.uart1, "Failed to read Odometer value in EEPROM");
      vehicle.odometer = atof((char*) e24lc02.pData);
}


void EEPROM_Write_Data(uint32_t odo)
{
      // Writing odometer data to EEPROM
      char msg[12] ={0};
      sprintf(msg, "%lu", odo);
      e24lc02.size = strlen((char*) msg);
      e24lc02.status = e24lc02.write(e24lc02.eepromI2C, e24lc02.devAddress, e24lc02.memAddress, (uint8_t*)msg);
      // e24lc02.status = e24lc02.write(e24lc02.eepromI2C, e24lc02.devAddress, e24lc02.memAddress, (uint8_t*)"        ");
      if(e24lc02.status != EEPROM_OK) 
      serial.print(serial.uart1, "Failed to write Odometer value to EEPROM");
}
