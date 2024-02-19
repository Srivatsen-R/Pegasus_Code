#ifndef EEPROM_CRC_H
#define EEPROM_CRC_H

#include <stdio.h>
#include <stdlib.h>

#define CRC_ADDRESS_CALC(address)   ((address) + 0x20)
#define CRC_SIZE                    8

uint32_t calculateFlashCRC(volatile uint64_t *data, uint16_t size);

#endif