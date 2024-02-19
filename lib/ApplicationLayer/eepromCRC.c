#include "eepromCRC.h"

uint32_t calculateFlashCRC(volatile uint64_t *data, uint16_t size)
{
	if(	(NULL == data) || (size <= 0))
	{
		return 0;
	}

	uint32_t poly = 0xEDB88320;
	uint32_t crc = 0xFFFFFFFF;

	for (uint16_t i = 0; i < size; i++)
	{
		crc ^= (uint32_t)data[i];

		for (int j = 0; j < 32; j++)
		{
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (poly & mask);
		}
	}

	return (~crc);
}