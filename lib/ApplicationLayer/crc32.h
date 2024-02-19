#ifndef __CRC32_H
#define __CRC32_H

#include <stdint.h>

uint32_t crc32_for_byte(uint32_t r);
uint32_t crc32(const void *data, uint32_t n_bytes);


#endif