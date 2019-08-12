#ifndef __DS18B20_H
#define __DS18B20_H
#include "main.h"
#include "stdio.h"

uint8_t DS18B20_Init(void);
void DS18B20_ReadId( uint8_t * ds18b20_id);
float DS18B20_GetTemp_SkipRom(void);
float DS18B20_GetTemp_MatchRom(uint8_t *ds18b20_id);

#endif

