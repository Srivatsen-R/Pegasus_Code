#ifndef __TEMP_AL_H
#define __TEMP_AL_H

#include "main.h"
#include "macros.h"
#include "structures.h"


int16_t KTY84150_Motor_Temp(void);
double NTC_Board_Temp(double ntc);
TEMP_STATUS IsOvertemp (void);


#endif