#ifndef FOC_MTPA_LUT
#define FOC_MTPA_LUT

#include "main.h"
#include "macros.h"
#include "enums.h"
#include <stdbool.h>


typedef struct mtpa_lut {

       int  torque_index,
            rpm_index,
            torque,
            speed;
                 
}mtpa_lut;


int TORQUE_INDEX_CALCULATE(int);
int RPM_INDEX_CALCULATE(int);

#endif 