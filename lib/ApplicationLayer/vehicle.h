#ifndef VEHICLE
#define VEHICLE

#include "main.h"
#include "macros.h"
#include "enums.h"
#include <stdbool.h>

typedef struct vehicle_t {

       float    odometer,
                 speed,
                 trip;
       char      odo_change_status;
       void (*calculate_ots)(uint32_t);          
}vehicle_t;


void Calculate_OTS(uint32_t) ;
void CAN_Logging(void);
void CAN_Communication(float,float,float);
void READ_FNR();
void READ_THROTTLE();
void READ_MOTOR_PHASE_CURRENT();
void READ_MOTOR_POSITION();


#endif 
