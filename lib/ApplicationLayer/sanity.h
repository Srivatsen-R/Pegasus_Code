#ifndef SANITY
#define SANITY

#include "main.h"
#include "macros.h"
#include "enums.h"
#include <stdbool.h>
#include <structures.h>
#include <dr_devices.h>


typedef struct sanity_t {  
        char boardTemperature_u,
             boardTemperature_v,
             boardTemperature_w,   
             motorTemperature,
             motorStall,
             busVoltage,
             throttle,
             currentSensor,
             encoder,
             overCurrent,
             throttleStarting,
             status,
             generic;

        uint32_t fault_code;

        void (*run_sanity)(void);
}sanity_t;

typedef struct motorControl_t {  
        char          alarm,
                      assistGear,
                      direction,
                      status,
                      vehicleSpeedLow,
                      vehicleSpeedHigh;
        int           trip;
        float         busVoltage;   
        drive_t       drive;
        encoder_t     encoder;
        temperature_t temperature;
        void    (*stop)   (void);
        void    (*sanity) (void);
        void    (*analogReading) (void);  
        void    (*faultReading)  (void);
}motorControl_t;



void RUN_SANITY(void) ;
void ANALOG_READING(void);
void FAULT_READING(void);
int  Encoder_Fault(int,int);
void Encoder_Check(int);
void SAFETY_AND_ERRORS(void);


#endif 