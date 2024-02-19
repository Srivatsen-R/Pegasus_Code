#ifndef FOC_BLOCKSET
#define FOC_BLOCKSET


float  READ_POSITION(int);
float  READ_SPEED(float);
float  SPEED_FILTER(float  ,float ,float );
float  TORQUE_PI_LOOP(float ,float );
float  CALCULATE_SYNC_SPEED(float  , float );
float  READ_ROTOR_ANGLE(float ,float ,float );
float  SPEED_PI_LOOP(float ,float ,float  );
float  MAGNETISING_PI_LOOP(float ,float );
float  FLUX_PI_LOOP(float ,float );
void   SPWM(int,int,int );
void   SVPWM_MODE_DRIVE_FUNCTION(int,int,int);
void   MotorControl_Init (void);
void   DRIVE_STOP(void);
float  CALCULATE_SLIP_SPEED(float,float);
float  map(float,float,float,float ,float);
float  TORQUE_PI_LOOP_2(float,float,float,float);


#endif 