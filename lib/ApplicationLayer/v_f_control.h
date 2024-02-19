
#ifndef V_F_CONTROL
#define V_F_CONTROL

#define F_ISR            1000.0
#define F_MAX           25.0
#define T_PRD           2500.0
#define OFFSET_DUTY     0.5
#define PWM_DUTY_FACTOR 0.25
#define TWO_PI          2.0 * 3.14159


void VF_Control(void) ;
void VF_Init(void);

#endif 