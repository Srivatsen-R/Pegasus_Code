/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with Vector FOC Control for motor control applications.

*/

#include <math.h>
#include "math_func.h"
#include "foc_blockset.h"
#include "vector_control.h"
#include "structures.h"
#include "tim.h"
#include "sanity.h"
#include "foc_mtpa_lut.h"
#include "motor_param.h"


extern mtpa_lut mtpa;
extern terminal_t terminal;
extern foc_t foc;
extern motorControl_t motorControl;

extern int forward_flag;
extern int reverse_flag;
extern int neutral_flag;

//Calculate rotor speed from position.
float READ_SPEED(float angle) {

        static float sine_rotor ,sine_rotor_prev;
        static float derivative_sine_rotor;
        static float cosine_rotor,cosine_rotor_prev;
        static float derivative_cosine_rotor;

        sine_rotor = sin(angle);
        derivative_sine_rotor = DERIVATIVE_CONSTANT*(sine_rotor-sine_rotor_prev);
        cosine_rotor = cos(angle);
        derivative_cosine_rotor = DERIVATIVE_CONSTANT*(cosine_rotor-cosine_rotor_prev);
        sine_rotor_prev = sine_rotor;
        cosine_rotor_prev = cosine_rotor;
        return ((cosine_rotor*derivative_sine_rotor)-(sine_rotor*derivative_cosine_rotor));

}

//Read rotor position from Encoder pulses.
float READ_POSITION(int position) {
    return (position*PPR_TO_RAD_CONSTANT);
}

//Filter for  speed of rotor.
float  SPEED_FILTER(float speed ,float speed_prev,float speed_filtered_prev) {
    return ((SPEED_FILTER_CONSTANT_1*speed_filtered_prev) +(SPEED_FILTER_CONSTANT_2*(speed+speed_prev)));
}

//Slip Speed Routine
float CALCULATE_SLIP_SPEED(float iq_sen,float imr_ref)
{
    float slip;
    slip = iq_sen/(ROTOR_TIME_CONSTANT*imr_ref);//0.22 5Nm 0.59 7Nm 0.89 10Nm 0.99Nm  
    if(slip>SLIP_LIMIT){slip =SLIP_LIMIT;}
    if(slip<0.0){slip=0.0;}
    return slip ;        
}


//Torque PI Control
float TORQUE_PI_LOOP(float iq_ref,float iq_sen)
{
static float error;
static float gain;
static float intg;
static float intg_prev;
static float error_prev;  
static float excess;
float u;
float limit;
float out;

#if REGEN_OFF

terminal.iq.ref  = iq_ref*IQ_PU_TO_A;

#endif

error = iq_ref - iq_sen;

gain = KP_IQ*error;

if(gain > foc.speed_limit){gain = foc.speed_limit;}
else if(gain < -foc.speed_limit){gain = -foc.speed_limit;}


u = gain +intg;

if(u>foc.speed_limit){out = foc.speed_limit;}
else if(u<-foc.speed_limit){out = -foc.speed_limit;}  
else{out = u;}


intg = intg_prev + ((IQ_INTG_CONST)*(error+error_prev));

if(intg > foc.speed_limit){intg = foc.speed_limit;}
else if(intg < -foc.speed_limit){intg = -foc.speed_limit;}

intg_prev = intg;
error_prev = error;

return out; 
}

//PI LOOP 2
float TORQUE_PI_LOOP_2(float ac_max, float ac_input  , float torque_input , float vq_prev )
    {
        float ac_error;
        static float ac_intg;

        ac_error = ((ac_max) - (ac_input));   
        if((ac_error < (400.0)) && (ac_error > (-400.0))){ac_intg = (0.0) ;}

        if(forward_flag)
        {
            if(ac_error >= (0.0)) // current less than max current
            {
            ac_intg += (ac_error *(KI_AC_CURRENT));             
            ac_error = ((KP_AC_CURRENT_UP) *ac_error);
            }
            else // current more than max  current.
            {
            ac_intg += (ac_error *(KI_AC_CURRENT));
            ac_error = ((KP_AC_CURRENT_DOWN)*ac_error);
            }
        }

        if(reverse_flag)
        {
            if(ac_error <= (0.0)) // current less than max current
            {
            ac_intg += (ac_error *(KI_AC_CURRENT));             
            ac_error = ((KP_AC_CURRENT_UP) *ac_error);
            }
            else // current more than max  current.
            {
            ac_intg += (ac_error *(KI_AC_CURRENT));
            ac_error = ((KP_AC_CURRENT_DOWN)*ac_error);
            }
        }

        if(neutral_flag)
        {
            if(motorControl.drive.fnr_status==1)
            {
                if(ac_error >= (0.0)) // current less than max current
                {
                ac_intg += (ac_error *(KI_AC_CURRENT));             
                ac_error = ((KP_AC_CURRENT_UP) *ac_error);
                }
                else // current more than max  current.
                {
                ac_intg += (ac_error *(KI_AC_CURRENT));
                ac_error = ((KP_AC_CURRENT_DOWN)*ac_error);
                }
            }

            if(motorControl.drive.fnr_status==2)
            {
                if(ac_error <= (0.0)) // current less than max current
                {
                ac_intg += (ac_error *(KI_AC_CURRENT));             
                ac_error = ((KP_AC_CURRENT_UP) *ac_error);
                }
                else // current more than max  current.
                {
                ac_intg += (ac_error *(KI_AC_CURRENT));
                ac_error = ((KP_AC_CURRENT_DOWN)*ac_error);
                }
            }
        }

        ac_intg = (ac_intg >=(868.0))?(868.0):ac_intg;
        ac_intg = (ac_intg <=(-868.0))?(-868.0):ac_intg;

        ac_error += ac_intg;       
        ac_error += vq_prev ;

        if(forward_flag)
        {
            if(ac_error >= torque_input) ac_error = torque_input ;
            if(ac_error <= 0) ac_error = 0 ;
        }

        if(reverse_flag)
        {
            if(ac_error <= torque_input) ac_error = torque_input ;
            if(ac_error > 0) ac_error = 0 ;
        }

        if(neutral_flag)
        {
            if(motorControl.drive.fnr_status==1)
            {
                if(ac_error >= torque_input) ac_error = torque_input ;
                if(ac_error <= 0) ac_error = 0 ;
            }

            if(motorControl.drive.fnr_status==2)
            {
                if(ac_error <= torque_input) ac_error = torque_input ;
                if(ac_error > 0) ac_error = 0 ;
            }
        }

        return ac_error ;
    }

// PI LOOP 2 END


//Synchronous Speed Routine
float CALCULATE_SYNC_SPEED(float slip , float rotor_speed) {
    return (slip+(POLEPAIRS*(rotor_speed*628.0)));
}

//Electrical angle calculation
float  READ_ROTOR_ANGLE(float rho_prev,float w_s,float w_s_prev) {
    return (rho_prev+((0.5*T_S)*(w_s+w_s_prev)));
}

//Speed PI Control
float SPEED_PI_LOOP(float w_ref,float w_sen,float phase_current_limit ) {
      
static float error;
static float gain;
static float intg;
static float intg_prev;
static float error_prev;  
float out;
float u;
float excess;

error = w_ref - w_sen;
terminal.w.err = error;
gain = KP_W*error;
terminal.w.gain = gain;

u = gain +intg;

if(u<=0.0){u=0.0;}
if(u>phase_current_limit){out = phase_current_limit;}
else if(u<=0.0){out=0.0;}
else{out = u;}

excess = u - out;
intg = intg_prev + ((SPEED_INTG_CONST)*(error+error_prev))- (KC_W*excess);
terminal.w.intg = intg;
intg_prev=intg;
error_prev=error;

return out;

}


//Magnetizing Current PI Loop
float  MAGNETISING_PI_LOOP(float imr_ref,float imr_sen) {

    static float error;
    static float gain;
    static float intg;
    static float intg_prev;
    static float error_prev;  
    float  out;

    error = imr_ref - imr_sen;

    terminal.imr.err = error;

    gain = KP_IMR*error;
    gain = (gain>MAGNETISING_CURRENT_REF)?MAGNETISING_CURRENT_REF:gain;
    gain = (gain<-MAGNETISING_CURRENT_REF)?-MAGNETISING_CURRENT_REF:gain;	

    terminal.imr.gain = gain;

    intg = intg_prev + ((IMR_INTG_CONST)*(error+error_prev));
    intg = (intg>MAGNETISING_CURRENT_REF)?MAGNETISING_CURRENT_REF:intg;
    intg = (intg<-MAGNETISING_CURRENT_REF)?-MAGNETISING_CURRENT_REF:intg;

    terminal.imr.intg = intg;

    out = gain +intg;
    out = (out>MAGNETISING_CURRENT_REF)?MAGNETISING_CURRENT_REF:out;
    out = (out<-MAGNETISING_CURRENT_REF)?-MAGNETISING_CURRENT_REF:out;


    intg_prev = intg;
    error_prev = error;

    return out; 
}

//Flux Control PI Loop
float FLUX_PI_LOOP(float id_ref,float id_sen)
{
static float error;
static float gain;
static float intg;
static float intg_prev;
static float error_prev;  
static float excess;
float u;
float out;

error = id_ref - id_sen;

// terminal.id.err = error;

gain = KP_IQ*error;

if(gain > VD_LIMIT){gain = VD_LIMIT;} 
else if(gain < -VD_LIMIT){gain = -VD_LIMIT;}

u = gain +intg;

if(u>VD_LIMIT){out = VD_LIMIT;}
else if(u<-VD_LIMIT){out = -VD_LIMIT;}  
else{out = u;}

excess = u - out;

intg = intg_prev + ((ID_INTG_CONST)*(error+error_prev)) - (KC_W*excess);

if(intg > VD_LIMIT){intg = VD_LIMIT;}
else if(intg < -VD_LIMIT){intg = -VD_LIMIT;}

intg_prev = intg;
error_prev = error;

return out; 

}

void SPWM(int p_a ,int p_b , int p_c) {
    		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,p_a);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
				
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,p_b);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
			
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,p_c);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void SVPWM_MODE_DRIVE_FUNCTION(int A_Duty_Svpwm_iq15 , int B_Duty_Svpwm_iq15  , int C_Duty_Svpwm_iq15 ) {

    int MAX_Duty_Svpwm_iq15 , MIN_Duty_Svpwm_iq15 , OFF_Duty_Spwm_iq15;
    int A_Duty_Svpwm_Temp_iq15 , B_Duty_Svpwm_Temp_iq15 ,  C_Duty_Svpwm_Temp_iq15 ;

     //*******   MAX Value CAlculate ********//
    if(A_Duty_Svpwm_iq15 > B_Duty_Svpwm_iq15) MAX_Duty_Svpwm_iq15 = A_Duty_Svpwm_iq15 ;
    else MAX_Duty_Svpwm_iq15 = B_Duty_Svpwm_iq15 ;
    if(MAX_Duty_Svpwm_iq15 < C_Duty_Svpwm_iq15) MAX_Duty_Svpwm_iq15 = C_Duty_Svpwm_iq15 ;

    //*******   MIN Value CAlculate ********//
   if(A_Duty_Svpwm_iq15 < B_Duty_Svpwm_iq15) MIN_Duty_Svpwm_iq15 = A_Duty_Svpwm_iq15 ;
   else MIN_Duty_Svpwm_iq15 = B_Duty_Svpwm_iq15 ;
   if(MIN_Duty_Svpwm_iq15 > C_Duty_Svpwm_iq15) MIN_Duty_Svpwm_iq15 = C_Duty_Svpwm_iq15 ;

   OFF_Duty_Spwm_iq15 =  MAX_Duty_Svpwm_iq15 ;
   OFF_Duty_Spwm_iq15 += MIN_Duty_Svpwm_iq15 ;
   OFF_Duty_Spwm_iq15 = OFF_Duty_Spwm_iq15*0.50;

   A_Duty_Svpwm_Temp_iq15 = A_Duty_Svpwm_iq15 ;
   A_Duty_Svpwm_Temp_iq15 -= OFF_Duty_Spwm_iq15 ;
   A_Duty_Svpwm_Temp_iq15 = A_Duty_Svpwm_Temp_iq15*(1.0) ;
   A_Duty_Svpwm_Temp_iq15 += (PWM_DUTY_ADD) ;

   B_Duty_Svpwm_Temp_iq15 = B_Duty_Svpwm_iq15 ;
   B_Duty_Svpwm_Temp_iq15 -= OFF_Duty_Spwm_iq15 ;
   B_Duty_Svpwm_Temp_iq15 = (B_Duty_Svpwm_Temp_iq15*(1.0)) ;
   B_Duty_Svpwm_Temp_iq15 += (PWM_DUTY_ADD) ;

   C_Duty_Svpwm_Temp_iq15 = C_Duty_Svpwm_iq15 ;
   C_Duty_Svpwm_Temp_iq15 -= OFF_Duty_Spwm_iq15 ;
   C_Duty_Svpwm_Temp_iq15 = C_Duty_Svpwm_Temp_iq15*(1.0) ;
   C_Duty_Svpwm_Temp_iq15 += (PWM_DUTY_ADD) ;

   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,A_Duty_Svpwm_Temp_iq15);
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,B_Duty_Svpwm_Temp_iq15);
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,C_Duty_Svpwm_Temp_iq15);
		
}


void MotorControl_Init (void) {

        if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
        if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
        if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

        if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
      	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
        if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
			
        DRIVE_STOP();	
}

void DRIVE_STOP(void) { 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}


float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





