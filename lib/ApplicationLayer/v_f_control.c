/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with V/F control of ACIM.

*/
#include "v_f_control.h"
#include "tim.h"

int n_samples_sine=0;
int f_sine=1;

int count,count1;  
float sine_a,sine_b,sine_c;

void VF_Init(void)
{

n_samples_sine = (F_ISR / f_sine);

}


void VF_Control(void)
{

  


    if((count) <= (n_samples_sine - 1))
    {
        sine_a = OFFSET_DUTY+ (f_sine / F_MAX) * PWM_DUTY_FACTOR* (sin((TWO_PI / n_samples_sine) * (count)));
        sine_b = OFFSET_DUTY+ (f_sine / F_MAX) * PWM_DUTY_FACTOR* (sin((TWO_PI / n_samples_sine) * (count)+ TWO_PI/ 3.0));
        sine_c = 1.0 - sine_b - sine_a + OFFSET_DUTY;

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,sine_a*T_PRD);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,sine_b*T_PRD);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,sine_c*T_PRD);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

        count = count + 1;
    }

    else
    {
        count = 0;
        count1 = count1 + 1;
        n_samples_sine = (F_ISR / f_sine);
        
        if ((f_sine) <= (F_MAX) && (count1 <= (25000 / n_samples_sine))) 
        {
            f_sine = f_sine;
        }
        else if ((f_sine) < (F_MAX)&& (count1) > (25000 / n_samples_sine)) 
        {
            f_sine = f_sine + 1;
        }
        else 
        {
            f_sine = F_MAX;
            count1 = 0;
        }
    }

}