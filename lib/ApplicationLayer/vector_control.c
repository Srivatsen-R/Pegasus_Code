/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with algorithm implementation for Field oriented control for ACIM/PMSM.

VECTOR_FOC_Control(void) : This functions contains all the FOC blockset functions responsible for motor control.

VECTOR CONTROL SUMMARY
To summarize the steps required for indirect vector
control:

1. The 3-phase stator currents are measured. This measurement provides ia, ib and ic. The rotor velocity is also measured.

2. The 3-phase currents are converted to a 2-axis system. This conversion provides the variables iα and iβ from the measured ia, ib and ic 
values.iα and iβ are time varying quadrature current values as viewed from the perspective of the stator.

3. The 2-axis coordinate system is rotated to align with the rotor flux using a transformation angle information calculated at the last 
iteration of the control loop. This conversion provides the Id and Iq variables from iα and iβ. Id and Iq are the quadrature currents 
transformed to the rotating coordinate system. For steady state conditions, Id and Iq will be constant.

4. Error signals are formed using Id, Iq and reference values for each. The Id reference controls rotor magnetizing flux. 
The Iq reference controls the torque output of the motor. The error signals are input to PI controllers. The output of the 
controllers provide Vd and Vq, which is a voltage vector that will be sent to the motor.

5. A new coordinate transformation angle is calculated. The motor speed, rotor electrical time constant, Id and Iq are the inputs 
to this calculation.The new angle tells the algorithm where to place the next voltage vector to produce an amount of slip for the present 
operating conditions.

6. The Vd and Vq output values from the PI controllers are rotated back to the stationary reference frame using the new angle. 
This calculation provides quadrature voltage values vα and vβ.

7. The vα and vβ values are transformed back to 3-phase values va, vb and vc. The 3-phase voltage values are used to calculate new PWM
duty cycle values that generate the desired voltage vector.

*/

#include <math.h>
#include "math_func.h"
#include "vector_control.h"
#include "dr_devices.h"
#include "structures.h"
#include "tim.h"
#include "sanity.h"
#include "foc_blockset.h"
#include "foc_mtpa_lut.h"
#include <stdint.h>
#include "motor_param.h"

extern int a_current;
extern int b_current;
extern int dc_current_u;
extern int dc_current_v;
extern int dc_current_w;
extern int speed_filtered;
extern float ref_output;

extern int forward_flag;
extern int reverse_flag;
extern int neutral_flag;
extern float Duty;
extern uint8_t reset_flag;
uint8_t duty_state=1;
uint8_t speed_fix_flag=0;
extern uint8_t acc_flag;
extern uint8_t deacc_flag;

float log_gain=0;
float log_intg=0;
extern motorControl_t motorControl;

int state=0;

foc_t foc; 
extern mtpa_lut mtpa;

extern const int mtpa_lut_id[16][6];
extern const int mtpa_lut_iq[16][6];


terminal_t terminal = {
       .rotor.rpm            = 10000.0,
       .rotor.angle          = 0.0,
       .rotor.speed          = 0.0,
       .rotor.speed_filtered = 0.0,
       .phase.A_current      = 0,
       .phase.B_current      = 0,
       .phase.C_current      = 0,
       .clarke.alpha_current = 0.0,
       .clarke.beta_current  = 0.0,
       .park.ids             = 0.0,
       .park.iqs             = 0.0,
       .speed.slip           = 0.0,
       .speed.sync           = 0.0,
       .speed.angular        = 0.0,
       .rho_                 = 0.0,
       .rho.sin              = 0.0,
       .rho.cos              = 0.0,
       .vd.ref               = 0.0,
       .vq.ref               = 0.0,
       .imr.ref              = 0.0,
       .imr.sen              = 0.0,
       .imr.err              = 0.0,
       .imr.gain             = 0.0,
       .imr.intg             = 0.0, 
       .w.ref                = 0.0,
       .w.sen                = 0.0,
       .w.err                = 0.0,
       .w.gain               = 0.0,
       .w.intg               = 0.0,
       .iq.err               = 0.0,
       .iq.gain              = 0.0,
       .iq.intg              = 0.0,
       .id.err               = 0.0,
       .id.gain              = 0.0,
       .id.intg              = 0.0, 
};



void THROTTLE_PROFILE(int config)
{
    if(config==1)
    {
        foc.torque_current_ref = (0.84*foc.speed_ref);
    }
}

void POWER_MAPPING(int config)
{

if(config==1)
{

    static int average_controller_temperature;
    static int bms_dynamic_current_limit_out;

                average_controller_temperature = ((motorControl.temperature.u+motorControl.temperature.v+motorControl.temperature.w)/3);    
                bms_dynamic_current_limit_out=250;

            //limiting torque current on basis on measured speed.
            if(average_controller_temperature<60 && motorControl.temperature.motor<70 && bms_dynamic_current_limit_out>=250)
            {    
                if((foc.speed_sense*SPEED_PU_TO_RPM) <= POWER_MAPPING_LOWER_RPM){foc.phase_limit = POWER_MAPPING_UPPER_IQ_LIMIT_PU;}
                else if((foc.speed_sense*SPEED_PU_TO_RPM) >= POWER_MAPPING_UPPER_RPM){foc.phase_limit = POWER_MAPPING_LOWER_IQ_LIMIT_PU;}
                else{foc.phase_limit = -11.1*(foc.speed_sense*SPEED_PU_TO_RPM) + 34421.0;}

                if(foc.torque_current_ref > foc.phase_limit){foc.torque_current_ref = foc.phase_limit;}
            }  

            //limiting torque current on basis of controller temperature
            else if(average_controller_temperature>=60)
            {    
                foc.phase_limit = -82.6*motorControl.temperature.motor + 16568;
        
                if(foc.torque_current_ref > foc.phase_limit){
                    foc.torque_current_ref = foc.phase_limit;
                }
            }

            //limiting torque current on basis of motor temperature
            else if(motorControl.temperature.motor>=70)
            {    
                    foc.phase_limit = -82.6*motorControl.temperature.motor + 16568;

                    if(foc.torque_current_ref > foc.phase_limit){
                        foc.torque_current_ref = foc.phase_limit;
                    }           

            }
                    

            //limiting torque current on basis of bus voltage

            //limiting torque current on basis of DCLO
            else if(bms_dynamic_current_limit_out<190)
            {    

                    //TO BE IMPLEMENTED     
            }
}

if(config==2)
{

    static int average_controller_temperature;
    static int bms_dynamic_current_limit_out;

                average_controller_temperature = ((motorControl.temperature.u+motorControl.temperature.v+motorControl.temperature.w)/3);    
                bms_dynamic_current_limit_out=250;

            //limiting torque current on basis on measured speed.
            if(average_controller_temperature<60 && motorControl.temperature.motor<120 && bms_dynamic_current_limit_out>=250)
            {    
                if((foc.speed_sense*SPEED_PU_TO_RPM) <= POWER_MAPPING_LOWER_RPM){foc.phase_limit = -POWER_MAPPING_UPPER_IQ_LIMIT_PU;}
                else if((foc.speed_sense*SPEED_PU_TO_RPM) >= POWER_MAPPING_UPPER_RPM){foc.phase_limit = -POWER_MAPPING_LOWER_IQ_LIMIT_PU;}
                else
                {
                    foc.phase_limit = -11.1*(foc.speed_sense*SPEED_PU_TO_RPM) + 34421.0;
                    foc.phase_limit = -foc.phase_limit;
                }

                if(foc.torque_current_ref < foc.phase_limit){foc.torque_current_ref = foc.phase_limit;}
            }  

            //limiting torque current on basis of controller temperature
            else if(average_controller_temperature>=60)
            {    
                foc.phase_limit = -217*average_controller_temperature + 30411;
                foc.phase_limit = -foc.phase_limit;
        
                if(foc.torque_current_ref < foc.phase_limit){
                    foc.torque_current_ref = foc.phase_limit;
                }
            }

            //limiting torque current on basis of motor temperature
            else if(motorControl.temperature.motor>=120)
            {    
                    foc.phase_limit = -434*motorControl.temperature.motor + 69510;
                    foc.phase_limit = -foc.phase_limit;

                    if(foc.torque_current_ref < foc.phase_limit){
                        foc.torque_current_ref = foc.phase_limit;
                    }           

            }
                    

            //limiting torque current on basis of bus voltage

            //limiting torque current on basis of DCLO
            else if(bms_dynamic_current_limit_out<190)
            {    

                    //TO BE IMPLEMENTED     
            }

}    


if(config==3)
{

    static int average_controller_temperature;
    static int bms_dynamic_current_limit_out;

                average_controller_temperature = ((motorControl.temperature.u+motorControl.temperature.v+motorControl.temperature.w)/3);    
                bms_dynamic_current_limit_out=250;

            //limiting torque current on basis on measured speed.
            if(average_controller_temperature<60 && motorControl.temperature.motor<70 && bms_dynamic_current_limit_out>=250)
            {    
                if((foc.speed_sense*SPEED_PU_TO_RPM) <= POWER_MAPPING_LOWER_RPM){foc.phase_limit = POWER_MAPPING_UPPER_IQ_LIMIT_PU;}
                else if((foc.speed_sense*SPEED_PU_TO_RPM) >= POWER_MAPPING_UPPER_RPM){foc.phase_limit = POWER_MAPPING_LOWER_IQ_LIMIT_PU;}
                else{foc.phase_limit = -11.1*(foc.speed_sense*SPEED_PU_TO_RPM) + 34421.0;}

            }  

            //limiting torque current on basis of controller temperature
            else if(average_controller_temperature>=60)
            {    
                foc.phase_limit = -82.6*motorControl.temperature.motor + 16568;
        
            }

            //limiting torque current on basis of motor temperature
            else if(motorControl.temperature.motor>=70)
            {    
                    foc.phase_limit = -82.6*motorControl.temperature.motor + 16568;
            }
                    

            //limiting torque current on basis of bus voltage

            //limiting torque current on basis of DCLO
            else if(bms_dynamic_current_limit_out<190)
            {    

                    //TO BE IMPLEMENTED     
            }
}


if(config==4)
{

    static int average_controller_temperature;
    static int bms_dynamic_current_limit_out;

                average_controller_temperature = ((motorControl.temperature.u+motorControl.temperature.v+motorControl.temperature.w)/3);    
                bms_dynamic_current_limit_out=250;

            //limiting torque current on basis on measured speed.
            if(average_controller_temperature<60 && motorControl.temperature.motor<120 && bms_dynamic_current_limit_out>=250)
            {    
                if((foc.speed_sense*SPEED_PU_TO_RPM) <= POWER_MAPPING_LOWER_RPM){foc.phase_limit = -POWER_MAPPING_UPPER_IQ_LIMIT_PU;}
                else if((foc.speed_sense*SPEED_PU_TO_RPM) >= POWER_MAPPING_UPPER_RPM){foc.phase_limit = -POWER_MAPPING_LOWER_IQ_LIMIT_PU;}
                else
                {
                    foc.phase_limit = -11.1*(foc.speed_sense*SPEED_PU_TO_RPM) + 34421.0;
                    foc.phase_limit = -foc.phase_limit;
                }

            }  

            //limiting torque current on basis of controller temperature
            else if(average_controller_temperature>=60)
            {    
                foc.phase_limit = -217*average_controller_temperature + 30411;
                foc.phase_limit = -foc.phase_limit;
        
            }

            //limiting torque current on basis of motor temperature
            else if(motorControl.temperature.motor>=120)
            {    
                    foc.phase_limit = -434*motorControl.temperature.motor + 69510;
                    foc.phase_limit = -foc.phase_limit;

            }
                    

            //limiting torque current on basis of bus voltage

            //limiting torque current on basis of DCLO
            else if(bms_dynamic_current_limit_out<190)
            {    

                    //TO BE IMPLEMENTED     
            }

}    



    
}


void FW_AND_MTPA_CONFIG(int config)
{
 

if(config==1)
{
                if((foc.speed_sense*SPEED_PU_TO_RPM)>=FW_RPM)
                {
                    foc.flux_current_ref = - map((foc.speed_sense*SPEED_PU_TO_RPM), FW_RPM, MAX_RPM, FW_MIN_CURRENT_PU, FW_MAX_CURRENT_PU);    
                    foc.flux_current_ref = FW_FLUX_CURRENT_SCALING_FACTOR*foc.flux_current_ref;
                }
            
                if((foc.speed_sense*SPEED_PU_TO_RPM)<FW_RPM)
                {
                        if(((foc.speed_sense*SPEED_PU_TO_RPM) >= D_CURRENT_DERATING_RPM_1) && ((foc.speed_sense*SPEED_PU_TO_RPM) <= D_CURRENT_DERATING_RPM_2))
                        {
                            foc.flux_current_ref = map(foc.speed_sense*SPEED_PU_TO_RPM, D_CURRENT_DERATING_RPM_1, D_CURRENT_DERATING_RPM_2, MTPA_MAX_CURRENT_PU, 0);
                        }
                        else if(((foc.speed_sense*SPEED_PU_TO_RPM) <D_CURRENT_DERATING_RPM_1))
                        {
                            foc.flux_current_ref = map(foc.speed_ref, 0.0, 27305.0, 0.0, MTPA_MAX_CURRENT_PU);//not correct   
                        }

                        if(foc.flux_current_ref >= MTPA_MAX_CURRENT_PU){foc.flux_current_ref = MTPA_MAX_CURRENT_PU;}
                }

                if(foc.flux_current_ref <= -FW_MAX_CURRENT_PU){foc.flux_current_ref = -FW_MAX_CURRENT_PU;}
}

if(config==2)
{
                    if((foc.speed_sense*SPEED_PU_TO_RPM)>=FW_RPM)
                    {
                    foc.flux_current_ref = - map((foc.speed_sense*SPEED_PU_TO_RPM), FW_RPM, MAX_RPM, FW_MIN_CURRENT_PU, FW_MAX_CURRENT_PU) - (foc.speed_sense*SPEED_PU_TO_RPM)*0.1*1.414;

                    if(foc.flux_current_ref <= -FW_MAX_CURRENT_PU){foc.flux_current_ref = -FW_MAX_CURRENT_PU;}
                    }
            
                if((foc.speed_sense*SPEED_PU_TO_RPM)<FW_RPM)
                {
                       
                        if(((foc.speed_sense*SPEED_PU_TO_RPM) >= 1000.0) && ((foc.speed_sense*SPEED_PU_TO_RPM) <= 2000.0)){
                            //foc.flux_current_ref = map((foc.speed_sense*SPEED_PU_TO_RPM), 1000.0, 2000.0, MTPA_MAX_CURRENT_PU, 0.0);
                            //foc.flux_current_ref=0;
                            foc.flux_current_ref = map(foc.speed_sense*SPEED_PU_TO_RPM, 1000.0, 2000.0, -5500, 0);
                         
                        }else if(((foc.speed_sense*SPEED_PU_TO_RPM) <1000.0)){
                            //foc.flux_current_ref = map((foc.speed_sense*SPEED_PU_TO_RPM), 0.0, 999.9, 0.0, MTPA_MAX_CURRENT_PU);//not correct
                            foc.flux_current_ref = map(foc.speed_ref, 0.0, 27305.0, 0.0, -5500);//not correct   
                        }

                        if(foc.flux_current_ref <= -5500){
                            foc.flux_current_ref = -5500;
                        }
                }
  
}
}

/*
This function is used to read motor position using AB/ABZ+PWM sensor from motor. It calculates
rotor mechanical angle from encoder pulses. Rotor angle is differentiated wrt time to get rotor mechanical
angular velocity.

Last status : 

1. foc.speed_sense variable updated for reverse operation.


*/
void FOC_READ_MOTOR_POSITION(void)
{
            foc.rotor_angle =  READ_POSITION(motorControl.encoder.value);
            foc.rotor_speed =  READ_SPEED(foc.rotor_angle);

            if(forward_flag)
            {
                    if(foc.rotor_speed<0.0){foc.rotor_speed=0.0;}
                    foc.rotor_speed_filtered = SPEED_FILTER(foc.rotor_speed,foc.rotor_speed_prev,foc.rotor_speed_filtered_prev); 
                    foc.speed_sense = (foc.rotor_speed_filtered)*PU*1.0;
                    if(foc.speed_sense>MAX_PU_SPEED){foc.speed_sense=MAX_PU_SPEED;} 
            }

            if(reverse_flag)
            {
                    if(foc.rotor_speed>0.0){foc.rotor_speed=0.0;}
                    foc.rotor_speed_filtered = SPEED_FILTER(foc.rotor_speed,foc.rotor_speed_prev,foc.rotor_speed_filtered_prev);
                    foc.speed_sense = (foc.rotor_speed_filtered)*PU*1.0;
                    foc.speed_sense = -foc.speed_sense;
                    if(foc.speed_sense>MAX_PU_SPEED){foc.speed_sense=MAX_PU_SPEED;}      
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status==1){
                    if(foc.rotor_speed<0.0){foc.rotor_speed=0.0;}
                    foc.rotor_speed_filtered = SPEED_FILTER(foc.rotor_speed,foc.rotor_speed_prev,foc.rotor_speed_filtered_prev);
                    foc.speed_sense = (foc.rotor_speed_filtered)*PU*1.0;
                    if(foc.speed_sense>MAX_PU_SPEED){foc.speed_sense=MAX_PU_SPEED;} 
                }

                if(motorControl.drive.fnr_status==2)
                {
                    if(foc.rotor_speed>0.0){foc.rotor_speed=0.0;}
                    foc.rotor_speed_filtered = SPEED_FILTER(foc.rotor_speed,foc.rotor_speed_prev,foc.rotor_speed_filtered_prev);
                    foc.speed_sense = (foc.rotor_speed_filtered)*PU*1.0;
                    foc.speed_sense = -foc.speed_sense;
                    if(foc.speed_sense>MAX_PU_SPEED){foc.speed_sense=MAX_PU_SPEED;}    
                }
            }
}

/*
This function is used to read phase current U,V from the motor.
3 phase motor is a balanced system so W current is calculated as:

U+V+W = 0 
W = 0 - V - W

Clark transform is used for U,V,W to alpha , beta (refer vector control.pdf)
Park transform is used for alpha,beta to dq conversion (refer vector control.pdf)

Last status : 

1. Variable names changed from A,B,C to U,V,W

*/

void FOC_CLARK_PARK_TRANSFORM()
{
            //Read Phase Currents
            foc.phase_U_current = a_current;    
            foc.phase_V_current = b_current;
            foc.phase_W_current = 0.0 - foc.phase_U_current - foc.phase_V_current;//phase c current
            //Clark Transform
            foc.alpha_current = foc.phase_U_current - 0.5*(foc.phase_V_current+foc.phase_W_current);//alpha current 
            foc.beta_current  = 0.866*(foc.phase_V_current-foc.phase_W_current);//beta current
            //Park Transform
            foc.flux_current_sense     = (foc.cos_rho*foc.alpha_current) + (foc.sin_rho*foc.beta_current);//flux current
            if(foc.flux_current_sense>MAX_PU_CURRENT){foc.flux_current_sense = MAX_PU_CURRENT;}    
            if(foc.flux_current_sense<-MAX_PU_CURRENT){foc.flux_current_sense = -MAX_PU_CURRENT;}    
            foc.torque_current_sense   = (foc.cos_rho*foc.beta_current) - (foc.sin_rho*foc.alpha_current);//torque current  
            if(foc.torque_current_sense>MAX_PU_CURRENT){foc.torque_current_sense = MAX_PU_CURRENT;}    
            if(foc.torque_current_sense<-MAX_PU_CURRENT){foc.torque_current_sense = -MAX_PU_CURRENT;}    
}


/*This function compares commanded and measured speed and computes voltage vectors accordingly

foc.speed_ref = 0 to 27305 ==> (27305/32767)*6000 = 4999 rpm

*/
void FOC_SPEED_TORQUE_PI_CONTROL()
{
   foc.speed_ref = ref_output;
   foc.torque_current_max = 10000;
   foc.torque_current_ref = SPEED_PI_LOOP(foc.speed_ref,foc.speed_sense,foc.torque_current_max); 
   foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense);
   if(foc.vq_ref>VQ_LIMIT){foc.vq_ref = VQ_LIMIT;}

}


/*
This function is used to compute output voltage for a specific 
referenced torque current value from throttle.

Last Changes:
1. THROTTLE_PROFILE function added to compute torque current profile 
wrt throttle input.
2. POWER MAPPING function updated to derate torque current on basis of
controller temperature, motor temperature , DCLO , measured speed.

*/

void FOC_TORQUE_PI_CONTROL()
{
    
            foc.speed_ref = ref_output;

            THROTTLE_PROFILE(1);

            if(forward_flag){foc.torque_current_ref = foc.torque_current_ref;}
            if(reverse_flag){foc.torque_current_ref = -foc.torque_current_ref;}

             if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1){foc.torque_current_ref = foc.torque_current_ref;}
                if(motorControl.drive.fnr_status == 2){foc.torque_current_ref = -foc.torque_current_ref;}
            }

            #if MTPA_OFF

            mtpa.speed = terminal.w.sen;
            mtpa.torque = foc.speed_ref;

            foc.torque_current_ref = mtpa_lut_iq[TORQUE_INDEX_CALCULATE(mtpa.torque)][RPM_INDEX_CALCULATE(mtpa.speed)];
            foc.flux_current_ref =   mtpa_lut_id[TORQUE_INDEX_CALCULATE(mtpa.torque)][RPM_INDEX_CALCULATE(mtpa.speed)];

            #endif



#if TORQUE_PI_1

            if(forward_flag){POWER_MAPPING(1);}
            if(reverse_flag){POWER_MAPPING(2);}
 
             if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1){POWER_MAPPING(1);}
                if(motorControl.drive.fnr_status == 2){POWER_MAPPING(2);}
            }


            #if REGEN_OFF
            foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense); 
            #endif

            #if REGEN_ON

            if(forward_flag)
            {
                    foc.regen_speed = (foc.speed_sense*SPEED_PU_TO_RPM);

                    //add limit for regen speed.

                    if (foc.speed_ref<=0 && foc.regen_speed>300 && forward_flag==1)
                    {

                        foc.regeneration_current_limit = -foc.regen_speed * 0.05;
                        foc.regen_current = foc.regen_current - REGEN_DOWN_FACTOR;

                        if(foc.regen_current<foc.regeneration_current_limit)
                        {
                            foc.regen_current = foc.regeneration_current_limit;
                        }
                        terminal.iq.ref               = foc.regen_current*0.00575*2.0;
                        // torque_current_commanded = regen_current;
                        foc.vq_ref = TORQUE_PI_LOOP(foc.regen_current,foc.torque_current_sense);//torque loop

                    }

                    else
                    {
                        foc.regen_current = foc.regen_current  + REGEN_DOWN_FACTOR;

                        if(foc.regen_current>= foc.torque_current_ref)
                        {
                            foc.regen_current = foc.torque_current_ref;
                        }
                        terminal.iq.ref = foc.regen_current*0.00575*2.0;
                        // torque_current_commanded = regen_current;
                        foc.vq_ref = TORQUE_PI_LOOP(foc.regen_current,foc.torque_current_sense);//torque loop
                    }
            }

            if(reverse_flag)
            {
                terminal.iq.ref  = foc.torque_current_ref*0.00575*2.0;
                // torque_current_commanded = torque_current_ref;
                foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense);
            }

            if(reverse_flag == 0 && forward_flag == 0){
                terminal.iq.ref  = foc.torque_current_ref*0.00575*2.0;
                foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense);
            }

            #endif

            //limiting voltage wrt throttle.
            if(forward_flag)
            {    
                foc.speed_limit = foc.speed_ref*VQ_LIMIT_FACTOR ;
                if(foc.vq_ref>foc.speed_limit) {foc.vq_ref = foc.speed_limit;}
                if(foc.vq_ref<-foc.speed_limit){foc.vq_ref = -foc.speed_limit;}
            }

            if(reverse_flag)
            {    
                foc.speed_limit = foc.speed_ref*VQ_LIMIT_FACTOR;    
                if(foc.vq_ref>foc.speed_limit) {foc.vq_ref = foc.speed_limit;}
                if(foc.vq_ref<-foc.speed_limit){foc.vq_ref = -foc.speed_limit;}
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {
                    foc.speed_limit = foc.speed_ref*VQ_LIMIT_FACTOR;
                    if(foc.vq_ref>foc.speed_limit){foc.vq_ref = foc.speed_limit;}
                    if(foc.vq_ref<-foc.speed_limit){foc.vq_ref = -foc.speed_limit;}
                }

                if(motorControl.drive.fnr_status == 2)
                {
                    foc.speed_limit = foc.speed_ref*VQ_LIMIT_FACTOR;                    
                    if(foc.vq_ref>foc.speed_limit) {foc.vq_ref = foc.speed_limit;}
                    if(foc.vq_ref<-foc.speed_limit){foc.vq_ref = -foc.speed_limit;}

                }
            }

 #endif


#if TORQUE_PI_2


            if(forward_flag){POWER_MAPPING(3);}
            if(reverse_flag){POWER_MAPPING(4);}

             if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1){POWER_MAPPING(3);}
                if(motorControl.drive.fnr_status == 2){POWER_MAPPING(4);}
            }

            #if REGEN_OFF
            terminal.iq.ref  = foc.phase_limit*IQ_PU_TO_A;
            // terminal.iq.ref  = -3200*IQ_PU_TO_A;

            foc.vq_ref = TORQUE_PI_LOOP_2(foc.phase_limit ,foc.torque_current_sense,foc.torque_current_ref, foc.vq_ref); 
            // foc.vq_ref = TORQUE_PI_LOOP_2(-3200.0 ,foc.torque_current_sense,foc.torque_current_ref, foc.vq_ref); 
            

            

            if(forward_flag)
            {
            if(foc.vq_ref>30000){foc.vq_ref = 30000;}  
            if(foc.vq_ref<0){foc.vq_ref =0;}
            }

            if(reverse_flag)
            {
            if(foc.vq_ref>0){foc.vq_ref = 0;}  
            if(foc.vq_ref<-30000){foc.vq_ref =-30000;}
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {
                    if(foc.vq_ref>30000){foc.vq_ref = 30000;}  
                    if(foc.vq_ref<0){foc.vq_ref =0;}
                }

                if(motorControl.drive.fnr_status == 2)
                {
                   if(foc.vq_ref>0){foc.vq_ref = 0;}  
                   if(foc.vq_ref<-30000){foc.vq_ref =-30000;}
                }
            }



            #endif

            #if REGEN_ON

            if(forward_flag)
            {
                    foc.regen_speed = (foc.speed_sense*SPEED_PU_TO_RPM);

                    //add limit for regen speed.

                    if (foc.speed_ref<=0 && foc.regen_speed>300 && forward_flag==1)
                    {

                        foc.regeneration_current_limit = -foc.regen_speed * 0.05;
                        foc.regen_current = foc.regen_current - REGEN_DOWN_FACTOR;

                        if(foc.regen_current<foc.regeneration_current_limit)
                        {
                            foc.regen_current = foc.regeneration_current_limit;
                        }
                        terminal.iq.ref               = foc.regen_current*0.00575*2.0;
                        // torque_current_commanded = regen_current;
                        foc.vq_ref = TORQUE_PI_LOOP(foc.regen_current,foc.torque_current_sense);//torque loop

                    }

                    else
                    {
                        foc.regen_current = foc.regen_current  + REGEN_DOWN_FACTOR;

                        if(foc.regen_current>= foc.torque_current_ref)
                        {
                            foc.regen_current = foc.torque_current_ref;
                        }
                        terminal.iq.ref = foc.regen_current*0.00575*2.0;
                        // torque_current_commanded = regen_current;
                        foc.vq_ref = TORQUE_PI_LOOP(foc.regen_current,foc.torque_current_sense);//torque loop
                    }
            }

            if(reverse_flag)
            {
                terminal.iq.ref  = foc.torque_current_ref*0.00575*2.0;
                // torque_current_commanded = torque_current_ref;
                foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense);
            }

            if(reverse_flag == 0 && forward_flag == 0){
                terminal.iq.ref  = foc.torque_current_ref*0.00575*2.0;
                foc.vq_ref = TORQUE_PI_LOOP(foc.torque_current_ref,foc.torque_current_sense);
            }

            #endif

 #endif





            //limiting voltage wrt voltage max limit , vs^2 = vq^2 + vd^2
            if(forward_flag){
                foc.vq_ref_max =sqrt(SQ_MAX_PU_VOLTAGE - powf(foc.vd_ref, 2.0));     
                if(foc.vq_ref >foc.vq_ref_max ){foc.vq_ref = foc.vq_ref_max;}
            }

            if(reverse_flag){
                foc.vq_ref_max = -sqrt(SQ_MAX_PU_VOLTAGE - powf(foc.vd_ref, 2.0));
                if(foc.vq_ref < foc.vq_ref_max){foc.vq_ref = foc.vq_ref_max;}
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {
                    foc.vq_ref_max =sqrt(SQ_MAX_PU_VOLTAGE - powf(foc.vd_ref, 2.0));     
                    if(foc.vq_ref >foc.vq_ref_max ){foc.vq_ref = foc.vq_ref_max;}
                }

                if(motorControl.drive.fnr_status == 2)
                {
                foc.vq_ref_max = -sqrt(SQ_MAX_PU_VOLTAGE - powf(foc.vd_ref, 2.0));
                if(foc.vq_ref < foc.vq_ref_max){foc.vq_ref = foc.vq_ref_max;}
                }
            }
            
}

/*
This function is used to compute electrical angle from mechanical angular velocity,
pole pairs.

electrical angle = previous angle + (polepair*mechanical angular velocity*time)

Last Changes:
1. Function updated for operation for Z pulse and PWM pulse.

Z pulse = For correcting angle at each mechanical revolution.
PWM pulse = To calculate initial rotor mechanical angle. 

*/

void FOC_ELECTRICAL_ANGLE_CALCULATION()
{

    static float angle_mech;

            foc.slip_speed = 0.0;
            //Synchronous Speed Calculation
            foc.sync_speed = CALCULATE_SYNC_SPEED(foc.slip_speed,foc.rotor_speed_filtered);//sync speed


            #if ULTRON_MOTOR_PWM

            if(forward_flag)
            {
                if(foc.speed_sense*SPEED_PU_TO_RPM<100)
                {
                    //pwm signal        
                    if(angle_mech<0){angle_mech = angle_mech+6.28;}
                    else if(angle_mech>=0){angle_mech = (Duty)*DUTY_TO_RADIAN;}   

                    // angle_mech = (Duty)*DUTY_TO_RADIAN; //mech angle
                    foc.rho_prev = 3.0*angle_mech; // elec angle
                    if (angle_mech>2.095 && angle_mech<4.1866){foc.rho_prev = foc.rho_prev - 6.28;}
                    else  if(angle_mech>=4.1886){foc.rho_prev = foc.rho_prev - 12.56;}
                }
            }

            if(reverse_flag)
            {
                if(foc.speed_sense*SPEED_PU_TO_RPM<100)
                {
                    //pwm signal        
                    angle_mech = -(6.28-(Duty)*DUTY_TO_RADIAN); //mech angle
                    foc.rho_prev = 3.0*angle_mech; // elec angle
                    if (angle_mech<-2.095 && angle_mech>-4.1866){foc.rho_prev = foc.rho_prev + 6.28;}
                    else  if(angle_mech<=-4.1886){foc.rho_prev = foc.rho_prev + 12.56;}
                }

            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {  
                        if(foc.speed_sense*SPEED_PU_TO_RPM<100)
                        {
                         
                         //pwm signal        
                         if(angle_mech<0){angle_mech = angle_mech+6.28;}
                         else if(angle_mech>=0){angle_mech = (Duty)*DUTY_TO_RADIAN;}   

                         // angle_mech = (Duty)*DUTY_TO_RADIAN; //mech angle

                            foc.rho_prev = 3.0*angle_mech; // elec angle
                            if (angle_mech>2.095 && angle_mech<4.1866){foc.rho_prev = foc.rho_prev - 6.28;}
                            else  if(angle_mech>=4.1886){foc.rho_prev = foc.rho_prev - 12.56;}
                        }

                }

                if(motorControl.drive.fnr_status == 2)
                {
                        if(foc.speed_sense*SPEED_PU_TO_RPM<100)
                        {
                            //pwm signal        
                            angle_mech = -(6.28-(Duty)*DUTY_TO_RADIAN); //mech angle
                            foc.rho_prev = 3.0*angle_mech; // elec angle
                            if (angle_mech<-2.095 && angle_mech>-4.1866){foc.rho_prev = foc.rho_prev + 6.28;}
                            else  if(angle_mech<=-4.1886){foc.rho_prev = foc.rho_prev + 12.56;}
                        }
                }
            }

             if(reset_flag == 1)
            {

                //foc.rho_prev = PPR_TO_RAD_CONSTANT*motorControl.encoder.value;
                foc.rho_prev = 0.0;
                reset_flag=0;
            }
     
            foc.rho = READ_ROTOR_ANGLE(foc.rho_prev,foc.sync_speed,foc.sync_speed_prev);//electrical angle
            #endif

            #if NIDEC_MOTOR_PWM

         /*   // if(forward_flag)
            {
                if(foc.speed_sense*SPEED_PU_TO_RPM<10.0)
                {
                    //pwm signal        
                    // if(angle_mech<0){angle_mech = angle_mech+6.28;}
                    // else if(angle_mech>=0){angle_mech = (100-Duty)*DUTY_TO_RADIAN;}   

                    angle_mech = (100-Duty)*DUTY_TO_RADIAN;

                    // angle_mech = (Duty)*DUTY_TO_RADIAN; //mech angle
                    foc.rho_prev = POLEPAIRS*angle_mech; // elec angle
                    if (angle_mech>2.095 && angle_mech<4.1866){foc.rho_prev = foc.rho_prev - 6.28;}
                    else  if(angle_mech>=4.1886){foc.rho_prev = foc.rho_prev - 12.56;}
                }

            }

            if(reverse_flag)
            {
                if(foc.speed_sense*SPEED_PU_TO_RPM<10.0)
                {
                    //pwm signal        
                    angle_mech = -(6.28-(100-Duty)*DUTY_TO_RADIAN); //mech angle
                    foc.rho_prev = POLEPAIRS*angle_mech; // elec angle
                    if (angle_mech<-2.095 && angle_mech>-4.1866){foc.rho_prev = foc.rho_prev + 6.28;}
                    else  if(angle_mech<=-4.1886){foc.rho_prev = foc.rho_prev + 12.56;}
                }

            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {  
                        if(foc.speed_sense*SPEED_PU_TO_RPM<10.0)
                        {
                         
                         //pwm signal        
                        //  if(angle_mech<0){angle_mech = angle_mech+6.28;}
                        //  else if(angle_mech>=0){angle_mech = (100-Duty)*DUTY_TO_RADIAN;}   

                            angle_mech = (100-Duty)*DUTY_TO_RADIAN;


                         // angle_mech = (Duty)*DUTY_TO_RADIAN; //mech angle

                            foc.rho_prev = POLEPAIRS*angle_mech; // elec angle
                            if (angle_mech>2.095 && angle_mech<4.1866){foc.rho_prev = foc.rho_prev - 6.28;}
                            else  if(angle_mech>=4.1886){foc.rho_prev = foc.rho_prev - 12.56;}
                        }

                }

                if(motorControl.drive.fnr_status == 2)
                {
                        if(foc.speed_sense*SPEED_PU_TO_RPM<10.0)
                        {
                            //pwm signal        
                            angle_mech = -(6.28-(100-Duty)*DUTY_TO_RADIAN); //mech angle
                            foc.rho_prev = POLEPAIRS*angle_mech; // elec angle
                            if (angle_mech<-2.095 && angle_mech>-4.1866){foc.rho_prev = foc.rho_prev + 6.28;}
                            else  if(angle_mech<=-4.1886){foc.rho_prev = foc.rho_prev + 12.56;}
                        }
                }
            }
*/
            if(reset_flag == 1)
            {

                //foc.rho_prev = PPR_TO_RAD_CONSTANT*motorControl.encoder.value;
                foc.rho_prev = 0.0;
                reset_flag=0;
            }
          
            foc.rho = READ_ROTOR_ANGLE(foc.rho_prev,foc.sync_speed,foc.sync_speed_prev);//electrical angle
            #endif




            #if SKIP_PWM
           //reset angle to 0
            if(reset_flag == 1)
            {

                //foc.rho_prev = PPR_TO_RAD_CONSTANT*motorControl.encoder.value;
                foc.rho_prev = 0.0;
                reset_flag=0;
            }
            //adding initial stationary angle to calculation
            if(duty_state == 1)
            {
                if(forward_flag){foc.rho_prev = (Duty)*DUTY_TO_RADIAN;}
                if(reverse_flag){foc.rho_prev = -(Duty)*DUTY_TO_RADIAN;}
                duty_state = 0;
                //Angle Calculation
                foc.rho = READ_ROTOR_ANGLE(foc.rho_prev,foc.sync_speed,foc.sync_speed_prev);//electrical angle
            }
            else
            {
            //Angle Calculation
            foc.rho = READ_ROTOR_ANGLE(foc.rho_prev,foc.sync_speed,foc.sync_speed_prev);//electrical angle

            }
            #endif


            
            if (foc.rho>=6.28){foc.rho=0.0;} 
            else if (foc.rho<=-6.28){foc.rho=0.0;}

            //Sin and Cos

            if(forward_flag)
            {
            foc.cos_rho = cos(foc.rho+ANGLE_OFFSET_FW);
            foc.sin_rho = sin(foc.rho+ANGLE_OFFSET_FW);            
            }

            if(reverse_flag)
            {
            foc.cos_rho = cos(foc.rho+ANGLE_OFFSET_RW);
            foc.sin_rho = sin(foc.rho+ANGLE_OFFSET_RW);
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {  
                    foc.cos_rho = cos(foc.rho+ANGLE_OFFSET_FW);
                    foc.sin_rho = sin(foc.rho+ANGLE_OFFSET_FW);            
                }

                if(motorControl.drive.fnr_status == 2)
                {
                    foc.cos_rho = cos(foc.rho+ANGLE_OFFSET_RW);
                    foc.sin_rho = sin(foc.rho+ANGLE_OFFSET_RW);
                }
            }
}
/*
This function is used to compute field weakening and mtpa operations.

Field weakening is done to run motor beyond rated speed for higher
rpm operation.

Maximum torque per ampere operation is done to provide initial pickup torque 
for smooth operation. It also improves controller efficiency while climbing 
ramps with payloads. 

flux_current_ref is manipulated for FW and MTPA.

Last Changes:
1. MTPA and Field Weakening transitions tuned to better response during drive.



*/

void FOC_FIELD_WEAKENING_AND_MTPA()
{
            if(forward_flag)
            {
                FW_AND_MTPA_CONFIG(1);
            }

            if(reverse_flag)
            {
                if((foc.speed_sense*SPEED_PU_TO_RPM)<FW_RPM)
                {       
                    if(((foc.speed_sense*SPEED_PU_TO_RPM) >= D_CURRENT_DERATING_RPM_1) && ((foc.speed_sense*SPEED_PU_TO_RPM) <= D_CURRENT_DERATING_RPM_2))
                     {foc.flux_current_ref = map(foc.speed_sense*SPEED_PU_TO_RPM, D_CURRENT_DERATING_RPM_1, D_CURRENT_DERATING_RPM_2, MTPA_MAX_CURRENT_PU, 0);}
                    else if(((foc.speed_sense*SPEED_PU_TO_RPM) <D_CURRENT_DERATING_RPM_1))
                     {foc.flux_current_ref = map(foc.speed_ref, 0.0, 27305.0, 0.0, MTPA_MAX_CURRENT_PU);}

                    if(foc.flux_current_ref >= MTPA_MAX_CURRENT_PU){foc.flux_current_ref = MTPA_MAX_CURRENT_PU;}
                }
            }

            if(neutral_flag)
            {
                if(motorControl.drive.fnr_status == 1)
                {
                    FW_AND_MTPA_CONFIG(1);
                }

                if(motorControl.drive.fnr_status == 2)
                {
                
                    if((foc.speed_sense*SPEED_PU_TO_RPM)<FW_RPM)
                    {       
                        if(((foc.speed_sense*SPEED_PU_TO_RPM) >= D_CURRENT_DERATING_RPM_1) && ((foc.speed_sense*SPEED_PU_TO_RPM) <= D_CURRENT_DERATING_RPM_2))
                        {foc.flux_current_ref = map(foc.speed_sense*SPEED_PU_TO_RPM, D_CURRENT_DERATING_RPM_1, D_CURRENT_DERATING_RPM_2, MTPA_MAX_CURRENT_PU, 0);}
                        else if(((foc.speed_sense*SPEED_PU_TO_RPM) <D_CURRENT_DERATING_RPM_1))
                        {foc.flux_current_ref = map(foc.speed_ref, 0.0, 27305.0, 0.0, MTPA_MAX_CURRENT_PU);}

                        if(foc.flux_current_ref >= MTPA_MAX_CURRENT_PU){foc.flux_current_ref = MTPA_MAX_CURRENT_PU;}
                    }

                }
            }
/*
            if(neutral_flag)
            {

                if(foc.flux_current_ref > 0)
                {
                   foc.flux_current_ref = foc.flux_current_ref - 0.1;     
                   if(foc.flux_current_ref<=0){foc.flux_current_ref=0;}
                }
                
                if(foc.flux_current_ref < 0)
                {
                   foc.flux_current_ref = foc.flux_current_ref + 0.1;     
                   if(foc.flux_current_ref>=0){foc.flux_current_ref=0;}     
                }
                 
            }
*/            
}



/*
This function is used to compute output voltage for a specific 
referenced flux current value from throttle.

Last Changes:
1. PI controller logic updated..

*/

void FOC_FLUX_PI_CONTROL()
{
            //PI loop flux current
           //foc.flux_current_ref=0.0;
   
            foc.vd_ref = FLUX_PI_LOOP(foc.flux_current_ref,foc.flux_current_sense);//Flux Loop
            if(foc.vd_ref>VD_LIMIT){foc.vd_ref =VD_LIMIT;}
            if(foc.vd_ref<-VD_LIMIT){foc.vd_ref =-VD_LIMIT;}
}
/*
This function is used to convert  voltage vectors Vd,Vq back to valpha , vbeta.
and valpha , vbeta to Va,Vb,Vc.

Last status : 

1. No changes.Working fine.

*/

void FOC_INVERSE_CLARK_PARK_TRANSFORM()
{
                //Park Inverse
                foc.v_alpha = foc.cos_rho*foc.vd_ref - foc.sin_rho*foc.vq_ref;
                foc.v_alpha = (foc.v_alpha>UL)?UL:foc.v_alpha;
                foc.v_alpha = (foc.v_alpha<LL)?LL:foc.v_alpha;	

                foc.v_beta  = foc.cos_rho*foc.vq_ref + foc.sin_rho*foc.vd_ref;
                foc.v_beta = (foc.v_beta>UL)?UL:foc.v_beta;
                foc.v_beta = (foc.v_beta<LL)?LL:foc.v_beta;

                //Clark Inverse
                foc.va_ref=foc.v_alpha;	

                foc.vb_ref = -0.5*foc.v_alpha + 0.866*foc.v_beta;		
                foc.vb_ref = (foc.vb_ref>UL)?UL:foc.vb_ref;
                foc.vb_ref = (foc.vb_ref<LL)?LL:foc.vb_ref;	

                foc.vc_ref = -0.5*foc.v_alpha - 0.866*foc.v_beta;		
                foc.vc_ref = (foc.vc_ref>UL)?UL:foc.vc_ref;
                foc.vc_ref = (foc.vc_ref<LL)?LL:foc.vc_ref;

                //Delta to Star conversion code
                #if CODE_NOT_IN_USE   
                vab_ref=foc.v_alpha;	

                vbc_ref = -0.5*foc.v_alpha + 0.866*foc.v_beta;		
                vbc_ref = (vbc_ref>UL)?UL:vbc_ref;
                vbc_ref = (vbc_ref<LL)?LL:vbc_ref;	

                vca_ref = -0.5*foc.v_alpha - 0.866*foc.v_beta;		
                vca_ref = (vca_ref>UL)?UL:vca_ref;
                vca_ref = (vca_ref<LL)?LL:vca_ref;


                foc.va_ref=(1.0/3.0)*(vab_ref-vca_ref);
                if	(foc.va_ref>=UL) {foc.va_ref=UL;} else if (foc.va_ref<=LL) {foc.va_ref=LL;}
                foc.vb_ref=(1.0/3.0)*(vbc_ref-vab_ref);
                if	(foc.vb_ref>=UL) {foc.vb_ref=UL;} else if (foc.vb_ref<=LL) {foc.vb_ref=LL;}
                foc.vc_ref=(1.0/3.0)*(vca_ref-vbc_ref);
                if	(foc.vc_ref>=UL) {foc.vc_ref=UL;} else if (foc.vc_ref<=LL) {foc.vc_ref=LL;}

                #endif

}


/*
This function is used to convert Va,Vb,Vc voltage vectors 
to PWM duty values for inverter operation.

Last status : 

1. SVPWM bug resolved. 1.15 changed to 1.0 to prevent overmodulation.

*/

void FOC_SPACE_VECTOR_MODULATION()
{
            //PWM DUTY VARIABLES

            foc.pwm_a = (PWM_CONST_2*foc.va_ref)  + PWM_CONST_1;
            foc.pwm_b = (PWM_CONST_2*foc.vb_ref)  + PWM_CONST_1;
            foc.pwm_c = (PWM_CONST_2*foc.vc_ref)  + PWM_CONST_1;   

            //Modulation  Techniques
            //SPWM(foc.pwm_a,foc.pwm_b,foc.pwm_c);
            if(motorControl.drive.check == DRIVE_ENABLE) {SVPWM_MODE_DRIVE_FUNCTION(foc.pwm_a,foc.pwm_b,foc.pwm_c);}
            else {DRIVE_STOP();}
}
        
/*
This is the main function for motor control operation. Please refer to 
vector control .pdf for understanding the algorithm and theory behind 
vector control.

*/
void VECTOR_FOC_Control(void) {
   
            FOC_READ_MOTOR_POSITION();
            FOC_CLARK_PARK_TRANSFORM();

            if(TORQUE_MODE){FOC_TORQUE_PI_CONTROL();}
            else{FOC_SPEED_TORQUE_PI_CONTROL();}

            FOC_ELECTRICAL_ANGLE_CALCULATION();
            FOC_FIELD_WEAKENING_AND_MTPA();
            FOC_FLUX_PI_CONTROL();
            FOC_INVERSE_CLARK_PARK_TRANSFORM();
            FOC_SPACE_VECTOR_MODULATION();
        
            foc.rotor_speed_prev          = foc.rotor_speed;  
            foc.rotor_speed_filtered_prev = foc.rotor_speed_filtered;
            foc.flux_current_sense_prev   = foc.flux_current_sense;
            foc.sync_speed_prev           = foc.sync_speed;
            foc.rho_prev                  = foc.rho;
            foc.magnetising_current_sense_prev = foc.magnetising_current_sense;
            foc.reverse_flag_prev = reverse_flag;
            foc.forward_flag_prev = forward_flag;
            foc.vq_ref_prev  = foc.vq_ref;
            foc.speed_ref_prev = foc.speed_ref;

            terminal.rotor.speed          = foc.rotor_speed;
            terminal.rotor.speed_filtered = foc.rotor_speed_filtered;
            terminal.w.sen                = ((foc.speed_sense)*SPEED_PU_TO_RPM);
            terminal.phase.A_current      = foc.phase_U_current;
            terminal.phase.B_current      = foc.phase_V_current;
            terminal.phase.C_current      = foc.phase_W_current;
            terminal.id.sen               = foc.flux_current_sense* ID_PU_TO_A;
            terminal.iq.sen               = foc.torque_current_sense*IQ_PU_TO_A;
            terminal.w.ref                = (((foc.speed_ref))*SPEED_PU_TO_RPM);
            terminal.imr.sen              = foc.magnetising_current_sense;
            terminal.speed.slip           = Duty;
            terminal.speed.sync           = foc.sync_speed;
            terminal.speed.angular        = foc.angular_speed;
            terminal.rho_                 = foc.rho;
            terminal.rho.sin              = foc.sin_rho;
            terminal.rho.cos              = foc.cos_rho;
            
            terminal.vd.ref               = (foc.vd_ref)/10.0;

            terminal.vq.ref               = (foc.vq_ref)/10.0;
            terminal.imr.ref              = foc.imr_ref;
            terminal.id.ref               = foc.flux_current_ref* ID_PU_TO_A;

}






