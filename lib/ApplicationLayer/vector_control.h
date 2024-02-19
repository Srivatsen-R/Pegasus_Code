
#ifndef VECTOR_CONTROL
#define VECTOR_CONTROL

#include <math.h>
#include "math_func.h"
#include "main.h"
#include "macros.h"
#include "enums.h"
#include <stdbool.h>

typedef struct foc_t {

             float speed_ref;
             float speed_limit;
             float speed_ref_output;
             float torque_current_ref;
             float torque_current_max;
             float flux_current_ref;
             float imr_ref;
             float imr_ref_min;
             float vd_ref;
             float vq_ref;
             float vq_ref_max;
             float vd_ref_max;
             float vq_ref_limit;
             float vd_ref_prev;
             float vq_ref_prev;
             int   vq_flag;
             float v_alpha;
             float v_beta;
             int   va_ref;
             int   vb_ref;
             int   vc_ref;
             int   vab_ref;
             int   vbc_ref;
             int   vca_ref;
             int   pwm_a;
             int   pwm_b;
             int   pwm_c;
             float rho; 
             float rotor_angle;
             float rotor_speed;
             float rotor_speed_filtered;
             float phase_U_current;
             float phase_V_current;
             float phase_W_current;  
             float alpha_current;
             float beta_current;
             float torque_current_sense;
             float flux_current_sense;
             float magnetising_current_sense;
             float speed_sense;
             float speed_rpm;
             float decay;
             float sin_rho;
             float cos_rho;
             float sync_speed;
             float slip_speed;
             float sync_speed_prev;
             float flux_current_sense_prev;
             float rotor_speed_prev;
             float rotor_speed_filtered_prev;
             float rho_prev;
             float magnetising_current_sense_prev;
             float speed_incr;
             float imr_incr;
             float flux_ref;
             float new_flux_ref;
             float flux_decrement;
             float flux_increment;

             char fw_flag;
             int  flag_cnt;
        
             int angular_position;
             int angular_position_prev;
             int angular_count;
             int angular_tic;
             int angular_speed;

             float phase_limit;
             float phase_decay;
             float regeneration_current_limit;
             float regen_current;
             float regen_speed;
             float torque_current_commanded;
             float speed_limit_prev;

             int reverse_flag_prev;
             int forward_flag_prev;
             float speed_ref_prev;

}foc_t;



//Motor-Controller Specific Parameters


#define PPR                     16384.0//NIDEC        
//#define PPR                     2048.0//ULTRON    

// #define POLEPAIRS               4.0// Nidec 14kg
#define POLEPAIRS               4.0// Nidec 22kg & ULTRON

#define FW_RPM                  1100.0 //FW 58V system.
//#define FW_RPM                  2650.0 //FW 72V system.

#define FW_MAX_CURRENT_PU       6800.0 //58V system
//#define FW_MAX_CURRENT_PU       1800.0 //72V system

#define TIM1_PSCLR              3
#define TIM1_PERIOD             2500// frequency = 200MHz/(2*(TIM1_PSCLR+1)*TIM1_PERIOD)
#define TIM1_DEADTIME           185
#define MAX_RPM                 5000.0
#define FW_MIN_CURRENT_PU       0.0
#define MTPA_MAX_CURRENT_PU     3500.0
#define D_CURRENT_DERATING_RPM_1  1000.0
#define D_CURRENT_DERATING_RPM_2  1100.0
#define FW_FLUX_CURRENT_SCALING_FACTOR  1.2
#define POWER_MAPPING_LOWER_RPM     1000.0
#define POWER_MAPPING_UPPER_RPM     2000.0    
#define POWER_MAPPING_UPPER_IQ_LIMIT_PU 22936.20
#define POWER_MAPPING_LOWER_IQ_LIMIT_PU 16000.0
#define VD_LIMIT                            25000.0
#define VQ_LIMIT                            15000.0
#define VQ_LIMIT_FACTOR                     1.0

//#define ANGLE_OFFSET                        4.188//Proto -3 Ultron
//#define ANGLE_OFFSET                        0.0//Proto -12 Ultron
//#define ANGLE_OFFSET                        0.523//Nidec 6 pole.

//#define ANGLE_OFFSET_FW                            2.094//ICAT Motor
//#define ANGLE_OFFSET_RW                            -4.1887//ICAT Motor

#define ANGLE_OFFSET_FW                               0 //Resolver motor
#define ANGLE_OFFSET_RW                               0

#define NIDEC_MOTOR_PWM             1
#define ULTRON_MOTOR_PWM            0
#define SKIP_PWM                    0


/*
30->0.523 
60->1.047 
90->1.570 
120->2.094 
150->2.617 
180->3.141
210->3.665
240->4.1887
270->4.7123
300->5.235
330->5.759
360->6.283
*/

//General parameters

#define ROTOR_TIME_CONSTANT                 0.11             // ROTOR TIME CONSTANT L/R
#define T_F_ROTOR_SPEED                     0.03           // Constant to filter rotor speed
#define MAGNETISING_CURRENT_REF             0.0//6000.0//6750.0
#define OUTPUT_LIMIT                        29200.0//17000.0       // OUTPUT LIMIT VD VQ
#define KP_W                                0.6//2.5//1.2//GAIN CONSTANT
#define KI_W                                1.2//50.0//0.48//INTG CONSTANT
#define KP_IQ                               0.6
#define KI_IQ                               1.2
#define KP_ID                               0.6
#define KI_ID                               1.2
#define SLIP_LIMIT                          20.0
#define FW_DIRECTION                        0
#define RW_DIRECTION                        0
#define RPM_IMR_CONST1                      0.0004251// (1-0.001)/(5000-2650)


#define F_SW                                20000.0//FCLK/(PSCR*ARR) //100MHz/(10*1000)         // ISR Sampling frequency
#define T_PRD                               2500.0         // PWM Period TIM1
#define OFFSET_DUTY                         0.5            // DUTY OFFSET used in modulation.
#define T_S                                 1.0/F_SW       // Sampling Time
#define PU                                  32767.0        // PU unit calculation max. value
#define LL                                  -1.0*PU        // LOWER LIMIT PU
#define UL                                  1.0*PU         // UPPER LIMIT PU
#define PI                                  3.14159        // PI Value
#define PPR_TO_RAD_CONSTANT                 (2.0*PI)/PPR   // Encoder value to mechanical angle conversion
#define DERIVATIVE_CONSTANT                 (2.0*F_SW)/(1256.0) // Constant to calculate speed
#define SPEED_FILTER_CONSTANT_1             ((2.0*T_F_ROTOR_SPEED-T_S)/(2.0*T_F_ROTOR_SPEED+T_S)) // Constant to filter speed
#define SPEED_FILTER_CONSTANT_2             (T_S/(2.0*T_F_ROTOR_SPEED+T_S)) // Constant to filter speed
#define PWM_CONST_1                         0.5*T_PRD        // CONSTANT FOR PWM Modulation
#define PWM_CONST_2                         (0.5*T_PRD)/(PU) // CONSTANT FOR PWM Modulation
#define ROTOR_TIME_CONSTANT_NEW             0.11             // ROTOR TIME CONSTANT L/R
#define IMR_CONST1                          (2.0*ROTOR_TIME_CONSTANT_NEW-T_S)/(2.0*ROTOR_TIME_CONSTANT_NEW+T_S) // MAGNETIZING CURRENT CONSTANT
#define IMR_CONST2                          T_S/(2.0*ROTOR_TIME_CONSTANT_NEW+T_S) // MAGNETIZING CURRENT CONSTANT
#define IMR_INTG_CONST                      0.5*T_S*KI_IMR // PI LOOP INTG
#define ID_INTG_CONST                       0.5*T_S*KI_ID  // PI LOOP INTG
#define SPEED_INTG_CONST                    0.5*T_S*KI_W   // PI LOOP INTG
#define IQ_INTG_CONST                       0.5*T_S*KI_IQ  // PI LOOP INTG
#define SLIP_SPEED_CONST                    1/(ROTOR_TIME_CONSTANT*MAGNETISING_CURRENT_REF) // SLIP SPEED CALCULATION CONSTANT
#define PWM_DUTY_ADD                        (0.5 * T_PRD) // PWM MODULATION CONSTANT
#define IMR_INCR                            0.016//0.0005*MAGNETISING_CURRENT_REF
#define DIV_FACTOR_CURRENT                  1//8
#define OFFSET_CURRENT                      32767/DIV_FACTOR_CURRENT             // ADC PHASE CURRENT OFFSET FOR 30000 ~ 1.5V
#define RAMP_UP                             0.05           // RAMP UP CONSTANT
#define KP_IMR                              2.0           // GAIN CONSTANT
#define KI_IMR                              20.0          // INTG CONSTANT
#define KC                                  0.0005
#define KC_W                                0.0005
#define MAX_RATE_Q                          250.0
#define MAX_RATE_D                          500.0
#define MAX_RATE_SPEED_LIMIT                800.0
#define REGEN_DOWN_FACTOR                   0.1
#define REGEN_ON                            0
#define REGEN_OFF                           1
#define MTPA_OFF                            0
#define MTPA_ON                             1

#define ID_PU_TO_A              0.0081*2.0/1.414
#define SPEED_PU_TO_RPM         0.183
#define IQ_PU_TO_A              0.00575*2.0


#define DUTY_TO_RADIAN          ((2*PI)/100.0)

#define SQ_MAX_PU_VOLTAGE          powf(30000.0, 2.0)


#define MAX_PU_CURRENT              39321.0
#define MAX_PU_SPEED                65535.0

#define KI_AC_CURRENT               0.0
#define KP_AC_CURRENT_UP            0.0001
#define KP_AC_CURRENT_DOWN          0.0001

#define TORQUE_PI_1  1
#define TORQUE_PI_2  0

#define AC_MAX       10000


void   VECTOR_FOC_Control(void) ;
void   FOC_CLARK_PARK_TRANSFORM(void);
void   FOC_READ_MOTOR_CURRENT(void);
void   FOC_TORQUE_PI_CONTROL(void);
void   FOC_ELECTRICAL_ANGLE_CALCULATION(void);
void   FOC_FLUX_PI_CONTROL(void);
void   FOC_INVERSE_CLARK_PARK_TRANSFORM(void);
void   FOC_SPACE_VECTOR_MODULATION(void);
void   FW_AND_MTPA_CONFIG(int);
void   POWER_MAPPING(int);
void   THROTTLE_PROFILE(int);
void   FOC_SPEED_TORQUE_PI_CONTROL();


#endif 
