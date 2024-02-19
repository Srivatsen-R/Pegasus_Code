
#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H


//Drive parameters
#define NTC_PULL_UP_RESISTOR                10000//NIDEC NTC VALUE
//#define NTC_PULL_UP_RESISTOR                52600//ULTRON

#define UNDER_VOLT_LIMIT                48.0   
#define OVER_VOLT_LIMIT                 75.0//72 corresponds to 80V


#define TORQUE_MODE                     1.0
#define SPEED_LIMIT                     5000.0
#define SPORTS_MODE                     0.0
#define ECO_MODE                        0.0

#define THROTTLE_THRESHOLD_A            14000.0
#define THROTTLE_THRESHOLD_B            14000.0    
#define ACCELERATION_CONST              300.0//15.0
#define DEACCELERATION_CONST            100.0//7.0
#define HEAT_SINK_MAX_TEMP_LIMIT        75.0
#define MOTOR_TEMP_LIMIT                130.0   

#define MAX_SPEED                       30000    
#define SPEED_MUL_FACTOR                ((SPEED_LIMIT*32767.0)/(6000.0*MAX_SPEED))//0.364//0.273
#define HEAT_SINK_TEMP_HYS              10.0 
#define THROTTLE_ADC_OFFSET_COUNT       1700 //0.8V
#define TH_MIN_VAL                      1000
#define KI_FOR_DC_CURRENT               0.00000005
#define KP_FOR_DC_CURR_UP               0.001
#define KP_FOR_DC_CURR_DOWN             0.001

#define VBUS_OFFSET                      0.0   
#define OVER_VOLT_HYS                    0.5   
#define UNDER_VOLT_HYS                   0.5   
#define BATT_CURRENT_ADC_OFFSET          0.0
#define Adc_max_COUNT                    65535
#define ADC_REF                          3.3
#define BATT_VOLT_FINAL_OFFSET           0
#define BATT_VOLTAGE_ADC_DIVD            31.0// (310/10)
#define BATT_VOLT_RESOLUTION             1
#define BATT_VOLT_MULTIPLY               ((ADC_REF * BATT_VOLTAGE_ADC_DIVD * BATT_VOLT_RESOLUTION) / Adc_max_COUNT)
#define BATT_VOLT_ADC_OFFSET             0
#define MAX_TARGET_SPEED                    3000.0
#define TH_AVG_MAX_ADC_COUNT                (Adc_max_COUNT -  THROTTLE_ADC_OFFSET_COUNT)
#define TH_AVG_TO_SPEED_MUL_FACTOR          ((MAX_TARGET_SPEED / TH_AVG_MAX_ADC_COUNT) * SPEED_MUL_FACTOR)
#define PU_TO_RPM                           21.85//65535/3000
#define TH_START_LIMIT                      20.0
#define TH_HYSS_LIMIT                       5.0
#define ENCODER_TIMEOUT_LIMIT               60000

#define RPM_TO_KMPH                         0.0083246
#define HOUR_TO_MS_100                      0.0002777
#define ODO_UPDATE                          1
#define NO_ODO_UPDATE                       0
#define KMPH_CAN_SCALING                    10
#define ODO_CAN_SCALING                     10
#define THROTTLE_CAN_ADC_SCALING            10.0
#define THROTTLE_ENCODER_THRESHOLD          1000
#define SANITY_COUNT                        150
#define CURRENT_SENSOR_MAX                  34000
#define CURRENT_SENSOR_MIN                  30000
#define RPM_TO_FREQ                         2.0/120.0
#define RPM_TO_THROTTLE_PERCENT             100.0/5000.0   
#define CURRENT_AVG_FACTOR                  20.0 
#define BETA_VALUE                          4000.0   
#define T_AMBIENT                           25.0
#define KELVIN_TO_CELSIUS                   273.15 
#define THROTTLE_AVG                        1.0
#define TEMP_AVG                            15.0
#define VOLTAGE_AVG                         60.0
#define ROOT2                               1.414



unsigned int Get_Batt_Current_measured(unsigned int);  // 0.1 amp resolution
int Get_Phase_Current_measured(int);  // 0.1 amp resolution
unsigned int Get_Batt_Voltage_measured(unsigned int);  // 0.1 volt resolution
unsigned char Get_temperature(unsigned int) ;
float moving_Batt_current_measured_fun(float  , float ) ;  // 0.1 amp Batt_current_measured
float moving_Batt_voltage_measured_fun(float  , float ) ;  // 0.1 amp Batt_current_measured
float moving_AC_voltage_measured_fun(float  , float ) ;  // 0.1 amp Batt_current_measured
float moving_Temperature_measured_fun_u(float ,float);
float moving_Temperature_measured_fun_v(float ,float);
float moving_Temperature_measured_fun_w(float ,float);
float moving_Temperature_measured_fun_M(float ,float);
float moving_Q_current_measured_fun(float  , float ) ;  // 0.1 amp Batt_current_measured
float moving_D_current_measured_fun(float  , float ) ;  // 0.1 amp Batt_current_measured



int moving_Throttle_measured_fun(int,int);
int Temperature_Fault(int);
int Temperature_Fault_Controller(int);
int BUS_Voltage_OV_UV_Fault(float);
int Motor_Stall_Fault();
float Throttle_Control(int,float,int,int);
int Rpm_Target_Function(int);
int Throttle_Switching_ON_OFF(int);
int Throttle_Fault(int);
int Current_Sensor_Fault(int , int);
int Overcurrent_Fault(int);
int Read_Throttle(int);
float PI_Loop_DC_CURRENT(float , float   , float  , float  );


#endif 
