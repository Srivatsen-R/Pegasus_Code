#ifndef __MACROS_H
#define __MACROS_H

#define CARRIER_FREQUENCY  (int16_t) 16000    // 16kHz carrrier frequency
#define MAX_DUTY_CYCLE     (int16_t) 1000-1   // 999 duty cycle pulse value at 3.3V
#define NOMINAL_FREQUENCY  (int16_t) 90       // Nominal Frequency at 90Hz
#define MAX_MOTOR_FREQ     (int16_t) 200      // 200Hz  max motor frequency
#define ADC_NoOfConversion (int16_t) 11       // Number of ADC Channels
#define Adc_max_Value      (uint16_t) 65535   // 16bit conversion
#define MAX_INPUT_VOLTAGE  (double) 3.3       // Max MCU ADC reference voltage
#define ADC_Vref           (double) 3.3

/* Parameters for Sine Wave table generation*/
#define TOTAL_ANGLE         (int16_t) 180    // total PWM angle is 180° 
#define PHASE_ANGLE         (int16_t) 120    // phase angle of motor
#define RAD_TO_DEGREES      (double) (PI/TOTAL_ANGLE) // converts radian to degree
   
 /* Parameters for battery pack voltage measurement */
#define Vb_R2              (int16_t) 10   // R2 resistance for voltage divider
                                          // subject to changes as per hardware design
#define Vb_R1              (int16_t) 370  // R1 resistance for voltage divider
                                          // subject to changes as per hardware design
#define MIN_VOLTAGE        (float) 33.00
#define MAX_VOLTAGE        (float) 60.00


/* Parameters for KTY150/130 Temperature coefficient sensor */
#define Re                      (int16_t) 1000   // exemplary resistance
#define KTY_ADC_MIN             (int16_t) 359    // minimum temp resistance
#define KTY_Adc_max             (int16_t) 2624   // maximum temp resistance
#define KTY_POINTS              (int16_t) 36     
#define KTY_m                   (double)  1.202229299  
#define KTY_c                   (double)  357.797770701    
#define MOTOR_VERIFIER          (int16_t) 5
#define RST_MOTOR_TEMP          (int16_t) 80
#define MAX_MOTOR_TEMP          (int16_t) 120          


/* Parameters for Motor Monitoring */
#define BOARD_VERIFIER       5   
#define MAX_BOARD_TEMP       65.0    
#define RST_BOARD_TEMP       50.0    



/* Parameters for Motor Speed and Torque */
#define MAX_Torque          82    
#define MIN_Torque          0x00  
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               

  


/* Parameters for UART */
#define UART_PRINT_TIMER   15
#define UART_BUFFER_SIZE   512 
#define UART_TIMEOUT       500 

/* Parameters for motor status */ 
#define FNR                 4     
#define NEUTRAL             0    


// #define serial1            &huart1     // pointer to uart1 handle
#define OK                 HAL_OK


/* Parameters for Colors selection */
#define BLACK             0U 
#define GREEN             1U
#define BLUE              2U
#define RED               3U
#define CYAN              4U
#define PURPLE            5U
#define YELLOW            6U
#define WHITE             7U

#define LED_GREEN_WORKING_TIMER     5
#define LED_PURPLE_OVERTEMP_TIMER   5



/* Parameters for Motor Control */
#define Numbers_Of_Poles      4 
#define ENCODER_TIMEOUT       300     
   
/* Debug Parameter */

#define DEBUG_NOT_IN_USE      0
#define DEBUG_IN_USE          1

#define CODE_NOT_IN_USE       0
#define CODE_IN_USE           1




#endif 