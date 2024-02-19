/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with vehicle such as throttle, FNR , CAN communication.

*/



#include <math.h>
#include "math_func.h"
#include "vehicle.h"
#include "motor_param.h"
#include "vector_control.h"
#include "fdcan_AL.h"
#include "foc_blockset.h"


vehicle_t vehicle = {

           .odometer=0.0,
           .speed=0.0,
           .trip=0.0,
           .odo_change_status=0.0,
           .calculate_ots = Calculate_OTS
                    };

extern can_t can;
extern terminal_t terminal;
extern __IO float dc_current;
extern sanity_t fault;
extern float motorT;
extern motorControl_t motorControl;
extern adc_t analog;
extern float Duty;
extern volatile uint32_t z_trig;
extern uint8_t start_flag;

static float kmph_can;
static float trip_can;
static uint32_t odo_can;

extern float avg_board_temp ;
extern float v_rms ;
extern float throttle_percent ;
float throttle_adc_voltage=0;
extern float busVoltage;
extern float freq_rpm;
int forward_flag=0;
int reverse_flag=0;
int neutral_flag=0;
float encoder_a_state=0;
float encoder_b_state=0;


int reverse_pin_state=1;
int forward_pin_state=1;
int throttle_enable_flag=0;
float ref_output=0;

__IO int a_current                = 0,
         b_current                = 0,
         throttle_input           = 0,
         target_speed_reference   = 0;

__IO float speed_filtered         = 0;         

extern int count_duty;

void  Calculate_OTS(uint32_t wheel_rpm)
{

  static int odo_prev;

  vehicle.speed = wheel_rpm*RPM_TO_KMPH;
  vehicle.trip = vehicle.trip +(vehicle.speed*HOUR_TO_MS_100)/2.0;
  vehicle.odometer = vehicle.odometer + (vehicle.speed*HOUR_TO_MS_100)/2.0;
  if((vehicle.odometer-odo_prev)>=1.0)
  {
    vehicle.odo_change_status = ODO_UPDATE;

  }
  odo_prev = vehicle.odometer;
}


void CAN_Logging()
{

    terminal.iq.sen = moving_Q_current_measured_fun(terminal.iq.sen,10);
    terminal.id.sen = moving_D_current_measured_fun(terminal.id.sen,10);
  //Logging CAN data.
    can.dataLoggingForPythonScript(terminal, 
                                  dc_current, 
                                  fault.status, 
                                  motorT, 
                                  motorControl, 
                                  analog, 
                                  count_duty,
                                  z_trig, 
                                  (float)odo_can, 
                                  (float)kmph_can, 
                                  (float)trip_can, 
                                  (float)fault.fault_code, 
                                  avg_board_temp, 
                                  v_rms);
                                  //angle, (float)odo_can, (float)kmph_can, (float)trip_can, (float)fault.fault_code, avg_board_temp, v_rms);
    HAL_Delay(10);
}


void CAN_Communication(float odo, float trip, float kmph)
{
  
      // odo = 54321;//0.1 scaling
      // trip = 200;//no scaling
      // kmph = 365;//0.1 scaling

      kmph_can = kmph*KMPH_CAN_SCALING;
      odo_can = odo*ODO_CAN_SCALING;
      trip_can =trip;

  
      // 705
      can.txMsg[0][0] = (uint8_t) (motorT + 50.0);
      can.txMsg[0][1] = (uint8_t) (avg_board_temp + 50.0);
      if(dc_current < 0.0) {can.txMsg[0][2] = (uint8_t) ((dc_current/200.0) * 100.0);}
      else {can.txMsg[0][2] = 0;}
      can.txMsg[0][4] = ((uint32_t) odo_can & 0xFF);
      can.txMsg[0][5] = ((uint32_t) odo_can)>>8;
      can.txMsg[0][6] = ((uint32_t) odo_can)>>16;
      can.txMsg[0][7] = ((uint32_t) odo_can)>>24;

      // 706
      if(forward_flag){can.txMsg[1][0] = 0x02;}
      else if(reverse_flag){can.txMsg[1][0] = 0x04;}
      else if(neutral_flag){can.txMsg[1][0] = 0x01;}

      if(dc_current >= 0.0){can.txMsg[1][1] &= ~(1 << 2);}
      else if(dc_current < 0.0){can.txMsg[1][1] |= (1 << 2);}

      if(terminal.rotor.angle < 44.0){can.txMsg[1][1] |= (1 << 3);}
      else{can.txMsg[1][1] &= ~(1 << 3);}

      if(start_flag){can.txMsg[1][1] |= (1 << 5);}
      else{can.txMsg[1][1] &= ~(1 << 5);}

      can.txMsg[1][2] = 0x00;

      if(fault.status == FAULT_OVER_SPEED){can.txMsg[1][3] |= (1 << 1);}
      else{can.txMsg[1][3] &= ~(1 << 1);}
      if(fault.status == FAULT_MOTOR_LOAD){can.txMsg[1][3] |= (1 << 2);}
      else{can.txMsg[1][3] &= ~(1 << 2);}
      if(fault.status == FAULT_MOTOR_TEMPERATURE){can.txMsg[1][3] |= (1 << 3);}
      else{can.txMsg[1][3] &= ~(1 << 3);}
      if(busVoltage > 75.0){can.txMsg[1][3] |= (1 << 4);}
      else if(busVoltage >= 48.0 && busVoltage <= 75.0){can.txMsg[1][3] &= ~(1 << 4);}
      if(busVoltage < 48.0){can.txMsg[1][3] |= (1 << 5);}
      else if(busVoltage >= 48.0 && busVoltage <= 75.0){can.txMsg[1][3] &= ~(1 << 5);}
      if(fault.status == FAULT_TEMP_LOW){can.txMsg[1][3] |= (1 << 6);}
      else{can.txMsg[1][3] &= ~(1 << 6);}

      can.txMsg[1][4] = ((uint16_t) (kmph_can*1.1));
      can.txMsg[1][5] = ((uint16_t) (kmph_can*1.1)>>8);

      can.txMsg[1][6] = ((uint16_t) (trip_can));
      can.txMsg[1][7] = ((uint16_t) (trip_can)>>8);

      // 708
      if(fault.status >= FAULT_MOTOR_TEMPERATURE && fault.status <= FAULT_DIRECTION_ERR){can.txMsg[2][0] |= (1 << fault.status);}
      if(fault.status == FAULT_ENCODER){can.txMsg[2][1] |= (1 << 1);}
      else{can.txMsg[2][1] &= ~(1 << 1);}
      if(fault.status == FAULT_HARDWARE_OVER_VOLTAGE){can.txMsg[2][1] |= (1 << 2);}
      else{can.txMsg[2][1] &= ~(1 << 2);}

      //710
      can.txMsg[3][0] = (uint8_t)((int)throttle_percent);
      can.txMsg[3][3] = (uint8_t)(throttle_adc_voltage*THROTTLE_CAN_ADC_SCALING);

      //715
      can.txMsg[4][4] = (uint8_t)busVoltage;
      can.txMsg[4][6] = (uint8_t)((int)terminal.w.sen & 0x00FF);
      can.txMsg[4][7] = (uint8_t)(((int)terminal.w.sen & 0xFF00) >> 8);

      //716
      can.txMsg[5][6] = (uint8_t)((int)terminal.iq.sen & 0x00FF);
      can.txMsg[5][7] = (uint8_t)(((int)terminal.iq.sen & 0xFF00) >> 8);

      //717
      can.txMsg[6][0] = (uint8_t)((int)freq_rpm & 0x00FF);
      can.txMsg[6][1] = (uint8_t)(((int)freq_rpm & 0xFF00) >> 8);

      //724
      can.txMsg[7][0] = (uint8_t)(motorControl.temperature.u + 50.0);
      can.txMsg[7][1] = (uint8_t)(motorControl.temperature.v + 50.0);
      can.txMsg[7][2] = (uint8_t)(motorControl.temperature.w + 50.0);
      can.txMsg[7][3] = (uint8_t)(v_rms);

      //726
      can.txMsg[8][0] = 0x6E;
      can.txMsg[8][1] = 0xE1;
      can.txMsg[8][2] = 0x04;
      can.txMsg[8][3] = 0x04;
      can.txMsg[8][4] = 0x01;
      can.txMsg[8][5] = 0x00;
      can.txMsg[8][6] = 0x03;
      can.txMsg[8][7] = 0x01;

      //7A0
      can.txMsg[9][0] = 0x03;
      can.txMsg[9][1] = 0x01;
      can.txMsg[9][2] = 0x00;
      can.txMsg[9][3] = 0x03;
      #if APP1
        can.txMsg[9][7] = 0x01;
      #endif
      #if APP2
        can.txMsg[9][7] = 0x02;
      #endif

      can.write();

      HAL_Delay(10);

}


void READ_FNR()
{
        reverse_pin_state = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4);
        forward_pin_state = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3);

        //if speed less than 100 rpm
          if(abs((int)terminal.w.sen) <= 100){


          if(forward_pin_state==FW_DIRECTION)
          {
            forward_flag = 1;
            reverse_flag = 0;
            neutral_flag = 0;
            throttle_enable_flag = 1;
            motorControl.drive.fnr_status=1;
            
          }

          if(reverse_pin_state==RW_DIRECTION)
          {
            reverse_flag = 1;
            forward_flag = 0;
            neutral_flag = 0;
            throttle_enable_flag = 1;
            motorControl.drive.fnr_status=2;
            
          }

          if(reverse_pin_state == 1 && forward_pin_state == 1){
            reverse_flag = 0;
            forward_flag = 0;
            neutral_flag = 1;
            throttle_enable_flag = 0; 
          }
        }

        //if speed greater than 100rpm
        if(reverse_pin_state == 1 && forward_pin_state == 1){
            
            reverse_flag = 0;
            forward_flag = 0;
            neutral_flag = 1;
            throttle_enable_flag = 0; 
          }

}
void READ_THROTTLE()
{
        //if(throttle_enable_flag){throttle_input    = moving_Throttle_measured_fun(Read_Throttle(analog.bufferData[THROTTLE]),1);}
        if(reverse_pin_state == 1 && forward_pin_state == 1)
        {
          target_speed_reference -= 5000;
          if(target_speed_reference <= 0){target_speed_reference = 0;}

        }
        else if(forward_pin_state == 0 && motorControl.drive.fnr_status==1) 
        {
          throttle_input    = moving_Throttle_measured_fun(Read_Throttle(analog.bufferData[THROTTLE]),1);
          target_speed_reference  = Rpm_Target_Function(throttle_input);    
        }
        else if(reverse_pin_state == 0 && motorControl.drive.fnr_status==2)
        {
          throttle_input    = moving_Throttle_measured_fun(Read_Throttle(analog.bufferData[THROTTLE]),1);
          target_speed_reference  = Rpm_Target_Function(throttle_input); 
        }


        if(forward_pin_state == 0 && motorControl.drive.fnr_status==2)
        {
          target_speed_reference -= 5000;
          if(target_speed_reference <= 0){target_speed_reference = 0;} 
        }
        else if (reverse_pin_state == 0 && motorControl.drive.fnr_status==1)
        {
          target_speed_reference -= 5000;
          if(target_speed_reference <= 0){target_speed_reference = 0;} 
        }
        
        // if(target_speed_reference<1000.0)
        // {
        //   // motorControl.drive.check == DRIVE_DISABLE;
        //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
        // }

        // else if (target_speed_reference >=1000.0 && fault.status==0)
        // {
        //   motorControl.drive.check == DRIVE_ENABLE;
        //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
        // }
        
        speed_filtered = Throttle_Control(target_speed_reference,speed_filtered,forward_flag,reverse_flag);                  
        ref_output = speed_filtered;

}

void READ_MOTOR_PHASE_CURRENT()
{

    int temp_a=0;
    int temp_b=0;
    int count=0;
  
    for (count=0;count<20;count++)
    {
            temp_a  = temp_a + ((analog.bufferData[PHASE_CURRENT_W]/DIV_FACTOR_CURRENT) - OFFSET_CURRENT);
            temp_b  = temp_b + ((analog.bufferData[PHASE_CURRENT_V]/DIV_FACTOR_CURRENT) - OFFSET_CURRENT);
    }    

    a_current = temp_a/CURRENT_AVG_FACTOR;
    b_current = temp_b/CURRENT_AVG_FACTOR;

    temp_a=0;
    temp_b=0;

}
void READ_MOTOR_POSITION()
{
  motorControl.encoder.value =  TIM2->CNT;   // Encoder Count 
  encoder_a_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
  encoder_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}





