/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with sanity checks for vehicle.

*/

#include <stdlib.h>
#include <math.h>
#include "math_func.h"
#include "vehicle.h"
#include "motor_param.h"
#include "vector_control.h"
#include "sanity.h"
#include "adc_AL.h"
#include "dr_devices.h"

#define FDCAN_ERROR_BIT (0x4000A400 + 0x0044)

sanity_t fault = {
        .boardTemperature_u = 0,
        .boardTemperature_v = 0,
        .boardTemperature_w = 0,
        .motorTemperature = 0,
        .motorStall       = 0,
        .busVoltage       = 0,
        .throttle         = 0,
        .currentSensor    = 0,
        .encoder          = 0,
        .overCurrent      = 0,
        .throttleStarting = 0,
        .status           = 0,
        .generic          = 0,
        .fault_code       = 0x00000000,
        .run_sanity = RUN_SANITY
};

motorControl_t motorControl = {
        .temperature.u      = 0.0,
        .temperature.v      = 0.0,
        .temperature.w      = 0.0,
        .temperature.motor  = 0.0,
        .encoder.errorCount = 0,
        .encoder.previous   = 0,
        .encoder.value      = 0,
        .drive.check        = 0,
        .drive.status       = 0,
        .drive.fnr_status  =  0,
       
};

extern adc_t          analog;
extern terminal_t     terminal;
extern motorControl_t mControl;

__IO float dc_current = 0.0; 
extern __IO float speed_filtered;
float motorT=0;
float busVoltage=0;
float avg_board_temp=0;
float v_rms=0;

extern float encoder_a_state;
extern float encoder_b_state;
extern float Duty;
extern int counter_current_ms;
extern int counter_encoder_ms;
extern int counter_abnormal_run;
extern float throttle_adc_voltage;

extern int reverse_pin_state;
extern int forward_pin_state;

extern float freq_rpm;
extern float throttle_percent;

extern int count_duty;
extern int forward_flag;
extern int reverse_flag;


void RUN_SANITY(void)
{

    int sanity_val=0;

    for (sanity_val=0;sanity_val<SANITY_COUNT;sanity_val++){ANALOG_READING();}

    HAL_Delay(1000);
    
    FAULT_READING();

     // Current sensor W
    if(analog.bufferData[PHASE_CURRENT_W]>=CURRENT_SENSOR_MAX || analog.bufferData[PHASE_CURRENT_W]<=CURRENT_SENSOR_MIN)
    {
         fault.fault_code |= FAULT_CURRENT_SENSE_HEX;
         motorControl.drive.check = DRIVE_DISABLE;
         fault.status             = FAULT_CURRENT_SENSE;
    }

   // Current sensor U   
   if(analog.bufferData[PHASE_CURRENT_V]>=CURRENT_SENSOR_MAX || analog.bufferData[PHASE_CURRENT_V]<=CURRENT_SENSOR_MIN)
    {
         fault.fault_code |= FAULT_CURRENT_SENSE_HEX;
         motorControl.drive.check = DRIVE_DISABLE;
         fault.status             = FAULT_CURRENT_SENSE;
    }

   //Drive not neutral while sanity.
   // reverse_pin_state = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3);
   // forward_pin_state = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4);

   //  if(forward_pin_state==0 || reverse_pin_state==0)
   //  {
   //     motorControl.drive.check      = DRIVE_DISABLE;
   //     fault.status                  = FAULT_FNR;
   //  }

    //If encoder not connected   
    //Encoder_Check(3);  

}


void FAULT_READING()
{
      // motor_stall_error = Motor_Stall_Fault();
      fault.boardTemperature_u = Temperature_Fault_Controller((motorControl.temperature.u));
      fault.boardTemperature_v = Temperature_Fault_Controller((motorControl.temperature.v));
      fault.boardTemperature_w = Temperature_Fault_Controller((motorControl.temperature.w));
      // fault.motorTemperature = Temperature_Fault(motorControl.temperature.motor);
      fault.busVoltage       = BUS_Voltage_OV_UV_Fault(busVoltage);
      fault.throttle         = Throttle_Fault(moving_Throttle_measured_fun(analog.bufferData[THROTTLE],THROTTLE_AVG));
      fault.currentSensor    = Current_Sensor_Fault(analog.bufferData[PHASE_CURRENT_W],analog.bufferData[PHASE_CURRENT_V]);
      fault.overCurrent      = Overcurrent_Fault(analog.bufferData[BUS_CURRENT_U]+analog.bufferData[BUS_CURRENT_V]+analog.bufferData[BUS_CURRENT_W]);

      //controller temperature fault
      // if((fault.boardTemperature_u == SANITY_FAULT)||(fault.boardTemperature_v == SANITY_FAULT)||(fault.boardTemperature_w == SANITY_FAULT) ) {
      //    fault.fault_code |= FAULT_BOARD_TEMPERAUTRE_HEX;
      //    motorControl.drive.check = DRIVE_DISABLE;
      //    fault.status             = FAULT_BOARD_TEMPERAUTRE;
      // }

      // //motor temperature fault
      // if(fault.motorTemperature == SANITY_FAULT) {
      //    fault.fault_code |= FAULT_MOTOR_TEMPERATURE_HEX;
      //    motorControl.drive.check = DRIVE_DISABLE;
      //    fault.status             = FAULT_MOTOR_TEMPERATURE;
      // }

      // //throttle fault
      // if(fault.throttle == SANITY_FAULT) {
      //    fault.fault_code |= FAULT_THROTTLE_HEX;
      //    motorControl.drive.check = DRIVE_DISABLE;
      //    fault.status             = FAULT_THROTTLE;
      // }

      // //can error fault - to be tested.
      // if((FDCAN_ERROR_BIT & 0x0000001) == 1 && (FDCAN_ERROR_BIT & 0x000001) == 0){
      //   fault.fault_code |= FAULT_CAN_ERROR_HEX;
      //   motorControl.drive.check = DRIVE_DISABLE;
      //   fault.status = FAULT_CAN_ERROR;
      // }

      // //over speed error fault.
      // if(terminal.w.sen >= 7000.0){
      //   fault.fault_code |= FAULT_OVER_SPEED_HEX;
      //   motorControl.drive.check = DRIVE_DISABLE;
      //   fault.status = FAULT_OVER_SPEED;
      // }

      // if(terminal.w.sen >= 5500.0){
      //   fault.fault_code |= FAULT_OVER_SPEED_HEX;
      //   // motorControl.drive.check = DRIVE_DISABLE;
      //   fault.status = FAULT_SPEED_LIMIT;
      // }

}

void ANALOG_READING()
{
     float current=0; 
     float dcV=0;

     //controller temperature 
     motorControl.temperature.u     = moving_Temperature_measured_fun_u(mControl.temperature.read(analog.read(BOARD_TEMP_U)),TEMP_AVG);
     motorControl.temperature.v     = moving_Temperature_measured_fun_v(mControl.temperature.read(analog.read(BOARD_TEMP_V)),TEMP_AVG);
     motorControl.temperature.w     = moving_Temperature_measured_fun_w(mControl.temperature.read(analog.read(BOARD_TEMP_W)),TEMP_AVG);
     avg_board_temp = (motorControl.temperature.u + motorControl.temperature.v + motorControl.temperature.w)/3.0;

     //motor temperature
     double resistance = (analog.bufferData[MOTOR_TEMP_SENSE] * 10000)/(Adc_max_COUNT - analog.bufferData[MOTOR_TEMP_SENSE]);
     double temp_K = resistance/NTC_PULL_UP_RESISTOR;
     temp_K = log(temp_K);
     temp_K /= BETA_VALUE;
     temp_K += 1.0/(T_AMBIENT + KELVIN_TO_CELSIUS);
     temp_K = 1.0/temp_K;
     temp_K -= KELVIN_TO_CELSIUS;
     motorControl.temperature.motor = moving_Temperature_measured_fun_M(temp_K, TEMP_AVG);
     motorT = motorControl.temperature.motor;

     //bus voltage
     dcV = moving_Batt_voltage_measured_fun(0.00211*analog.bufferData[BUS_VOLTAGE] +VBUS_OFFSET,VOLTAGE_AVG);
     busVoltage = dcV; 
     terminal.rotor.angle = busVoltage;
   
     //ac phase voltage    
     v_rms = 10*sqrt(terminal.vd.ref * terminal.vd.ref + terminal.vq.ref * terminal.vq.ref);
     v_rms /= 32767.0;
     v_rms *= busVoltage/ROOT2;
     v_rms = moving_AC_voltage_measured_fun(v_rms,VOLTAGE_AVG);

     //dc current  
     current = (10*sqrt(terminal.vq.ref * terminal.vq.ref + terminal.vd.ref * terminal.vd.ref));
     current /= 32767.0;
     if(terminal.id.ref < 0.0){ current *= terminal.id.ref * -1.0;}
     else if(terminal.id.ref >= 0.0 && terminal.id.ref <= 40.0){current *= terminal.id.ref;}
     else if(terminal.id.sen > 40.0){current *= terminal.id.sen * -1.0;}
     dc_current = current;

   //   if(terminal.iq.sen<=0)
   //   {
   //   current = (10*sqrt(terminal.vq.ref * terminal.vq.ref + terminal.vd.ref * terminal.vd.ref));
   //   current /= 32767.0;
   //   current *= sqrt(terminal.iq.sen*terminal.iq.sen + terminal.id.sen*terminal.id.sen);
   //   dc_current = current;
   //   dc_current = -moving_Batt_current_measured_fun(dc_current,VOLTAGE_AVG);
   //   }

   //    if(terminal.iq.sen>0)
   //   {
   //   current = (10*sqrt(terminal.vq.ref * terminal.vq.ref + terminal.vd.ref * terminal.vd.ref));
   //   current /= 32767.0;
   //   current *= sqrt(terminal.iq.sen*terminal.iq.sen + terminal.id.sen*terminal.id.sen);
   //   dc_current = current;
   //   dc_current = moving_Batt_current_measured_fun(dc_current,VOLTAGE_AVG);
   //   }
 
     throttle_adc_voltage = 2.0 * moving_Throttle_measured_fun(analog.bufferData[THROTTLE],THROTTLE_AVG) * (3.3/65535.0);
     //motor frequency 
     freq_rpm = terminal.w.sen * (POLEPAIRS*RPM_TO_FREQ);
     //throttle percentage
     throttle_percent = (terminal.w.ref*RPM_TO_THROTTLE_PERCENT);
   
}

void Encoder_Check(int test_case) {

   //If throttle given but drive doesn't move for 3 sec
   if(test_case==1)
   {
     if(speed_filtered > THROTTLE_ENCODER_THRESHOLD) {
          if(motorControl.encoder.value == motorControl.encoder.previous) {
              if(motorControl.encoder.errorCount++ >= ENCODER_TIMEOUT_LIMIT) {
                 motorControl.drive.check      = DRIVE_DISABLE;
                 fault.status                  = FAULT_ENCODER;
                 motorControl.encoder.previous = ENCODER_TIMEOUT_LIMIT;         
              }
          }
          else {
              motorControl.encoder.errorCount = 0;
          }
          motorControl.encoder.previous = motorControl.encoder.value;
      }
   }

   //Disconnection during drive
   if(test_case==2){Encoder_Fault(encoder_a_state,encoder_b_state);}

   //Not connected during sanity itself.
   if(test_case==3)
   {
      
      if(count_duty==0 )// to be edited, Duty maybe zero!!!
      {
             motorControl.drive.check      = DRIVE_DISABLE;
             fault.status                  = FAULT_ENCODER;
      }
   }


 
}

int Encoder_Fault(int a,int b) {
  int encoder_retValue;
  encoder_t *encoder = (encoder_t*)malloc(sizeof(encoder_t));
  
  while(encoder->check == 0) {
      if((a==1) && (b==1)) {
          if(encoder->errorCount++ > 25000) {
             encoder->timeoutCount++;
             encoder->errorCount  = 0;
          }
          if(encoder->timeoutCount > 3) {
             encoder->timeoutCount = 3;
             encoder->errorValue   = 1;
             encoder->state        = 1;
          }
      }
      else {
          encoder->errorCount    = 0;
          encoder->timeoutCount  = 0;
          encoder->errorValue    = 0;
          encoder->state         = 1;
      }
  }
     encoder_retValue = encoder->errorValue;
     free(encoder);
     return encoder_retValue;
}




void SAFETY_AND_ERRORS()
{
         static uint8_t count_spike;
         static uint16_t time_count_iq;
         static uint8_t fault_iq_count;
         static uint8_t peak_curr;

         //if drive command >1000 and motor doesn't move for 3 sec   
         //Encoder_Check(1);

         // if torque current sense is greater than 270A and speed less than 500 rpm 10 sec
      //   if(terminal.iq.sen >= 130.0 && terminal.w.sen <= 100.0){

      //     time_count_iq++;
      //     if(time_count_iq >= 20000){
      //       time_count_iq = 0;
      //       fault_iq_count++;
      //     }

      //     if(fault_iq_count >= 6){
      //       fault_iq_count = 0;
      //       time_count_iq = 0;
      //       fault.fault_code |= FAULT_UNDER_SPEED_STALL_HEX;
      //       motorControl.drive.check = DRIVE_DISABLE;
      //       fault.status = FAULT_STALL;
      //     }
      //   }

      //  if(terminal.iq.sen >= 300.0){
      //    peak_curr++;
      //    if(peak_curr >= 60000){
      //       peak_curr = 0;
      //       fault.fault_code |= FAULT_HARDWARE_OVER_CURRENT_HEX;
      //       motorControl.drive.check = DRIVE_DISABLE;
      //       fault.status = FAULT_HARDWARE_OVER_CURRENT;
      //    }
      //  }

      //   //if peak 200A computed. 
      //   if(dc_current >= 210.0){
      //     fault.fault_code |= FAULT_DC_OVER_CURR_HEX;
      //     motorControl.drive.check = DRIVE_DISABLE;
      //     fault.status = FAULT_DC_OVER_CURR;
      //   }

      // //overvoltage/undervoltage fault
      // if(busVoltage < UNDER_VOLT_LIMIT) {
      //    fault.fault_code |= FAULT_BUS_VOLTAGE_HEX;
      //    motorControl.drive.check = DRIVE_DISABLE;
      //    fault.status             = FAULT_UNDER_VOLTAGE;
      // }

      // if(busVoltage > OVER_VOLT_LIMIT) {
      //    fault.fault_code |= FAULT_BUS_VOLTAGE_HEX;
      //    motorControl.drive.check = DRIVE_DISABLE;
      //    fault.status             = FAULT_HARDWARE_OVER_VOLTAGE;
      // }
}
