/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with measuring, filtering , error and fault detection for 
various physical parameters such as throttle, voltage , current , temperature.

*/

#include "motor_param.h"
#include "stdint.h"
#include "vector_control.h"
#include "foc_blockset.h"


int Fault_Status=0;
char for_rev_check=1;

extern float dc_total;

extern uint8_t acc_flag;
extern uint8_t deacc_flag;

extern foc_t foc;



unsigned int Get_Batt_Voltage_measured(unsigned int voltage_adc_count)  // 0.1 volt resolution
{
	if(voltage_adc_count > BATT_VOLT_ADC_OFFSET)
	{
		voltage_adc_count = (voltage_adc_count - BATT_VOLT_ADC_OFFSET);
	}
	else
	{
		voltage_adc_count = 0;
	}
	voltage_adc_count *= BATT_VOLT_MULTIPLY ;
	voltage_adc_count += BATT_VOLT_FINAL_OFFSET ;
	
	return (unsigned int)voltage_adc_count;
}
	




      int moving_Throttle_measured_fun(int current_val,int MOV_AVG_SAMPLE )
      {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
      }

     float moving_Temperature_measured_fun_u( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }
    float moving_Temperature_measured_fun_v( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }
    float moving_Temperature_measured_fun_w( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }
    float moving_Temperature_measured_fun_M( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }

     float moving_Batt_current_measured_fun( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }

	 float moving_Batt_voltage_measured_fun( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }	

	 float moving_AC_voltage_measured_fun( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }

    float moving_Q_current_measured_fun( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }	

    float moving_D_current_measured_fun( float current_val , float MOV_AVG_SAMPLE)   // 0.1 amp Batt_current_measured
    {
        static float Prev_current_val;
        float Bus_Current_Error_value;
        Bus_Current_Error_value = (current_val - Prev_current_val);
        Prev_current_val += (Bus_Current_Error_value / MOV_AVG_SAMPLE);
        current_val = Prev_current_val ;
        return current_val ;
    }




int Temperature_Fault_Controller(int  heatsink_T_temp)
{
    static int H_sink_temp_Hys_var;
    int Return_H_temp_fault;

    if(heatsink_T_temp >= HEAT_SINK_MAX_TEMP_LIMIT)
    {
        H_sink_temp_Hys_var = 1;
        Return_H_temp_fault = 1;
    }
    else if(heatsink_T_temp < (HEAT_SINK_MAX_TEMP_LIMIT - HEAT_SINK_TEMP_HYS))
    {
        H_sink_temp_Hys_var = 0;
        Return_H_temp_fault = 0;
    }
    else
    {
        if(H_sink_temp_Hys_var == 1) Return_H_temp_fault = 1 ;
        else Return_H_temp_fault = 0 ;
    }

    if(Return_H_temp_fault == 0)Fault_Status =0;
    else Fault_Status =1 ;

    return Return_H_temp_fault ;
}


int Temperature_Fault(int  heatsink_T_temp)
{
    static int H_sink_temp_Hys_var;
    int Return_H_temp_fault;

    if(heatsink_T_temp >= MOTOR_TEMP_LIMIT)
    {
        H_sink_temp_Hys_var = 1;
        Return_H_temp_fault = 1;
    }
    else if(heatsink_T_temp < (MOTOR_TEMP_LIMIT - HEAT_SINK_TEMP_HYS))
    {
        H_sink_temp_Hys_var = 0;
        Return_H_temp_fault = 0;
    }
    else
    {
        if(H_sink_temp_Hys_var == 1) Return_H_temp_fault = 1 ;
        else Return_H_temp_fault = 0 ;
    }

    if(Return_H_temp_fault == 0)Fault_Status =0;
    else Fault_Status =1 ;

    return Return_H_temp_fault ;
}


int BUS_Voltage_OV_UV_Fault(float measure_Vbus_temp)
   {
       static int UV_Hys_var;
       static int OV_Hys_var;
       int UV_Bus_fault;
       int OV_Bus_fault;
       int V_Bus_fault;

       /** OVER VLOTAGE FAULT**/
       if(measure_Vbus_temp >= OVER_VOLT_LIMIT)
       {
           OV_Hys_var = 1;
           OV_Bus_fault = 1;
       }
       else if(measure_Vbus_temp < (OVER_VOLT_LIMIT - OVER_VOLT_HYS))
       {
           OV_Hys_var = 0;
           OV_Bus_fault = 0;
       }
       else
       {
           if(OV_Hys_var == 1) OV_Bus_fault = 1 ;
           else OV_Bus_fault = 0 ;
       }

       if(OV_Bus_fault == 0)Fault_Status =0 ;
       else Fault_Status =2 ;

       /** UNDER VLOTAGE FAULT**/
       if(measure_Vbus_temp <= UNDER_VOLT_LIMIT)
       {
           UV_Hys_var = 1;
           UV_Bus_fault = 1;
       }
       else if(measure_Vbus_temp > (UNDER_VOLT_LIMIT + UNDER_VOLT_HYS))
       {
           UV_Hys_var = 0;
           UV_Bus_fault = 0;
       }
       else
       {
           if(UV_Hys_var == 1) UV_Bus_fault = 1 ;
           else UV_Bus_fault = 0 ;
       }

    //    if(UV_Bus_fault == 0)Fault_Status =0 ;
    //    else Fault_Status =2 ;

       if((OV_Bus_fault == 1)) V_Bus_fault = 2;
       else if(UV_Bus_fault == 1) V_Bus_fault = 1;
       else V_Bus_fault = 0 ;
       
       return V_Bus_fault;
   }


float Throttle_Control(int target_speed,float prev_output,int f_flag,int r_flag){
    
    float th_increment,th_decrement;

        if(target_speed >= prev_output)
        {
            acc_flag = 1;
            deacc_flag = 0;
      
            if(foc.speed_sense*SPEED_PU_TO_RPM<100)
            {    
                //th_increment = ACCELERATION_CONST;
                th_increment = 180.0;
                
            } 
            else
            {    
                th_increment = ACCELERATION_CONST; 
            }


            if(target_speed >= prev_output)
            {
                if(target_speed >= (prev_output + th_increment)) prev_output += th_increment ;
                else prev_output = target_speed ;
            }
        }
        else
        {
            deacc_flag = 1;
            acc_flag = 0;

            if((foc.speed_sense*SPEED_PU_TO_RPM) < 2000.0)
            {
                th_decrement = 40;
            }
            else if((foc.speed_sense*SPEED_PU_TO_RPM) >= 2000.0 && (foc.speed_sense*SPEED_PU_TO_RPM) <= 3500.0)
            {
                th_decrement = 60;
            }
            else
            {
                th_decrement =DEACCELERATION_CONST;
            }

            if(target_speed <= (prev_output - th_decrement)) prev_output -= th_decrement ;
            else prev_output = target_speed ;
        }

    return prev_output;
}


int Rpm_Target_Function(int  throttle_input)
{

    int rpm_th_temp ;

    rpm_th_temp = ((throttle_input) - THROTTLE_ADC_OFFSET_COUNT)*SPEED_MUL_FACTOR ;// 0.27~7950
    if(rpm_th_temp<0){rpm_th_temp=0;}
    if(rpm_th_temp>MAX_SPEED*SPEED_MUL_FACTOR){rpm_th_temp=MAX_SPEED*SPEED_MUL_FACTOR;}
    return ((rpm_th_temp));
}

int Throttle_Switching_ON_OFF(int  Th_val_adc_sense)
{
    static int Th_Hys_var;
    int Return_Th_selection;
    if(Th_val_adc_sense >= TH_START_LIMIT)
    {
        Th_Hys_var = 1;
        Return_Th_selection = 0;
    }
    else if(Th_val_adc_sense < (TH_START_LIMIT - TH_HYSS_LIMIT))
    {
            Th_Hys_var = 0;
            Return_Th_selection = 1;
    }
    else
    {
        if(Th_Hys_var == 1) Return_Th_selection = 1 ;
        else Return_Th_selection = 1 ;
    }
    return Return_Th_selection ;
}

int Throttle_Fault(int  Th_val_adc_sense)
{

    int error=0;
    
    if((Th_val_adc_sense <= TH_MIN_VAL) ||(Th_val_adc_sense > TH_AVG_MAX_ADC_COUNT) || (Th_val_adc_sense >= 48000))
    {
       error=1; 
    }

    return error ;

}


int Current_Sensor_Fault(int A_phase , int B_Phase)
{


return 0;

}



int Overcurrent_Fault (int Current)
{
    return 0;
}



int Read_Throttle(int adc_data)
{

    adc_data  = adc_data - 7300 ;
 
    if(adc_data<0)
    {
        adc_data=0;
    }


    return adc_data;

}


float PI_Loop_DC_CURRENT(float dc_max, float dc_input  , float speed_filtered , float ref_output )
    {
        float dc_error;
        static float dc_intg;

        dc_error = ((dc_max) - (dc_input));   //Difference is positive during ramp up and negative during ramp down

        if((dc_error < (5.0)) && (dc_error > (-5.0)))
        {
            dc_intg = (0.0) ;
        }

        if(dc_error >= (0.0))
        {
            dc_intg += (dc_error *(KI_FOR_DC_CURRENT));             //Calculate the integral term
            dc_error = ((KP_FOR_DC_CURR_UP) *dc_error);
        }
        else
        {
            dc_intg += (dc_error *(KI_FOR_DC_CURRENT));
            dc_error = ((KP_FOR_DC_CURR_DOWN)*dc_error);
        }

        dc_intg = (dc_intg >=(10.0))?(10.0):dc_intg;
        dc_intg = (dc_intg <=(-10.0))?(-10.0):dc_intg;

        dc_error += dc_intg;       //P_term + I_term is the final error
        dc_error += ref_output ;

        if(dc_error >= speed_filtered) dc_error = speed_filtered ;
        if(dc_error <= (0.0)) dc_error = (0.0) ;

        return dc_error ;
    }
