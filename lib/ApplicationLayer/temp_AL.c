/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with temperature sensing for Motor.

*/

#include "temp_AL.h"
#include "math_func.h"
#include "adc_AL.h"
#include "sanity.h"
#include "structures.h"

extern adc_t analog;
motorControl_t mControl = {
              .temperature.Rp   = 10000.0,         // Zero Power Resistance - 10kΩ
              .temperature.R1   = 100000.0,        // R1 from voltage divider - 100kΩ
              .temperature.T2   = 273.15+25.0,     // temp in kelvin at 25°C
              .temperature.Bx   = 4037.0,          // B constant at norminal temperature
              .temperature.Ka   = 273.15,          // temp at 0°C
              .temperature.read = NTC_Board_Temp,
              .temperature.overtemp_verifier = 0,
              .temperature.rst_temp_verifier = 0,
};
temperature_t temp =  {
               .isOverTemp = IsOvertemp,
};
motor_t motor = {
        .overtemp_verifier = 0,
        .temperature       = KTY84150_Motor_Temp,
};
/************************** Motor Temperature Reading ************************************/
const int16_t KTY84150_POINTS = 36;
const int16_t KTY84150_Ta_Rt[36][3] = {
      {-40, 359,  386},  {-30, 391,  419},  {-20, 424,  455},   {-10, 460,  492},  {0,   498,  532},   {10,  538,  574},  {20,  581,  618},  {25,  603,  641},  {30,  626,  665},
      {40,  672,  713},  {50,  722,  764},  {60,  773,  817},   {70,  826,  872},  {80,  882,  929},   {90,  940,  989},  {100, 1000, 1050}, {110, 1062, 1117}, {120, 1127, 1187},
      {130, 1194, 1259}, {140, 1262, 1334}, {150, 1334, 1412},  {160, 1407, 1492}, {170, 1482, 1574},  {180, 1560, 1659}, {190, 1640, 1747}, {200, 1722, 1837}, {210, 1807, 1931},
      {220, 1893, 2026}, {230, 1982, 2125}, {240, 2073, 2226},  {250, 2166, 2329}, {260, 2261, 2436},  {270, 2357, 2543}, {280, 2452, 2650}, {290, 2542, 2751}, {300, 2624, 2844},
};
int16_t KTY84150_Motor_Temp(void) {
        uint16_t motor_temp_value = analog.bufferData[MOTOR_TEMP_SENSE];
        double Rx  = (motor_temp_value * Re)/((Adc_max_Value-1)-motor_temp_value);  // Get Rx value
        Rx         = (((Rx + 1)-KTY_ADC_MIN)*KTY_m)+KTY_c;
        if(Rx < KTY84150_Ta_Rt[0][1]) return  KTY84150_Ta_Rt[0][0];
        else if(Rx > KTY84150_Ta_Rt[KTY84150_POINTS-1][1]) return  KTY84150_Ta_Rt[KTY84150_POINTS-1][0];
        else {
             int16_t findTable, tableBottom = 0, tableTop = KTY84150_POINTS;
             double Ta;
             double y, m, c;
             for (findTable = (KTY84150_POINTS/2); (tableTop-tableBottom) != 1;) {
                       if(Rx <  KTY84150_Ta_Rt[findTable][1]) {
                             tableTop  = findTable;
                             findTable = (tableTop + tableBottom)/2;
                       }
                       else if(Rx > KTY84150_Ta_Rt[findTable][1]) {
                              tableBottom = findTable;
                              findTable   = (tableTop + tableBottom)/2;
                       }
                       else {
                            /* implementing linear algebra = y = mx+c */
                            y  = KTY84150_Ta_Rt[findTable][2] - KTY84150_Ta_Rt[findTable][1];
                            m  = (KTY84150_Ta_Rt[findTable][0] + 9) - KTY84150_Ta_Rt[findTable][0];
                            m  = y/m;
                            c  =  KTY84150_Ta_Rt[findTable][1] - (KTY84150_Ta_Rt[findTable][0] * m);
                            Ta = (y + c)/m;
                            return (int16_t) Ta;
                       }
             }
              /* implementing linear algebra = y = mx+c */
              y  = KTY84150_Ta_Rt[findTable][2] - KTY84150_Ta_Rt[findTable][1];
              m  = (KTY84150_Ta_Rt[findTable][0] + 9) - KTY84150_Ta_Rt[findTable][0];
              m  = y/m;
              c  = KTY84150_Ta_Rt[findTable][1] - (KTY84150_Ta_Rt[findTable][0] * m);
              y  = Rx;
              Ta = (y - c)/m;
              return (int16_t) Ta;
        }
}
/********************************************************************************************/
/*************************** Board Temp Measurement *****************************************/
double NTC_Board_Temp(double ntc) {
          double temp_resistance,
                 temp_value,
                 temp_ln;
          // temp_resistance = (ntc * 100000) / (ADC_Vref - ntc);
          temp_resistance = (ntc * 10000) / (ADC_Vref - ntc);
          // temp_ln = log(temp_resistance);
          temp_ln=(298)*log(10000/temp_resistance);
          /*  Steinhart-Hart Thermistor Equation:
              Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
              where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8
          */
     //     temp_value = ( 1 / ( 0.001129148 + ( 0.000234125 * temp_ln ) + ( 0.0000000876741 * temp_ln * temp_ln * temp_ln ))); /* Temperature in Kelvin */
          temp_value=(4080*(273+25))/(4080-temp_ln);
          temp_value = temp_value - mControl.temperature.Ka; /* Temperature in degree Celsius */
          return temp_value;
}
/***********************************************************************************************/
/**
 * @brief Returns Motor and Board temperature status
 *
 * @return true
 * @return false
 */
TEMP_STATUS IsOvertemp (void) {
     bool overtemp = false;
     if(mControl.temperature.u >= MAX_BOARD_TEMP || mControl.temperature.v >= MAX_BOARD_TEMP || mControl.temperature.w >= MAX_BOARD_TEMP) {
          if (mControl.temperature.overtemp_verifier++ > BOARD_VERIFIER) {
                       overtemp = true;
                       mControl.temperature.rst_temp_verifier = 0,
                       mControl.temperature.overtemp_verifier-= 1;
          }
     }
     else if(mControl.temperature.u < RST_BOARD_TEMP || mControl.temperature.v < RST_BOARD_TEMP || mControl.temperature.w < RST_BOARD_TEMP) {
          if (mControl.temperature.rst_temp_verifier++ > BOARD_VERIFIER) {
                       overtemp = false;
                       mControl.temperature.overtemp_verifier = 0;
                       mControl.temperature.rst_temp_verifier-= 1;
          }
     }
     else if(analog.motor_temp >= MAX_MOTOR_TEMP) {
          if (motor.overtemp_verifier++ > MOTOR_VERIFIER) {
                       overtemp = true;
                       motor.rst_temp_verifier = 0;
                       motor.overtemp_verifier-= 1;
          }
     }
      else if(analog.motor_temp < RST_MOTOR_TEMP) {
          if (mControl.temperature.rst_temp_verifier++ > MOTOR_VERIFIER) {
                       overtemp = false;
                       motor.overtemp_verifier = 0;
                       motor.rst_temp_verifier-= 1;
          }
     }
     return overtemp;
}