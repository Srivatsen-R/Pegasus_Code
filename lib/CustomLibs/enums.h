#ifndef __ENUMS_H
#define __ENUMS_H

#include <stdio.h>
#include <stdbool.h>


typedef enum {
	 SANITY_ERROR = false, 
     SANITY_OK    = true,
	 SANITY_FAULT = true,
}SANITY_STATUS;

typedef enum {
     FAULT_MOTOR_TEMPERATURE 	 = 1,
	 FAULT_STALL             	 = 2,
	 FAULT_SOFTWARE_OVER_CURR 	 = 3,
	 FAULT_HARDWARE_OVER_VOLTAGE = 4,
	 FAULT_MOTOR_OUT_SHORT       = 5,
	 FAULT_PHASE_DISCONNECT      = 6,
	 FAULT_DIRECTION_ERR         = 7,
	 FAULT_UNDER_VOLTAGE	 = 8,	
	 FAULT_TEMP_LOW          = 9,
	 FAULT_SPEED_LIMIT       = 10,
	 FAULT_BOARD_TEMPERAUTRE = 11,
	 FAULT_THROTTLE          = 12,
	 FAULT_ENCODER           = 13,
	 FAULT_CURRENT_SENSE     = 14,
	 FAULT_OVER_CURRENT      = 15,
	 FAULT_FNR				 = 16,
	 FAULT_OVER_SPEED        = 17,
	 FAULT_DC_OVER_CURR      = 18,
	 FAULT_UNDER_SPEED       = 19,
	 FAULT_UNDER_SPEED_THROTTLE = 20,
	 FAULT_UNDER_SPEED_STALL = 21,
	 FAULT_ENCODER_DICNT     = 22,
	 FAULT_Z_DICNT           = 23,
	 FAULT_PWM_DICNT 		 = 24,
	 FAULT_A_B_5V_SHORT      = 25,
	 FAULT_CAN_ERROR		 = 26,
	 FAULT_ABNORMAL_RUNNING  = 27,
	 FAULT_MOTOR_LOAD        = 28,
	 FAULT_HARDWARE_OVER_CURRENT = 29,
}FAULT_STATUS;

typedef enum {
	FAULT_BUS_VOLTAGE_HEX	     = 0x00000001,
	FAULT_BOARD_TEMPERAUTRE_HEX  = 0x00000002,
	FAULT_MOTOR_TEMPERATURE_HEX  = 0x00000004,
	FAULT_THROTTLE_HEX           = 0x00000008,
	FAULT_CURRENT_SENSE_HEX      = 0x00000010,
	FAULT_OVER_CURRENT_HEX       = 0x00000020,
	FAULT_OVER_SPEED_HEX         = 0x00000040,
	FAULT_DC_OVER_CURR_HEX       = 0x00000080,
	FAULT_ENCODER_DICNT_HEX      = 0x00000100,
	FAULT_Z_DICNT_HEX            = 0x00000200,
	FAULT_PWM_DICNT_HEX          = 0x00000400,
	FAULT_A_B_5V_SHORT_HEX       = 0x00000800,
	FAULT_UNDER_SPEED_STALL_HEX  = 0x00001000,
	FAULT_CAN_ERROR_HEX			 = 0x00002000,
	FAULT_ABNORMAL_RUNNING_HEX   = 0x00004000,
	FAULT_HARDWARE_OVER_CURRENT_HEX  = 0x00008000
}FAULT_STATUS_HEX;

/*CAN Enumerators*/
typedef enum {
	E 	= 0,
	S	= 1,
}TypeofCANID;

typedef enum{
	FIFO0_CAN1 = 0,
	FIFO1_CAN1 = 1,
}FIFOs_CAN1;

typedef enum {
	FBANK0		= 0,
	FBANK1		= 1,
	FBANK2		= 2,
	FBANK3		= 3,
	FBANK4		= 4,
	FBANK5		= 5,
	FBANK6		= 6,
	FBANK7		= 7,
	FBANK8		= 8,
	FBANK9		= 9,
	FBANK10		= 10,
	FBANK11		= 11,
	FBANK12		= 12,
	FBANK13		= 13,
}FilterBanks_CAN1;

typedef enum{
	FIFO0_CAN2	= 0,
	FIFO1_CAN2	= 1,
}FIFOs_CAN2;

typedef enum{
	FDCAN_NOT_AVAILABLE	= false,
	FDCAN_AVAILABLE	    = true,
}FDCAN_STATUS;

typedef enum{
	TEMP_NORMAL	    = false,
	TEMP_NOT_NORMAL	= true,
}TEMP_STATUS;

typedef enum {
     EEPROM_OK      = 0,
	 EEPROM_ERROR   = 1,
	 EEPROM_BUSY    = 2,
	 EEPROM_TIMEOUT = 3,
}EEPROM_STATUS;

typedef enum{
	FBANK14		= 14,
	FBANK15		= 15,
	FBANK16		= 16,
	FBANK17		= 17,
	FBANK18		= 18,
	FBANK19		= 19,
	FBANK20		= 20,
	FBANK21		= 21,
	FBANK22		= 22,
	FBANK23		= 23,
	FBANK24		= 24,
	FBANK25		= 25,
	FBANK26		= 26,
	FBANK27		= 27,
}FilterBanks_CAN2;


typedef enum{
	OFF		= 0,
	ON		= 1,
}PinStateType;


typedef enum {
     PHASE_CURRENT_W   = 0,
     BOARD_TEMP_U      = 1,
     BOARD_TEMP_W      = 2,
     MOTOR_TEMP_SENSE  = 3,
     BOARD_TEMP_V      = 4,
     BUS_CURRENT_W     = 5,
     BUS_VOLTAGE       = 6,
     BUS_CURRENT_U     = 7, 
     BUS_CURRENT_V     = 8,
     PHASE_CURRENT_V   = 9,
	 THROTTLE		   = 10,	
}ADC_Channels_t;

typedef enum {
      PWM_W_Channel =  TIM_CHANNEL_1,
      PWM_V_Channel =  TIM_CHANNEL_2,
      PWM_U_Channel =  TIM_CHANNEL_3,
}TIM_Channels_t;

typedef enum {
      DRIVE_ENABLE  = 0,
      DRIVE_DISABLE = 1,
}DRIVE_STATUS;

#endif
