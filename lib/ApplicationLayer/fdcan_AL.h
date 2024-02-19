#ifndef __CAN_AL_H
#define __CAN_AL_H

#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "fdcan.h"
#include "enums.h"
#include "structures.h"
#include "dr_devices.h"
#include "can_tp_app.h"
#include "firmware_upgrade_app.h"

#include "led_AL.h"
#include "sanity.h"



/* Parameters for CAN */
#define rx_controller_110       0x110
#define rx_controller_112       0x112
#define rx_controller_12A       0x12A 
#define rx_Controller_6FA       0x6FA
#define rx_Controller_704       0x704
#define tx_Controller_705       0x705
#define tx_Controller_706       0x706
#define tx_Controller_708       0x708
#define tx_Controller_710       0x710
#define tx_Controller_715       0x715
#define tx_Controller_716       0x716
#define tx_Controller_717       0x717
#define tx_Controller_724       0x724
#define tx_Controller_725       0x725
#define tx_Controller_726       0x726
#define tx_Controller_727       0x727
#define tx_Controller_7A0       0x7A0
#define rx_BMS_109              0x109 
#define FDCAN_DLC_BYTES         0x008  
#define FDCAN_TX_BUFFER_NUMBER  22 
#define rx_Controller_7FE       0x7FE
#define rx_Controller_109       0x109
#define tx_controller_6FA       0x6FA

#define FDCAN_DEBUG_ID_300     0x300
#define FDCAN_DEBUG_ID_301     0x301
#define FDCAN_DEBUG_ID_302     0x302
#define FDCAN_DEBUG_ID_303     0x303
#define FDCAN_DEBUG_ID_304     0x304
#define FDCAN_DEBUG_ID_305     0x305
#define FDCAN_DEBUG_ID_306     0x306
#define FDCAN_DEBUG_ID_307     0x307
#define FDCAN_DEBUG_ID_308     0x308


#define SCALING_FACTOR    1

typedef struct can_t {
        bool         rx_flag;
        char         verifier;
        uint8_t      BufferForMessageToTransmit[8];
        uint8_t      txMsg[FDCAN_TX_BUFFER_NUMBER][FDCAN_DLC_BYTES];
        uint8_t      rxMsg[FDCAN_DLC_BYTES];
        void         (*transmit) (uint32_t U32_transmitCANid, TypeofCANID U8_idType, uint8_t *U8_dataarr, uint8_t U8_DLC);
        void         (*write) (void);
        void         (*setup) (void);
        FDCAN_STATUS (*available) (void);
        FDCAN_RxHeaderTypeDef RxMessageBuf;
        void (*dataLoggingForPythonScript)(terminal_t terminal, float dcCurrent, uint8_t fault_status, 
                                      float motorTemperature, motorControl_t motorControllerTemperatures,
									  adc_t phaseCurrent, float encoderAState, float encoderBState, float Ododmeter, float Speed, float Trip, float current, float avg_board_temp, float v_rms);
}can_t;

void CAN_Filter_IDList(uint32_t U32_receiveCANid1, TypeofCANID U8_idType, FilterBanks_CAN2 U8_filterbank, FIFOs_CAN2 U8_fifo);
void Transmit_on_CAN(uint32_t U32_transmitCANid, TypeofCANID U8_idType, uint8_t *U8_dataarr, uint8_t U8_DLC);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_Write(void);
FDCAN_STATUS isFDCAN_Available(void);
void FDCAN_ApplicationSetup (void);
void __fdcan_transferMessagesOnID6FA(firmware_upgrade_error_codes_e message, uint8_t flag_msg);
void _fdcan_transmit_on_can(uint32_t arbitration_id, TypeofCANID format, uint8_t * can_data, uint8_t dlc);
#endif