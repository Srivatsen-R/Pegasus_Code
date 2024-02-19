/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with CAN/Logging.

*/

#include "fdcan_AL.h"
#include "sanity.h"
#include "can_tp_app.h"

void FDCAN_dataLoggingForPythonScript(terminal_t, float, uint8_t, float, motorControl_t, adc_t, float, float, float, float, float, float, float, float);

extern gpio_t  io;
extern terminal_t terminal;
extern motorControl_t motorControl;
extern motor_t motor;
float SOC_Data = 0.0, DCLI = 0.0, DCLO = 0.0;
float Pack_V = 0.0, Pack_Ah = 0.0, Pack_C = 0.0;
extern float avg_board_temp;
extern float v_rms;

extern IsoTpShims firmware_up_recv_shim;
extern IsoTpReceiveHandle firmware_up_recv_handle;
extern IsoTpMessage firmware_up_recv_message;

#define FUPFG_REQUEST 1
#define FUPFG_STATUS  0

#define PEGASUS_ID    3
#define STARK_ID 	  0

extern TIM_HandleTypeDef htim7;

can_t can = {
          .txMsg     = {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},
		  .rx_flag   = false,
		  .verifier  = 0,
		  .write     = CAN_Write,
		  .transmit  = Transmit_on_CAN,
		  .available = isFDCAN_Available, 
		  .setup     = FDCAN_ApplicationSetup,
		  .dataLoggingForPythonScript = FDCAN_dataLoggingForPythonScript,
	
};

/* Function to Transmit CAN Ids on CAN1 Port
 *
 * Returns:
 * 		Void (Check Error Handler for Error Messages if any)
 *
 * Parameters:
 * 		U32_transmitCANid: 11-bit or 29-bit CAN ID for Transmitting
 * 		U8_idType: If U8_idType == 0, then Extended ID is transmitted. Else Standard ID.
 * 		U8_dataarr: CAN Data Bytes for transmission (Range: data_arr[0] to data_arr[7])
 * 		U8_DLC: Data Length Count (Range: 0 to 8). No CAN-FD support by CAN Transceiver.
 * */
void Transmit_on_CAN(uint32_t U32_transmitCANid, TypeofCANID U8_idType, uint8_t *U8_dataarr, uint8_t U8_DLC) {
	FDCAN_TxHeaderTypeDef	TxMsg;
    
	if(U8_idType == E)      TxMsg.IdType = FDCAN_EXTENDED_ID;
	else if(U8_idType == S) TxMsg.IdType = FDCAN_STANDARD_ID;

	TxMsg.TxFrameType           = FDCAN_DATA_FRAME;
	TxMsg.Identifier            = U32_transmitCANid;
    TxMsg.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    TxMsg.BitRateSwitch         = FDCAN_BRS_OFF;
    TxMsg.FDFormat              = FDCAN_CLASSIC_CAN;
    TxMsg.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    TxMsg.MessageMarker         = 0;
	
	switch(U8_DLC) { 
		case 0:  TxMsg.DataLength  = FDCAN_DLC_BYTES_0; break;  /*!< 0 bytes data field  */
		case 1:  TxMsg.DataLength  = FDCAN_DLC_BYTES_1; break;  /*!< 1 bytes data field  */
		case 2:  TxMsg.DataLength  = FDCAN_DLC_BYTES_2; break;  /*!< 2 bytes data field  */
		case 3:  TxMsg.DataLength  = FDCAN_DLC_BYTES_3; break;  /*!< 3 bytes data field  */
		case 4:  TxMsg.DataLength  = FDCAN_DLC_BYTES_4; break;  /*!< 4 bytes data field  */
		case 5:  TxMsg.DataLength  = FDCAN_DLC_BYTES_5; break;  /*!< 5 bytes data field  */
		case 6:  TxMsg.DataLength  = FDCAN_DLC_BYTES_6; break;  /*!< 6 bytes data field  */
		case 7:  TxMsg.DataLength  = FDCAN_DLC_BYTES_7; break;  /*!< 7 bytes data field  */
		case 8:  TxMsg.DataLength  = FDCAN_DLC_BYTES_8; break;  /*!< 8 bytes data field  */
		default: TxMsg.DataLength  = FDCAN_DLC_BYTES_0; break;  /*!< 0 bytes data field  */
	}

    
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxMsg, U8_dataarr) != HAL_OK) 
	   __HAL_FDCAN_CLEAR_FLAG(&hfdcan2, FDCAN_TXBC_TFQM);

}









/* Function to configure Hardware CAN Filters by ID Filtering method. Function configured for CAN1 only.
 *
 * Returns:
 * 		Void (Check Error Handler for Error Messages if any)
 *
 * Parameters:
 * 		U32_receiveCANid1: 11-bit or 29-bit CAN ID for filtering (First ID)
 * 		U32_receiveCANid2: 11-bit or 29-bit CAN ID for filtering (Second ID)
 * 		U8_idType: If U8_idType == 0, then Extended ID is configured. Else Standard ID. Both IDs have to be of same type for the filter to work.
 * 		U8_filterbank: Selection of Filter Bank (Range: 0 to 13).  No overlap of filters should be there from ID Masking techniques if used.
 * 		U8_fifo: If U8_fifo = 0, FIFO0 is selected. If U8_fifo = 1, FIFO1 is selected.
 * 		FunctionalState: ENABLE to initialize and start filtering. DISABLE to deinit the filter and stop filtering.
 * */

void CAN_Filter_IDList(uint32_t U32_receiveCANid, TypeofCANID U8_idType, FilterBanks_CAN2 U8_filterbank, FIFOs_CAN2 U8_fifo) {
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterIndex  = U8_filterbank;

    if(U8_idType == E) {
		sFilterConfig.IdType    = FDCAN_EXTENDED_ID;
		sFilterConfig.FilterID1 = (U32_receiveCANid & 0x1FFFE000)>>13;
		sFilterConfig.FilterID2 = ((U32_receiveCANid & 0x00001FFF)<<3)|0x04;
	}
	else if(U8_idType == S) {
		sFilterConfig.IdType    = FDCAN_STANDARD_ID;
		sFilterConfig.FilterID1 = U32_receiveCANid;
		sFilterConfig.FilterID2 = 0x7FF;
	}

	if(U8_fifo == FIFO0_CAN2) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }
	else if(U8_fifo == FIFO1_CAN2) {
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	}

	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		//Error Handler Code to be added later to act as redundancy.
		Error_Handler();
	}

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* Configure Rx FIFO 0 watermark to 2 */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 6);

    /* Activate Rx FIFO 0 watermark notification */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}




void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) 
{

	    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &can.RxMessageBuf, can.rxMsg);
        can.rx_flag = true;
		if(can.RxMessageBuf.Identifier == rx_Controller_6FA){
			if(can.rxMsg[1] == PEGASUS_ID){
				if ( (can.rxMsg[2] == FUPFG_REQUEST) && ( can.rxMsg[3] == RECEIVE_CONF))
				{
					uptype = COTA;
					upgrade_state = UPGRADE_INIT;
					motorControl.drive.check = DRIVE_DISABLE;
					HAL_TIM_Base_Start_IT(&htim7);
				}
				else if ( (can.rxMsg[2] == FUPFG_REQUEST) && ( can.rxMsg[3] == PERFORM_UPGRADE) )
				{
					set_complete_flag(1);
				}
				else if ( (can.rxMsg[2] == FUPFG_REQUEST) && ( can.rxMsg[3] == PAUSE_UPGRADE) )
				{
					set_pause_flag(1);
				}
				else if ( (can.rxMsg[2] == FUPFG_REQUEST) && ( can.rxMsg[3] == RESUME_UPGRADE) )
				{
					upgrade_state = UPGRADE_RESUME;
					HAL_TIM_Base_Start_IT(&htim7);
				}
				else if ( (can.rxMsg[2] == FUPFG_REQUEST) && ( can.rxMsg[3] == RECEIVE_BIN) )
				{
					uptype = FOTA;
					upgrade_state = UPGRADE_INIT;
					motorControl.drive.check = DRIVE_DISABLE;
					HAL_TIM_Base_Start_IT(&htim7);
				}
			}
		}

		if(can.RxMessageBuf.Identifier == rx_Controller_7FE)
		{
			CAN_TP_Receive_interrupt(rx_Controller_7FE, (uint8_t *)can.rxMsg, can.RxMessageBuf.DataLength >> 16, &firmware_up_recv_shim, &firmware_up_recv_message, &firmware_up_recv_handle);
		}

		if(can.RxMessageBuf.Identifier == rx_Controller_109){
			SOC_Data = (float)(((can.rxMsg[1]<<8) | can.rxMsg[0]) * 0.01);
			Pack_V = (float)((can.rxMsg[7]<<8 | can.rxMsg[6]) * 0.1);
		}

		if(can.RxMessageBuf.Identifier == rx_controller_12A){
			DCLI = (float)(((can.rxMsg[1]<<8 | can.rxMsg[0])) * 0.1);
			DCLO = (float)(((can.rxMsg[3]<<8 | can.rxMsg[2])) * 0.1);
		}

		if(can.RxMessageBuf.Identifier == rx_controller_112){
			Pack_Ah = (float)((can.rxMsg[7]<<8 | can.rxMsg[6]) * 0.1);
		}

		if(can.RxMessageBuf.Identifier == rx_controller_110){
			Pack_C = (float)((can.rxMsg[7] << 24 | can.rxMsg[6] << 16 | can.rxMsg[5] << 8 | can.rxMsg[0]) * 0.01);
		}
}

void CAN_Write(void) {
     can.transmit(tx_Controller_705, S, can.txMsg[0],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_706, S, can.txMsg[1],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_708, S, can.txMsg[2],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_710, S, can.txMsg[3],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_715, S, can.txMsg[4],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_716, S, can.txMsg[5],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_717, S, can.txMsg[6],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_724, S, can.txMsg[7],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_726, S, can.txMsg[8],  FDCAN_DLC_BYTES);
	 can.transmit(tx_Controller_7A0, S, can.txMsg[9],  FDCAN_DLC_BYTES);

}


FDCAN_STATUS isFDCAN_Available(void) {
	         FDCAN_STATUS returnVal;

			 returnVal   = can.rx_flag;
			 can.rx_flag = false;
			 return returnVal;
}

void FDCAN_ApplicationSetup (void) {

  /* Configure Rx filter */
  CAN_Filter_IDList(rx_Controller_6FA, S, FBANK0, FIFO0_CAN2);
  CAN_Filter_IDList(rx_Controller_7FE, S, FBANK1, FIFO0_CAN2);
  CAN_Filter_IDList(rx_Controller_109, S, FBANK2, FIFO0_CAN2);
  CAN_Filter_IDList(rx_controller_12A, S, FBANK3, FIFO0_CAN2);
  CAN_Filter_IDList(rx_controller_112, S, FBANK4, FIFO0_CAN2);
  CAN_Filter_IDList(rx_controller_110, S, FBANK5, FIFO0_CAN2);

  /* Start the FDCAN module */
  if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
	  Error_Handler();
  };
}

static void __motorController_calibrateTerminalReading(terminal_t *terminal) {
     terminal->id.ref += 1000;
     terminal->id.sen += 1000;
	 terminal->iq.ref += 1000;
	 terminal->iq.sen += 1000;
	 terminal->w.sen +=10000;
	 terminal->vd.ref +=32767;
	 terminal->vq.ref +=32767;
	 terminal->rotor.angle *=1.0;
	 terminal->speed.slip += 10000;
     terminal->speed.sync+=10000;
	 terminal->w.ref += 10000;
}

static uint16_t __fdcan_addScalingFactorToMessage(float scaleResult) {
	   return (scaleResult * SCALING_FACTOR);
}

static void __fdcan_transferMessagesOnID300(terminal_t terminal) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(terminal.w.ref) & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(terminal.w.ref) & 0xFF00) >> 8 ;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(terminal.w.sen) & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(terminal.w.sen) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(terminal.id.ref) & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(terminal.id.ref) & 0xFF00) >> 8;
       __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(terminal.id.sen ) & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(terminal.id.sen) & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_300, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}


static void __fdcan_transferMessagesOnID301(terminal_t terminal) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(terminal.iq.ref)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(terminal.iq.ref) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(terminal.iq.sen)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(terminal.iq.sen) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(terminal.vd.ref)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(terminal.vd.ref) & 0xFF00) >> 8;
       __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(terminal.vq.ref)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(terminal.vq.ref) & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_301, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID302(terminal_t terminal, float dcCurrent, float motorTemperature) {
	   dcCurrent += 1000.0;
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(dcCurrent)             & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(dcCurrent)            & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(terminal.w.sen)        & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(terminal.w.sen)       & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(terminal.rotor.angle)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(terminal.rotor.angle) & 0xFF00) >> 8;
       __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(motorTemperature)      & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(motorTemperature)     & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_302, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID303(motorControl_t motorController, char fault_status) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(motorController.temperature.u)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(motorController.temperature.u) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(motorController.temperature.v)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(motorController.temperature.v) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(motorController.temperature.w)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(motorController.temperature.w) & 0xFF00) >> 8;
       __fdcan->BufferForMessageToTransmit[6] = (uint8_t)fault_status; 
	   __fdcan->BufferForMessageToTransmit[7] = 0;
	   __fdcan->transmit(FDCAN_DEBUG_ID_303, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID304(adc_t phaseCurrent, motorControl_t motorController) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   
	   __fdcan->BufferForMessageToTransmit[0] =  phaseCurrent.bufferData[PHASE_CURRENT_W] & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (phaseCurrent.bufferData[PHASE_CURRENT_W] & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] =  phaseCurrent.bufferData[PHASE_CURRENT_V] & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (phaseCurrent.bufferData[PHASE_CURRENT_V] & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] =  motorController.encoder.value            & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (motorController.encoder.value            & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[6] = ((uint32_t)Pack_C & 0x00FF);
	   __fdcan->BufferForMessageToTransmit[7] = ((uint32_t)Pack_C & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_304, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID305(terminal_t terminal, float encoderAState, float encoderBState) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(terminal.speed.slip)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(terminal.speed.slip) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(terminal.speed.sync)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(terminal.speed.sync) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(encoderAState)        & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(encoderAState)       & 0xFF00) >> 8;
       __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(encoderBState)        & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(encoderBState)       & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_305, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID306(float Odometer, float Speed, float Trip, float current) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(Odometer)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(Odometer) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(Speed)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(Speed) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(Trip)        & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(Trip)       & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(current)        & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(current)       & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_306, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

static void __fdcan_transferMessagesOnID307(float avg_board_temp, float v_rms, float SOC) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(Pack_V * Pack_Ah)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[1] = (__fdcan_addScalingFactorToMessage(Pack_V * Pack_Ah) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(DCLO)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[3] = (__fdcan_addScalingFactorToMessage(DCLO) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(SOC)  & 0x00FF; 
	   __fdcan->BufferForMessageToTransmit[5] = (__fdcan_addScalingFactorToMessage(SOC) & 0xFF00) >> 8;
	   __fdcan->BufferForMessageToTransmit[6] = __fdcan_addScalingFactorToMessage(DCLI) & 0x00FF;
	   __fdcan->BufferForMessageToTransmit[7] = (__fdcan_addScalingFactorToMessage(DCLI) & 0xFF00) >> 8;
	   __fdcan->transmit(FDCAN_DEBUG_ID_307, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

void __fdcan_transferMessagesOnID6FA(firmware_upgrade_error_codes_e message, uint8_t flag_msg) {
	   can_t *__fdcan = malloc(sizeof(can_t));
       
	   __fdcan->transmit = can.transmit;
	   __fdcan->BufferForMessageToTransmit[0] = __fdcan_addScalingFactorToMessage(PEGASUS_ID); 
	   __fdcan->BufferForMessageToTransmit[1] = __fdcan_addScalingFactorToMessage(STARK_ID);
	   __fdcan->BufferForMessageToTransmit[2] = __fdcan_addScalingFactorToMessage(FUPFG_STATUS); 
	   __fdcan->BufferForMessageToTransmit[3] = __fdcan_addScalingFactorToMessage((float)message);
	   __fdcan->BufferForMessageToTransmit[4] = __fdcan_addScalingFactorToMessage(PEGASUS_ID);
	   __fdcan->BufferForMessageToTransmit[5] = 0;
       __fdcan->BufferForMessageToTransmit[6] = 0; 
	   __fdcan->BufferForMessageToTransmit[7] = __fdcan_addScalingFactorToMessage(flag_msg);
	   __fdcan->transmit(tx_controller_6FA, S, __fdcan->BufferForMessageToTransmit, 8);

	   free(__fdcan);
}

void _fdcan_transmit_on_can(uint32_t arbitration_id, TypeofCANID format, uint8_t * can_data, uint8_t dlc){
	can_t *__fdcan = malloc(sizeof(can_t));
	__fdcan->transmit = can.transmit;
	for (uint8_t i = 0; i<dlc; i++)
	{
		__fdcan->BufferForMessageToTransmit[i] = __fdcan_addScalingFactorToMessage(can_data[i]); 
	}
	__fdcan->transmit(arbitration_id, format, __fdcan->BufferForMessageToTransmit, dlc);
	free(__fdcan);
}

void FDCAN_dataLoggingForPythonScript(terminal_t terminal, float dcCurrent, uint8_t fault_status, 
                                      float motorTemperature, motorControl_t motorControllerTemperatures,
									  adc_t phaseCurrent, float encoderAState, float encoderBState, float Odometer, float Speed, float Trip, float current, float avg_board_temp, float v_rms) {
    //  dcCurrent *= 0.044;
     __motorController_calibrateTerminalReading(&terminal);
	 __fdcan_transferMessagesOnID300(terminal);
	 __fdcan_transferMessagesOnID301(terminal);
	 __fdcan_transferMessagesOnID302(terminal, dcCurrent, motorTemperature);
     __fdcan_transferMessagesOnID303(motorControllerTemperatures, fault_status);
	 __fdcan_transferMessagesOnID304(phaseCurrent, motorControllerTemperatures);
	 __fdcan_transferMessagesOnID305(terminal, encoderAState, encoderBState);
	 __fdcan_transferMessagesOnID306(Odometer, Speed, Trip, current);
	 __fdcan_transferMessagesOnID307(avg_board_temp,v_rms,SOC_Data);
}
