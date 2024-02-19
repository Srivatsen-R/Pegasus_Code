#include "firmware_upgrade_app.h"
#include <stdlib.h>
#include "can_tp_app.h"
#include "memoryMap.h"
#include "main.h"
#include <string.h>
#include "main.h"
#include "flash.h"
#include "fdcan_AL.h"
#include "config.h"
#include <stdint.h>

extern motorControl_t motorControl;

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim17;

// #define APP1
upgrade_type uptype = COTA;
firmware_upgrade_states upgrade_state = UPGRADE_INIT;
firmware_upgrade_states previous_upgrade_state = UPGRADE_INIT;
volatile IsoTpShims firmware_up_recv_shim;
volatile IsoTpReceiveHandle firmware_up_recv_handle;
volatile IsoTpMessage firmware_up_recv_message;

uint8_t firmware_up_flag = 0;
uint8_t target_id = STARK_ID;

volatile flash_write_progress_struct write_progress;

uint8_t fail_mess = 0;
uint8_t success_mess = 1;
uint32_t app_address = 0;

uint16_t curr_frame = 0;
uint16_t total_frames = 0;
uint8_t firmware_up_complete = 0;
uint8_t up_pause_flag = 0;

uint8_t get_pause_flag(){
    return up_pause_flag;
}

void set_pause_flag(uint8_t val){
    up_pause_flag = val;
}



uint8_t get_complete_Flag(){
    return firmware_up_complete;
}

void set_complete_flag(uint8_t val){
    firmware_up_complete = val;
}

uint8_t response[5] = {PEGASUS_ID, STARK_ID, FUPG_STATUS, 1, PEGASUS_ID};


uint32_t crc32_for_b(uint32_t r)
{
	for (int j = 0; j < 8; ++j) r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
	return r ^ (uint32_t)0xFF000000L;
}


uint32_t calc_crc32(const void *data, uint32_t n_bytes)
{
	uint32_t crc = 0;
	static uint32_t table[0x100];
	if (!*table)
		for (size_t i = 0; i < 0x100; ++i) table[i] = crc32_for_b(i);
	for (size_t i = 0; i < n_bytes; ++i)
		crc = table[(uint8_t)crc ^ ((uint8_t *)data)[i]] ^ crc >> 8;

	return crc;
}


void Transmit_on_CAN1(uint32_t arbitration_id, TypeofCANID format, uint8_t * can_data, uint8_t dlc)
{
    _fdcan_transmit_on_can(arbitration_id, format, can_data, dlc);
}

void handle_upgrade_init()
{
    switch (uptype)
    {
    case FOTA:

#ifdef APP1
        app_address =  &__app2_start__;
        uint8_t res = clearApp2();
#endif
#ifdef APP2
        app_address =  &__app1_start__;
        uint8_t res = clearApp1();
#endif
        if(res != 1)
        {
            response[3] = FILE_CREATE_ERROR;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5);
            firmware_up_flag = 0;

            // FAILED
            upgrade_state = UPGRADE_FAILED;
            return;
        }

        response[3] = SUCCESS_MESS;
        upgrade_state = UPGRADE_RECEIVE_DATA;
        
        break;
    case COTA:
        // clear flash from that area
        Flash_Erase_Data(0x081E0000, 64);
        response[3] = SUCCESS_MESS;
        upgrade_state = UPGRADE_RECEIVE_DATA;
        break;

    default:
        upgrade_state = UPGRADE_FAILED; 
        break;
    }

    Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5); 
}

void handle_rceive_data()
{

    static uint8_t mess_buff[525] = {0};
    static uint16_t mess_len = 0;
    uint32_t words[128] = {0};
    uint32_t recv_crc = 0;
    uint32_t offset = 0;
    uint32_t computed_crc = 0;
    uint8_t message_type = 3;
    static uint16_t timeout_counter = 0;

    uint8_t timeout_count = 0;
    uint8_t retry = 0;

    uint8_t can_data[5] = {0};
    Transmit_on_CAN1(0x310, S, can_data, 5);
    

    

    if (get_interrupt_flag() == 1)
    {
        set_interrupt_flag(0);

        memcpy(&message_type, firmware_up_recv_message.payload, 1);

        switch (message_type)
        {
        case COTA_MESSAGE:
            memcpy(&recv_crc, firmware_up_recv_message.payload + 1, 4); 
            update_config();
            response[3] = SUCCESS_MESS;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5);
            break;
        
        case FOTA_MESSAGE:
            memcpy(mess_buff, firmware_up_recv_message.payload+9, firmware_up_recv_message.size-9);
            memcpy(&recv_crc, firmware_up_recv_message.payload + 1, 4); 
            memcpy(&offset, firmware_up_recv_message.payload + 5, 4);

            mess_len = firmware_up_recv_message.size-9;
            computed_crc = calc_crc32(mess_buff, mess_len);
            if(computed_crc != recv_crc)
            {
                response[3] = CRC_ERROR;
                Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5);
                firmware_up_flag = 0;

                // FAILED
                upgrade_state = UPGRADE_FAILED;
                return;
            }

            for(int word_c = 0 ; word_c < mess_len/4 ; word_c++){
                uint32_t* word = &mess_buff[word_c*4];
                words[word_c] = *word;
            }

            unsigned int address = app_address + offset;
            unsigned int ret_code; 
            ret_code = Flash_Write_Data(address, words, mess_len);

            // uint8_t can_data[5] = {0};
            // can_data[0] = ret_code;

            // Transmit_on_CAN1(0x310, S, can_data, 5);


            curr_frame++;
            write_progress.offset = offset;

            // Transmit_on_CAN1(tx_flash_write_progress, S, &write_progress, sizeof(write_progress));
            response[3] = SUCCESS_MESS;
            // HAL_Delay(100);
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5);
            timeout_counter = 0;
            break;
        
        default:
            break;
        }
    }
    else 
    {
        timeout_counter++;
        if (timeout_counter >= TIMEOUT_COUNT ){
            retry++;
            response[3] = CAN_TP_TIMEOUT;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response,5);
            timeout_counter = 0;
        }
        if (retry > MAX_RETRY) {upgrade_state = UPGRADE_FAILED; return ;}
    }

    if (get_complete_Flag() == 1)
    {
        set_complete_flag(0);
        upgrade_state = UPGRADE_COMPLETE;
        return;
    }

    if (get_pause_flag() == 1){
        set_pause_flag(0);
        upgrade_state = UPGRADE_PAUSE;
        return;
    }
    

}

void handle_upgrade_pause()
{
    previous_upgrade_state = upgrade_state;

}


void handle_upgrade_resume()
{
    response[3] = RESUME_UPGRADE;
    Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, S, response, 5);
    upgrade_state = previous_upgrade_state;
}

void handle_upgrade_complete()
{
    switch (uptype)
    {
    case FOTA:
        HAL_TIM_Base_Stop_IT(&htim7);
        switch_partition_and_reset();
        break;
    case COTA:
        HAL_NVIC_SystemReset();
        break;
    default:
        upgrade_state = UPGRADE_FAILED;
        break;
    }
}


// uint32_t value;

// bool verifyStringFromServer(char *rest, uint64_t * data, uint32_t len)
// {
//     char delimiter[2] = ",";
//     if (rest == NULL || *rest == '\0')
//     {
//         return false;
//     }
//     int i = 0;
//     while (*rest != '\0')
//     {
//         if (i == len)
//         {
//             break;
//         }
//         value = (uint32_t)atoll(strtok_r(NULL, delimiter, &rest));
        
//         data[i] = value;
//         i++;
//     }
//     if (i < len)
//     {
//         return false;
//     }
//     return true;
// }

// uint8_t dataParserCOTA(uint64_t *configData, char * server_cmd, char * delimiter)
// {
//     bmsStatus_t status;
//     unsigned int recv_crc = 0;
//     memcpy(&recv_crc, firmware_up_recv_message.payload + 1, 4); 

//     if (verifyStringFromServer(server_cmd + 5, configData, FLASH_CONFIG_TOTAL_LENGTH - 1))
//     {
        
//         status = writeDataToFlash(FLASH_COTA_UPDATE, FLASH_CONFIG_ADDRESS, configData, FLASH_CONFIG_TOTAL_LENGTH, (uint8_t *)&recv_crc);
//         return BMS_OK;
//     }
//     else
//     {
//         return BMS_COTA_UPDATE_ERROR;
//     }
// }

// void update_config(uint64_t *configData)
// {
//     bmsStatus_t status;
//     char delimiter[2] = ",";
//     status = dataParserCOTA(configData, ((char *)firmware_up_recv_message.payload), delimiter);  

//     //set flag and ackowledge to Stark here
// }
