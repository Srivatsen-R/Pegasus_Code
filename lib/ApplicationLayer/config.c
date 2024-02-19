#include "config.h"
#include "flash.h"
#include "main.h"
#include <string.h>
#include "firmware_upgrade_app.h"

#define NO_OF_CONFIG_PARAMS 27

extern IsoTpMessage firmware_up_recv_message;

const uint32_t __configdata__  = 0x081E0000;

configFile config;

bool verifyStringFromServer(char *rest, uint32_t data[], uint32_t len)
{
    char delimiter[2] = ",";
	if (rest == NULL || *rest == '\0')
    {
        return false;
    }
    int i = 0;
    while (*rest != '\0')
    {
        if (i == len)
        {
            break;
        }
        data[i] = (uint32_t)atoll(strtok_r(NULL, delimiter, &rest));
        i++;
    }
    if (i < len)
    {
        return false;
    }
    return true;
}

uint8_t parse_config(configFile * received_conf, char * server_cmd, char * delimiter)
{
    uint32_t data[NO_OF_CONFIG_PARAMS];
    if (verifyStringFromServer(server_cmd, data, NO_OF_CONFIG_PARAMS))
    {
        received_conf->config_set_flag = data[0];
        received_conf->version_major = data[1];
        received_conf->version_minor = data[2];
        received_conf->version_pat = data[3];
        received_conf->ppr = data[4];
        received_conf->POLE_PAIR = data[5];
        received_conf->TOP_SPEED_FW = data[6];
        received_conf->TOP_SPEED_RW = data[7];
        received_conf->PEAK_CURR_RMS = data[8];
        received_conf->PEAK_CURR_DC = data[9];
        received_conf->ACC = data[10];
        received_conf->DEACC = data[11];
        received_conf->CON_TEMP_CUTTOFF = data[12];
        received_conf->MOTOR_TEMP_CUTOFF = data[13];
        received_conf->UNDER_VOLTAGE = data[14];
        received_conf->OVER_VOLTAGE = data[15];
        received_conf->POS_SEN_TYPE = data[16];
        received_conf->kp_iq = data[17];
        received_conf->kp_id = data[18];
        received_conf->ki_iq = data[19];
        received_conf->ki_id = data[20];
        received_conf->CONTROL_MODE = data[21];
        received_conf->STALL_IDEN_TIME = data[22];
        received_conf->MIN_THR_VAL = data[23];
        received_conf->MAX_THR_VAL = data[24];
        received_conf->file_size = data[25];
        received_conf->test_var = data[26];
        return 1;
    }
    else
    {
        return 0;
    }
}

void bootup_config(void)
{
    Flash_Read_Data(__configdata__, (uint32_t *)&config, 16);
    if (config.config_set_flag == 1)
    {
        // pass
        // variables->PPR = config.ppr;
        // variables->POLEPAIRS = config.POLE_PAIR;
        // variables->MAX_RPM = config.TOP_SPEED_FW;
    }
    else
    {
        // TODO set up a dummpy config 
        // set some values to each parameter
        config.ppr = 4096;
        config.POLE_PAIR = 3;
        config.TOP_SPEED_FW = 5000;
        config.TOP_SPEED_RW = 3000;
        config.PEAK_CURR_RMS = 250;
        config.PEAK_CURR_DC = 200;
        config.ACC = 2.1;
        config.DEACC = 0.3;
        config.CON_TEMP_CUTTOFF = 80;
        config.MOTOR_TEMP_CUTOFF = 130;
        config.UNDER_VOLTAGE = 40;
        config.OVER_VOLTAGE = 90;
        config.POS_SEN_TYPE = 2;
        config.kp_iq = 1.2;
        config.kp_id = 3.5;
        config.ki_iq = 1.2;
        config.ki_id = 3.5;
        config.CONTROL_MODE = 2;
        config.STALL_IDEN_TIME = 3;
        config.MIN_THR_VAL = 0;
        config.MAX_THR_VAL = 5000;
    }

}

void update_config()
{
    // TODO do crc check
    // configFile received_config;
    
    configFile received_conf;
    char delimiter[2] = ",";
    uint8_t res = parse_config(&received_conf, ((char *)firmware_up_recv_message.payload) + 9, delimiter);

    Flash_Write_Data(__configdata__, (uint32_t *)&received_conf, sizeof(received_conf));
    set_config_flag(0);
}
