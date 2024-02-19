#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

#define bool _Bool


typedef struct __attribute__((packed, aligned(1))) {
       uint8_t config_set_flag;
       uint8_t version_major;
       uint8_t version_minor;
       uint8_t version_pat;
       uint16_t ppr;
       uint8_t POLE_PAIR;
       uint16_t TOP_SPEED_FW;
       uint16_t TOP_SPEED_RW;
       uint16_t PEAK_CURR_RMS;
       uint8_t PEAK_CURR_DC;
       uint8_t ACC;
       uint8_t DEACC;
       uint32_t CON_TEMP_CUTTOFF;
       uint32_t MOTOR_TEMP_CUTOFF;
       uint32_t UNDER_VOLTAGE;
       uint32_t OVER_VOLTAGE;
       uint8_t POS_SEN_TYPE;
       uint32_t kp_iq;
       uint32_t kp_id;
       uint32_t ki_iq;
       uint32_t ki_id;
       uint8_t CONTROL_MODE;
       uint8_t STALL_IDEN_TIME;
       uint32_t MIN_THR_VAL;
       uint32_t MAX_THR_VAL;
       uint8_t file_size;
       uint32_t test_var; 
}configFile;

extern configFile config;

void bootup_config();

void update_config();

bool verifyStringFromServer(char *rest, uint32_t data[], uint32_t len);

uint8_t parse_config(configFile * received_conf, char * server_cmd, char * delimiter);


#endif
