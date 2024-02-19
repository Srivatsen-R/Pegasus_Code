
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint8_t get_conifg_flag();
void set_config_flag(uint8_t value);
void set_interrupt_flag(uint8_t value);
uint8_t get_interrupt_flag();
void switch_partition_and_reset();

/* Private defines -----------------------------------------------------------*/
#define Ibus_U_Pin                GPIO_PIN_0
#define Ibus_U_GPIO_Port          GPIOA
#define Ibus_V_Pin                GPIO_PIN_1
#define Ibus_V_GPIO_Port          GPIOA
#define Ibus_W_Pin                GPIO_PIN_2
#define Ibus_W_GPIO_Port          GPIOA
#define Vbus_sense_Pin            GPIO_PIN_3
#define Vbus_sense_GPIO_Port      GPIOA
#define UCurrent_sensor_Pin       GPIO_PIN_4
#define UCurrent_sensor_GPIO_Port GPIOA
#define VCurrent_sensor_Pin       GPIO_PIN_5
#define VCurrent_sensor_GPIO_Port GPIOA
#define WCurrent_sensor_Pin       GPIO_PIN_6
#define WCurrent_sensor_GPIO_Port GPIOA
#define Utemp_sensor_Pin          GPIO_PIN_4
#define Utemp_sensor_GPIO_Port    GPIOC
#define motor_temp_Pin            GPIO_PIN_5
#define motor_temp_GPIO_Port      GPIOC
#define Vtemp_sensor_Pin          GPIO_PIN_0
#define Vtemp_sensor_GPIO_Port    GPIOB
#define Wtemp_sensor_Pin          GPIO_PIN_1
#define Wtemp_sensor_GPIO_Port    GPIOB
#define brake_pot_Pin             GPIO_PIN_11
#define brake_pot_GPIO_Port       GPIOF
#define throttle_pot_Pin          GPIO_PIN_12
#define throttle_pot_GPIO_Port    GPIOF
#define Hs_W_N_Pin                GPIO_PIN_8
#define Hs_W_N_GPIO_Port          GPIOE
#define Hs_W_Pin                  GPIO_PIN_9
#define Hs_W_GPIO_Port            GPIOE
#define Hs_V_N_Pin                GPIO_PIN_10
#define Hs_V_N_GPIO_Port          GPIOE
#define Hs_V_Pin                  GPIO_PIN_11
#define Hs_V_GPIO_Port            GPIOE
#define Hs_U_N_Pin                GPIO_PIN_12
#define Hs_U_N_GPIO_Port          GPIOE
#define Hs_U_Pin                  GPIO_PIN_13
#define Hs_U_GPIO_Port            GPIOE
#define serial_tx_Pin             GPIO_PIN_14
#define serial_tx_GPIO_Port       GPIOB
#define serial_rx_Pin             GPIO_PIN_15
#define serial_rx_GPIO_Port       GPIOB
#define LED_R_Pin                 GPIO_PIN_10
#define LED_R_GPIO_Port           GPIOD
#define LED_G_Pin                 GPIO_PIN_11
#define LED_G_GPIO_Port           GPIOD
#define LED_B_Pin                 GPIO_PIN_12
#define LED_B_GPIO_Port           GPIOD
#define DRV1_Pin                  GPIO_PIN_13
#define DRV1_GPIO_Port            GPIOD
#define DRV2_Pin                  GPIO_PIN_14
#define DRV2_GPIO_Port            GPIOD
#define DRV3_Pin                  GPIO_PIN_15
#define DRV3_GPIO_Port            GPIOD
#define FWD_Pin                   GPIO_PIN_0
#define FWD_GPIO_Port             GPIOD
#define BKW_Pin                   GPIO_PIN_1
#define BKW_GPIO_Port             GPIOD
#define SW4_Pin                   GPIO_PIN_2
#define SW4_GPIO_Port             GPIOD
#define SW5_Pin                   GPIO_PIN_3
#define SW5_GPIO_Port             GPIOD
#define SW6_Pin                   GPIO_PIN_4
#define SW6_GPIO_Port             GPIOD
#define SW7_Pin                   GPIO_PIN_5
#define SW7_GPIO_Port             GPIOD
#define SW8_Pin                   GPIO_PIN_6
#define SW8_GPIO_Port             GPIOD
#define SW16_Pin                  GPIO_PIN_7
#define SW16_GPIO_Port            GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
