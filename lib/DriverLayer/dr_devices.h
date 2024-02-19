#ifndef __DR_DEVICES_H
#define __DR_DEVICES_H

#include <stdbool.h>

#include "main.h"
#include "enums.h"
#include "macros.h"
#include "structures.h"


typedef struct uart_t {
       uint8_t            bufferData[UART_BUFFER_SIZE];
       UART_HandleTypeDef *uart1, *uart2;
       void (*print)      (UART_HandleTypeDef *huart, const char* format, ...);
       HAL_StatusTypeDef  (*write) (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
       HAL_StatusTypeDef  (*read)  (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
       HAL_StatusTypeDef  (*write_interrupt) (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
       HAL_StatusTypeDef  (*read_interrupt) (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
       HAL_StatusTypeDef  (*write_dma) (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
       HAL_StatusTypeDef  (*read_dma)  (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
       HAL_StatusTypeDef  (*pause_dma) (UART_HandleTypeDef *huart);
       HAL_StatusTypeDef  (*resume_dma)(UART_HandleTypeDef *huart);
       HAL_StatusTypeDef  (*stop_dma)  (UART_HandleTypeDef *huart);
}uart_t;

typedef struct adc_t {
       uint16_t          Ta, motor_temp;
       __IO uint16_t     bufferData[ADC_NoOfConversion]; 
       double            vbus, throttle, brake_pot;
       double (*read)    (uint8_t channel);
       struct voltage_t  voltage;
       HAL_StatusTypeDef (*start) (ADC_HandleTypeDef *hadc);
       HAL_StatusTypeDef (*stop)  (ADC_HandleTypeDef *hadc);
       HAL_StatusTypeDef (*start_dma) (ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
       HAL_StatusTypeDef (*stop_dma)  (ADC_HandleTypeDef *hadc);
}adc_t;

typedef struct gpio_t {  
       fnr_state_t    fnr;            
       GPIO_PinState  (*read) (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
       void (*write)  (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
       void (*toggle) (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
}gpio_t;

typedef struct timers_t {
        bool              motor_frequency;
        HAL_StatusTypeDef (*start) (TIM_HandleTypeDef *htim, uint32_t Channel);
        HAL_StatusTypeDef (*start_it) (TIM_HandleTypeDef *htim, uint32_t Channel);
        HAL_StatusTypeDef (*stop_it) (TIM_HandleTypeDef *htim, uint32_t Channel);
        HAL_StatusTypeDef (*stop)  (TIM_HandleTypeDef *htim, uint32_t Channel);
        HAL_StatusTypeDef (*base_stop) (TIM_HandleTypeDef *htim);
        HAL_StatusTypeDef (*base_start) (TIM_HandleTypeDef *htim);
        HAL_StatusTypeDef (*base_stop_int) (TIM_HandleTypeDef *htim);
        HAL_StatusTypeDef (*base_start_int) (TIM_HandleTypeDef *htim);
        void              (*elapsed_callback) (TIM_HandleTypeDef *htim);
}timers_t;


typedef struct eeprom_t {
               uint8_t   pData [12];         /* Pointer to data buffer */
               uint8_t   size;               /* Amount of data to be sent or read */
               uint16_t  devAddress;         /* Target device address: The device 7 bits address value
                                                in datasheet must be shifted to the left before calling the interface */
               uint16_t  memAddress;         /* Internal memory address */
               I2C_HandleTypeDef* eepromI2C; /* Pointer to a I2C_HandleTypeDef structure that contains
                                                the configuration information for the specified I2C. */
              EEPROM_STATUS status;
              EEPROM_STATUS (*write) (I2C_HandleTypeDef* hi2c, uint16_t devAddress, uint16_t memAddress, uint8_t  *pData); 
              EEPROM_STATUS (*read)  (I2C_HandleTypeDef* hi2c, uint16_t devAddress, uint16_t memAddress, uint8_t  *pData, uint16_t size);

}eeprom_t;



typedef struct motor_t {
        int      overtemp_verifier,
                 rst_temp_verifier;
        int16_t  (*temperature) (void);
        float    torque;

}motor_t;

#endif