/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with ADC.

*/



#include "adc_AL.h"

adc_t analog = {
           .start_dma    = HAL_ADC_Start_DMA,
           .stop_dma     = HAL_ADC_Stop_DMA,
           .read         = Get_ADC_Reading,
};

/**
  * @brief  Conversion complete callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {

  /* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &analog.bufferData[0], ADC_NoOfConversion);
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &analog.bufferData[ADC_NoOfConversion], ADC_NoOfConversion);
}

 /**
  * @brief  Gets and Returns adc coverted value
  * @param  channel 
  * @return double 
  */
double Get_ADC_Reading(uint8_t channel) {
      return (analog.bufferData[channel] * ((double)ADC_Vref/Adc_max_Value));
}