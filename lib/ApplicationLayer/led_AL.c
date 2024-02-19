/*
Pegasus - Motor Controller Unit software for HL,HLP,Storm,Vortex and future projects.

Contributors: Pujit Gandhi,Pratik Rout,Olay,Saksham

This file contains functions associated with GPIO read/write.

*/

#include "led_AL.h"


gpio_t io = {
            .fnr.neutral = true,
            .fnr.forward = false,
            .fnr.reverse = false,
            .read   = HAL_GPIO_ReadPin,
            .toggle = HAL_GPIO_TogglePin,
            .write  = HAL_GPIO_WritePin,     
};

led_t led = {
      .toggle = toggleColor,
      .write  = writeColor,
};

void run_disco(void) {
               writeColor(BLACK);
               HAL_Delay(500);
               writeColor(GREEN);
               HAL_Delay(500);
               writeColor(BLUE);
               HAL_Delay(500);
               writeColor(RED);
               HAL_Delay(500);
               writeColor(CYAN);
               HAL_Delay(500);
               writeColor(PURPLE); 
               HAL_Delay(500);
               writeColor(YELLOW);
               HAL_Delay(500);
               writeColor(WHITE);
               HAL_Delay(500);
}

void writeColor(uint16_t color) {
     switch (color) {
            case BLACK:  
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case GREEN:  
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, RESET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case BLUE:   
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, RESET);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case RED:     
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);     
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, RESET);
                           break;
            case CYAN:
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, RESET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, RESET);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case PURPLE:   
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, RESET);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, RESET);
                           break;
            case YELLOW:  
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, RESET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);   
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, RESET);
                           break;
            case WHITE:   
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, RESET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, RESET);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, RESET);
                           break;
            default: 
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
     }
}

void toggleColor(uint16_t color) {
     switch (color) {
            case BLACK:  
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case GREEN:  
                           io.toggle(LED_G_GPIO_Port, LED_G_Pin);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case BLUE:   
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.toggle(LED_B_GPIO_Port, LED_B_Pin);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case RED:     
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);     
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.toggle(LED_R_GPIO_Port, LED_R_Pin);
                           break;
            case CYAN:
                           io.toggle(LED_G_GPIO_Port, LED_G_Pin);
                           io.toggle(LED_B_GPIO_Port, LED_B_Pin);  
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
            case PURPLE:   
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.toggle(LED_B_GPIO_Port, LED_B_Pin);  
                           io.toggle(LED_R_GPIO_Port, LED_R_Pin);
                           break;
            case YELLOW:  
                           io.toggle(LED_G_GPIO_Port, LED_G_Pin);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);   
                           io.toggle(LED_R_GPIO_Port, LED_R_Pin);
                           break;
            case WHITE:   
                           io.toggle(LED_G_GPIO_Port,  LED_G_Pin);
                           io.toggle(LED_B_GPIO_Port,  LED_B_Pin);  
                           io.toggle(LED_R_GPIO_Port,  LED_R_Pin);
                           break;
            default: 
                           io.write(LED_G_GPIO_Port,  LED_G_Pin, SET);
                           io.write(LED_B_GPIO_Port,  LED_B_Pin, SET);    
                           io.write(LED_R_GPIO_Port,  LED_R_Pin, SET);
                           break;
     }
}