/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "stm32h7xx_it.h"

#include "bootloaderFunctions.h"
#include "memoryMap.h"
#include "bootloaderMain.h"
#include "flash.h"

/*  Removed can init from bootloader 

*/

/* Variable declaration ------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void increment_boot_loop_count(void);

typedef struct __attribute__((packed))
{
	
	uint32_t flags;
  uint32_t magic;
} shared_memory_t;

/* USER CODE END 0 */

/**
 * @brief  The bootloader entry point.
 * @retval int
 */
shared_memory_t sharedmemory __attribute__((section(".shared_memory")));

char message[50] = {0};

//main function. 
int main(void) {

  //system clock init.
  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();
  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
#if NDEBUG == 1
	switchToSecondaryApp();  // Comment if not testing
#endif
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  //can init
  
  //while loop running on CLK frequency.
  while (1) 
  {
    // for (int blinkCounter = 0; blinkCounter <= 1; blinkCounter++)
    // {
    //   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
    //   HAL_Delay(100);
    // }

    // switchToPrimaryApp();
    if (sharedmemory.magic == SHARED_MAGIC)
    {
      // get flag
      bootloader_flag_handler(&sharedmemory.flags);
    }
    else
    {
      // Magic Byte incorrect
      // Reboot Detected
      // Clear flags
      sharedmemory.flags = 0;
    }

    // Set magic byte
    sharedmemory.magic = SHARED_MAGIC;

    increment_boot_loop_count();
    uint16_t app_partition = getAppPartition();
    if (app_partition == 1)
    {
      // setApp1Active();
      switchToPrimaryApp();
    }
    else if (app_partition == 2)
    {
    //  setApp2Active();
     switchToSecondaryApp();
    }
    else
    {
      // no valid app partition found

      // Checking if any valid app exists
      if (verifyApp1() == 1)
      {
        setApp1Active();
        switchToPrimaryApp();
      }
      else if (verifyApp2() == 1)
      {
        setApp2Active();
        switchToSecondaryApp();
      }
    }
  }

}

void increment_boot_loop_count()
{
  uint32_t flags, boot_loop_count;
  boot_loop_count = (sharedmemory.flags >> 24) & 0xFF;
  flags = (sharedmemory.flags);

  flags = sharedmemory.flags & 0xFFFF;

  boot_loop_count += 1;

  flags = (flags | (boot_loop_count << 24));
  sharedmemory.flags = flags;
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {

  // 400MHz
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
   
