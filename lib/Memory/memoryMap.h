/* Header Guard --------------------------------------------------------------*/
#ifndef MEMORYMAP_H_
#define MEMORYMAP_H_

/* Standard C includes -------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern uint32_t __sharedram_start__;
extern uint32_t __sharedram_size__;
extern uint32_t __bootloader_start__;
extern uint32_t __bootloader_size__;
extern uint32_t __app1_start__;
extern uint32_t __app1_size__;
extern uint32_t __app2_start__;
extern uint32_t __app2_size__;
extern uint32_t __commondata_start__;

/* Private variables ---------------------------------------------------------*/

/* Inline functions ----------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


#endif
/* End of File ---------------------------------------------------------------*/




