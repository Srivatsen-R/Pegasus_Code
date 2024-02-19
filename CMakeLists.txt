cmake_minimum_required(VERSION 3.10)

# set the project name
project(Pegasus3.1v)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

execute_process (
    COMMAND "git rev-parse --short HEAD"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(COMMON_INCLUDE
    Inc/
    Drivers/
    Drivers/STM32H7xx_HAL_Driver/Inc/
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/
    Drivers/CMSIS/Device/ST/STM32H7xx/Include/
    Drivers/CMSIS/Include/

    Utility/can-tp-module-threadx
    Utility/can-tp-module-threadx/isotp-c/src
    Utility/can-tp-module-threadx/isotp-c/deps/bitfield-c/src

    lib/ApplicationLayer/
    lib/CustomLibs/
    lib/DriverLayer/

    lib/Bootloader/
    lib/Memory/

    )

set(COMMON_FILES
    src/adc.c
    src/fdcan.c
    src/gpio.c
    src/i2c.c
    src/tim.c
    src/usart.c
    src/dma.c

    src/stm32h7xx_it.c
    src/stm32h7xx_hal_msp.c
    src/system_stm32h7xx.c

    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c

    

    

    startup_stm32h743zitx.s)

set(EXECUTABLE_BOOTLDR ${PROJECT_NAME}-bootloader.elf)
set(EXECUTABLE_APP_PRI ${PROJECT_NAME}-app1.elf)
set(EXECUTABLE_APP_SEC ${PROJECT_NAME}-app2.elf)

set(BOOTLOADER_MAIN
    lib/Bootloader/bootloaderMain.c)

set(BOOTLOADER_FILES
    lib/Bootloader/bootloaderFunctions.c

    lib/ApplicationLayer/flash.c
    lib/ApplicationLayer/crc32.c
    )   

set(FIRMWARE_MAIN
   src/main.c)

set(FIRMWARE_FILES
    lib/ApplicationLayer/adc_AL.c
    lib/ApplicationLayer/config.c
    lib/ApplicationLayer/crc32.c
    lib/ApplicationLayer/eeprom_AL.c
    lib/ApplicationLayer/eepromCRC.c
    lib/ApplicationLayer/fdcan_AL.c
    lib/ApplicationLayer/flash.c
    lib/ApplicationLayer/foc_blockset.c
    lib/ApplicationLayer/foc_mtpa_lut.c
    lib/ApplicationLayer/led_AL.c
    lib/ApplicationLayer/microcontroller.c
    lib/ApplicationLayer/motor_param.c
    lib/ApplicationLayer/sanity.c
    lib/ApplicationLayer/temp_AL.c
    lib/ApplicationLayer/usart_AL.c
    lib/ApplicationLayer/v_f_control.c
    lib/ApplicationLayer/vector_control.c
    lib/ApplicationLayer/vehicle.c

    lib/CustomLibs/printf.c
    lib/CustomLibs/math_func.c

    lib/CustomLibs/firmware_upgrade_app.c

    Utility/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/8byte.c
    Utility/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/bitarray.c
    Utility/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/bitfield.c
    Utility/can-tp-module-threadx/isotp-c/src/isotp/isotp.c
    Utility/can-tp-module-threadx/isotp-c/src/isotp/receive.c
    Utility/can-tp-module-threadx/isotp-c/src/isotp/send.c
    Utility/can-tp-module-threadx/can_tp_app.c
    )

add_executable(${EXECUTABLE_BOOTLDR}
                ${COMMON_FILES}
                ${BOOTLOADER_MAIN} ${BOOTLOADER_FILES})        
    
add_executable(${EXECUTABLE_APP_PRI} 
                ${COMMON_FILES} 
                ${FIRMWARE_MAIN} ${FIRMWARE_FILES})

add_executable(${EXECUTABLE_APP_SEC} 
                ${COMMON_FILES} 
                ${FIRMWARE_MAIN} ${FIRMWARE_FILES})

target_compile_definitions(${EXECUTABLE_BOOTLDR} PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32H743xx
    -DGIT_SHA=\"${GIT_SHA}\"
    -DNDEBUG=${NDEBUG}
    )
                
target_compile_definitions(${EXECUTABLE_APP_PRI} PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32H743xx
    -DGIT_SHA=\"${GIT_SHA}\"
    -DAPP1
    -DNDEBUG=${NDEBUG}
    )

target_compile_definitions(${EXECUTABLE_APP_SEC} PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32H743xx
    -DGIT_SHA=\"${GIT_SHA}\"
    -DAPP2
    -DNDEBUG=${NDEBUG}
    )

target_include_directories(${EXECUTABLE_BOOTLDR} PUBLIC ${COMMON_INCLUDE})    
target_include_directories(${EXECUTABLE_APP_PRI} PUBLIC ${COMMON_INCLUDE})
target_include_directories(${EXECUTABLE_APP_SEC} PUBLIC ${COMMON_INCLUDE})

target_compile_options(${EXECUTABLE_BOOTLDR} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -fdata-sections
    -ffunction-sections

    -std=gnu99
    
    -Wall
    -Wno-comment
    
    $<$<CONFIG:Debug>:-Og>
    )

target_compile_options(${EXECUTABLE_APP_PRI} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -fdata-sections
    -ffunction-sections

    -std=gnu99
    
    -Wall
    -Wno-comment
    
    $<$<CONFIG:Debug>:-Og>
    )

target_compile_options(${EXECUTABLE_APP_SEC} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -fdata-sections
    -ffunction-sections

    -std=gnu99
    
    -Wall
    -Wno-comment
    
    $<$<CONFIG:Debug>:-Og>
    )

target_link_options(${EXECUTABLE_BOOTLDR} PRIVATE
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/memory_map.ld"
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/master.ld"
    -mcpu=cortex-m7
    --specs=nosys.specs

    -Wl,-Map=${PROJECT_NAME}-bootloader.map,--cref
    -Wl,--gc-sections -static

    -fno-math-errno

    --specs=nano.specs
    
    -mthumb
    -u_printf_float
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -Wl,--start-group
    -lc 
    -lm 
    -Wl,--end-group

    -lnosys
    )

target_link_options(${EXECUTABLE_APP_PRI} PRIVATE
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/memory_map.ld"
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/app1.ld"
    -mcpu=cortex-m7
    --specs=nosys.specs

    -Wl,-Map=${PROJECT_NAME}-app1.map,--cref
    -Wl,--gc-sections -static

    -fno-math-errno

    --specs=nano.specs
    
    -mthumb
    -u_printf_float
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -Wl,--start-group
    -lc 
    -lm 
    -Wl,--end-group

    -lnosys
    )

target_link_options(${EXECUTABLE_APP_SEC} PRIVATE
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/memory_map.ld"
    "SHELL:-T ${CMAKE_SOURCE_DIR}/linkerScripts/app2.ld"
    -mcpu=cortex-m7
    --specs=nosys.specs

    -Wl,-Map=${PROJECT_NAME}-app2.map,--cref
    -Wl,--gc-sections -static

    -fno-math-errno

    --specs=nano.specs
    
    -mthumb
    -u_printf_float
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

    -Wl,--start-group
    -lc 
    -lm 
    -Wl,--end-group

    -lnosys
    )

add_custom_command(TARGET ${EXECUTABLE_BOOTLDR}
    POST_BUILD
    COMMAND arm-none-eabi-size ${EXECUTABLE_BOOTLDR})

add_custom_command(TARGET ${EXECUTABLE_APP_PRI}
    POST_BUILD
    COMMAND arm-none-eabi-size ${EXECUTABLE_APP_PRI})

add_custom_command(TARGET ${EXECUTABLE_APP_SEC}
    POST_BUILD
    COMMAND arm-none-eabi-size ${EXECUTABLE_APP_SEC})

add_custom_command(TARGET ${EXECUTABLE_BOOTLDR}
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_BOOTLDR} ${PROJECT_NAME}-bootloader.hex
    COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_BOOTLDR} ${PROJECT_NAME}-bootloader.bin)

add_custom_command(TARGET ${EXECUTABLE_APP_PRI}
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_APP_PRI} ${PROJECT_NAME}-app1.hex
    COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_APP_PRI} ${PROJECT_NAME}-app1.bin)

add_custom_command(TARGET ${EXECUTABLE_APP_SEC}
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_APP_SEC} ${PROJECT_NAME}-app2.hex
    COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_APP_SEC} ${PROJECT_NAME}-app2.bin)
