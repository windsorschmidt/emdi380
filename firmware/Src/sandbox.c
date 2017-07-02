#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "gfx.h"

// Support OpenOCD's thread awareness
const int __attribute__((used)) uxTopUsedPriority = configMAX_PRIORITIES - 1;

// FreeRTOS idle task, defined __weak in auto-generated main.c
void DefaultTask(void const * argument) {

    gfxInit(); // after FreeRTOS starts we can initialize uGFX

    while(1) {
        GPIOE->BSRR = (1<<0); // green LED ON
        osDelay(200);
        GPIOE->BSRR = (1<<16); // green LED off
        osDelay(200);
        GPIOE->BSRR = (1<<1); // red LED on
        osDelay(200);
        GPIOE->BSRR = (1<<(1+16)); // red LED off
        osDelay(200);
    }
}
