#include "exti.h"
#include "stm32h750xx.h"
#include "stm32h7xx_hal_cortex.h"

void MK_EXTI_Init(void)
{
    // user button (blue) (PC13)
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    // joystick button (PG3)
    HAL_NVIC_SetPriority(EXTI3_IRQn, 12, 1);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}