#include "helper_functions.h"
#include "cmsis_gcc.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"

float_t calc_intern_temp_sens(float_t adc3_ch18_reading)
{
    uint16_t TS_CAL1 = *(uint16_t*)(0x1FF1E820);
    uint16_t TS_CAL2 = *(uint16_t*)(0x1FF1E840);

    float_t temperature =
        ((110.0f - 30.0f) / ((float)TS_CAL2 - (float)TS_CAL1)) * ((float)adc3_ch18_reading - (float)TS_CAL1) + 30.0f;

    return temperature;
}

void delay_us(uint16_t delay)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < delay)
    {
        __NOP();
    }
}