#include "mg90s.h"

void mg90s_set_angle(TIM_HandleTypeDef* htim, uint32_t channel, uint8_t angle)
{

    if (angle > 180)
    {
        angle = 180;
    }

    uint32_t pulse_length = 500 + (angle * (1000) / 180); // 2500 - 500

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}