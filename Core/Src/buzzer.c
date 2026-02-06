#include "buzzer.h"
#include "tim.h"

extern TIM_HandleTypeDef htim15;

static uint32_t beep_start_time = 0;
static uint8_t beep_active = 0;

volatile uint8_t buzzer_mode_active = 0;

void buzzer_short_beep_start(void)
{
    if (!beep_active && buzzer_mode_active) // zacni samo ce trenutno ni beepa IN je vkljuceno buzzanje
    {
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, (__HAL_TIM_GET_AUTORELOAD(&htim15) >> 1));
        beep_start_time = HAL_GetTick();
        beep_active = 1;
    }
}

void buzzer_update(void)
{
    if (beep_active)
    {
        uint32_t now = HAL_GetTick();
        if (now - beep_start_time >= 5) // minilo 5ms
        {
            __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0); // ugasni
            beep_active = 0;
        }
    }
}

void buzzer_off(void) { __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0); }

void toggle_buzzer_mode(void) { buzzer_mode_active = !buzzer_mode_active; }

void set_buzzer_mode_active(void) { buzzer_mode_active = 1; }

void set_buzzer_mode_inactive(void) { buzzer_mode_active = 0; }