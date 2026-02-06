#include "hcsr04.h"
#include "buzzer.h"
#include "display.h"
#include "helper_functions.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include <math.h>
#include <stdint.h>

#define HCSR04_MAX_RANGE 400.0f

extern TIM_HandleTypeDef htim12;

uint8_t last_captured_rising_edge = 0;
uint32_t IC_value_rising_edge = 0;
uint32_t IC_value_falling_edge = 0;
volatile float_t calculated_distance = 0.0f;

volatile uint8_t has_new_measurement = 0;

volatile uint32_t last_echo_rising_edge_tick_ms = 0; // HAL_GetTick od takrat ko smo videli rising edge

volatile float_t measurement_angle = 0.0f; // kot pri katerem smo sprozili meritev

void HCSR04_Trigger(void)
{
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);
}

uint8_t HCSR04_ReadDistance(float_t* addr)
{
    if (!has_new_measurement)
    {
        return 0;
    }

    *addr = calculated_distance;
    has_new_measurement = 0;

    return 1;
}

void HCSR04_OnCapture(void)
{
    /* dodaten check da preverimo ce je timeout ze potekel, lahko se zgodi da bi izgubili meritev ce pride nov trigger
     * med obdelavo falling edge-a
     */
    if (last_captured_rising_edge)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_echo_rising_edge_tick_ms > HCSR04_ECHO_TIMEOUT_MS)
        {
            last_captured_rising_edge = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim12, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
            return;
        }
    }

    // zacetek celovite meritve
    if (!last_captured_rising_edge)
    {
        IC_value_rising_edge = htim12.Instance->CCR2; //  HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_2)

        last_echo_rising_edge_tick_ms = HAL_GetTick(); // zacnemo timeout okno

        last_captured_rising_edge = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim12, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
    }
    else
    {
        IC_value_falling_edge = htim12.Instance->CCR2;

        uint32_t diff;
        if (IC_value_falling_edge >= IC_value_rising_edge)
        {
            diff = IC_value_falling_edge - IC_value_rising_edge;
        }
        else
        {
            // ce je v casu merjenja counter prised do vrednosti ARR moramo pristeti ARR + 1 - rising_edge
            diff = (htim12.Init.Period + 1u - IC_value_rising_edge) + IC_value_falling_edge;
        }

        calculated_distance = (diff * 0.034f) / 2.0f;
        has_new_measurement = 1;

        if (calculated_distance > 0.0f && calculated_distance < HCSR04_MAX_RANGE)
        {
            has_new_measurement = 1;
            add_new_radar_dot(calculated_distance, measurement_angle);
        }

        last_captured_rising_edge = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim12, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
    }
}

void HCSR04_CheckEchoTimeout(void)
{
    if (last_captured_rising_edge)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_echo_rising_edge_tick_ms) > HCSR04_ECHO_TIMEOUT_MS)
        {
            last_captured_rising_edge = 0; // zavrzemo prejsnjo meritev

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim12, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);

            __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC2);
        }
    }
}

void HCSR04_SetCurrentSweepAngle(float_t angle) { measurement_angle = angle; }