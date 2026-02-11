#include "tim.h"
#include "cmsis_gcc.h"
#include "main.h"
#include "stm32h750xx.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"

#define DISPLAY_FPS 30U

void MK_Internal_TIM15_Init(void);
void MK_Internal_TIM3_Init(void);
void MK_Internal_TIM6_Init(void);
void MK_Internal_TIM12_Init(void);
void MK_Internal_TIM7_Init(void);

TIM_HandleTypeDef htim15 = {0};
TIM_HandleTypeDef htim3 = {0};
TIM_HandleTypeDef htim6 = {0};
TIM_HandleTypeDef htim12 = {0};
TIM_HandleTypeDef htim7 = {0};

void MK_TIM_Init(void)
{
    MK_Internal_TIM15_Init();
    MK_Internal_TIM12_Init();
    MK_Internal_TIM6_Init();
    MK_Internal_TIM3_Init();
    MK_Internal_TIM7_Init();
}

/*
 * passive buzzer
 */
void MK_Internal_TIM15_Init(void)
{
    __HAL_RCC_TIM15_CLK_ENABLE(); // APB2 Timer clocks 125MHz

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    htim15.Instance = TIM15;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Prescaler = 0;      // PSC
    htim15.Init.Period = 70000 - 1; // ARR
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // CCR, zacni v tisini
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * HC-SR04 ECHO
 * 1MHz - 1us
 */
void MK_Internal_TIM12_Init(void)
{
    __HAL_RCC_TIM12_CLK_ENABLE(); // APB1 Timer clocks 125MHz
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim12.Instance = TIM12;
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1e6) - 1U; // PSC
    htim12.Init.Period = 0xffffU;                                // ARR
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 8;
    if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
}

/*
 * micro second delay
 */
void MK_Internal_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE(); // APB1
    htim6.Instance = TIM6;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1e6) - 1U;
    htim6.Init.Period = 0xffffU;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * MG90S
 */
void MK_Internal_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1e6) - 1U; // 125-1
    htim3.Init.Period = 20000 - 1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);

    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.Pulse = 1500;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * osvezevanje ekrana
 */
void MK_Internal_TIM7_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    htim7.Instance = TIM7;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1e5) - 1U; // 100kHz
    htim7.Init.Period = (100000U / DISPLAY_FPS) - 1U;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}