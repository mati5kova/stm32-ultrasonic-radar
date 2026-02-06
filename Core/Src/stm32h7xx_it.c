/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    stm32h7xx_it.c
* @brief   Interrupt Service Routines.
******************************************************************************
* @attention
*
* Copyright (c) 2026 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_it.h"
#include "buzzer.h"
#include "cmsis_gcc.h"
#include "display.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hcsr04.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t last_interrupt_exti_15_10_pin13 = 0;
uint32_t last_interrupt_exti_3_pin3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim3;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {}
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void USART3_IRQHandler(void)
{

    HAL_UART_IRQHandler(&huart3);

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) && __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE))
    {
        uint8_t c = (uint8_t)huart3.Instance->RDR; // pocisti rxne
        (void)c;
    }
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc3);

    /* debug */
    // volatile uint32_t err = hdma_adc3.ErrorCode;
    // (void)err;
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
    // HAL_TIM_IRQHandler nastavi htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2 in klice HAL_TIM_IC_CaptureCallback
    HAL_TIM_IRQHandler(&htim12);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM12 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        HCSR04_OnCapture();
    }
}

void TIM7_IRQHandler(void)
{
    // pocisti flage in poklice HAL_TIM_PeriodElapsedCallback
    HAL_TIM_IRQHandler(&htim7);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM7)
    {
        refresh_frame_flag = 1; // refresh frame
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13))
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);

        uint32_t now = HAL_GetTick();
        if (now - last_interrupt_exti_15_10_pin13 < 50)
        {
            return;
        }
        last_interrupt_exti_15_10_pin13 = now;

        toggle_buzzer_mode(); // togglaj buzzer on/off
    }
}

void EXTI3_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3))
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

        uint32_t now = HAL_GetTick();
        if (now - last_interrupt_exti_3_pin3 < 50)
        {
            return;
        }
        last_interrupt_exti_3_pin3 = now;
    }
}

/* USER CODE END 1 */
