#include "uart.h"

UART_HandleTypeDef huart3 = {0};

void MK_UART_Init(void)
{
    /* USART3 VCP */
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // prispel nov znak
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_PE); // parrity error
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE); // naprava pripravljena na prenos
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC); // transmit complete
}