#include "adc.h"

ADC_HandleTypeDef hadc3 = {0};
DMA_HandleTypeDef hdma_adc3 = {0};

/*
 * [0] - internal temperature sensor
 * [1] - joystick x
 * [2] - joysitck y
 */
__attribute__((section(".dma_buffer"), aligned(32))) volatile uint16_t adc3_dma_values[3];

volatile uint16_t* internal_temperature_sensor_reading = &adc3_dma_values[0];

// glej AN3116 - STM32's ADC modes and their applications
void MK_ADC_Init(void)
{
    /*multichannel (scan) continous conversion mode*/
    __HAL_RCC_ADC3_CLK_ENABLE();

    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
    hadc3.Init.Resolution = ADC_RESOLUTION_16B;
    // continious conversion -> ne ustavi se po prvi rundi konverzij ampak se vrne na zacetek in to ponavlja v nedogled
    hadc3.Init.ContinuousConvMode = ENABLE;
    hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE; // scan = multichannel
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.NbrOfDiscConversion = 0;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc3.Init.NbrOfConversion = 3; // temp sensor + (x + y)kordinata joysticka
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.LowPowerAutoWait = DISABLE; // poglej mogoce daj to ENABLE in se bo sprozila nova pretvorba sele kot
                                           // softwaresko prebers vrednosti v timerju npr.
    hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.OversamplingMode = DISABLE;
    hadc3.Init.Oversampling.Ratio = 1;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }

    /*ADC3 channel config*/
    ADC_ChannelConfTypeDef adc3_channel_config = {0};

    /* temp sensor*/
    adc3_channel_config.Rank = ADC_REGULAR_RANK_1;
    adc3_channel_config.Channel = ADC_CHANNEL_TEMPSENSOR;
    adc3_channel_config.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    adc3_channel_config.OffsetNumber = ADC_OFFSET_NONE;
    adc3_channel_config.OffsetSignedSaturation = DISABLE;
    adc3_channel_config.SingleDiff = ADC_SINGLE_ENDED;
    if (HAL_ADC_ConfigChannel(&hadc3, &adc3_channel_config) != HAL_OK)
    {
        Error_Handler();
    }

    /* x kordinata joysticka*/
    adc3_channel_config.Rank = ADC_REGULAR_RANK_2;
    adc3_channel_config.Channel = ADC_CHANNEL_7;
    adc3_channel_config.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    adc3_channel_config.OffsetNumber = ADC_OFFSET_NONE;
    adc3_channel_config.OffsetSignedSaturation = DISABLE;
    adc3_channel_config.SingleDiff = ADC_SINGLE_ENDED;
    if (HAL_ADC_ConfigChannel(&hadc3, &adc3_channel_config) != HAL_OK)
    {
        Error_Handler();
    }

    /* y kordinata joysticka */
    adc3_channel_config.Rank = ADC_REGULAR_RANK_3;
    adc3_channel_config.Channel = ADC_CHANNEL_10;
    adc3_channel_config.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    adc3_channel_config.OffsetNumber = ADC_OFFSET_NONE;
    adc3_channel_config.OffsetSignedSaturation = DISABLE;
    adc3_channel_config.SingleDiff = ADC_SINGLE_ENDED;
    if (HAL_ADC_ConfigChannel(&hadc3, &adc3_channel_config) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_adc3.Instance = DMA1_Stream0;
    hdma_adc3.Init.Request = DMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(&hadc3, DMA_Handle, hdma_adc3);

    // debug
    // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 7, 7);
    // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}