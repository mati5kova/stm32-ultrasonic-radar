#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;

extern volatile uint16_t* internal_temperature_sensor_reading;

extern volatile uint16_t adc3_dma_values[3];

void MK_ADC_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ADC_H__ */
