#ifndef __HELPER_FUNCTIONS_H__
#define __HELPER_FUNCTIONS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

float_t calc_intern_temp_sens(float_t adc3_ch18_reading);

void delay_us(uint16_t delay);

#ifdef __cplusplus
}
#endif
#endif /*__HELPER_FUNCTIONS_H__ */
