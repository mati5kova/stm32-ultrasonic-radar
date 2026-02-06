#ifndef __MG90S_H__
#define __MG90S_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void mg90s_set_angle(TIM_HandleTypeDef* htim, uint32_t channel, uint8_t angle);

#ifdef __cplusplus
}
#endif
#endif /*__MG90S_H__ */
