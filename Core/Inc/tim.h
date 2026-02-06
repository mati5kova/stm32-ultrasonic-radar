#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim7;

void MK_TIM_Init(void);


#ifdef __cplusplus
}
#endif
#endif /*__TIM_H__ */
