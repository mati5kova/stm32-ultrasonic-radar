#ifndef __HCSR04_H__
#define __HCSR04_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define HCSR04_ECHO_TIMEOUT_MS 45

void HCSR04_Trigger(void);

uint8_t HCSR04_ReadDistance(float_t* addr);

void HCSR04_OnCapture(void);

void HCSR04_CheckEchoTimeout(void);

void HCSR04_SetCurrentSweepAngle(float_t angle);

uint8_t HCSR04_IsBusy(void);

#ifdef __cplusplus
}
#endif
#endif /*__HCSR04_H__ */
