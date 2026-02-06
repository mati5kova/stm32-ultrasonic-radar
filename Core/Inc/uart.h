#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart3;

void MK_UART_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__UART_H__ */

