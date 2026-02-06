#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void buzzer_short_beep_start(void);

void buzzer_update(void);

void buzzer_off(void);

void toggle_buzzer_mode(void);

void set_buzzer_mode_active(void);

void set_buzzer_mode_inactive(void);

#ifdef __cplusplus
}
#endif
#endif /*__BUZZER_H__ */
