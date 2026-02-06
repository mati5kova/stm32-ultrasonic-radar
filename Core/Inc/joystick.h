#ifndef __JOYSTICK__H__
#define __JOYSTICK__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"

#define NEAR_MIN_ACCEPT 500
#define NEAR_MAX_ACCEPT 65000
#define LOW_MIDDLE 10000
#define HIGH_MIDDLE 50000

enum JoystickAimDirection
{
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3,
    CENTER = 4
};

enum JoystickAimDirection get_joystick_aim_dir(void);

uint8_t* get_str_from_aim_dir(void);

#ifdef __cplusplus
}
#endif
#endif /*__JOYSTICK__H__ */
