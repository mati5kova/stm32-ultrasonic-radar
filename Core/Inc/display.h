#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BACKGROUND_LAYER 0
#define DYNAMIC_LAYER 1

extern volatile uint8_t refresh_frame_flag;

extern float_t sweeper_handle_angle;

void wait_for_vsync(void);

void MK_Display_Init(void);

void draw_background_static_once(void);

void draw_dynamic_content(void);

void add_new_radar_dot(float_t distance, float_t angle);


#ifdef __cplusplus
}
#endif
#endif /*__DISPLAY_H__ */
