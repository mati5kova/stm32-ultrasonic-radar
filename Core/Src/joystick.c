#include "joystick.h"

volatile uint16_t* joystick_x = &adc3_dma_values[1];
volatile uint16_t* joystick_y = &adc3_dma_values[2];

static inline uint8_t other_centered(uint16_t other) { return (uint8_t)(LOW_MIDDLE <= other && other <= HIGH_MIDDLE); }

static inline uint8_t near_min(uint16_t v) { return (uint8_t)(v <= NEAR_MIN_ACCEPT); }
static inline uint8_t near_max(uint16_t v) { return (uint8_t)(v >= NEAR_MAX_ACCEPT); }

enum JoystickAimDirection get_joystick_aim_dir(void)
{
    uint16_t x = *joystick_x;
    uint16_t y = *joystick_y;

    enum JoystickAimDirection ret = CENTER;

    if (near_min(x) && other_centered(y))
    {
        ret = LEFT;
    }
    else if (near_max(x) && other_centered(y))
    {
        ret = RIGHT;
    }
    else if (near_min(y) && other_centered(x))
    {
        ret = UP;
    }
    else if (near_max(y) && other_centered(x))
    {
        ret = DOWN;
    }

    return ret;
}

uint8_t* get_str_from_aim_dir(void)
{
    enum JoystickAimDirection aim_dir = get_joystick_aim_dir();
    switch ((uint8_t)aim_dir)
    {
        case (uint8_t)LEFT:
            return (uint8_t*)"left";
        case (uint8_t)RIGHT:
            return (uint8_t*)"right";
        case (uint8_t)UP:
            return (uint8_t*)"up";
        case (uint8_t)DOWN:
            return (uint8_t*)"down";
    }

    return (uint8_t*)"center";
}