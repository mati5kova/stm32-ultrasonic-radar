#include "display.h"
#include "adc.h"
#include "buzzer.h"
#include "fonts.h"
#include "hcsr04.h"
#include "helper_functions.h"
#include "joystick.h"
#include "main.h"
#include "stm32_lcd.h"
#include "stm32h750b_discovery_errno.h"
#include "stm32h750b_discovery_lcd.h"
#include "stm32h750b_discovery_sdram.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t refresh_frame_flag = 0;

extern volatile uint16_t* internal_temperature_sensor_reading;

#define F_PI 3.14159265358979323846f

/*
 * 1.571 = 90°/sec (0->180° ~4 sekund)
 * 0.785 = 45°/sec (0->180° ~8 sekund)
 * 0.524 = 30°/sec (0->180° ~12 sekund)
 * 0.262 = 15°/sec (0->180° ~24 sekund)
 */
#define SWEEPER_ANGULAR_VELOCITY 0.785f
/*50ms max da preprecimo prevelike skoke*/
#define FRAME_TIME_LIMIT 0.05f
/* popravek za floating point aritmetiko ce ne pridemo na tocno 2*F_PI (0->180) oziroma F_PI (180->0)
 * clampamo na vrednost in se še vedno obrnemo v drugo smer
 */
#define ANGLE_EPSILON 0.001f
#define SWEEPER_HANDLE_THICKNESS_HALF 0.04f

#define LCD_HALF_WIDTH (LCD_DEFAULT_WIDTH / 2)
#define BACKGROUND_COLOR 0xFF202613UL

/*crte na ozadju, diagonalne crte*/
#define NBR_OF_VERTICAL_LINES 9
#define NBR_OF_HORIZONTAL_LINES 10
#define NBR_OF_DIAGONAL_LINES_INNER 7
#define NBR_OF_DIAGONAL_LINES_OUTER 40
#define BACKGROUND_LINES_COLOR 0xFF2D4719UL
#define BACKGROUND_DIAGONAL_LINES_COLOR 0xFF909C11UL

/*vse kar se zadeva vogalov*/
#define NBR_OF_CORNERS 4
#define CORNER_ARM_LENGTH 35
#define CORNER_OFFSET_X 15
#define CORNER_OFFSET_Y 15
#define CORNER_COLOR LCD_COLOR_ARGB8888_ST_GREEN_LIGHT

#define OUTER_HALF_CIRCLE_RADIUS 200
#define INNER_FULL_CIRCLE_RADIUS 18
#define MAIN_SWEEPER_BOTTOM_OFFSET 45
#define OUTER_HALF_CIRCLE_COLOR LCD_COLOR_ARGB8888_ST_GREEN_LIGHT

/*vse kar se zadeva pozicij texta na spodnjem delu displaya*/
// izpis temperaturnega senzorja
#define INTERNAL_TEMP_TEXT_X (0 + CORNER_OFFSET_X + 7)
#define INTERNAL_TEMP_TEXT_Y (LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y - 13)
#define INTERNAL_TEMP_TEXT_WIDTH 80
// izpis distance
#define DISTANCE_TEXT_Y (LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET + INNER_FULL_CIRCLE_RADIUS + 10)
#define DISTANCE_TEXT_WIDTH 100
// izpise smeri joysticka
#define JOYSTICK_TEXT_Y (LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y - 13)
#define JOYSTICK_TEXT_X_MAX (LCD_DEFAULT_WIDTH - CORNER_OFFSET_X - 4)
#define JOYSTICK_TEXT_WIDTH 90

/*radar dots*/
#define MAX_DISTANCE_TO_RENDER_DOT 200 //[cm]
#define MIN_DISTANCE_TO_RENDER_DOT 10  //[cm]
#define NBR_OF_RADAR_DOTS 150
#define RADAR_DOT_LIFE_SPAN 6000 // [ms]

static uint32_t last_tick = 0;
float_t sweeper_handle_angle = F_PI + SWEEPER_HANDLE_THICKNESS_HALF;
static uint8_t sweeper_direction = 1; // 1 = smer urinega kazalca (0->180), 0 = obratna smer urinega kazalca (180->0)

/*uporabi se za ucinkovito clearanje samo potrebnih delov ekrana*/
static Point last_sweeper_points[4] = {0};
static uint8_t first_frame = 1;

static inline void set_dot_data_zero(void);

static inline void internal_draw_corner(void);
static void internal_draw_half_circle(uint16_t x_center, uint16_t y_center, uint16_t radius, uint32_t color);
static inline uint16_t internal_center_text_x_cordinate(uint8_t* str, sFONT font);
static void internal_draw_diagonal_lines(uint16_t x_center, uint16_t y_center, uint16_t outer_radius,
                                         uint16_t inner_radius);

static inline void internal_clear_sweep_area(void);
static inline void internal_dynamic_layer_draw_temp_sens(void);
static inline void internal_dynamic_layer_draw_distance(void);
static inline void internal_dynamic_layer_draw_joystick_aim_dir(void);
static void internal_dynamic_layer_draw_sweeper_line(float_t angle, uint16_t x_center, uint16_t y_center,
                                                     uint16_t outer_radius, uint16_t inner_radius);
static void internal_dynamic_layer_draw_radar_dots(void);

typedef struct
{
    uint8_t valid;
    uint16_t distance;
    float_t angle;
    uint32_t timestamp;

} RadarDot_TypeDef;

RadarDot_TypeDef radar_dots[NBR_OF_RADAR_DOTS];
uint16_t radar_dots_next_index = 0;

static Point corner_cordinates[4][2] = {
    /*levo zgoraj*/
    {
        {0 + CORNER_OFFSET_X, 0 + CORNER_OFFSET_Y},
        {0 + CORNER_OFFSET_X, 0 + CORNER_OFFSET_Y},
    },
    /*levo spodaj*/
    {{0 + CORNER_OFFSET_X, LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y},
     {0 + CORNER_OFFSET_X, LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y - CORNER_ARM_LENGTH}},
    /*desno zgoraj*/
    {{LCD_DEFAULT_WIDTH - CORNER_OFFSET_X - CORNER_ARM_LENGTH, 0 + CORNER_OFFSET_Y},
     {LCD_DEFAULT_WIDTH - CORNER_OFFSET_X, 0 + CORNER_OFFSET_Y}},
    /*desno spodaj*/
    {{LCD_DEFAULT_WIDTH - CORNER_OFFSET_X - CORNER_ARM_LENGTH, LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y},
     {LCD_DEFAULT_WIDTH - CORNER_OFFSET_X, LCD_DEFAULT_HEIGHT - CORNER_OFFSET_Y - CORNER_ARM_LENGTH}}};

static inline void set_dot_data_zero(void) { memset(radar_dots, 0, sizeof(RadarDot_TypeDef) * NBR_OF_RADAR_DOTS); }

void MK_Display_Init(void)
{
    if (BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    if (BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE) != BSP_ERROR_NONE)
    {
        Error_Handler();
    } // BSP_LCD_Init konfigurira layer 0 po defaultu

    BSP_LCD_LayerConfig_t layer1_config;
    layer1_config.X0 = 0;
    layer1_config.X1 = LCD_DEFAULT_WIDTH;
    layer1_config.Y0 = 0;
    layer1_config.Y1 = LCD_DEFAULT_HEIGHT;
    layer1_config.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
    layer1_config.Address = LCD_LAYER_1_ADDRESS;

    if (BSP_LCD_ConfigLayer(0, DYNAMIC_LAYER, &layer1_config) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    // naredimo oba layerja vidna
    if (BSP_LCD_SetLayerVisible(0, BACKGROUND_LAYER, ENABLE) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }
    if (BSP_LCD_SetLayerVisible(0, DYNAMIC_LAYER, ENABLE) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    // transparency = 255 -> NE vidno skozi
    if (BSP_LCD_SetTransparency(0, BACKGROUND_LAYER, 255) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }
    if (BSP_LCD_SetTransparency(0, DYNAMIC_LAYER, 255) != BSP_ERROR_NONE)
    {
        Error_Handler();
    } // layer pixli kontrolirajo svoj alpha

    if (BSP_LCD_SetActiveLayer(0, BACKGROUND_LAYER) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    UTIL_LCD_SetFuncDriver(&LCD_Driver);

    if (BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    // 0(zero) set za vse tocke na radarju - enkratni dogodek
    set_dot_data_zero();
}

void wait_for_vsync(void)
{
    /*pocakamo na VSYNC za preprecimo tearing*/
    /*mogoce implementiraj se pravi double buffering - mogoce se splaca*/

    // CDSR - current display status register
    // ce je current display v fazi vertical synchronization je VSYNCS bit postavljen na 1
    while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS)) {} // pocakaj da se VSYNC zacne
    while ((LTDC->CDSR & LTDC_CDSR_VSYNCS)) {}  // pocakaj da se VSYNC zakljuci
}

void draw_background_static_once(void)
{
    if (BSP_LCD_SetActiveLayer(0, DYNAMIC_LAYER) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }
    UTIL_LCD_Clear(0x00000000UL);

    if (BSP_LCD_SetActiveLayer(0, BACKGROUND_LAYER) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }
    UTIL_LCD_Clear(BACKGROUND_COLOR);

    /*vertikalne crte ozadja*/
    for (uint8_t i = 1; i < NBR_OF_VERTICAL_LINES; i++)
    {
        UTIL_LCD_DrawVLine((uint32_t)(i * LCD_DEFAULT_WIDTH / NBR_OF_VERTICAL_LINES), 0, LCD_DEFAULT_HEIGHT,
                           BACKGROUND_LINES_COLOR);
    }

    /*horizontalne crte ozadja*/
    for (uint8_t i = 1; i < NBR_OF_HORIZONTAL_LINES; i++)
    {
        UTIL_LCD_DrawHLine(0, (uint32_t)(i * LCD_DEFAULT_HEIGHT / NBR_OF_VERTICAL_LINES), LCD_DEFAULT_WIDTH,
                           BACKGROUND_LINES_COLOR);
    }

    /*veliki polkrog po katerem hodi sweeper*/
    internal_draw_half_circle(LCD_HALF_WIDTH, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET, OUTER_HALF_CIRCLE_RADIUS,
                              OUTER_HALF_CIRCLE_COLOR);

    /*manjsi polkrog znotraj velikega polkroga*/
    internal_draw_half_circle(LCD_HALF_WIDTH, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET, 40,
                              BACKGROUND_DIAGONAL_LINES_COLOR);

    /*najmanjsi polkrog znotraj velikega polkroga*/
    internal_draw_half_circle(LCD_HALF_WIDTH, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET, 80,
                              BACKGROUND_DIAGONAL_LINES_COLOR);

    /*mali krog na dnu (izhodisce za sweeper handle)*/
    UTIL_LCD_DrawCircle(LCD_HALF_WIDTH, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET, INNER_FULL_CIRCLE_RADIUS,
                        LCD_COLOR_ARGB8888_ST_GREEN_LIGHT);

    /*crta v desno od majhnega kroga na dnu*/
    UTIL_LCD_DrawRect(LCD_HALF_WIDTH + INNER_FULL_CIRCLE_RADIUS, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET,
                      OUTER_HALF_CIRCLE_RADIUS - INNER_FULL_CIRCLE_RADIUS, 1, LCD_COLOR_ARGB8888_ST_GREEN_LIGHT);

    /*crta v levo od majhnega kroga na dnu*/
    UTIL_LCD_DrawRect(LCD_HALF_WIDTH - OUTER_HALF_CIRCLE_RADIUS, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET,
                      OUTER_HALF_CIRCLE_RADIUS - INNER_FULL_CIRCLE_RADIUS, 1, LCD_COLOR_ARGB8888_ST_GREEN_LIGHT);

    /*vsi robovi ekrana (4x)*/
    internal_draw_corner();

    /*NBR_OF_DIAGONAL_LINES_INNER diagonalnih crt od roba malega notranjega kroga do tock na zunanjem polkrogu*/
    /* + NBR_OF_DIAGONAL_LINES_OUTER majhnih diagonalnih crt od roba velike kroznice navzven*/
    internal_draw_diagonal_lines(LCD_HALF_WIDTH, LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET,
                                 OUTER_HALF_CIRCLE_RADIUS, INNER_FULL_CIRCLE_RADIUS);
}

/*pocisti sweeper handle + text boxe namesto UTIL_LCD_Clear - bolj performant*/
static inline void internal_clear_sweep_area(void)
{
    if (BSP_LCD_SetActiveLayer(0, DYNAMIC_LAYER) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    if (!first_frame)
    {
        // pocisti zadnjo sweeper crto
        UTIL_LCD_FillPolygon(last_sweeper_points, 4, 0x00000000UL);
    }

    // pocisti text boxe
    // UTIL_LCD_FillRect(INTERNAL_TEMP_TEXT_X, INTERNAL_TEMP_TEXT_Y, INTERNAL_TEMP_TEXT_WIDTH, Font12.Height,
    //                   0x00000000UL);
    // UTIL_LCD_FillRect(LCD_HALF_WIDTH - (DISTANCE_TEXT_WIDTH / 2), DISTANCE_TEXT_Y, DISTANCE_TEXT_WIDTH,
    // Font12.Height,
    //                   0x00000000UL);
    // UTIL_LCD_FillRect(JOYSTICK_TEXT_X_MAX - JOYSTICK_TEXT_WIDTH, JOYSTICK_TEXT_Y, JOYSTICK_TEXT_WIDTH, Font12.Height,
    //                   0x00000000UL);
}

/*render vseh dinamicnih elementov*/
void draw_dynamic_content(void)
{
    uint32_t now = HAL_GetTick();
    /* izracunamo casovno razliko (delta time) (v sekundah) od zadnjega frame-a
     *  brez tega ne upostevamo frame rate-a in bi se sweeper premikal razlicno glede na fps (nastavljen v initu za
     *  TIM7)
     *
     */
    float_t dt = (now - last_tick) * 0.001f;
    last_tick = now;

    /* omejimo vrednost dt, ce se npr. zaradi printf ali neke druge blocking stvari zatakne za dlje casa ne bomo
     * izgubili sweepanja za minili cas
     */
    if (dt > FRAME_TIME_LIMIT || dt <= 0.0f)
    {
        dt = FRAME_TIME_LIMIT;
    }

    if (BSP_LCD_SetActiveLayer(0, DYNAMIC_LAYER) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /*pocisti obmocje sweeperja*/
    internal_clear_sweep_area();

    /*razdalja ki jo trenutno meri HC-SR04*/
    internal_dynamic_layer_draw_distance();
    /*trenutna temperatura internega temperaturnega senzorja*/
    internal_dynamic_layer_draw_temp_sens();
    /*trenutna smer joysticka*/
    internal_dynamic_layer_draw_joystick_aim_dir();

    /*sweeper handle na trenutnem kotu (offsetu od base-a)*/
    internal_dynamic_layer_draw_sweeper_line(sweeper_handle_angle, LCD_HALF_WIDTH,
                                             LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET, OUTER_HALF_CIRCLE_RADIUS,
                                             INNER_FULL_CIRCLE_RADIUS);

    internal_dynamic_layer_draw_radar_dots();

    // izracun novega kota - odvisen od dt
    float_t angle_delta = SWEEPER_ANGULAR_VELOCITY * dt;

    if (sweeper_direction) // 0->180
    {
        sweeper_handle_angle += angle_delta;
        /*SWEEPER_HANDLE_THICKNESS_HALF je offset zaradi debeline sweeper handle-a, prepreci da gre handle cez spodnjo
         * mejo (crto) sweeper ekrana
         */
        if (sweeper_handle_angle >= (2 * F_PI - ANGLE_EPSILON - SWEEPER_HANDLE_THICKNESS_HALF))
        {
            sweeper_handle_angle = 2 * F_PI - SWEEPER_HANDLE_THICKNESS_HALF;
            sweeper_direction = 0;
        }
    }
    else // 180->0
    {
        sweeper_handle_angle -= angle_delta;
        /*-||-*/
        if (sweeper_handle_angle <= (F_PI + ANGLE_EPSILON + SWEEPER_HANDLE_THICKNESS_HALF))
        {
            sweeper_handle_angle = F_PI + SWEEPER_HANDLE_THICKNESS_HALF;
            sweeper_direction = 1;
        }
    }
}

static inline void internal_draw_corner(void)
{
    /* za vsak entry v corner_cordinates izrise kotni (corner) element
     * ∀i: corner_cordinates[i][0] - X in Y kordinati za zacetek risanja polovice celote roba v desno
     *     corner_cordinates[i][1] - X in Y kordinati za zacetek risanja polovice celote roba navzdol
     */
    for (uint8_t i = 0; i < NBR_OF_CORNERS; i++)
    {
        UTIL_LCD_FillRect(corner_cordinates[i][0].X, corner_cordinates[i][0].Y,
                          CORNER_ARM_LENGTH + (i == 0 || i == 1 ? 3 : 0), 4,
                          CORNER_COLOR); // rise crto v desno
        UTIL_LCD_FillRect(corner_cordinates[i][1].X, corner_cordinates[i][1].Y, 4,
                          CORNER_ARM_LENGTH + (i == NBR_OF_CORNERS - 1 ? 4 : 0),
                          CORNER_COLOR); // rise crto navzdol
    }
}

static void internal_draw_half_circle(uint16_t x_center, uint16_t y_center, uint16_t radius, uint32_t color)
{
    float_t start_angle = F_PI, end_angle = 2 * F_PI;

    // povecaj/zmanjsaj angle_step za vecjo/manjso natancnost
    for (float_t angle = start_angle; angle <= end_angle; angle += 0.0005f)
    {
        uint16_t x = x_center + (int16_t)(radius * cosf(angle));
        uint16_t y = y_center + (int16_t)(radius * sinf(angle));
        UTIL_LCD_SetPixel(x, y, color);
    }
}

/*uporabi za izracun x kordinate za centriran text na displayu*/
static inline uint16_t internal_center_text_x_cordinate(uint8_t* str, sFONT font)
{
    return (LCD_HALF_WIDTH - (strlen((const char*)str) * font.Width / 2));
}

static void internal_draw_diagonal_lines(uint16_t x_center, uint16_t y_center, uint16_t outer_radius,
                                         uint16_t inner_radius)
{
    // large_slice je razmik med diagonalnimi crtami znotraj polkroga
    float_t large_slice = F_PI / (NBR_OF_DIAGONAL_LINES_INNER + 1);
    // small_slice je razmik med kratkimi diagonalnimi crtami zunaj polkroga
    float_t small_slice = F_PI / (NBR_OF_DIAGONAL_LINES_OUTER);

    for (uint8_t i = 1; i <= NBR_OF_DIAGONAL_LINES_INNER; i++)
    {
        uint16_t x_outer = x_center + (int16_t)(outer_radius * cosf(F_PI + i * large_slice));
        uint16_t y_outer = y_center + (int16_t)(outer_radius * sinf(F_PI + i * large_slice));

        uint16_t x_inner = x_center + (int16_t)(inner_radius * cosf(F_PI + i * large_slice));
        uint16_t y_inner = y_center + (int16_t)(inner_radius * sinf(F_PI + i * large_slice));

        UTIL_LCD_DrawLine(x_inner, y_inner, x_outer, y_outer, BACKGROUND_DIAGONAL_LINES_COLOR);
    }

    for (uint8_t i = 0; i <= NBR_OF_DIAGONAL_LINES_OUTER; i++)
    {
        // crte na indexih deljivih s 5 lezijo na large_slice diagonalah in so malo vecje
        uint8_t extension = (i % 5) == 0 ? 4 : 0;

        uint16_t x_outer = x_center + (int16_t)((outer_radius + 9 + extension) * cosf(F_PI + i * small_slice));
        uint16_t y_outer = y_center + (int16_t)((outer_radius + 9 + extension) * sinf(F_PI + i * small_slice));

        uint16_t x_inner = x_center + (int16_t)((outer_radius + 6 - extension * 0.75f) * cosf(F_PI + i * small_slice));
        uint16_t y_inner = y_center + (int16_t)((outer_radius + 6 - extension * 0.75f) * sinf(F_PI + i * small_slice));

        UTIL_LCD_DrawLine(x_inner, y_inner, x_outer, y_outer, BACKGROUND_DIAGONAL_LINES_COLOR);
    }
}

static inline void internal_dynamic_layer_draw_temp_sens(void)
{
    static uint8_t str[9];
    snprintf((char*)str, 9, "%2.2f$C", (double_t)calc_intern_temp_sens(*internal_temperature_sensor_reading));
    str[8] = '\0';

    // pocistimo obmocje kjer je prej lezal text
    UTIL_LCD_FillRect(INTERNAL_TEMP_TEXT_X, INTERNAL_TEMP_TEXT_Y, INTERNAL_TEMP_TEXT_WIDTH, Font12.Height,
                      0x00000000UL);

    UTIL_LCD_SetFont(&Font12);
    UTIL_LCD_SetBackColor(0x00000000UL);
    UTIL_LCD_SetTextColor(LCD_COLOR_ARGB8888_WHITE);
    UTIL_LCD_DisplayStringAt(INTERNAL_TEMP_TEXT_X, INTERNAL_TEMP_TEXT_Y, str, LEFT_MODE);
}

static inline void internal_dynamic_layer_draw_distance(void)
{
    float_t read_distance;
    if (!HCSR04_ReadDistance(&read_distance))
    {
        return;
    }

    uint8_t str[9];
    snprintf((char*)str, 9, "%3.2fcm", (double_t)read_distance);
    str[8] = '\0';

    // pocistimo obmocje kjer je prej lezal text
    UTIL_LCD_FillRect(LCD_HALF_WIDTH - (DISTANCE_TEXT_WIDTH / 2), DISTANCE_TEXT_Y, DISTANCE_TEXT_WIDTH, Font12.Height,
                      0x00000000UL);

    UTIL_LCD_SetFont(&Font12);
    UTIL_LCD_SetBackColor(0x00000000UL);
    UTIL_LCD_SetTextColor(LCD_COLOR_ARGB8888_WHITE);
    UTIL_LCD_DisplayStringAt(internal_center_text_x_cordinate((uint8_t*)str, Font12), DISTANCE_TEXT_Y, (uint8_t*)str,
                             LEFT_MODE);
}

static inline void internal_dynamic_layer_draw_joystick_aim_dir(void)
{
    uint8_t* str = get_str_from_aim_dir();

    // pocistimo obmocje kjer je prej lezal text
    UTIL_LCD_FillRect(JOYSTICK_TEXT_X_MAX - JOYSTICK_TEXT_WIDTH, JOYSTICK_TEXT_Y, JOYSTICK_TEXT_WIDTH, Font12.Height,
                      0x00000000UL);

    UTIL_LCD_SetFont(&Font12);
    UTIL_LCD_SetBackColor(0x00000000UL);
    UTIL_LCD_SetTextColor(LCD_COLOR_ARGB8888_WHITE);
    UTIL_LCD_DisplayStringAt(JOYSTICK_TEXT_X_MAX - strlen((char*)str) * Font12.Width, JOYSTICK_TEXT_Y, str, LEFT_MODE);
}

static void internal_dynamic_layer_draw_sweeper_line(float_t angle, uint16_t x_center, uint16_t y_center,
                                                     uint16_t outer_radius, uint16_t inner_radius)
{
    // odmik kota levo od sweeper handle-a
    float_t angle_left = angle - SWEEPER_HANDLE_THICKNESS_HALF;
    // odmik kota desno od sweeper handle-a
    float_t angle_right = angle + SWEEPER_HANDLE_THICKNESS_HALF;

    float_t cos_left = cosf(angle_left);
    float_t sin_left = sinf(angle_left);
    float_t cos_right = cosf(angle_right);
    float_t sin_right = sinf(angle_right);

    Point points[4];

    // znotraj levo
    points[0].X = x_center + (int16_t)(inner_radius * cos_left);
    points[0].Y = y_center + (int16_t)(inner_radius * sin_left);

    // zunaj levo
    points[1].X = x_center + (int16_t)(outer_radius * cos_left);
    points[1].Y = y_center + (int16_t)(outer_radius * sin_left);

    // zunaj desno
    points[2].X = x_center + (int16_t)(outer_radius * cos_right);
    points[2].Y = y_center + (int16_t)(outer_radius * sin_right);

    // znotraj desno
    points[3].X = x_center + (int16_t)(inner_radius * cos_right);
    points[3].Y = y_center + (int16_t)(inner_radius * sin_right);

    UTIL_LCD_FillPolygon(points, 4, BACKGROUND_DIAGONAL_LINES_COLOR);

    // Shrani za naslednji clear
    memcpy(last_sweeper_points, points, sizeof(points));
    first_frame = 0;
}

static void internal_dynamic_layer_draw_radar_dots(void)
{
    uint32_t now = HAL_GetTick();

    for (uint16_t i = 0; i < NBR_OF_RADAR_DOTS; i++)
    {
        if (!radar_dots[i].valid)
        {
            continue;
        }

        // fade out cez RADAR_DOT_LIFE_SPAN sekund
        uint32_t age = now - radar_dots[i].timestamp;
        if (age > RADAR_DOT_LIFE_SPAN)
        {
            radar_dots[i].valid = 0;
            continue;
        }

        uint8_t alpha = 0xFFU - (age * 0xFFU / RADAR_DOT_LIFE_SPAN);

        float_t scaled_distance =
            (radar_dots[i].distance / (float_t)MAX_DISTANCE_TO_RENDER_DOT) * OUTER_HALF_CIRCLE_RADIUS;
        int16_t x = LCD_HALF_WIDTH + (int16_t)(scaled_distance * cosf(radar_dots[i].angle));
        int16_t y =
            LCD_DEFAULT_HEIGHT - MAIN_SWEEPER_BOTTOM_OFFSET + (int16_t)(scaled_distance * sinf(radar_dots[i].angle));

        uint32_t color = (alpha << 24) | 0x0018AB18;
        UTIL_LCD_FillCircle(x, y, 4, color);
    }
}

void add_new_radar_dot(float_t distance, float_t angle)
{
    if (distance > MAX_DISTANCE_TO_RENDER_DOT || distance < MIN_DISTANCE_TO_RENDER_DOT)
    {
        return;
    }

    if (radar_dots_next_index >= NBR_OF_RADAR_DOTS)
    {
        radar_dots_next_index = 0;
    }

    radar_dots[radar_dots_next_index].angle = angle;
    radar_dots[radar_dots_next_index].distance = distance;
    radar_dots[radar_dots_next_index].valid = 1;
    radar_dots[radar_dots_next_index].timestamp = HAL_GetTick();
    ++radar_dots_next_index;

    buzzer_short_beep_start();
}