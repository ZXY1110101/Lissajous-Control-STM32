#include "lcd_touch_ui.h"

#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include "lissajous_pll.h"
#include "touch.h"

#define LCD_TOUCH_SCAN_PERIOD_MS  15U
#define LCD_TOUCH_MARGIN_PX       10U

typedef struct
{
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
} LCDTouchUI_Rect;

static LCDTouchUI_Rect button_m1;
static LCDTouchUI_Rect button_m2;
static uint8_t selected_multiplier = LISSAJOUS_PLL_DEFAULT_MULTIPLIER;
static uint8_t touch_was_down;
static uint8_t pressed_multiplier;
static uint32_t last_touch_scan_ms;

static uint8_t LCDTouchUI_PointInside(const LCDTouchUI_Rect *rect,
                                      uint16_t x,
                                      uint16_t y)
{
  return (uint8_t)((x >= rect->x1) && (x <= rect->x2) &&
                   (y >= rect->y1) && (y <= rect->y2));
}

static void LCDTouchUI_DrawCenteredText(const LCDTouchUI_Rect *rect,
                                        char *text,
                                        uint8_t font_size,
                                        uint16_t color)
{
  uint16_t text_width = (uint16_t)(strlen(text) * (font_size / 2U));
  uint16_t x = rect->x1;
  uint16_t y = rect->y1;

  if ((rect->x2 - rect->x1 + 1U) > text_width)
  {
    x = (uint16_t)(rect->x1 +
        ((rect->x2 - rect->x1 + 1U - text_width) / 2U));
  }
  if ((rect->y2 - rect->y1 + 1U) > font_size)
  {
    y = (uint16_t)(rect->y1 +
        ((rect->y2 - rect->y1 + 1U - font_size) / 2U));
  }

  lcd_show_string(x, y, (uint16_t)(rect->x2 - x + 1U),
                  (uint16_t)(rect->y2 - y + 1U), font_size, text, color);
}

static void LCDTouchUI_DrawButton(uint8_t multiplier, uint8_t pressed)
{
  const LCDTouchUI_Rect *rect = (multiplier == 1U) ? &button_m1 : &button_m2;
  uint16_t fill_color;
  uint16_t text_color;
  char *label = (multiplier == 1U) ? "M = 1" : "M = 2";

  if (pressed != 0U)
  {
    fill_color = YELLOW;
    text_color = RED;
  }
  else if (selected_multiplier == multiplier)
  {
    fill_color = GREEN;
    text_color = BLACK;
  }
  else
  {
    fill_color = LIGHTBLUE;
    text_color = BLUE;
  }

  lcd_fill(rect->x1, rect->y1, rect->x2, rect->y2, fill_color);
  lcd_draw_rectangle(rect->x1, rect->y1, rect->x2, rect->y2, BLACK);
  lcd_draw_rectangle((uint16_t)(rect->x1 + 1U), (uint16_t)(rect->y1 + 1U),
                     (uint16_t)(rect->x2 - 1U), (uint16_t)(rect->y2 - 1U), BLACK);
  LCDTouchUI_DrawCenteredText(rect, label, 24U, text_color);
}

static void LCDTouchUI_DrawStatus(char *prefix, uint8_t multiplier,
                                  uint16_t color)
{
  char status[24];
  uint16_t y = (uint16_t)(button_m1.y2 + 20U);

  (void)snprintf(status, sizeof(status), "%s M=%u", prefix, multiplier);
  lcd_fill(LCD_TOUCH_MARGIN_PX, y,
           (uint16_t)(lcddev.width - LCD_TOUCH_MARGIN_PX - 1U),
           (uint16_t)(y + 20U), WHITE);
  lcd_show_string(LCD_TOUCH_MARGIN_PX, y,
                  (uint16_t)(lcddev.width - (2U * LCD_TOUCH_MARGIN_PX)),
                  20U, 16U, status, color);
}

static void LCDTouchUI_DrawScreen(void)
{
  uint16_t usable_width;
  uint16_t button_width;

  lcd_clear(WHITE);
  lcd_show_string(LCD_TOUCH_MARGIN_PX, 18U,
                  (uint16_t)(lcddev.width - (2U * LCD_TOUCH_MARGIN_PX)),
                  24U, 16U, "LISSAJOUS MULTIPLIER", BLUE);
  lcd_show_string(LCD_TOUCH_MARGIN_PX, 48U,
                  (uint16_t)(lcddev.width - (2U * LCD_TOUCH_MARGIN_PX)),
                  20U, 16U, "Touch a button", BLACK);

  usable_width = (uint16_t)(lcddev.width - (3U * LCD_TOUCH_MARGIN_PX));
  button_width = usable_width / 2U;
  button_m1.x1 = LCD_TOUCH_MARGIN_PX;
  button_m1.x2 = (uint16_t)(button_m1.x1 + button_width - 1U);
  button_m2.x1 = (uint16_t)(button_m1.x2 + LCD_TOUCH_MARGIN_PX + 1U);
  button_m2.x2 = (uint16_t)(lcddev.width - LCD_TOUCH_MARGIN_PX - 1U);
  button_m1.y1 = (uint16_t)(lcddev.height / 3U);
  button_m1.y2 = (uint16_t)((lcddev.height * 2U) / 3U);
  button_m2.y1 = button_m1.y1;
  button_m2.y2 = button_m1.y2;

  LCDTouchUI_DrawButton(1U, 0U);
  LCDTouchUI_DrawButton(2U, 0U);
  LCDTouchUI_DrawStatus("ACTIVE:", selected_multiplier, GREEN);
}

void LCDTouchUI_Init(uint8_t initial_multiplier)
{
  lcd_init();
  (void)tp_dev.init();

  selected_multiplier = (initial_multiplier == 1U) ? 1U : 2U;
  touch_was_down = 0U;
  pressed_multiplier = 0U;
  last_touch_scan_ms = HAL_GetTick();
  LCDTouchUI_DrawScreen();
}

void LCDTouchUI_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t touch_is_down;
  uint8_t hit_multiplier = 0U;

  if ((uint32_t)(now_ms - last_touch_scan_ms) < LCD_TOUCH_SCAN_PERIOD_MS)
  {
    return;
  }
  last_touch_scan_ms = now_ms;

  (void)tp_dev.scan(0U);
  touch_is_down = (uint8_t)((tp_dev.sta & TP_PRES_DOWN) != 0U);

  if ((touch_is_down != 0U) && (touch_was_down == 0U))
  {
    if (LCDTouchUI_PointInside(&button_m1, tp_dev.x[0], tp_dev.y[0]) != 0U)
    {
      hit_multiplier = 1U;
    }
    else if (LCDTouchUI_PointInside(&button_m2, tp_dev.x[0], tp_dev.y[0]) != 0U)
    {
      hit_multiplier = 2U;
    }

    if (hit_multiplier != 0U)
    {
      pressed_multiplier = hit_multiplier;
      LCDTouchUI_DrawButton(hit_multiplier, 1U);
      LCDTouchUI_DrawStatus("PRESSED:", hit_multiplier, RED);

      if (LissajousPLL_SetMultiplier(hit_multiplier) == HAL_OK)
      {
        selected_multiplier = hit_multiplier;
      }
    }
  }
  else if ((touch_is_down == 0U) && (touch_was_down != 0U))
  {
    if (pressed_multiplier != 0U)
    {
      LCDTouchUI_DrawButton(1U, 0U);
      LCDTouchUI_DrawButton(2U, 0U);
      LCDTouchUI_DrawStatus("ACTIVE:", selected_multiplier, GREEN);
      pressed_multiplier = 0U;
    }
  }

  touch_was_down = touch_is_down;
}
