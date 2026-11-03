#include "amplitude_panel.h"

#define KEY_COUNT       4U
#define KEY_DEBOUNCE_MS 20U

/*
 * CubeMX GPIO labels required by this file:
 * A0, A1, EN
 * KEY1, KEY2, KEY3, KEY4
 * LED1, LED2, LED3, LED4
 */
static GPIO_TypeDef *const key_ports[KEY_COUNT] =
{
    KEY1_GPIO_Port, KEY2_GPIO_Port, KEY3_GPIO_Port, KEY4_GPIO_Port
};

static const uint16_t key_pins[KEY_COUNT] =
{
    KEY1_Pin, KEY2_Pin, KEY3_Pin, KEY4_Pin
};

static GPIO_TypeDef *const led_ports[KEY_COUNT] =
{
    LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port, LED4_GPIO_Port
};

static const uint16_t led_pins[KEY_COUNT] =
{
    LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin
};

/*
 * 每个幅度档对应的 ADG704 地址码，数值的 bit1:bit0 = A1:A0：
 * 0 -> S1，1 -> S2，2 -> S3，3 -> S4。
 *
 * 当前硬件为 LEVEL1->S1、LEVEL2->S2、LEVEL3->S3、LEVEL4->S4。
 * 如果实际焊接的四路顺序不同，只修改这一张表。
 */
static const uint8_t mux_code_for_level[KEY_COUNT] = {0U, 1U, 2U, 3U};

static uint8_t raw_pressed[KEY_COUNT];
static uint8_t stable_pressed[KEY_COUNT];
static uint32_t raw_change_tick[KEY_COUNT];
static AmpLevel current_level = AMP_LEVEL_NONE;

static uint8_t KeyIsPressed(uint8_t index)
{
    return (HAL_GPIO_ReadPin(key_ports[index], key_pins[index]) == GPIO_PIN_RESET)
               ? 1U
               : 0U;
}

static void ShowOneLed(AmpLevel level)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; ++i)
    {
        HAL_GPIO_WritePin(
            led_ports[i],
            led_pins[i],
            (i == (uint8_t)level) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void AmpPanel_Disable(void)
{
    uint8_t i;

    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

    for (i = 0U; i < KEY_COUNT; ++i)
    {
        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
    }

    current_level = AMP_LEVEL_NONE;
}

void AmpPanel_SetLevel(AmpLevel level)
{
    uint8_t code;

    if ((uint8_t)level >= KEY_COUNT)
    {
        return;
    }

    code = mux_code_for_level[(uint8_t)level];

    /* 切换地址期间先关闭模拟开关，避免输出瞬态串档。 */
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
        A0_GPIO_Port,
        A0_Pin,
        (code & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
        A1_GPIO_Port,
        A1_Pin,
        (code & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

    ShowOneLed(level);
    current_level = level;
}

void AmpPanel_Init(AmpLevel default_level)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    AmpPanel_Disable();

    for (i = 0U; i < KEY_COUNT; ++i)
    {
        raw_pressed[i] = KeyIsPressed(i);
        stable_pressed[i] = 0U;
        raw_change_tick[i] = now;
    }

    if ((uint8_t)default_level < KEY_COUNT)
    {
        AmpPanel_SetLevel(default_level);
    }
}

void AmpPanel_Task(void)
{
    uint8_t i;
    uint8_t pressed;
    uint32_t now = HAL_GetTick();

    for (i = 0U; i < KEY_COUNT; ++i)
    {
        pressed = KeyIsPressed(i);

        if (pressed != raw_pressed[i])
        {
            raw_pressed[i] = pressed;
            raw_change_tick[i] = now;
        }
        else if ((pressed != stable_pressed[i]) &&
                 ((uint32_t)(now - raw_change_tick[i]) >= KEY_DEBOUNCE_MS))
        {
            stable_pressed[i] = pressed;

            /* 只响应确认后的按下沿，长按不会重复执行。 */
            if (pressed != 0U)
            {
                AmpPanel_SetLevel((AmpLevel)i);
            }
        }
    }
}

AmpLevel AmpPanel_GetLevel(void)
{
    return current_level;
}
