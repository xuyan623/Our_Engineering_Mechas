#include "bsp/board_led.h"

#include "stm32f4xx_hal.h"

#define BOARD_LED_GREEN_PORT          GPIOF
#define BOARD_LED_GREEN_PIN           GPIO_PIN_14
#define BOARD_LED_RED_PORT            GPIOE
#define BOARD_LED_RED_PIN             GPIO_PIN_11
#define BOARD_LED_USER8_PORT          GPIOG
#define BOARD_LED_USER8_FIRST_PIN     GPIO_PIN_1
#define BOARD_LED_USER8_ALL_PINS      (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)

static OmBool g_board_led_ready = OM_FALSE;

static void board_led_write(GPIO_TypeDef* port, uint16_t pin, OmBool on)
{
    /* 开发板 A 型红绿灯按 active-low 处理：输出低电平点亮。 */
    HAL_GPIO_WritePin(port, pin, (on == OM_TRUE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

OmRet board_led_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    gpio_init.Pin = BOARD_LED_RED_PIN;
    HAL_GPIO_Init(BOARD_LED_RED_PORT, &gpio_init);

    gpio_init.Pin = BOARD_LED_GREEN_PIN;
    HAL_GPIO_Init(BOARD_LED_GREEN_PORT, &gpio_init);

    gpio_init.Pin = BOARD_LED_USER8_ALL_PINS;
    HAL_GPIO_Init(BOARD_LED_USER8_PORT, &gpio_init);

    board_led_write(BOARD_LED_RED_PORT, BOARD_LED_RED_PIN, OM_FALSE);
    board_led_write(BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN, OM_FALSE);
    HAL_GPIO_WritePin(BOARD_LED_USER8_PORT, BOARD_LED_USER8_ALL_PINS, GPIO_PIN_SET);

    g_board_led_ready = OM_TRUE;
    return OM_OK;
}

void board_led_set_green(OmBool on)
{
    if (g_board_led_ready != OM_TRUE)
    {
        (void)board_led_init();
    }

    board_led_write(BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN, on);
}

void board_led_set_red(OmBool on)
{
    if (g_board_led_ready != OM_TRUE)
    {
        (void)board_led_init();
    }

    board_led_write(BOARD_LED_RED_PORT, BOARD_LED_RED_PIN, on);
}

void board_led_set_user8(uint8_t index, OmBool on)
{
    uint16_t pin = 0u;

    if (g_board_led_ready != OM_TRUE)
    {
        (void)board_led_init();
    }

    if (index >= 8u)
    {
        return;
    }

    pin = (uint16_t)(BOARD_LED_USER8_FIRST_PIN << index);
    /* PG1-PG8 用户 LED 与板载红绿灯保持同样的 active-low 驱动方式。 */
    HAL_GPIO_WritePin(BOARD_LED_USER8_PORT, pin, (on == OM_TRUE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void board_led_set_user8_mask(uint8_t on_mask)
{
    uint32_t index = 0u;

    for (index = 0u; index < 8u; index++)
    {
        board_led_set_user8((uint8_t)index, ((on_mask & (1u << index)) != 0u) ? OM_TRUE : OM_FALSE);
    }
}

void board_led_set_user8_all(OmBool on)
{
    board_led_set_user8_mask((on == OM_TRUE) ? 0xFFu : 0x00u);
}

void board_led_set_running(void)
{
    board_led_set_red(OM_FALSE);
    board_led_set_green(OM_TRUE);
}

void board_led_set_fault(void)
{
    board_led_set_green(OM_FALSE);
    board_led_set_red(OM_TRUE);
}
