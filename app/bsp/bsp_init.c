#include "bsp/bsp_init.h"

#include "bsp.h"
#include <string.h>

#define BSP_PWM_TIMER_FREQUENCY_HZ (1000000U)
#define BSP_PWM_PERIOD_US          (20000U)
#define BSP_PWM_DEFAULT_PULSE_US   (1000U)
#define BSP_SPI5_TIMEOUT_TICKS     (0xFFFFU)

static BspDeviceRegistry g_bsp_devices = {0};
static OmBool g_bsp_ready = OM_FALSE;
static OmBool g_spi5_ready = OM_FALSE;

static OmRet bsp_register_device_handle(Device** out_device, char* name)
{
    Device* device = OM_NULL;

    if (out_device == OM_NULL || name == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    device = device_find(name);
    if (device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *out_device = device;
    return OM_OK;
}

static void bsp_gpio_outputs_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    gpio_init.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOE, &gpio_init);

    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOF, &gpio_init);

    /* PB5 作为 IMU 加热控制输出，默认关闭。 */
    gpio_init.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void bsp_pwm_timer_common_init(TIM_TypeDef* instance)
{
    uint32_t prescaler = (uint32_t)((SystemCoreClock / 2U) / BSP_PWM_TIMER_FREQUENCY_HZ) - 1U;

    instance->CR1 = 0U;
    instance->PSC = prescaler;
    instance->ARR = BSP_PWM_PERIOD_US - 1U;
    instance->CCR1 = BSP_PWM_DEFAULT_PULSE_US;
    instance->CCR2 = BSP_PWM_DEFAULT_PULSE_US;
    instance->CCR3 = BSP_PWM_DEFAULT_PULSE_US;
    instance->CCR4 = BSP_PWM_DEFAULT_PULSE_US;

    instance->CCMR1 =
        (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    instance->CCMR2 =
        (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    instance->CR1 |= TIM_CR1_ARPE;
    instance->EGR = TIM_EGR_UG;
    instance->CR1 |= TIM_CR1_CEN;
}

static void bsp_tim4_tim5_pwm_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_init.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &gpio_init);

    gpio_init.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio_init.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &gpio_init);

    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOI, &gpio_init);

    bsp_pwm_timer_common_init(TIM4);
    bsp_pwm_timer_common_init(TIM5);
}

OmRet bsp_spi5_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (g_spi5_ready == OM_TRUE)
    {
        return OM_OK;
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_SPI5_FORCE_RESET();
    __HAL_RCC_SPI5_RELEASE_RESET();

    /* PF6 作为 IMU 片选。 */
    gpio_init.Pin = GPIO_PIN_6;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio_init);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);

    /* PF7/PF8/PF9 分别映射到 SPI5_SCK/MISO/MOSI。 */
    gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &gpio_init);

    SPI5->CR1 = 0U;
    SPI5->CR2 = 0U;
    SPI5->CRCPR = 7U;
    SPI5->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
    SPI5->CR1 |= SPI_CR1_SPE;

    g_spi5_ready = OM_TRUE;
    return OM_OK;
}

const BspDeviceRegistry* bsp_get_device_registry(void)
{
    return &g_bsp_devices;
}

OmRet bsp_register_all(void)
{
    OmRet ret = OM_OK;

    if (g_bsp_ready == OM_TRUE)
    {
        return OM_OK;
    }

    memset(&g_bsp_devices, 0, sizeof(g_bsp_devices));

    ret = bsp_register_device_handle(&g_bsp_devices.can1, "can1");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.can2, "can2");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.usart1, "usart1");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.usart3, "usart3");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.usart6, "usart6");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.uart7, "uart7");
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = bsp_register_device_handle(&g_bsp_devices.uart8, "uart8");
    if (ret != OM_OK)
    {
        return ret;
    }

    bsp_gpio_outputs_init();
    bsp_tim4_tim5_pwm_init();

    g_bsp_ready = OM_TRUE;
    return OM_OK;
}

OmRet bsp_init_all(void)
{
    return bsp_register_all();
}

uint8_t SPI5_ReadWriteByte(uint8_t tx_data)
{
    uint32_t timeout = BSP_SPI5_TIMEOUT_TICKS;

    if (g_spi5_ready != OM_TRUE)
    {
        return 0xFFU;
    }

    while ((SPI5->SR & SPI_SR_TXE) == 0U)
    {
        if (timeout-- == 0U)
        {
            return 0xFFU;
        }
    }

    *(__IO uint8_t*)&SPI5->DR = tx_data;
    timeout = BSP_SPI5_TIMEOUT_TICKS;
    while ((SPI5->SR & SPI_SR_RXNE) == 0U)
    {
        if (timeout-- == 0U)
        {
            return 0xFFU;
        }
    }

    return *(__IO uint8_t*)&SPI5->DR;
}

void mpu6500_SPI_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
}

void mpu6500_SPI_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
}

void mpu6500_delay_ms(uint16_t ms)
{
    om_cpu_delay_ms((float)ms);
}

void mpu6500_delay_us(uint32_t us)
{
    DWT_Delay((float)us / 1000000.0f);
}
