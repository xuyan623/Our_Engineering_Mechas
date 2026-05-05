#include "bsp/imu_bsp.h"

#include "bsp.h"
#include "bsp/bsp_init.h"
#include "core/om_interrupt.h"
#include "driver/mpu6500/mpu6500.h"
#include "driver/mpu6500/mpu6500_reg.h"
#include <string.h>

#define IMU_BSP_DRDY_PORT            GPIOB
#define IMU_BSP_DRDY_PIN             GPIO_PIN_8
#define IMU_BSP_DMA_FRAME_LEN        (21u)
#define IMU_BSP_RX_DMA_STREAM        DMA2_Stream5
#define IMU_BSP_RX_DMA_CHANNEL       DMA_CHANNEL_7
#define IMU_BSP_TX_DMA_STREAM        DMA2_Stream4
#define IMU_BSP_TX_DMA_CHANNEL       DMA_CHANNEL_2
#define IMU_BSP_IRQ_PRIORITY         (5u)

/* 板级 IMU 采样运行时状态：
 * - dma_rx_frame 保存本次 DMA 读回的完整 21 字节帧
 * - raw_buffers 做双缓冲，向上层提供“最近完成的一帧”
 * - dma_busy 用来保证同一时刻只允许一笔 SPI5 DMA 采样在飞
 */
typedef struct
{
    DMA_HandleTypeDef rx_dma;
    DMA_HandleTypeDef tx_dma;
    uint8_t tx_frame[IMU_BSP_DMA_FRAME_LEN];
    uint8_t dma_rx_frame[IMU_BSP_DMA_FRAME_LEN];
    uint8_t raw_buffers[2][IMU_BSP_RAW_PAYLOAD_LEN];
    volatile uint8_t latest_slot;
    volatile uint8_t next_write_slot;
    volatile uint8_t dma_busy;
    OmBool ready;
} ImuBspRuntime;

static ImuBspRuntime g_imu_bsp_runtime = {0};
volatile ImuBspDebugState g_imu_bsp_debug = {0};

static void imu_bsp_reset_dma_handle(DMA_HandleTypeDef* dma_handle)
{
    if (dma_handle == OM_NULL)
    {
        return;
    }

    dma_handle->State = HAL_DMA_STATE_READY;
    dma_handle->ErrorCode = HAL_DMA_ERROR_NONE;
    __HAL_UNLOCK(dma_handle);
}

static void imu_bsp_stop_spi_dma_transfer(void)
{
    /* 停止本轮 DMA 请求并释放片选。
     * 这个动作既用于正常完成，也用于异常收尾。
     */
    CLEAR_BIT(SPI5->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE(&g_imu_bsp_runtime.rx_dma);
    __HAL_DMA_DISABLE(&g_imu_bsp_runtime.tx_dma);
    mpu6500_SPI_NS_H();
}

static void imu_bsp_tx_complete_callback(DMA_HandleTypeDef* dma_handle)
{
    (void)dma_handle;
}

static void imu_bsp_dma_error_callback(DMA_HandleTypeDef* dma_handle)
{
    uint32_t primask = om_hw_disable_interrupt();

    (void)dma_handle;

    /* 错误路径必须把总线恢复到可再次启动的状态，
     * 否则下一次 DRDY 到来时会一直卡在 busy。
     */
    imu_bsp_stop_spi_dma_transfer();
    imu_bsp_reset_dma_handle(&g_imu_bsp_runtime.rx_dma);
    imu_bsp_reset_dma_handle(&g_imu_bsp_runtime.tx_dma);
    g_imu_bsp_runtime.dma_busy = 0u;
    g_imu_bsp_debug.dma_error_count++;

    om_hw_restore_interrupt(primask);
}

static void imu_bsp_rx_complete_callback(DMA_HandleTypeDef* dma_handle)
{
    uint32_t primask = om_hw_disable_interrupt();
    uint8_t write_slot = 0u;

    (void)dma_handle;

    /* DMA 帧第 0 字节是发送阶段的寄存器地址回读，
     * 真正有效的采样数据从 rx[1] 开始，长度固定 20 字节。
     */
    write_slot = g_imu_bsp_runtime.next_write_slot;
    memcpy(g_imu_bsp_runtime.raw_buffers[write_slot], &g_imu_bsp_runtime.dma_rx_frame[1], IMU_BSP_RAW_PAYLOAD_LEN);

    imu_bsp_stop_spi_dma_transfer();

    /* 双缓冲只保留“最新完成的一帧”，不排历史队列。
     * 上层控制逻辑只关心最近状态，不需要补消费旧帧。
     */
    g_imu_bsp_runtime.latest_slot = write_slot;
    g_imu_bsp_runtime.next_write_slot = (uint8_t)(write_slot ^ 0x01u);
    g_imu_bsp_runtime.dma_busy = 0u;

    g_imu_bsp_debug.dma_done_count++;
    g_imu_bsp_debug.latest_seq++;

    om_hw_restore_interrupt(primask);
}

static void imu_bsp_prepare_dma_tx_frame(void)
{
    uint32_t index = 0u;

    /* 固定读取 MPU6500 的 0x3B..0x4E：
     * - 第 0 字节发寄存器起始地址 | 0x80
     * - 后面补 0xFF 作为时钟占位
     */
    g_imu_bsp_runtime.tx_frame[0] = (uint8_t)(MPU_ACCEL_XOUT_H | 0x80u);
    for (index = 1u; index < IMU_BSP_DMA_FRAME_LEN; index++)
    {
        g_imu_bsp_runtime.tx_frame[index] = 0xFFu;
    }
}

static void imu_bsp_configure_drdy_exti(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* 原理图锁定 MPU6500 的 IMU_INT 在 PB8，这里按 EXTI8 上升沿配置。 */
    gpio_init.Pin = IMU_BSP_DRDY_PIN;
    gpio_init.Mode = GPIO_MODE_IT_RISING;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IMU_BSP_DRDY_PORT, &gpio_init);

    __HAL_GPIO_EXTI_CLEAR_IT(IMU_BSP_DRDY_PIN);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, IMU_BSP_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static OmRet imu_bsp_configure_dma_handle(DMA_HandleTypeDef* dma_handle, DMA_Stream_TypeDef* instance, uint32_t channel,
                                          uint32_t direction, uint32_t priority)
{
    if (dma_handle == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(dma_handle, 0, sizeof(*dma_handle));
    dma_handle->Instance = instance;
    dma_handle->Init.Channel = channel;
    dma_handle->Init.Direction = direction;
    dma_handle->Init.PeriphInc = DMA_PINC_DISABLE;
    dma_handle->Init.MemInc = DMA_MINC_ENABLE;
    dma_handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_handle->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_handle->Init.Mode = DMA_NORMAL;
    dma_handle->Init.Priority = priority;
    dma_handle->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma_handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    dma_handle->Init.MemBurst = DMA_MBURST_SINGLE;
    dma_handle->Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(dma_handle) != HAL_OK)
    {
        return OM_ERROR;
    }

    return OM_OK;
}

static OmRet imu_bsp_configure_spi_dma(void)
{
    OmRet ret = OM_OK;

    /* 资源映射沿用旧工程在开发板 A 型上的 SPI5 DMA 分配：
     * - RX: DMA2_Stream5 / Channel7
     * - TX: DMA2_Stream4 / Channel2
     */
    __HAL_RCC_DMA2_CLK_ENABLE();

    ret = imu_bsp_configure_dma_handle(&g_imu_bsp_runtime.rx_dma, IMU_BSP_RX_DMA_STREAM, IMU_BSP_RX_DMA_CHANNEL,
                                       DMA_PERIPH_TO_MEMORY, DMA_PRIORITY_VERY_HIGH);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = imu_bsp_configure_dma_handle(&g_imu_bsp_runtime.tx_dma, IMU_BSP_TX_DMA_STREAM, IMU_BSP_TX_DMA_CHANNEL,
                                       DMA_MEMORY_TO_PERIPH, DMA_PRIORITY_MEDIUM);
    if (ret != OM_OK)
    {
        return ret;
    }

    if (HAL_DMA_RegisterCallback(&g_imu_bsp_runtime.rx_dma, HAL_DMA_XFER_CPLT_CB_ID, imu_bsp_rx_complete_callback) != HAL_OK)
    {
        return OM_ERROR;
    }
    if (HAL_DMA_RegisterCallback(&g_imu_bsp_runtime.rx_dma, HAL_DMA_XFER_ERROR_CB_ID, imu_bsp_dma_error_callback) != HAL_OK)
    {
        return OM_ERROR;
    }
    if (HAL_DMA_RegisterCallback(&g_imu_bsp_runtime.tx_dma, HAL_DMA_XFER_CPLT_CB_ID, imu_bsp_tx_complete_callback) != HAL_OK)
    {
        return OM_ERROR;
    }
    if (HAL_DMA_RegisterCallback(&g_imu_bsp_runtime.tx_dma, HAL_DMA_XFER_ERROR_CB_ID, imu_bsp_dma_error_callback) != HAL_OK)
    {
        return OM_ERROR;
    }

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, IMU_BSP_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, IMU_BSP_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

    return OM_OK;
}

static void imu_bsp_start_dma_transfer_from_isr(void)
{
    HAL_StatusTypeDef rx_status = HAL_OK;
    HAL_StatusTypeDef tx_status = HAL_OK;

    if (g_imu_bsp_runtime.ready != OM_TRUE)
    {
        return;
    }

    /* DRDY 到来时如果上一笔采样还没收完，本次直接记一次 drop。
     * 这里不排队，不重入，保持 ISR 快进快出。
     */
    if (g_imu_bsp_runtime.dma_busy != 0u)
    {
        g_imu_bsp_debug.dma_drop_count++;
        return;
    }

    g_imu_bsp_runtime.dma_busy = 1u;

    while ((SPI5->SR & SPI_SR_RXNE) != 0u)
    {
        (void)*(__IO uint8_t*)&SPI5->DR;
    }
    (void)SPI5->SR;

    /* 先拉低片选，再同时启动 RX/TX DMA，最后打开 SPI DMA 请求位。 */
    mpu6500_SPI_NS_L();

    rx_status = HAL_DMA_Start_IT(&g_imu_bsp_runtime.rx_dma, (uint32_t)&SPI5->DR, (uint32_t)g_imu_bsp_runtime.dma_rx_frame,
                                 IMU_BSP_DMA_FRAME_LEN);
    tx_status = HAL_DMA_Start_IT(&g_imu_bsp_runtime.tx_dma, (uint32_t)g_imu_bsp_runtime.tx_frame, (uint32_t)&SPI5->DR,
                                 IMU_BSP_DMA_FRAME_LEN);

    if (rx_status != HAL_OK || tx_status != HAL_OK)
    {
        imu_bsp_stop_spi_dma_transfer();
        imu_bsp_reset_dma_handle(&g_imu_bsp_runtime.rx_dma);
        imu_bsp_reset_dma_handle(&g_imu_bsp_runtime.tx_dma);
        g_imu_bsp_runtime.dma_busy = 0u;
        g_imu_bsp_debug.dma_error_count++;
        return;
    }

    SET_BIT(SPI5->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    g_imu_bsp_debug.dma_start_count++;
}

OmRet imu_bsp_init(void)
{
    OmRet ret = OM_OK;

    if (g_imu_bsp_runtime.ready == OM_TRUE)
    {
        return OM_OK;
    }

    memset((void*)&g_imu_bsp_runtime, 0, sizeof(g_imu_bsp_runtime));
    memset((void*)&g_imu_bsp_debug, 0, sizeof(g_imu_bsp_debug));

    ret = bsp_spi5_init();
    if (ret != OM_OK)
    {
        return ret;
    }

    /* imu_bsp_init 只做“异步采样链”初始化：
     * SPI 同步寄存器读写依旧由 mpu6500_init()/ist8310_init() 在上电阶段完成。
     */
    imu_bsp_prepare_dma_tx_frame();
    ret = imu_bsp_configure_spi_dma();
    if (ret != OM_OK)
    {
        return ret;
    }

    imu_bsp_configure_drdy_exti();
    CLEAR_BIT(SPI5->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    mpu6500_SPI_NS_H();

    g_imu_bsp_runtime.ready = OM_TRUE;
    return OM_OK;
}

OmBool imu_bsp_fetch_latest_raw(uint8_t out_payload[IMU_BSP_RAW_PAYLOAD_LEN], uint32_t* out_seq)
{
    uint32_t primask = 0u;
    uint8_t latest_slot = 0u;
    uint32_t latest_seq = 0u;

    if (out_payload == OM_NULL || g_imu_bsp_runtime.ready != OM_TRUE)
    {
        return OM_FALSE;
    }

    /* 这里短暂关中断只为保证：
     * latest_seq 与对应槽的数据拷贝是同一时刻的一致快照。
     */
    primask = om_hw_disable_interrupt();
    latest_seq = g_imu_bsp_debug.latest_seq;
    latest_slot = g_imu_bsp_runtime.latest_slot;

    if (latest_seq != 0u)
    {
        memcpy(out_payload, g_imu_bsp_runtime.raw_buffers[latest_slot], IMU_BSP_RAW_PAYLOAD_LEN);
    }
    om_hw_restore_interrupt(primask);

    if (latest_seq == 0u)
    {
        return OM_FALSE;
    }

    if (out_seq != OM_NULL)
    {
        *out_seq = latest_seq;
    }

    return OM_TRUE;
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin != IMU_BSP_DRDY_PIN)
    {
        return;
    }

    /* HAL 统一 EXTI 回调最终汇到这里，再转成一次 IMU 采样启动。 */
    g_imu_bsp_debug.drdy_irq_count++;
    imu_bsp_start_dma_transfer_from_isr();
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(IMU_BSP_DRDY_PIN);
}

void DMA2_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_imu_bsp_runtime.rx_dma);
}

void DMA2_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_imu_bsp_runtime.tx_dma);
}
