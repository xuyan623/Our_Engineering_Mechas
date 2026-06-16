#include "module/motor_tx_dispatch/motor_tx_dispatch.h"

#include "atomic/atomic_simple.h"
#include "data_struct/mpsc_ringbuf.h"
#include <string.h>

#define MTX_QUEUE_CAPACITY (16u)

static MpscRingbuf g_motor_tx_dispatch_queue = {0};
static uint8_t g_motor_tx_dispatch_storage[MTX_QUEUE_CAPACITY] = {0u};
static OmAtomicU8 g_motor_tx_dispatch_ready[MTX_QUEUE_CAPACITY] = {0};
static OmAtomicUint g_motor_tx_dispatch_initialized = 0u;
static OmAtomicUint g_motor_tx_dispatch_overflow_flag = 0u;

void motor_tx_dispatch_init(void)
{
    if (OM_LOAD_ACQ(&g_motor_tx_dispatch_initialized) != 0u)
    {
        return;
    }

    (void)mpscrb_init(
        &g_motor_tx_dispatch_queue,
        g_motor_tx_dispatch_storage,
        g_motor_tx_dispatch_ready,
        sizeof(uint8_t),
        MTX_QUEUE_CAPACITY);
    OM_STORE_RLX(&g_motor_tx_dispatch_overflow_flag, 0u);
    OM_STORE_REL(&g_motor_tx_dispatch_initialized, 1u);
}

OmRet motor_tx_dispatch_submit(MotorTxRequestSource source)
{
    uint8_t source_value = (uint8_t)source;

    if ((uint32_t)source >= (uint32_t)MOTOR_TX_SOURCE_COUNT)
    {
        return OM_ERROR_PARAM;
    }

    if (OM_LOAD_ACQ(&g_motor_tx_dispatch_initialized) == 0u)
    {
        return OM_ERROR;
    }

    if (mpscrb_in(&g_motor_tx_dispatch_queue, &source_value) != true)
    {
        OM_STORE_RLX(&g_motor_tx_dispatch_overflow_flag, 1u);
        return OM_ERR_OVERFLOW;
    }

    return OM_OK;
}

uint32_t mtx_drain(void)
{
    uint32_t sources_mask = 0u;
    uint8_t source_value = 0u;

    if (OM_LOAD_ACQ(&g_motor_tx_dispatch_initialized) == 0u)
    {
        return 0u;
    }

    while (mpscrb_out(&g_motor_tx_dispatch_queue, &source_value) == true)
    {
        if ((uint32_t)source_value < (uint32_t)MOTOR_TX_SOURCE_COUNT)
        {
            sources_mask |= (1u << source_value);
        }
    }

    return sources_mask;
}

OmBool mtx_take_overflow(void)
{
    if (OM_LOAD_ACQ(&g_motor_tx_dispatch_initialized) == 0u)
    {
        return OM_FALSE;
    }

    return (OM_SWAP_ACQ(&g_motor_tx_dispatch_overflow_flag, 0u) != 0u) ? OM_TRUE : OM_FALSE;
}
