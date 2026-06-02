#include "task/motor_communications_task/mct_internal.h"

/* mct_vendor.c 只承接 vendor 维度的静态接线与启动期 bring-up。
 * 这里不跑主循环，也不导出观测快照。
 */

/* DJI 只作为 CAN1 上的一个 vendor bus 被正式接入。 */
static OmRet mct_register_dji(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MCT_DJI_CHASSIS_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_dji_chassis_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        ret = motor_attach_dji(
            &runtime->dji_chassis_motors[index],
            g_mct_dji_chassis_configs[index].name,
            &runtime->dji_bus,
            &runtime->dji_chassis_drivers[index],
            DJI_MOTOR_TYPE_C620,
            g_mct_dji_chassis_configs[index].id,
            DJI_CTRL_MODE_CURRENT,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        ret = motor_recovery_register_entry(
            &runtime->dji_chassis_motors[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    if (app_motor_profile_is_present(APP_MOTOR_ROLE_ROLL3) != OM_TRUE)
    {
        return OM_OK;
    }

    ret = motor_attach_dji(
        &runtime->dji_roll3_motor,
        APP_MOTOR_NAME_ROLL3,
        &runtime->dji_bus,
        &runtime->dji_roll3_driver,
        DJI_MOTOR_TYPE_GM6020,
        MCT_DJI_ROLL3_ID,
        DJI_CTRL_MODE_CURRENT,
        MOTOR_CONTROL_MODE_DISABLED);
    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_recovery_register_entry(&runtime->dji_roll3_motor);
}

/* P1010B 注册阶段只完成：
 * - bus/driver/motor 对象绑定
 * - 默认配置写入
 * - recovery entry 注册
 *
 * 真正依赖电机应答的 disable/set_mode/set_active_report/enable
 * 留到启动期 prepare。
 */
static OmRet mct_register_p1010b(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        ret = motor_attach_p1010b(
            &runtime->p1010b_motors[index],
            g_mct_p1010b_configs[index].name,
            &runtime->p1010b_bus,
            &runtime->p1010b_drivers[index],
            g_mct_p1010b_configs[index].id,
            P1010B_MODE_CURRENT,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        motor_recovery_configure_p1010b_driver(&runtime->p1010b_drivers[index]);

        ret = motor_recovery_register_entry(
            &runtime->p1010b_motors[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

/* Damiao 注册阶段默认保持 disabled。
 * profile_role 只影响：
 * - 是否正式接入当前应用
 * - 是否加入 recovery
 * - 是否参与启动期 enable
 */
static OmRet mct_register_damiao(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        ret = motor_attach_damiao(
            &runtime->damiao_motors[index],
            g_mct_damiao_configs[index].name,
            &runtime->damiao_bus,
            &runtime->damiao_drivers[index],
            g_mct_damiao_configs[index].type,
            g_mct_damiao_configs[index].can_id,
            g_mct_damiao_configs[index].master_id,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        ret = motor_recovery_register_entry(
            &runtime->damiao_motors[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

/* GO8010 仍然直接挂正式通信任务拥有的 USART6。 */
static OmRet mct_register_go8010(MctRuntime* runtime)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (app_motor_profile_is_present(APP_MOTOR_ROLE_PITCH2) != OM_TRUE)
    {
        return OM_OK;
    }

    ret = motor_attach_go8010(
        &runtime->go8010_pitch2_motor,
        APP_MOTOR_NAME_PITCH2,
        &runtime->go8010_bus,
        &runtime->go8010_pitch2_driver,
        MCT_GO8010_PITCH2_ID,
        MOTOR_CONTROL_MODE_DISABLED);
    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_recovery_register_entry(&runtime->go8010_pitch2_motor);
}

/* P1010B 启动期固定四步：
 * disable -> set_mode -> set_active_report -> enable
 */
static OmRet mct_prepare_p1010b_motor(Motor* motor)
{
    P1010BDriver* driver = OM_NULL;
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (motor == OM_NULL || motor->binding.p1010b.driver == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    driver = motor->binding.p1010b.driver;
    motor_recovery_configure_p1010b_driver(driver);

    ret = p1010b_disable(driver, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_mode(driver, driver->config.defaultMode, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_active_report(
        driver,
        &driver->runtime.activeReport,
        0u,
        &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_enable(driver, 0u, &response);
    if (ret == OM_OK)
    {
        motor_recovery_notify_p1010b_enabled(motor);
    }

    return ret;
}

/* 启动期对两台 P1010B 都尝试做一次完整 bring-up。
 * 单台缺席不应阻塞 owner 任务启动。
 */
static OmRet mct_prepare_p1010b(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet last_error = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        OmRet ret = mct_prepare_p1010b_motor(&runtime->p1010b_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    return last_error;
}

/* Damiao 启动期先显式写 MIT 模式，再进入 enable 态。
 * 这一行为是正式链当前已经验证过的必要 bring-up 步骤。
 */
static OmRet mct_prepare_damiao(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet last_error = OM_OK;
    OmRet ret = OM_OK;
    Motor* sync_motor = OM_NULL;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        ret = motor_owner_prepare_working_state(&runtime->damiao_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
        else if (sync_motor == OM_NULL)
        {
            sync_motor = &runtime->damiao_motors[index];
        }
    }

    osal_sleep_ms(MCT_DAMIAO_MODE_SETTLE_MS);

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (app_motor_profile_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        ret = motor_owner_enable(&runtime->damiao_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
            continue;
        }
        motor_recovery_notify_damiao_enabled(&runtime->damiao_motors[index]);
    }

    if (sync_motor != OM_NULL && motor_owner_sync_bus(sync_motor) != OM_OK)
    {
        last_error = OM_ERROR;
    }
    return last_error;
}

OmRet mct_register_vendors(MctRuntime* runtime)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    /* 注册顺序固定为：
     * DJI -> P1010B -> Damiao -> GO8010
     * 这里只做对象绑定，不做物理总线 START。
     */
    ret = mct_register_dji(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_register_p1010b(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_register_damiao(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    return mct_register_go8010(runtime);
}

OmRet mct_prepare_startup_motors(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    /* 启动期 bring-up 目前只包含依赖设备应答的两类 vendor。 */
    (void)mct_prepare_p1010b(runtime);
    (void)mct_prepare_damiao(runtime);
    return OM_OK;
}
