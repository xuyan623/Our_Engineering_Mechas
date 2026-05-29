#ifndef NEW_ROBOT_MOTOR_H
#define NEW_ROBOT_MOTOR_H

#include "core/om_def.h"
#include "driver/damiao/damiao.h"
#include "driver/dji/dji_motor.h"
#include "driver/go8010/go8010.h"
#include "driver/p1010b/P1010B.h"
#include <stdint.h>

/** 电机注册表最大容量 */
#define MOTOR_REGISTRY_CAPACITY  (32u)

typedef struct Motor Motor;

/**
 * @brief 电机厂商类型枚举
 */
typedef enum
{
    MOTOR_VENDOR_DJI = 0u,      /**< DJI 大疆电机 */
    MOTOR_VENDOR_DAMIAO,        /**< Damiao 达妙电机 */
    MOTOR_VENDOR_P1010B,        /**< P1010B 电机 */
    MOTOR_VENDOR_GO8010,        /**< GO8010 电机 */
} MotorVendor;

/**
 * @brief 电机控制模式枚举
 */
typedef enum
{
    MOTOR_CONTROL_MODE_DISABLED = 0u,   /**< 禁用模式 */
    MOTOR_CONTROL_MODE_CURRENT,         /**< 电流环控制 */
    MOTOR_CONTROL_MODE_SPEED,           /**< 速度环控制 */
    MOTOR_CONTROL_MODE_ANGLE,           /**< 角度环控制 */
    MOTOR_CONTROL_MODE_TORQUE,          /**< 力矩控制 */
} MotorControlMode;

typedef OmRet (*MotorComputeHook)(Motor* motor);
typedef OmRet (*MotorSyncHook)(Motor* motor);

/**
 * @brief 电机控制指令结构体
 */
typedef struct
{
    float angle;    /**< 目标角度（弧度） */
    float speed;    /**< 目标速度（RPM） */
    float current;  /**< 目标电流（A） */
    float torque;   /**< 目标力矩（N·m） */
    float kp;       /**< 位置环比例增益 */
    float kd;       /**< 位置环微分增益 */
} MotorCommand;

/**
 * @brief 电机反馈数据结构体
 */
typedef struct
{
    float angle;        /**< 当前角度（弧度，多圈累计值） */
    float speed;        /**< 当前速度（RPM） */
    float current;      /**< 当前电流（A） */
    float torque;       /**< 当前力矩（N·m） */
    OmBool online;      /**< 在线状态标志 */
    uint32_t timestamp_ms; /**< 反馈数据时间戳（毫秒） */
} MotorFeedback;

/**
 * @brief 电机反馈快照结构体，用于批量获取反馈数据
 */
typedef struct
{
    const char* name;           /**< 电机名称 */
    MotorVendor vendor;         /**< 电机厂商 */
    MotorFeedback feedback;     /**< 反馈数据 */
} MotorFeedbackSnapshot;

/**
 * @brief DJI 电机绑定结构体
 */
typedef struct
{
    DJIMotorBus* bus;       /**< CAN 总线句柄 */
    DJIMotorDrv* driver;    /**< 电机驱动句柄 */
} MotorDjiBinding;

/**
 * @brief Damiao 电机绑定结构体
 */
typedef struct
{
    DamiaoMotorBus* bus;        /**< CAN 总线句柄 */
    DamiaoMotorDrv* driver;     /**< 电机驱动句柄 */
} MotorDamiaoBinding;

/**
 * @brief P1010B 电机绑定结构体
 */
typedef struct
{
    P1010BBus* bus;         /**< CAN 总线句柄 */
    P1010BDriver* driver;   /**< 电机驱动句柄 */
} MotorP1010BBinding;

/**
 * @brief GO8010 电机绑定结构体
 */
typedef struct
{
    Go8010Bus* bus;             /**< CAN 总线句柄 */
    Go8010MotorDrv* driver;     /**< 电机驱动句柄 */
} MotorGo8010Binding;

/**
 * @brief 厂商特定绑定联合体，根据电机类型选择对应的绑定结构
 */
typedef union
{
    MotorDjiBinding dji;        /**< DJI 电机绑定 */
    MotorDamiaoBinding damiao;  /**< Damiao 电机绑定 */
    MotorP1010BBinding p1010b;  /**< P1010B 电机绑定 */
    MotorGo8010Binding go8010;  /**< GO8010 电机绑定 */
} MotorVendorBinding;

/**
 * @brief 电机配置结构体
 */
typedef struct
{
    const char* name;                   /**< 电机名称标识 */
    MotorVendor vendor;                 /**< 电机厂商类型 */
    MotorControlMode control_mode;      /**< 控制模式 */
    void* vendor_context;               /**< 厂商特定上下文指针 */
    MotorComputeHook compute_hook;      /**< 计算钩子函数，在控制计算前调用 */
    MotorSyncHook sync_hook;            /**< 同步钩子函数，在通信同步前调用 */
    OmBool output_limit_enabled;        /**< 输出限幅使能标志 */
    float output_min;                   /**< 最小输出限制 */
    float output_max;                   /**< 最大输出限制 */
} MotorConfig;

/**
 * @brief 电机核心结构体，封装不同厂商电机的统一抽象层
 */
struct Motor
{
    MotorConfig config;                     /**< 电机配置信息 */
    MotorVendorBinding binding;             /**< 厂商特定绑定信息 */
    OmBool registered_flag;                 /**< 注册状态标志 */
    float target_current;                   /**< 目标电流设定值 */
    float target_speed;                     /**< 目标速度设定值 */
    float target_angle;                     /**< 目标角度设定值 */
    float target_torque;                    /**< 目标力矩设定值 */
    float target_kp;                        /**< 目标位置环比例增益 */
    float target_kd;                        /**< 目标位置环微分增益 */
    float torque_feedforward;               /**< 力矩前馈补偿值 */
    float computed_output;                  /**< 计算后的最终输出值 */
    int16_t p1010b_last_synced_raw_target;  /**< P1010B 最近一次真正下发的原始目标值 */
    uint8_t p1010b_last_synced_mode;        /**< P1010B 最近一次下发时的底层模式 */
    OmBool p1010b_last_synced_valid;        /**< P1010B 最近一次下发缓存是否有效 */
    MotorCommand command;                   /**< 当前控制指令 */
    MotorFeedback feedback;                 /**< 实时反馈数据 */
};

/**
 * @brief 通用方式注册电机到系统
 * 
 * @param motor 电机实例指针
 * @param config 电机配置指针
 * @return OmRet 操作结果
 */
OmRet motor_register(Motor* motor, const MotorConfig* config);

/**
 * @brief 注册 DJI 电机到系统
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_register_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, MotorControlMode control_mode);

/**
 * @brief 注册 Damiao 电机到系统
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_register_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                            MotorControlMode control_mode);

/**
 * @brief 注册 P1010B 电机到系统
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_register_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver,
                            MotorControlMode control_mode);

/**
 * @brief 注册 GO8010 电机到系统
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_register_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver,
                            MotorControlMode control_mode);

/**
 * @brief 附加 DJI 电机并配置详细参数
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param type DJI 电机类型
 * @param id 电机物理 ID
 * @param dji_control_mode DJI 底层控制模式
 * @param control_mode 应用层控制模式
 * @return OmRet 操作结果
 */
OmRet motor_attach_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, DJIMotorType type,
                       uint8_t id, DJIMotorCtrlMode dji_control_mode, MotorControlMode control_mode);

/**
 * @brief 附加 Damiao 电机并配置详细参数
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param type Damiao 电机类型
 * @param can_id CAN 通信 ID
 * @param master_id 主站 ID
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_attach_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                          DamiaoMotorType type, uint16_t can_id, uint16_t master_id, MotorControlMode control_mode);

/**
 * @brief 附加 P1010B 电机并配置详细参数
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param id 电机 ID
 * @param default_mode 默认工作模式
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_attach_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver, uint8_t id,
                          P1010BMode default_mode, MotorControlMode control_mode);

/**
 * @brief 附加 GO8010 电机并配置详细参数
 * 
 * @param motor 电机实例指针
 * @param name 电机名称
 * @param bus CAN 总线句柄
 * @param driver 电机驱动句柄
 * @param id 电机 ID
 * @param control_mode 控制模式
 * @return OmRet 操作结果
 */
OmRet motor_attach_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver, uint8_t id,
                          MotorControlMode control_mode);

/**
 * @brief 设置电机控制模式
 * 
 * @param motor 电机实例指针
 * @param control_mode 目标控制模式
 * @return OmRet 操作结果
 */
OmRet motor_set_control_mode(Motor* motor, MotorControlMode control_mode);

/**
 * @brief 设置目标电流
 * 
 * @param motor 电机实例指针
 * @param current 目标电流值（A）
 * @return OmRet 操作结果
 */
OmRet motor_set_current(Motor* motor, float current);

/**
 * @brief 设置目标速度
 * 
 * @param motor 电机实例指针
 * @param speed 目标速度值（RPM）
 * @return OmRet 操作结果
 */
OmRet motor_set_speed(Motor* motor, float speed);

/**
 * @brief 设置目标角度
 * 
 * @param motor 电机实例指针
 * @param angle 目标角度值（弧度）
 * @return OmRet 操作结果
 */
OmRet motor_set_angle(Motor* motor, float angle);

/**
 * @brief 设置目标力矩
 * 
 * @param motor 电机实例指针
 * @param torque 目标力矩值（N·m）
 * @return OmRet 操作结果
 */
OmRet motor_set_torque(Motor* motor, float torque);

/**
 * @brief 设置位置环 PID 增益
 * 
 * @param motor 电机实例指针
 * @param kp 比例增益
 * @param kd 微分增益
 * @return OmRet 操作结果
 */
OmRet motor_set_position_gains(Motor* motor, float kp, float kd);

/**
 * @brief 设置力矩前馈补偿
 * 
 * @param motor 电机实例指针
 * @param torque_feedforward 力矩前馈值（N·m）
 * @return OmRet 操作结果
 */
OmRet motor_set_torque_feedforward(Motor* motor, float torque_feedforward);

/**
 * @brief 设置输出限幅范围
 * 
 * @param motor 电机实例指针
 * @param output_min 最小输出限制
 * @param output_max 最大输出限制
 * @return OmRet 操作结果
 */
OmRet motor_set_output_limit(Motor* motor, float output_min, float output_max);

/**
 * @brief 直接设置反馈数据（用于测试或仿真）
 * 
 * @param motor 电机实例指针
 * @param angle 角度值（弧度）
 * @param speed 速度值（RPM）
 * @param current 电流值（A）
 * @param torque 力矩值（N·m）
 * @return OmRet 操作结果
 */
OmRet motor_set_feedback(Motor* motor, float angle, float speed, float current, float torque);

/**
 * @brief 从底层驱动刷新反馈数据
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_refresh_feedback(Motor* motor);

/**
 * @brief 获取电机反馈数据指针
 * 
 * @param motor 电机实例指针
 * @return MotorFeedback* 反馈数据指针
 */
const MotorFeedback* motor_get_feedback(const Motor* motor);

/**
 * @brief 获取反馈数据的时间戳
 * 
 * @param motor 电机实例指针
 * @return uint32_t 时间戳（毫秒）
 */
uint32_t motor_get_feedback_timestamp_ms(const Motor* motor);

/**
 * @brief 检查反馈数据是否在有效时间窗口内
 * 
 * @param motor 电机实例指针
 * @param timeout_ms 超时阈值（毫秒）
 * @return OmBool 数据是否有效
 */
OmBool motor_is_feedback_recent(const Motor* motor, uint32_t timeout_ms);

/**
 * @brief 获取单圈角度（将多圈累计值转换为 0~2π 范围）
 * 
 * @param motor 电机实例指针
 * @param angle_rad 输出单圈角度（弧度）
 * @return OmBool 操作是否成功
 */
OmBool motor_get_single_turn_angle_rad(const Motor* motor, float* angle_rad);

/**
 * @brief 获取初始零位角度
 * 
 * @param motor 电机实例指针
 * @param zero_angle_rad 输出零位角度（弧度）
 * @return OmBool 操作是否成功
 */
OmBool motor_get_initial_zero_angle_rad(const Motor* motor, float* zero_angle_rad);

/**
 * @brief 捕获当前角度作为初始零位
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_capture_initial_zero(Motor* motor);

/**
 * @brief 准备电机工作状态（上电、使能等）
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_owner_prepare_working_state(Motor* motor);

/**
 * @brief 使能电机输出
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_owner_enable(Motor* motor);

/**
 * @brief 禁用电机输出
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_owner_disable(Motor* motor);

/**
 * @brief 查询电机反馈状态
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_owner_query_feedback(Motor* motor);

/**
 * @brief 同步电机总线通信数据
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_owner_sync_bus(Motor* motor);

/**
 * @brief 根据名称查找已注册的电机
 * 
 * @param name 电机名称
 * @return Motor* 电机实例指针，未找到返回 NULL
 */
Motor* motor_find_by_name(const char* name);

/**
 * @brief 批量复制所有电机的反馈快照
 * 
 * @param snapshots 输出快照数组
 * @param capacity 数组容量
 * @param snapshot_count 输出实际复制的快照数量
 * @return OmRet 操作结果
 */
OmRet motor_copy_feedback_snapshots(MotorFeedbackSnapshot* snapshots, uint32_t capacity, uint32_t* snapshot_count);

/**
 * @brief 设置计算钩子函数
 * 
 * @param motor 电机实例指针
 * @param compute_hook 计算钩子函数指针
 * @return OmRet 操作结果
 */
OmRet motor_set_compute_hook(Motor* motor, MotorComputeHook compute_hook);

/**
 * @brief 设置同步钩子函数
 * 
 * @param motor 电机实例指针
 * @param sync_hook 同步钩子函数指针
 * @return OmRet 操作结果
 */
OmRet motor_set_sync_hook(Motor* motor, MotorSyncHook sync_hook);

/**
 * @brief 设置厂商特定上下文
 * 
 * @param motor 电机实例指针
 * @param vendor_context 厂商上下文字典
 * @return OmRet 操作结果
 */
OmRet motor_set_vendor_context(Motor* motor, void* vendor_context);

/**
 * @brief 执行电机控制计算（PID 等）
 * 
 * @param motor 电机实例指针
 * @return OmRet 操作结果
 */
OmRet motor_control_compute(Motor* motor);

/**
 * @brief 获取电机最终计算输出值
 * 
 * @param motor 电机实例指针
 * @return float 输出值
 */
float motor_get_output(const Motor* motor);

/**
 * @brief 发送所有已注册电机的控制指令到总线
 * 
 * @return OmRet 操作结果
 */
OmRet motor_transmit_all(void);

/**
 * @brief 仅发送用于刷新反馈的观测帧，不下发正式控制目标
 *
 * 当前只覆盖：
 * - Damiao：零增益 MIT 空闲帧
 * - GO8010：保持当前位置的零增益目标帧
 *
 * DJI 与 P1010B 不走这条路径。
 *
 * @return OmRet 操作结果
 */
OmRet motor_transmit_observation_only(void);

/**
 * @brief 接收所有已注册电机的反馈数据从总线
 * 
 * @return OmRet 操作结果
 */
OmRet motor_receive_all(void);

#endif
