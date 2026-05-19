set_project("new_robot_code")
set_xmakever("3.0.7")
add_rules("mode.debug", "mode.release")

-- 这些宏必须对当前工程内的所有 target 生效，包括 tar_board。
add_defines("RM_A_SERIAL37_PROFILE=RM_A_SERIAL37_PROFILE_USART3_FULLDMA_UART7_INT")
add_defines("STM32F427xx", "USE_HAL_DRIVER")

-- 包含 oh-my-robot-framework
includes("../oh-my-robot-framework")

-- 选择开发板（根据你的硬件选择 rm-a-board 或 rm-c-board）
includes("../oh-my-robot-framework/platform/bsp/boards/rm-a-board")

target("robot_project")
    set_kind("binary")
    set_filename("robot_project.elf")
    
    -- 当前阶段只显式依赖已用到的框架能力，避免聚合目标的链接顺序干扰
    add_deps("tar_board")
    add_deps("tar_sync")
    add_deps("tar_osal")
    add_deps("tar_awcore")
    add_deps("tar_awalgo")
    
    -- 添加源文件
    add_files("app/main.c")
    add_files("driver/dji/dji_motor.c")
    add_files("driver/p1010b/P1010B.c")
    add_files("driver/damiao/damiao.c")
    add_files("driver/go8010/go8010.c")
    add_files("driver/motor/motor.c")
    add_files("app/algorithm/gravity_comp/gravity_comp.c")
    add_files("app/algorithm/kinematics/kinematics.c")
    add_files("app/bsp/bsp_init.c")
    add_files("app/bsp/imu_bsp.c")
    add_files("app/bsp/board_led.c")
    add_files("app/driver/ahrs/filters.c")
    add_files("app/driver/ahrs/MahonyAHRS.c")
    add_files("app/driver/mpu6500/mpu6500.c")
    add_files("app/driver/mpu6500/ist8310.c")
    add_files("app/driver/imu/imu.c")
    add_files("app/function/vofa/vofa.c")
    add_files("app/module/event_bus/event_bus.c")
    add_files("app/module/motor_tx_dispatch/motor_tx_dispatch.c")
    add_files("app/module/motor_recovery/motor_recovery.c")
    add_files("app/module/state_machine/state_machine.c")
    add_files("app/module/system_health/system_health.c")
    add_files("app/task/input_task/input_task.c")
    add_files("app/task/imu_task/imu_task.c")
    add_files("app/task/mode_task/mode_task.c")
    add_files("app/task/chassis_task/chassis_task.c")
    add_files("app/task/arm_task/arm_task.c")
    add_files("app/task/motor_communications_task/mct.c")
    add_files("app/task/motor_communications_task/mct_runtime.c")
    add_files("app/task/motor_communications_task/mct_vendor.c")
    add_files("app/task/motor_communications_task/mct_diag.c")
    add_files("app/task/vofa_task/vofa_task.c")
    
    -- 添加包含路径
    add_includedirs(".")
    add_includedirs("app")
    add_includedirs("driver")
    add_includedirs("../oh-my-robot-framework/platform/bsp/boards/rm-a-board/include")
    add_includedirs("../oh-my-robot-framework/platform/bsp/boards/rm-a-board/osal/freertos")
    add_includedirs("../oh-my-robot-framework/platform/bsp/vendor/STM32/CMSIS/Include")
    add_includedirs("../oh-my-robot-framework/platform/bsp/vendor/STM32/STM32F4/STM32F4xx/Include")
    add_includedirs("../oh-my-robot-framework/platform/bsp/vendor/STM32/STM32F4/STM32F4xx_HAL_Driver/Inc")
    add_includedirs("../oh-my-robot-framework/platform/bsp/vendor/STM32/STM32F4/STM32F4xx_HAL_Driver/Inc/Legacy")
    add_includedirs("../oh-my-robot-framework/platform/osal/freertos/FreeRTOS/include")
    add_includedirs("../oh-my-robot-framework/platform/osal/freertos/portable/gnu-rm/cortex-m4")
    
    -- 设置策略
    set_policy("check.auto_ignore_flags", false)
    
    -- 应用规则
    add_rules("oh_my_robot.context", "oh_my_robot.board_assets")
target_end()
