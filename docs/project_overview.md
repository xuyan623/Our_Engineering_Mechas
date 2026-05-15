# `new_robot_code` 项目概况

## 1. 目的

`new_robot_code` 是当前机器人的应用层工程，负责：

1. 组织正式启动链路。
2. 管理模式、底盘、机械臂和电机通信任务。
3. 基于 `oh-my-robot-framework` 提供的 BSP、OSAL、驱动与构建能力完成整机控制。

本文档只覆盖仓库级概况，不重复维护仿真参数、动作姿态表和控制公式。这些内容统一见 [`matlab_simulation_reference.md`](./matlab_simulation_reference.md)。

## 2. 当前目录结构

```text
<repo-root>/
├─ app/       # 应用层算法、模块、任务、入口
├─ docs/      # 仓库级说明与仿真参考
├─ driver/    # 项目内 vendor/适配层驱动
└─ xmake.lua  # 项目顶层构建入口
```

关键子目录：

1. `app/main.c`：正式启动链入口。
2. `app/task/`：`imu_task`、`input_task`、`mode_task`、`chassis_task`、`arm_task`、`motor_communications_task`、`vofa_task`。
3. `app/module/`：`event_bus`、`motor_recovery`、`system_health` 等基础模块。
4. `app/task/test/`：真实测试任务源码，属于正式仓库内容，不是临时垃圾目录。
5. `driver/`：项目内 `DJI`、`Damiao`、`P1010B`、`GO8010` 及统一 `motor` 适配。

## 3. 对 `oh-my-robot-framework` 的依赖方式

当前工程不是自包含仓库，构建时通过相对路径依赖同级的 `oh-my-robot-framework`：

- `xmake.lua` 使用 `includes("../oh-my-robot-framework")`
- 同时引用 `../oh-my-robot-framework/platform/bsp/boards/rm-a-board`

因此当前推荐工作区布局是：

```text
<workspace>/
├─ <this-repo>/
└─ oh-my-robot-framework/
```

本轮不改变这种依赖方式，也不引入 submodule / subtree。后续如果要把框架依赖固定到某个版本或改成子模块，应单独设计并落文档。

## 4. 正式启动链与 owner 边界

当前正式启动链位于 `app/main.c`，顺序为：

1. `bsp_register_all()`
2. `event_bus_init(...)`
3. `mpu_device_init(...)`
4. `imu_task_start()`
5. `input_task_start(...)`
6. `mode_task_start()`
7. `mct_start(...)`
8. `chassis_task_start()`
9. `arm_task_start()`
10. `vofa_task_start(...)`

owner 边界：

1. `mode_task` 只产出模式与动作状态。
2. `chassis_task` 只计算底盘与后腿目标，不直接碰物理总线。
3. `arm_task` 只计算机械臂目标与前馈，不直接碰物理总线。
4. `motor_communications_task` 是唯一正式电机通信 owner。
5. `vofa_task` 是统一观测上传路径。

## 5. 构建入口与命名关系

当前仓库的真实构建入口就是仓库根目录的 `xmake.lua`。

当前统一命名关系：

1. XMake 顶层目标名：`robot_project`
2. ELF 产物名：`robot_project.elf`
3. 烧录目标名：`flash.jlink.target = "robot_project"`

这三个名字必须保持一致链路中的语义对应：

- `target("robot_project")`：目标 ID
- `set_filename("robot_project.elf")`：产物文件名
- `flash.jlink.target = "robot_project"`：烧录任务定位目标名

## 6. 本地预设文件约定

独立仓库内提供：

- `om_preset.example.lua`：可提交模板

开发者本机使用：

- `om_preset.lua`：由开发者复制模板后按本机路径修改

约束：

1. `om_preset.lua` 只用于本机，不入库。
2. `om_preset.example.lua` 必须保持可用，作为新开发者配置起点。

## 7. 与仿真参考文档的边界

以下内容不在本文重复维护，统一以 [`matlab_simulation_reference.md`](./matlab_simulation_reference.md) 为事实源：

1. 14 路 VOFA 通道顺序
2. 机械臂姿态表与动作时间窗
3. 麦轮速度分配公式
4. 后腿控制参数
5. 重力补偿公式与物理参数
6. 各电机正式配置表

如果这些控制事实变化，应优先更新源码，再同步更新仿真参考文档，而不是在多个总览文档里重复修改。
