# `new_robot_code` 命名规则

本仓库命名以“**稳定前缀 + 语义核心**”为准，不以“把路径层级全部拼进名字”换可读性。

## 长度上限

- 目录名：`<= 20`
- 文件基名：`<= 24`
- 局部变量 / 参数：`<= 20`
- 结构体字段：`<= 24`
- 私有静态函数：`<= 28`
- 对外函数：`<= 32`
- typedef / struct / enum 类型名：`<= 24`
- enum 成员 / task id / 错误码：`<= 36`
- 宏：`<= 35`

豁免项：

- include guard
- 第三方 / HAL / FreeRTOS / CMSIS 外部符号

## 词序规则

- 函数：`<module>_<verb>_<object>[_qualifier]`
- 类型：`<Module><Noun>[_Role]`
- 宏：`<AREA>_<OBJECT>_<PROPERTY>[_UNIT]`
- enum 成员：`<GROUP>_<MEANING>`

禁止：

- 重复堆叠模块词  
  例如 `mode_task_submit_mode_task_*`
- 同义范围词叠加  
  例如 `current_active_runtime_state`
- 把路径层级原样摊平到符号名里  
  例如 `SH_ERR_MOTOR_COMMUNICATIONS_TASK_START_FAIL`

## 缩写白名单

只允许以下缩写；白名单外用完整单词：

- 模块/角色：`cfg` `ctx` `drv` `diag` `dbg` `pred`
- 运动量：`pos` `vel` `acc`
- 控制：`pid` `kp` `ki` `kd`
- 索引/计数：`idx` `cnt`
- 通信：`tx` `rx`
- 单位：`ms` `hz` `rad` `deg` `rpm` `mm`
- 已稳定短前缀：`mct` `sh` `vofa` `imu` `rc` `ik` `fk`

固定约定：

- `control` 统一写 `ctrl`
- `feedback` 统一写 `fb`
- `target` 统一写 `tgt`
- `progress` 统一写 `prog`
- `observation` / `observe` 统一写 `obs`
- `operational` 统一写 `op`
- `reference` 统一写 `ref`
- 模块前缀允许统一缩写：
  - `arm_task` -> `amt`
  - `mode_task` -> `mdt`
  - `chassis_task` -> `cst`
  - `input_task` -> `ipt`
  - `input_custom` -> `ipc`
  - `motor_recovery` -> `mtr`
  - `task_context_pool` -> `tcp`
  - `task_channel` -> `tch`
  - `vofa_layout` -> `vlo`
- 上述模块前缀只用于宏、常量和类型收敛；函数名前缀保持模块全拼，不缩成这些短前缀
- 同一语义只允许一种写法，不能同时保留缩写和全拼

## 当前仓库判例

- `APP_CT_FRONT_WHEEL_SPEED_PID_INTEGRAL_LIMIT`
  -> `APP_CT_FRONT_VEL_PID_I_MAX`
- `APP_MCT_OPERATIONAL_FORMAL_TRANSMIT_PERIOD_MS`
  -> `APP_MCT_OPERATIONAL_TX_MS`
- `MT_INIT_PROGRESS_IMU_READY`
  -> `MODE_INIT_IMU_READY`
- `SH_ERR_MOTOR_COMMUNICATIONS_TASK_START_FAIL`
  -> `SH_ERR_MCT_START_FAIL`
- `APP_MPF_ROLE_OBSERVE_ONLY`
  -> `APP_MR_OBSERVE_ONLY`
- `VOFA_LAYOUT_MAX_CHANNELS`
  -> `VL_MAX_CHANNELS`

## 执行顺序

后续命名收敛按这个顺序做：

1. 配置宏、枚举、错误码
2. public API 和 DTO
3. 私有函数、字段
4. 文件名与目录名

每轮都必须：

1. 先扫旧名字残留
2. 再完整构建
3. 确认只改命名，不带行为漂移
