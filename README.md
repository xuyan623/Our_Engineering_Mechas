# `new_robot_code`

`new_robot_code` 是当前机器人应用层工程，负责正式启动链、模式管理、底盘控制、机械臂控制、电机统一通信与调试观测上传。

## 1. 工作区布局

`new_robot_code` 是**代码区**，不是独立工作区根目录。正确工作区布局应是：

```text
<workspace>/
├─ .vscode/
│  ├─ c_cpp_properties.json
│  └─ launch.json
├─ .xmake/          # 构建后自动生成
├─ build/           # 构建后自动生成
├─ new_robot_code/  # 新代码区域
├─ oh-my-robot-framework/
├─ om_preset.lua
└─ xmake.lua
```

关键点：

1. **工作区根目录** 的 `xmake.lua` 是唯一正式构建入口。
2. **工作区根目录** 的 `om_preset.lua` 是唯一正式本机预设。
3. **工作区根目录** 的 `.vscode/` 是唯一正式 VSCode 调试配置。
4. `new_robot_code/` 内部只保留代码和内部构建描述文件 `workspace_target.lua`，不再作为新成员的直接配置入口。
5. 如果同级没有 `oh-my-robot-framework`，构建与调试都不会工作。

## 2. 快速开始

### 2.1 本机预设

1. 在**工作区根目录**创建 `om_preset.lua`
2. 可参考 [`docs/examples/om_preset.example.lua`](./docs/examples/om_preset.example.lua)
3. 按本机实际路径修改：
   - GNU Arm 工具链 `sdk/bin`
   - `armclang` 工具链 `sdk/bin`
   - `JLink.exe` 路径

`om_preset.lua` 仅用于本机，不应进入共享仓库。

### 2.2 构建

```powershell
cd <workspace>
xmake f -c --toolchain=gnu-rm -m debug
xmake build
```

当前统一命名关系：

1. XMake 目标名：`robot_project`
2. ELF 产物名：`robot_project.elf`
3. 烧录目标名：`flash.jlink.target = "robot_project"`

## 3. 文档入口

- [文档索引](./docs/README.md)
- [项目概况](./docs/project_overview.md)
- [Git 协作规范](./docs/process/git_collaboration_spec.md)
- [文档治理规范](./docs/process/document_governance_spec.md)
- [MATLAB 仿真参考](./docs/matlab_simulation_reference.md)
- [首版发布说明](./docs/release_notes/v0.1.0.md)

## 4. VSCode 调试

应使用**工作区根目录**的 `.vscode/launch.json` 与 `.vscode/c_cpp_properties.json`，可参考：

- [`docs/examples/.vscode/launch.json`](./docs/examples/.vscode/launch.json)
- [`docs/examples/.vscode/c_cpp_properties.json`](./docs/examples/.vscode/c_cpp_properties.json)

默认面向：

1. `rm-a-board`
2. `J-Link`
3. `gnu-rm` 与 `armclang` 两条调试链

前提：

1. `JLinkGDBServerCL.exe` 与 `arm-none-eabi-gdb` 已加入 `PATH`，或你按需自行改本地调试配置。
2. `oh-my-robot-framework` 与 `new_robot_code` 保持同级目录。

## 5. 当前范围

当前正式链包括：

1. `imu_task`
2. `input_task`
3. `mode_task`
4. `motor_communications_task`
5. `chassis_task`
6. `arm_task`
7. `vofa_task`

更细的 owner 边界、任务周期、姿态表、重力补偿和电机配置，统一以 `docs/project_overview.md` 和 `docs/matlab_simulation_reference.md` 为准。
