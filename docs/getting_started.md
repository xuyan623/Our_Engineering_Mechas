# 从零部署 `new_robot_code` 完整教程

本文档手把手教你从**一个空文件夹**开始，把 `new_robot_code` 源码成功编译、调试并烧录到 `rm-a-board`（RoboMaster 开发板 A 型，STM32F427IIH6）上。

---

## 1. 前置条件

在开始之前，请确认你的电脑已安装以下工具。如果你已经装好了，可以直接跳到第 2 步。

| 工具 | 作用 | 获取方式 |
|------|------|----------|
| Git | 克隆仓库 | [git-scm.com](https://git-scm.com/) |
| XMake | 构建系统 | [xmake.io](https://xmake.io/#/zh-cn/guide/installation) |
| GNU Arm Embedded Toolchain | 交叉编译器 | [developer.arm.com](https://developer.arm.com/downloads/-/gnu-rm) |
| SEGGER J-Link | 烧录与调试 | [segger.com/downloads/jlink](https://www.segger.com/downloads/jlink/) |
| VSCode（可选） | 代码编辑与调试 | [code.visualstudio.com](https://code.visualstudio.com/) |

> **提示**：安装完成后，把 `arm-none-eabi-gcc.exe` 和 `JLink.exe` 所在目录加入系统 `PATH`，后续步骤会更顺畅。

---

## 2. 创建空文件夹并克隆仓库

### 2.1 新建工作区目录

```powershell
# 在 D 盘根目录新建一个空文件夹（名字任意，示例用 robot_workspace）
mkdir D:\robot_workspace
cd D:\robot_workspace
```

### 2.2 克隆两个仓库

`new_robot_code` 不是自包含仓库，构建时必须与 `oh-my-robot-framework` 保持**同级目录**，并且工作区根目录还需要单独放置 `xmake.lua`、`om_preset.lua` 和 `.vscode/`。

```powershell
# 1) 克隆应用层工程（new_robot_code）—— 开发分支为 feature/new_robot_code
git clone -b feature/new_robot_code https://github.com/xuyan623/Our_Engineering_Mechas.git new_robot_code

# 2) 克隆框架依赖（oh-my-robot-framework）
git clone https://github.com/oh-my-robot/oh-my-robot-framework.git
```

> **注意**：如果你参与了框架层的开发，请按 [`docs/process/git_collaboration_spec.md`](./process/git_collaboration_spec.md) 使用 Forking Workflow，把 `oh-my-robot-framework` 的 `origin` 指向你自己的 Fork。

### 2.3 确认目录布局

克隆完成后，目录结构应如下：

```text
D:\robot_workspace/
├─ .vscode/
│  ├─ c_cpp_properties.json  ← 构建配置
│  └─ launch.json            ← 调试/烧录配置
├─ .xmake/                   ← 构建自动生成
├─ build/                    ← 构建自动生成
├─ new_robot_code/           ← 应用层代码区
│  ├─ app/
│  ├─ driver/
│  ├─ docs/
│  └─ workspace_target.lua   ← 内部 target 描述，不是用户入口
├─ oh-my-robot-framework/    ← 框架依赖（与 new_robot_code 同级）
├─ om_preset.lua             ← 工作区本机预设
└─ xmake.lua                 ← 工作区唯一正式构建入口
```

如果同级目录里没有 `oh-my-robot-framework`，或你把构建入口/预设/调试配置错误地放进 `new_robot_code/`，后续构建与调试都会失败或误导新成员。

---

## 3. 配置本机预设文件

### 3.1 准备工作区根模板

在工作区根目录创建 `xmake.lua`、`om_preset.lua` 和 `.vscode/`。推荐直接参考本仓库提供的示例文件：

```powershell
cd D:\robot_workspace
copy .\new_robot_code\docs\examples\xmake.lua .\xmake.lua
copy .\new_robot_code\docs\examples\om_preset.example.lua .\om_preset.lua
mkdir .\.vscode
copy .\new_robot_code\docs\examples\.vscode\launch.json .\.vscode\launch.json
copy .\new_robot_code\docs\examples\.vscode\c_cpp_properties.json .\.vscode\c_cpp_properties.json
```

> `om_preset.lua` 仅用于本机，**不要提交到共享仓库**。

### 3.2 按本机路径修改

用 VSCode 或记事本打开 `om_preset.lua`，修改三个关键路径：

1. **GNU Arm 工具链路径**
2. **ArmClang 工具链路径**（可选，默认用 `gnu-rm`）
3. **J-Link 可执行文件路径**

以下是一个**最小可用示例**，假设你解压了 `arm-gnu-toolchain-15.2.rel1` 到 `D:/arm-gnu-toolchain-15.2.rel1-mingw-w64-i686-arm-none-eabi`，J-Link 安装在 `D:/Jlink/JLink/JLink.exe`：

```lua
local preset = {
  board = {name = "rm-a-board"},      -- 当前硬件为 rm-a-board
  os = {name = "freertos"},

  toolchain_default = {
    name = "gnu-rm",
  },

  toolchain_presets = {
    ["gnu-rm"] = {
      sdk = "D:/arm-gnu-toolchain-15.2.rel1-mingw-w64-i686-arm-none-eabi",
      bin = "D:/arm-gnu-toolchain-15.2.rel1-mingw-w64-i686-arm-none-eabi/bin",
    },
    ["armclang"] = {
      sdk = "D:/keilstm32/ARM/ARMCLANG",      -- 按需修改
      bin = "D:/keilstm32/ARM/ARMCLANG/bin",
    },
  },

  flash = {
    jlink = {
      device = "STM32F427II",
      interface = "swd",
      speed = 4000,
      program = "D:/Jlink/JLink/JLink.exe",    -- 改成你本机 JLink.exe 路径
      target = "robot_project",
      firmware = "build/cross/arm/debug/robot_project.elf",
      prefer_hex = false,
      reset = true,
      run = true,
      native_output = false,
    },
  },
}

function get_preset()
  return preset
end
```

**必须修改的地方只有 `sdk`、`bin` 和 `program` 这三行，其余保持默认即可。**

---

## 4. 配置工程并编译

### 4.1 进入工作区根目录

```powershell
cd D:\robot_workspace
```

### 4.2 执行配置

```powershell
xmake f -c --toolchain=gnu-rm -m debug
```

命令解释：

- `xmake f`：`xmake config` 的简写，配置工程。
- `-c`：强制清理旧配置，从头重新检测。
- `--toolchain=gnu-rm`：使用 GNU Arm 工具链。
- `-m debug`：生成调试版本（带符号表）。

如果配置成功，你会看到类似输出：

```text
checking for arm-none-eabi-gcc ... ok
...
```

### 4.3 执行编译

```powershell
xmake build
```

编译成功时，产物位于：

```text
D:\robot_workspace\build\cross\arm\debug\robot_project.elf
```

如果编译失败，请检查：

1. `om_preset.lua` 中的工具链路径是否指向了存在的目录。
2. `oh-my-robot-framework` 是否与 `new_robot_code` 同级。
3. 是否误把 `xmake.lua` / `om_preset.lua` 放进了 `new_robot_code/` 而不是工作区根目录。

---

## 5. 烧录固件到硬件

### 5.1 连接 J-Link

1. 用 SWD 线连接 J-Link 与 rm-a-board。
2. 给开发板上电。
3. 确认 J-Link USB 已插入电脑。

### 5.2 一键烧录

```powershell
xmake run flash
```

如果 `om_preset.lua` 中 `program` 路径配置正确，XMake 会自动调用 J-Link Commander 完成烧录并运行。

烧录成功后，你应该能在串口终端（如 VOFA+）看到启动日志输出。

---

## 6. VSCode 调试（可选但推荐）

### 6.1 打开工程

用 VSCode 打开**工作区根目录**：

```powershell
code D:\robot_workspace
```

### 6.2 启动 J-Link GDB Server

烧录完成后保持 J-Link 连接，启动 GDB Server：

```powershell
JLinkGDBServerCL -device STM32F427II -if SWD -speed 4000
```

> **注意**：`JLinkGDBServerCL.exe` 需要在系统 `PATH` 中，或写完整路径调用。

### 6.3 在 VSCode 中启动调试

1. 按 `F5` 或点击左侧调试图标 → "运行与调试"。
2. 选择 `"robot_project (J-Link + GNU-RM)"`。
3. 如果 `launch.json` 中 `miDebuggerPath` 指向了正确的 `arm-none-eabi-gdb.exe`，调试器会自动连接并停在 `main()` 入口。

VSCode 调试配置应位于**工作区根目录** `.vscode/launch.json`，默认面向：

- `rm-a-board`
- `J-Link`
- `gnu-rm` 与 `armclang` 两条调试链

如果 `arm-none-eabi-gdb` 未在 `PATH` 中，请手动修改工作区根 `.vscode/launch.json` 里的对应路径。

---

## 7. 验证成功

完成以上步骤后，确认以下几点：

1. `xmake build` 无报错，生成 `robot_project.elf`。
2. `xmake run flash` 成功烧录，开发板 LED 或串口有输出。
3. VSCode 中按 `F5` 能断点停在 `main()`，并能单步执行到 `start_task()`。

---

## 8. 下一步

- 阅读 [`project_overview.md`](./project_overview.md) 了解正式启动链与任务边界。
- 阅读 [`matlab_simulation_reference.md`](./matlab_simulation_reference.md) 获取控制公式、电机配置表和姿态表。
- 准备参与开发时，务必阅读 [`docs/process/git_collaboration_spec.md`](./process/git_collaboration_spec.md)。
