# VSCode Git 协作操作指南

本指南针对 `Our_Engineering_Mechas` 项目，结合 `docs/git_collaboration_spec.md` 中的协作规范，说明如何使用 VSCode 界面进行 Git 操作。

## 前置准备

### 1. Fork 仓库（Web 端）
1. 访问 https://github.com/xuyan623/Our_Engineering_Mechas
2. 点击右上角的 `Fork` 按钮
3. 在自己的账号下创建 Fork 仓库

### 2. 克隆个人 Fork 仓库
**使用 VSCode 界面（推荐）：**
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Git: Clone` 并选择
3. 输入你的 Fork 仓库地址：`https://github.com/<你的用户名>/Our_Engineering_Mechas.git`
4. 选择本地存放位置

**命令行补充：**
```bash
git clone https://github.com/<你的用户名>/Our_Engineering_Mechas.git
cd Our_Engineering_Mechas
```

### 3. 添加 upstream 远端
**方法一：使用 VSCode 命令面板**
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Git: Add Remote` 并选择
3. 输入远程名称：`upstream`
4. 输入远程仓库 URL：`https://github.com/xuyan623/Our_Engineering_Mechas.git`

**方法二：命令行（推荐）：**
```bash
git remote add upstream https://github.com/xuyan623/Our_Engineering_Mechas.git
git remote -v  # 验证配置
```

## VSCode Git 界面基础

### Git 面板位置
- 左侧活动栏点击 **源代码管理** 图标（或按 `Ctrl+Shift+G`）
- 顶部状态栏显示当前分支、同步状态等

### 常用操作按钮
- **刷新**：重新扫描变更
- **提交**：提交暂存的更改
- **同步更改**：拉取并推送
- **拉取**：从远端拉取更新
- **推送**：推送本地提交到远端

## 标准开发流程

### 阶段一：创建并切换到功能分支

**使用 VSCode 界面：**
1. 点击左下角的分支名（默认是 `main`）
2. 选择 `创建新分支...`
3. 输入分支名：`feature/<简短英文描述>`
   - 例如：`feature/osal-mutex`
4. 新分支会自动从当前分支创建并切换

**命令行补充：**
```bash
git fetch upstream
git checkout -b feature/osal-mutex upstream/main
```

### 阶段二：开发与提交

#### 1. 查看变更
- 在 **源代码管理** 面板查看所有变更
- 点击文件名可以查看具体变更差异（左右分屏对比）

#### 2. 暂存文件
- 点击文件右侧的 `+` 号暂存单个文件
- 或点击 **更改** 右侧的 `+` 号暂存所有文件
- 点击已暂存文件右侧的 `-` 号可以取消暂存

#### 3. 提交变更
1. 在顶部输入框填写 Commit Message，必须符合规范：
   ```
   类型(作用域): 简短描述
   ```
   例如：
   ```
   feat(osal): 新增互斥锁抽象接口
   ```

2. 点击 **提交** 按钮（或按 `Ctrl+Enter`）

**Commit Message 类型：**
- `feat`：新增功能
- `fix`：修复 Bug
- `docs`：文档变更
- `style`：格式调整
- `refactor`：重构
- `chore`：构建/配置相关

### 阶段三：同步 upstream 并变基

#### 1. 拉取 upstream 更新
**方法一：使用 VSCode 命令面板**
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Git: Fetch` 并选择
3. 选择 `upstream` 远程仓库

**方法二：命令行（推荐）：**
```bash
git fetch upstream
```

#### 2. 检查是否落后
**命令行（必须）：**
```bash
git rev-list --left-right --count upstream/main...HEAD
```

如果 `behind > 0`，需要变基：

#### 3. 执行变基
**方法一：使用 VSCode 命令面板**
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Git: Rebase Branch` 并选择
3. 选择变基目标：`upstream/main`

**方法二：命令行（推荐）：**
```bash
git rebase upstream/main
```

**冲突处理（VSCode 界面）：**
1. VSCode 会提示有冲突文件，在 **源代码管理** 面板可以看到
2. 点击冲突文件，使用以下选项解决：
   - **当前更改**：保留你的修改
   - **传入更改**：使用 upstream 的修改
   - **比较更改**：打开分屏对比手动编辑
3. 解决后暂存文件（点击 `+` 号）
4. 在终端运行：
   ```bash
   git rebase --continue
   ```

### 阶段四：推送到个人 Fork

**使用 VSCode 界面：**
1. 点击顶部的 **推送** 按钮
2. 如果是第一次推送该分支，选择 `确定`

**命令行补充（推荐用于变基后）：**
```bash
git push origin feature/osal-mutex --force-with-lease
```

### 阶段五：创建 PR（Web 端）

1. 访问你的 Fork 仓库
2. 点击 `Compare & pull request` 按钮
3. 填写 PR 信息：
   - 标题：简要描述
   - 描述：包含范围、风险、验证结果
4. 选择目标分支：
   - 日常开发：`integration`
   - 发布/热修复：`main`
5. 创建 Pull Request

## 常见操作（VSCode 界面）

### 切换分支
1. 点击左下角分支名
2. 从列表选择要切换的分支

### 查看历史
1. 在 **源代码管理** 面板点击 `...` 菜单
2. 选择 `查看历史`
3. 或安装 Git Graph 扩展获得更好的可视化

### 撤销更改
- **未暂存的更改**：点击文件右侧的 `撤销更改` 图标（弯曲箭头）
- **已暂存的更改**：先点击文件右侧的 `-` 取消暂存，然后撤销

### Stash 临时更改
1. 在 **源代码管理** 面板点击 `...` 菜单
2. 选择 `Stash Changes`，输入 stash 信息（可选）
3. 恢复时选择 `Apply Stash` 或 `Pop Stash`

### 拉取远端更新
1. 点击顶部的 **拉取** 按钮
2. 或在 `...` 菜单中选择 `拉取`

## 重要提醒

1. **禁止直接在 main/integration 分支开发**
2. **同步 upstream 必须用 rebase，禁止 merge**
3. **提交信息必须符合规范**
4. **仅允许在个人功能分支使用强制推送**

## 参考文档

- 完整协作规范：`docs/git_collaboration_spec.md`
- Git 官方文档：https://git-scm.com/docs
- VSCode Git 文档：https://code.visualstudio.com/docs/sourcecontrol/overview