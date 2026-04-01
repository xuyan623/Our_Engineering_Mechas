# VSCode Git 协作操作指南

本指南针对 `Our_Engineering_Mechas` 项目，结合 `docs/git_collaboration_spec.md` 中的协作规范，说明如何使用 VSCode 界面进行 Git 操作。

## 前置准备

### 1. Fork 仓库（Web 端）
1. 访问 https://github.com/xuyan623/Our_Engineering_Mechas
2. 点击右上角的 `Fork` 按钮
3. 在自己的账号下创建 Fork 仓库

### 2. 克隆个人 Fork 仓库（VSCode 或命令行）
**使用 VSCode：**
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Git: Clone`
3. 输入你的 Fork 仓库地址：`https://github.com/<你的用户名>/Our_Engineering_Mechas.git`
4. 选择本地存放位置

**或使用命令行：**
```bash
git clone https://github.com/<你的用户名>/Our_Engineering_Mechas.git
cd Our_Engineering_Mechas
```

### 3. 添加 upstream 远端（命令行）
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

## 标准开发流程（VSCode 界面）

### 阶段一：创建 Issue（Web 端）
在官方仓库 https://github.com/xuyan623/Our_Engineering_Mechas 创建 Issue，设置：
- Assignees：自己
- Label：对应的标签（feature、bug 等）

### 阶段二：创建并切换到功能分支

**方法一：使用 VSCode 界面**
1. 点击左下角的分支名（默认是 `main`）
2. 选择 `创建新分支...`
3. 输入分支名：`feature/<issue编号>-<简短英文描述>`
   - 例如：`feature/15-osal-mutex`

**方法二：使用命令行**
```bash
git fetch upstream
git checkout -b feature/15-osal-mutex upstream/main
```

### 阶段三：开发与提交

#### 1. 查看变更
- 在 **源代码管理** 面板查看所有变更
- 点击文件名可以查看具体变更差异

#### 2. 暂存文件
- 点击文件右侧的 `+` 号暂存单个文件
- 或点击 **更改** 右侧的 `+` 号暂存所有文件

#### 3. 提交变更
1. 在顶部输入框填写 Commit Message，必须符合规范：
   ```
   类型(作用域): 简短描述 (#<issue编号>)
   ```
   例如：
   ```
   feat(osal): 新增互斥锁抽象接口 (#15)
   ```

2. 点击 **提交** 按钮（或按 `Ctrl+Enter`）

**Commit Message 类型：**
- `feat`：新增功能
- `fix`：修复 Bug
- `docs`：文档变更
- `style`：格式调整
- `refactor`：重构
- `chore`：构建/配置相关

### 阶段四：同步 upstream 并变基

#### 1. 拉取 upstream 更新（命令行）
```bash
git fetch upstream
```

#### 2. 检查是否落后
```bash
git rev-list --left-right --count upstream/main...HEAD
```

如果 `behind > 0`，需要变基：

#### 3. 执行变基（命令行）
```bash
git rebase upstream/main
```

**冲突处理：**
1. VSCode 会提示有冲突文件
2. 点击冲突文件，使用 **当前更改**、**传入更改** 或 **比较更改** 解决冲突
3. 解决后暂存文件
4. 在终端运行：
   ```bash
   git rebase --continue
   ```

### 阶段五：推送到个人 Fork

#### 使用 VSCode 界面
1. 点击顶部的 **推送** 按钮
2. 如果是第一次推送该分支，选择 `确定`
3. 如果之前已推送过且有变基，需要强制推送（使用命令行）

#### 使用命令行（推荐用于变基后）
```bash
git push origin feature/15-osal-mutex --force-with-lease
```

### 阶段六：创建 PR（Web 端）

1. 访问你的 Fork 仓库
2. 点击 `Compare & pull request` 按钮
3. 填写 PR 信息：
   - 标题：简要描述
   - 描述：包含范围、风险、验证结果
   - 使用 `Refs #15`（如果合入 integration）或 `Fixes #15`（如果合入 main）
4. 选择目标分支：
   - 日常开发：`integration`
   - 发布/热修复：`main`
5. 创建 Pull Request

## 常见操作（VSCode 界面）

### 切换分支
1. 点击左下角分支名
2. 从列表选择要切换的分支

### 查看历史
1. 在 **源代码管理** 面板点击 **查看历史**（时钟图标）
2. 或安装 Git Graph 扩展获得更好的可视化

### 撤销更改
- **未暂存的更改**：点击文件右侧的 `撤销更改` 图标
- **已暂存的更改**：点击文件右侧的 `-` 取消暂存，然后撤销

###  stash 临时更改
1. 在 **源代码管理** 面板点击 `...` 菜单
2. 选择 `Stash Changes`
3. 恢复时选择 `Apply Stash` 或 `Pop Stash`

## 重要提醒

1. **禁止直接在 main/integration 分支开发**
2. **所有变更必须绑定 Issue**
3. **同步上游必须用 rebase，禁止 merge**
4. **提交信息必须符合规范并包含 Issue 编号**
5. **仅允许在个人功能分支使用强制推送**

## 参考文档

- 完整协作规范：`docs/git_collaboration_spec.md`
- Git 官方文档：https://git-scm.com/docs
- VSCode Git 文档：https://code.visualstudio.com/docs/sourcecontrol/overview