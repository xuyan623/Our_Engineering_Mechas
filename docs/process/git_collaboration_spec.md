# Git 协作规范

## 1. 目的与边界

本规范用于约束 `new_robot_code` 仓库的日常协作方式，目标是：

1. 保持提交历史线性、可审查、可回滚。
2. 让功能开发、缺陷修复和紧急修复有明确分支边界。
3. 保证应用仓与外部依赖 `oh-my-robot-framework` 的同步关系可追溯。

本规范采用 OMR 精简版，不照搬只适用于 OMR 官方仓的 fork / upstream 发布纪律。

## 2. 分支模型

长期分支：

1. `main`：稳定主干，只接收经过评审的合入。
2. `integration`：日常集成主线，接收功能与常规修复。

任务分支：

1. `feature/<issue编号>-<slug>`：功能开发。
2. `fix/<issue编号>-<slug>`：常规缺陷修复，从 `integration` 切出并回到 `integration`。
3. `hotfix/<issue编号>-<slug>`：已发布问题的紧急修复，从 `main` 切出，先回 `main`，再回灌 `integration`。

## 3. 基本协作原则

1. 所有开发都应绑定任务编号或明确问题单，禁止无任务开发。
2. 每个任务分支只做一件事，避免大杂烩提交。
3. 提交 PR 前必须先把当前分支基于目标基线做 `rebase`，不使用 `merge` 同步基线。
4. 变基后若需要重写远端分支历史，使用 `--force-with-lease`，不使用裸 `--force`。
5. 对 `main` 和 `integration` 禁止强推。

## 4. 日常工作流

### 4.1 功能开发

1. 从 `integration` 切出 `feature/*`
2. 开发并做本地验证
3. 提交前 `rebase integration`
4. 推送分支并提 PR 到 `integration`

### 4.2 常规修复

1. 从 `integration` 切出 `fix/*`
2. 修复并验证
3. 提交前 `rebase integration`
4. 提 PR 到 `integration`

### 4.3 紧急修复

1. 从 `main` 切出 `hotfix/*`
2. 修复并验证
3. 提交前 `rebase main`
4. 先提 PR 到 `main`
5. 合入 `main` 后再把同一修复回灌到 `integration`

## 5. Commit 与 PR 规范

### 5.1 Commit Message

固定格式：

```text
类型(作用域): 简短描述
```

类型建议：

1. `feat`
2. `fix`
3. `docs`
4. `style`
5. `refactor`
6. `chore`

若当前团队使用 issue 编号锚点，提交信息末尾追加 `(#<id>)`。

### 5.2 原子提交

1. 一次提交只做一件事。
2. 格式化、重构、功能、修复分开提交。
3. 提交切入后应尽量保持可构建、可验证。

### 5.3 PR 说明

每个 PR 至少写清：

1. 变更范围
2. 风险点
3. 验证结果
4. 是否涉及 `oh-my-robot-framework` 依赖变化

## 6. 与 `oh-my-robot-framework` 的同步原则

`new_robot_code` 当前通过相对路径依赖外部同级 `oh-my-robot-framework`。因此必须遵守：

1. 不长期依赖只存在于本地、未沉淀的框架私改。
2. 若为了 `new_robot_code` 修改框架：
   - 要么把修改正式回推并纳入框架侧协作流，
   - 要么在本仓库文档中明确记录依赖的补丁状态、适用版本和风险。
3. 应用仓 PR 如依赖框架补丁，必须在描述中写清：
   - 依赖哪个框架修改
   - 当前是否已 upstream
   - 若未 upstream，如何在新环境复现

## 7. 文档与代码的同步要求

以下变化发生时，应同步更新仓库文档：

1. 顶层构建目标名、产物名、烧录名变化
2. 正式启动链变化
3. 目录结构与 owner 边界变化
4. 对外部框架依赖方式变化

## 8. 最小检查清单

提交前至少自检：

1. 当前分支是否基于正确基线切出
2. 是否已完成必要构建验证
3. 是否已对目标基线完成 `rebase`
4. 是否夹带无关文件
5. 若涉及框架依赖，是否已在文档或 PR 中说明
