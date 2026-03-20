# Git 管理规范（f1tenth_car_ws）

本文档用于统一本仓库的 Git 管理方式，避免出现“主仓库 + 多个嵌套仓库 + 异常子模块”导致的混乱状态。

## 1. 管理目标

1. 主仓库作为唯一开发入口。
2. 默认不跟踪编译产物、缓存、日志、大体积录包。
3. 三方代码管理策略明确：要么规范子模块，要么完整 vendor 到主仓库，禁止混用。

## 2. 分支与提交建议

1. `main`：稳定分支，仅合并通过验证的改动。
2. `feat/*`：功能分支，例如 `feat/fastlio-mid360-tuning`。
3. `fix/*`：修复分支。

提交建议：

1. 一次提交只做一件事（配置调整、功能新增、重构分开提交）。
2. 提交信息建议：`type(scope): summary`
3. 推荐 `type`：`feat` `fix` `refactor` `docs` `chore`

## 3. 三方代码策略（必须二选一）

### 方案 A：统一单仓库（推荐当前项目）

适用：你们长期在本仓库里改三方代码。

规则：

1. 删除嵌套 `.git`，让目录成为主仓库普通目录。
2. 主仓库直接跟踪这些目录的源码。

### 方案 B：保留子模块

适用：严格跟随上游、很少改三方源码。

规则：

1. 每个子模块必须在 `.gitmodules` 有正确映射。
2. 团队统一使用 `git submodule update --init --recursive`。
3. 禁止出现“索引是子模块，但 `.gitmodules` 缺失”的状态。

## 4. 当前仓库已发现问题（2026-03-20）

1. `src/car_hardware/livox_ros_driver2` 在索引中是 gitlink（mode `160000`），但 `.gitmodules` 缺失。
2. 存在嵌套仓库：
   - `src/car_localization/slam_toolbox/.git`
   - `src/car_localization/pcd2pgm/.git`
   - `src/car_hardware/livox_ros_driver2/.git`
3. 工作区存在大量新增/删除混合状态，需要分批整理提交。

## 5. 推荐落地流程（按顺序执行）

1. 先做一次状态快照：

```bash
git status -sb
git ls-files -s src/car_hardware/livox_ros_driver2
find . -type d -name .git
```

2. 确认三方策略（A 或 B）。
3. 按策略修复仓库结构。
4. 分批提交：

```bash
# 例：先提交 git 规范文件
git add .gitignore docs/GIT_MANAGEMENT.md
git commit -m "chore(git): standardize ignore rules and management guide"
```

5. 再分模块提交代码变更（`car_hardware`、`car_localization`、`car_navigation` 分开）。

## 6. 提交前检查清单

1. `git status` 中无 `build/install/log` 产物。
2. 不存在意外 `__pycache__`、`.pyc`、临时文件。
3. 不存在异常 gitlink 或丢失 `.gitmodules` 的子模块。
4. commit message 清晰可追溯。
