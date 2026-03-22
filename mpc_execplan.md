# Battle Fast2 + Odom-MPC 执行计划（不含 reverse，不复用 simple_mpc）

## 0. 文档目标
本计划用于指导在 **保留现有 battle_fast2 scan 前端逻辑** 的前提下，逐步引入基于里程计（odom）的局部参考点 MPC 控制后端，并确保可回退、可验证、可迭代。

适用边界：
- 不实现 reverse 安全接管（后续由独立 safety 节点承担）。
- 不复用仓库内 `simple_mpc` 包代码。
- 优先采用 **单节点最小侵入改造**。

---

## A. 对现有代码的理解

### A1. 当前控制闭环形态
`battle_fast2_node.py` 当前结构为单节点反应式控制：
1. 输入：`/scan` (`sensor_msgs/msg/LaserScan`)
2. 前端：
- 前向 180° 取样与滤波
- 障碍提取与膨胀
- 最大可行方向筛选
- `Follow / chaoche` 状态判定
3. 后端：
- 在同一回调里用启发式公式直接计算 `steering/speed`
- 发布 `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`)

### A2. 改造目标
在保留上述 scan 前端行为的前提下，将后端控制逐步替换为 MPC：
1. Phase 1：MPC 仅接管 steering。
2. Phase 2：MPC 接管 steering + speed。
3. 全程保持失败回退：MPC 失败或超时自动回退到启发式控制。

### A3. 现有系统依赖
- 车辆模型：F1TENTH Ackermann
- 里程计来源：`/odom` (`nav_msgs/msg/Odometry`)（由 vesc_to_odom 提供）

---

## B. 推荐总体架构

### B1. 架构原则
采用“**先单节点最小侵入，后工程化增强**”策略：
1. 保持 `battle_fast2_node.py` 为主节点。
2. 节点内新增 odom 订阅和 MPC 子模块。
3. 通过参数切换控制模式：`off / steer_only / full`。
4. 保持启发式后端作为可靠 fallback。

### B2. 节点内职责拆分
在单节点内按三层组织：
1. `frontend_scan_process()`：前端方向/避障/Follow/chaoche（保留原语义）
2. `heuristic_backend_control()`：当前启发式 steering/speed
3. `control_select_and_publish()`：根据配置选择 heuristic 或 MPC，并处理回退

---

## C. 分阶段实施计划（Phase 0/1/2/3）

## Phase 0：结构重整（行为不变）

### 目标
在不改变原始行为的前提下，把逻辑拆成前端、后端、选择器三层，并接入 odom 缓存。

### 任务清单
1. 拆分 `middle_line_callback` 流程：
- 前端处理函数
- 启发式后端函数
- 控制选择与发布函数
2. 新增 odom 订阅与缓存结构（仅保存状态，不直接控制）。
3. 增加模式/回退状态编码。
4. 增加调试输出 topic（heuristic、mpc、mode、mpc_ok）。

### 完成标准
- `mpc_enable=false` 下输出行为与原策略一致。
- 节点在无 odom 情况下仍可运行（MPC禁用或自动回退）。

### 回退策略
- 参数：`mpc_enable=false` 或 `mpc_mode=off`。

---

## Phase 1：MPC 接管 steering（速度沿用原逻辑）

### 目标
仅替换 steering 控制通道，速度继续使用原启发式速度链路。

### 任务清单
1. 局部参考点构建：`build_local_ref()`
- 输入：前端方向结果 + 当前目标速度
- 输出：短时域 local reference (`x/y/yaw/v`)
2. 新增 MPC 求解核心：
- 运动学自行车模型滚动预测
- 代价函数包含：横向误差、航向误差、速度误差、输入变化惩罚
3. 输出接管策略：
- `steer_only` 模式：`steering <- MPC`，`speed <- heuristic`
4. 失败回退：
- 条件：无 odom / 超时 / 求解失败
- 动作：当帧使用 heuristic steering

### 完成标准
- `mpc_mode=steer_only` 时 steering 由 MPC 主导。
- 发生求解异常时 1 帧内回退，不中断控制输出。

### 回退策略
- `mpc_mode=off` 即刻回到纯启发式。

---

## Phase 2：MPC 接管 steering + speed

### 目标
将速度也纳入 MPC 输出，实现 full 模式控制。

### 任务清单
1. 扩展求解变量或输出处理：
- 输出 `steering + speed`（或 `steering + accel` 推导 speed）
2. 保持行为边界：
- `Follow/chaoche` 继续保留（作为速度约束/上层限幅）
3. 控制选择器更新：
- `full` 模式启用 MPC speed
- 失败自动退回启发式 steering+speed

### 完成标准
- `mpc_mode=full` 下稳定输出 steering + speed。
- Follow/chaoche 场景中速度边界语义保留。

### 回退策略
- `full -> steer_only -> off` 逐级降级。

---

## Phase 3：工程化增强

### 目标
提升可调试性、可维护性和长期演进能力。

### 任务清单
1. 调试观测完善：
- 求解耗时
- 回退触发率
- 模式切换统计
2. 参数模板与调参说明文档化。
3. 预留与未来 safety 节点对接接口（本阶段不实现 safety）。

### 完成标准
- 可重复部署、可复现实验结果、可快速定位故障。

---

## D. 文件与函数改动清单

## D1. 核心代码
文件：`src/car_navigation/roboracer_china_2025/roboracer_china_2025/battle_fast2_node.py`

新增/重构重点：
1. 新增订阅：`odom_callback()`
2. 前端函数：`frontend_scan_process()`
3. 启发式后端：`heuristic_backend_control()`
4. MPC参考与求解：
- `build_local_ref()`
- `rollout_cost()`
- `solve_mpc()`
- `fallback_guard()`
5. 输出仲裁：`control_select_and_publish()`
6. 主回调编排：`middle_line_callback()`

## D2. 启动参数
文件：`src/car_navigation/roboracer_china_2025/launch/battle_fast2.launch.py`

新增参数：
- `odom_topic`
- `mpc_enable`
- `mpc_mode`
- `mpc_timeout_ms`
- `wheelbase`
- `horizon`
- `dt`
- `mpc_steer_candidates`
- `mpc_accel_candidates`
- `mpc_max_speed`
- `mpc_min_speed`
- `mpc_max_accel`
- `mpc_max_decel`

## D3. 依赖声明
文件：`src/car_navigation/roboracer_china_2025/package.xml`

新增依赖：
- `nav_msgs`
- `std_msgs`

---

## E. ROS2 接口说明

## E1. 保留接口
1. 输入：`/scan` (`sensor_msgs/msg/LaserScan`)
2. 输入：`/odom` (`nav_msgs/msg/Odometry`)
3. 输出：`/drive` (`ackermann_msgs/msg/AckermannDriveStamped`)

## E2. 新增调试接口
1. `battle_fast2/drive_heuristic` (`AckermannDriveStamped`)
2. `battle_fast2/drive_mpc` (`AckermannDriveStamped`)
3. `battle_fast2/control_mode` (`std_msgs/msg/UInt8`)
4. `battle_fast2/mpc_ok` (`std_msgs/msg/Bool`)

## E3. 控制模式编码
- `0`: Heuristic
- `1`: MPC Steer Only
- `2`: MPC Full
- `3`: Fallback

---

## F. 验证方案（仿真/实车）

## F1. Phase 0 验证
1. `mpc_enable=false` 下回归跑通。
2. 对比改造前后输出趋势一致性（允许微小浮点差异）。

## F2. Phase 1 验证（steer_only）
1. 低速仿真直道/缓弯：确认 steering 由 MPC 接管。
2. 故障注入（超时/无 odom）：确认回退 1 帧内生效。

## F3. Phase 2 验证（full）
1. 分级提速：低速 -> 中速 -> 目标竞速速度。
2. 动态障碍场景：确认 Follow/chaoche 语义保留。

## F4. Phase 3 验证
1. 长时间稳定性：统计 P95/P99 求解耗时。
2. 回退率与模式切换频次分析。

---

## G. 风险与规避策略

1. 局部参考点质量不稳定导致抖动。
- 规避：短时域、小步长、平滑参考点、增加输入变化惩罚。

2. 求解耗时超预算。
- 规避：固定候选网格规模 + timeout + 失败即回退。

3. 坐标语义混淆（局部/全局）。
- 规避：当前版本统一以 `base_link` 局部几何构建参考点，odom主要用于速度状态。

4. 模式切换突变。
- 规避：保留启发式作为连续 fallback，必要时增加切换限幅。

---

## H. 最小可行实施路径（MVP）

1. 先完成 Phase 0（结构化 + odom缓存 + 选择器）。
2. 直接进入 Phase 1（steer_only），先保证稳定回退链。
3. 稳定后切换到 Phase 2（full）。
4. Phase 3 做工程化，不在本轮引入 safety 节点实现。

---

## I. 推荐运行方式

1. 纯启发式：
```bash
ros2 launch roboracer_china_2025 battle_fast2.launch.py mpc_enable:=false mpc_mode:=off
```

2. Steering-only：
```bash
ros2 launch roboracer_china_2025 battle_fast2.launch.py mpc_enable:=true mpc_mode:=steer_only
```

3. Full MPC：
```bash
ros2 launch roboracer_china_2025 battle_fast2.launch.py mpc_enable:=true mpc_mode:=full
```
