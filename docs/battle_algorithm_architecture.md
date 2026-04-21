# Battle Fast2 纯启发式架构说明

## 1. 当前定位

`battle_fast2_node` 已回退为单一路径的纯启发式控制器：

**启发式前端 + 启发式后端 + 直接发布 `/drive`**

当前版本不再依赖 `/odom`，也不再包含任何 MPC 求解、模式切换或回退逻辑。

---

## 2. 输入输出

### 2.1 输入
- `/scan`
  `sensor_msgs/msg/LaserScan`

### 2.2 输出
- `/drive`
  `ackermann_msgs/msg/AckermannDriveStamped`

### 2.3 调试输出
- `battle_fast2/drive_heuristic`
- `battle_fast2/current_speed_mps`

### 2.4 可视化输出
- `marker_topic`
- `debug_scan_topic`

---

## 3. 整体控制链

一次 scan 回调的处理链固定为：

```text
/scan
  -> frontend_scan_process()
  -> heuristic_backend_control()
  -> control_select_and_publish()
  -> /drive
```

这意味着：
- 前端负责局部可通行方向搜索与行为状态判断
- 后端负责生成启发式转向和基础速度
- 输出阶段只做启发式收口，不再叠加 MPC 分支

---

## 4. 模块职责

### 4.1 前端感知
`frontend_scan_process()` 负责：
- 前向激光提取
- 零值补全与异常值平滑
- 障碍边界识别与膨胀
- 最大可通行方向搜索
- 动态障碍判断
- `Follow` / `chaoche` 状态判别

这些逻辑保持 battle_fast2 当前启发式语义不变。

### 4.2 启发式后端
`heuristic_backend_control()` 负责：
- 计算启发式转向角
- 计算基础速度 `base_speed`
- 结合 `speed_rate`、`turn_rate` 保留直道、弯道、过渡段语义

### 4.3 输出收口
`control_select_and_publish()` 负责：
- 直接使用启发式转向
- 保留 `Follow` 的低速语义：`speed = min(MIN_OBS_SPEED, heuristic_speed)`
- `chaoche` 和普通通行保持启发式速度输出
- 发布 `/drive` 和 `battle_fast2/drive_heuristic`

---

## 5. 保留与删除

### 5.1 保留
- `GO_STARIGHT`
- `TRANSITION`
- `speed_rate`
- `turn_rate`
- `Follow`
- `chaoche`
- `P / D`
- 方向选择、避障和速度公式

### 5.2 已删除
- `/odom` 输入
- 所有 `mpc_*` 参数
- MPC 求解函数和内部状态
- `battle_fast2/drive_mpc`
- `battle_fast2/mpc_ok`
- `battle_fast2/mpc_solve_time_ms`
- `battle_fast2/fallback_rate`
- `battle_fast2/control_mode`
- `battle_fast2/mode_switch_count`

---

## 6. 运行方式

```bash
ros2 launch roboracer_china_2025 battle_fast2.launch.py
```

默认只需要提供激光输入即可运行纯启发式控制。

---

## 7. 一句话总结

当前 `battle_fast2_node` 的真实形态是：

**只靠激光和启发式规则完成方向与速度控制，不再包含任何 MPC 或 odom 依赖。**
