# Local Planner

**全向移动机器人的局部路径规划与跟踪系统**

---

## 目录

- [概述](#概述)
- [系统架构](#系统架构)
- [核心组件](#核心组件)
- [路径跟踪算法](#路径跟踪算法)
- [快速开始](#快速开始)
- [配置参数](#配置参数)
- [测试与验证](#测试与验证)
- [文档](#文档)
- [常见问题](#常见问题)

---

## 概述

Local Planner 是一个为全向移动机器人设计的局部路径规划与跟踪系统，支持多种路径跟踪算法，能够在复杂环境中实现高精度的轨迹跟踪。

### 主要特性

- ✅ **多种路径跟踪算法**
  - 传统 Pure Pursuit 路径跟踪器
  - 先进的 MPCC (Model Predictive Contouring Control) 跟踪器
- ✅ **全向底盘支持**
  - 支持 3 自由度全向移动（vx, vy, ω）
  - 横向平移能力
  - 正/反向驱动
- ✅ **实时障碍物检测**
  - 基于激光点云的障碍物检测
  - 动态路径选择
  - 可配置的安全距离
- ✅ **精确对齐逻辑**
  - 目标点位置对齐
  - 航向精确对齐
  - 支持直线/曲线对齐模式
- ✅ **完善的测试框架**
  - 多种测试轨迹（圆形、8字形、直线、倒车）
  - RViz 可视化
  - 性能评估工具

### 应用场景

- 🤖 移动机器人自主导航
- 📦 AGV/AMR 物流运输
- 🏭 工业移动平台
- 🔬 移动机器人研究

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      Local Planner 系统                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌───────────────┐      ┌──────────────────┐                │
│  │               │      │                  │                │
│  │ localPlanner  │──────│  Path Generator  │                │
│  │               │      │                  │                │
│  └───────┬───────┘      └──────────────────┘                │
│          │                                                   │
│          │ /local_path                                       │
│          │                                                   │
│          ├──────────────┬────────────────────────────────┐  │
│          │              │                                │  │
│          ▼              ▼                                ▼  │
│  ┌──────────────┐ ┌──────────────┐             ┌──────────┐│
│  │              │ │              │             │          ││
│  │ pathFollower │ │ mpccfollower │             │  Other   ││
│  │ (Pure Pursuit│ │    (MPCC)    │             │ Followers││
│  │              │ │              │             │          ││
│  └──────┬───────┘ └──────┬───────┘             └────┬─────┘│
│         │                │                          │      │
│         └────────────────┴──────────────────────────┘      │
│                          │                                  │
│                          ▼                                  │
│                     /cmd_vel                                │
└─────────────────────────────────────────────────────────────┘
         ▲                                    │
         │                                    │
    /livox_points                        /cmd_vel
    /localization                             │
                                              ▼
                                      Mobile Base
```

### 数据流

```
传感器数据 ──► localPlanner ──► 候选路径 ──► pathFollower/mpccfollower ──► 速度指令
   │                                               │
   │                                               │
   └─────────────── 位置反馈 ◄───────────────────┘
```

---

## 核心组件

### 1. localPlanner (路径生成器)

**功能**：根据全局目标点和传感器数据生成局部候选路径。

**输入**：
- `/livox_points` (sensor_msgs/PointCloud2) - 激光点云
- `/localization` (nav_msgs/Odometry) - 机器人位姿
- `/goalPoint` (geometry_msgs/PointStamped) - 目标点

**输出**：
- `/local_path` (nav_msgs/Path) - 局部路径
- `/local_path_*` (可视化路径)

**关键文件**：
- `src/localPlanner.cpp`

**特性**：
- 多候选路径生成（不同方向和长度）
- 基于点云的障碍物检测
- 路径代价评估（方向、障碍物、地形）
- 路径平滑与滤波
- 目标点对齐路径生成（Hermite 样条）

### 2. pathFollower (Pure Pursuit 跟踪器)

**功能**：基于 Pure Pursuit 算法的传统路径跟踪器。

**输入**：
- `/local_path` (nav_msgs/Path) - 目标路径
- `/localization` (nav_msgs/Odometry) - 机器人位姿

**输出**：
- `/cmd_vel` (geometry_msgs/Twist) - 速度指令

**关键文件**：
- `src/pathFollower.cpp`

**特性**：
- 前视距离自适应
- 速度滤波与加速度限制
- 转向速率限制
- 坡度感知减速

### 3. mpccfollower (MPCC 跟踪器)

**功能**：基于模型预测轮廓控制的高精度路径跟踪器。

**输入**：
- `/local_path` (nav_msgs/Path) - 目标路径
- `/localization` (nav_msgs/Odometry) - 机器人位姿

**输出**：
- `/cmd_vel` (geometry_msgs/Twist) - 速度指令
- `/mpcc_pred_path` (nav_msgs/Path) - 预测轨迹（调试用）

**关键文件**：
- `src/mpccfollower.cpp`
- `config/mpccfollower.yaml`

**特性**：
- 轮廓误差 + 滞后误差最小化
- 路径参数优化
- 约束满足（速度、加速度）
- HPIPM 高性能 QP 求解器
- 30 Hz 实时控制

**详细原理**：参见 [MPCC 理论文档](docs/MPCC_THEORY.md)

### 4. mpcc_pred_visualizer (预测轨迹可视化)

**功能**：将 MPCC 预测轨迹可视化为彩色带状标记。

**输入**：
- `/mpcc_pred_path` (nav_msgs/Path) - 预测轨迹
- `/localization` (nav_msgs/Odometry) - 机器人位姿

**输出**：
- `/mpcc_pred_marker` (visualization_msgs/Marker) - 可视化标记

**关键文件**：
- `scripts/mpcc_pred_visualizer.py`

### 5. mpcc_test_framework (测试框架)

**功能**：MPCC 跟踪器的独立测试框架，支持多种轨迹类型。

**关键文件**：
- `scripts/mpcc_test_framework.py`
- `launch/test_mpcc_framework.launch`

**支持的轨迹**：
- 圆形轨迹 (CIRCLE)
- 8 字形轨迹 (FIGURE_EIGHT)
- 直线轨迹 (STRAIGHT)
- 倒车轨迹 (STRAIGHT_BACKWARD)
- 方形轨迹 (SQUARE)

---

## 路径跟踪算法

### Pure Pursuit

**优点**：
- 实现简单，计算量小
- 参数少，易于调试
- 适合低速场景

**缺点**：
- 转弯处容易切角
- 跟踪精度有限
- 不支持约束优化

**适用场景**：
- 对精度要求不高的导航任务
- 计算资源受限的平台
- 低速移动场景

### MPCC (推荐)

**优点**：
- 高精度跟踪（轮廓误差 < 5 cm）
- 支持约束优化（速度、加速度）
- 路径参数自适应
- 利用全向底盘横向能力

**缺点**：
- 计算量较大（需要求解 QP）
- 参数调优复杂
- 需要连续可导路径

**适用场景**：
- 高精度对齐任务
- 复杂轨迹跟踪
- 全向底盘机器人

**性能指标**（实测）：
- 控制频率：30 Hz
- 轮廓误差：< 0.05 m
- 滞后误差：< 0.10 m
- 最终对齐精度：位置 < 0.05 m，航向 < 3°

---

## 快速开始

### 环境要求

- **操作系统**：Ubuntu 20.04
- **ROS 版本**：ROS Noetic
- **依赖库**：
  - Eigen3
  - PCL (Point Cloud Library)
  - HPIPM (用于 MPCC)
  - BLASFEO (HPIPM 依赖)

### 编译

```bash
cd /path/to/your/catkin_ws
catkin_make
# 或
catkin build local_planner
```

### 运行

#### 方式 1：使用 Pure Pursuit 跟踪器

```bash
roslaunch local_planner local_planner.launch
```

**参数**：
- `goalX` - 目标点 X 坐标（默认：0）
- `goalY` - 目标点 Y 坐标（默认：0）
- `maxSpeed` - 最大速度（默认：1.0 m/s）
- `points_topic` - 点云话题（默认：`/livox_points`）
- `state_topic` - 位姿话题（默认：`/localization`）

**示例**：
```bash
roslaunch local_planner local_planner.launch goalX:=5.0 goalY:=3.0 maxSpeed:=0.5
```

#### 方式 2：使用 MPCC 跟踪器（推荐）

```bash
roslaunch local_planner local_planner_mpcc.launch
```

**参数**：同上，外加 MPCC 特有参数（在 `config/mpccfollower.yaml` 中配置）

#### 方式 3：测试 MPCC（无需真实传感器）

```bash
roslaunch local_planner test_mpcc_framework.launch trajectory_type:=CIRCLE
```

**支持的轨迹类型**：
- `CIRCLE` - 圆形
- `FIGURE_EIGHT` - 8 字形
- `STRAIGHT` - 直线
- `STRAIGHT_BACKWARD` - 倒车
- `SQUARE` - 方形

---

## 配置参数

### localPlanner 参数

参考：`launch/local_planner.launch`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `vehicleLength` | double | 0.5 | 机器人长度 [m] |
| `vehicleWidth` | double | 1.0 | 机器人宽度 [m] |
| `checkObstacle` | bool | true | 是否检查障碍物 |
| `obstacleHeightThre` | double | 0.35 | 障碍物高度阈值 [m] |
| `adjacentRange` | double | 5.0 | 点云裁剪距离 [m] |
| `maxSpeed` | double | 1.0 | 最大速度 [m/s] |
| `reach_goal_thre_g` | double | 0.5 | 触发对齐的距离阈值 [m] |
| `align_at_goal` | bool | true | 是否在目标点对齐 |
| `align_pos_thre_g` | double | 0.05 | 对齐位置阈值 [m] |
| `align_yaw_thre_deg` | double | 3.0 | 对齐航向阈值 [度] |
| `align_use_straight_path` | bool | true | 使用直线对齐路径 |

### pathFollower 参数

参考：`launch/local_planner.launch`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lookAheadDis` | double | 2.5 | 前视距离 [m] |
| `yawRateGain` | double | 2.5 | 转向速率增益 |
| `maxYawRate` | double | 25.0 | 最大转向速率 [deg/s] |
| `maxSpeed` | double | 1.0 | 最大速度 [m/s] |
| `maxAccel` | double | 1.0 | 最大加速度 [m/s²] |
| `stopDisThre` | double | 0.5 | 停止距离阈值 [m] |

### mpccfollower 参数

参考：`config/mpccfollower.yaml`

**核心参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mpc_hz` | double | 30.0 | MPC 控制频率 [Hz] |
| `mpc_horizon` | int | 15 | MPC 预测步数 |
| `vx_max` | double | 1.0 | X 方向最大速度 [m/s] |
| `vy_max` | double | 1.0 | Y 方向最大速度 [m/s] |
| `w_max` | double | 0.5 | 最大角速度 [rad/s] |
| `vs_max` | double | 0.5 | 最大路径速度 [m/s] |
| `vs_min` | double | -0.5 | 最小路径速度（允许倒车） [m/s] |

**代价函数权重**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `qC` | 20.0 | 轮廓误差权重 |
| `qL` | 5.0 | 滞后误差权重 |
| `qYaw` | 6.0 | 航向误差权重 |
| `qC_f` | 40.0 | 终端轮廓误差权重 |
| `qL_f` | 10.0 | 终端滞后误差权重 |
| `qYaw_f` | 12.0 | 终端航向误差权重 |
| `rVx` | 1.0 | vx 控制惩罚 |
| `rVy` | 1.0 | vy 控制惩罚 |
| `rW` | 1.0 | ω 控制惩罚 |
| `rVs` | 0.5 | vs 控制惩罚 |
| `qVs` | 0.05 | vs 速度奖励 |

**详细参数说明**：参见 [mpccfollower.yaml](config/mpccfollower.yaml)（所有参数都有中文注释）

---

## 测试与验证

### 1. MPCC 测试框架

**启动测试**：

```bash
# 圆形轨迹测试
roslaunch local_planner test_mpcc_framework.launch trajectory_type:=CIRCLE

# 8 字形轨迹测试
roslaunch local_planner test_mpcc_framework.launch trajectory_type:=FIGURE_EIGHT

# 倒车测试
roslaunch local_planner test_mpcc_framework.launch trajectory_type:=STRAIGHT_BACKWARD
```

**可视化**：

在 RViz 中添加以下话题：
- `/local_path` - 参考路径（红色）
- `/mpcc_pred_marker` - MPC 预测轨迹（彩色渐变带）
- `/robot_marker` - 机器人位姿（箭头）

### 2. 性能评估

测试框架会实时输出：
- 轮廓误差 (Contour Error)
- 滞后误差 (Lag Error)
- 航向误差 (Yaw Error)
- 控制输入 (vx, vy, ω, vs)

### 3. 参数调优指南

参见：[MPCC 理论文档 - 参数调优指南](docs/MPCC_THEORY.md#10-参数调优指南)

**常见问题**：

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 振荡/抖动 | qC, qL 过大 | 降低跟踪权重，增加 rVx, rVy |
| 切角严重 | qC 过小 | 增加轮廓误差权重 |
| 转弯慢 | rW 过大 | 降低角速度惩罚 |
| 冲出路径 | qVs 过大 | 降低速度奖励 |

---

## 文档

### 📚 技术文档

- **[MPCC 理论与实现](docs/MPCC_THEORY.md)**
  - 详细的数学推导
  - 运动学模型建立
  - MPC 框架与矩阵构造（G, E, H, Q, F, R 矩阵）
  - QP 转换与求解
  - 参数调优指南

### 📖 配置文件

- **[mpccfollower.yaml](config/mpccfollower.yaml)** - MPCC 控制器参数（含详细中文注释）

### 🔧 Launch 文件

- **[local_planner.launch](launch/local_planner.launch)** - Pure Pursuit 模式
- **[local_planner_mpcc.launch](launch/local_planner_mpcc.launch)** - MPCC 模式
- **[test_mpcc_framework.launch](launch/test_mpcc_framework.launch)** - MPCC 测试框架

### 📝 代码文档

核心源文件：
- `src/localPlanner.cpp` - 路径生成器实现（含对齐逻辑）
- `src/pathFollower.cpp` - Pure Pursuit 跟踪器
- `src/mpccfollower.cpp` - MPCC 跟踪器（含详细注释）
- `scripts/mpcc_test_framework.py` - 测试框架
- `scripts/mpcc_pred_visualizer.py` - 预测轨迹可视化

---

## 常见问题

### Q1: 如何选择 Pure Pursuit 还是 MPCC？

**选择 Pure Pursuit**：
- 计算资源受限
- 对精度要求不高（> 0.2 m）
- 低速场景（< 0.3 m/s）
- 简单路径跟踪

**选择 MPCC**：
- 需要高精度对齐（< 0.05 m）
- 复杂轨迹跟踪
- 全向底盘机器人
- 有充足计算资源

### Q2: MPCC 控制频率能否提高到 50 Hz？

可以，修改 `mpc_hz: 50.0`，但需要：
1. 确保 QP 求解时间 < 20 ms
2. 可能需要减少 `mpc_horizon`（如降至 10）
3. 测试实时性

### Q3: 如何启用倒车功能？

修改 `mpccfollower.yaml`：
```yaml
vs_min: -0.5  # 允许最大 -0.5 m/s 倒车速度
```

### Q4: 机器人在目标点附近振荡怎么办？

可能原因：
1. `align_pos_thre_g` 过小 → 增大到 0.08
2. `qVs` 过大 → 降低到 0.01
3. `vs_max` 过大 → 降低到 0.3

### Q5: 如何调整对齐精度？

修改 `local_planner.launch` 或 `local_planner_mpcc.launch`：
```xml
<param name="align_pos_thre_g" value="0.02" />    <!-- 更严格的位置阈值 -->
<param name="align_yaw_thre_deg" value="1.0" />   <!-- 更严格的航向阈值 -->
```

同时在 `mpccfollower.yaml` 中增加终端权重：
```yaml
qC_f: 80.0   # 加倍终端轮廓误差权重
qYaw_f: 24.0 # 加倍终端航向误差权重
```

### Q6: 点云话题不匹配怎么办？

修改 launch 文件中的 `points_topic` 参数：
```bash
roslaunch local_planner local_planner_mpcc.launch points_topic:=/your_point_cloud_topic
```

### Q7: 如何调试 MPCC 性能？

1. **启用预测轨迹可视化**：
   ```bash
   roslaunch local_planner local_planner_mpcc.launch
   # RViz 中添加 /mpcc_pred_marker 话题
   ```

2. **使用测试框架**：
   ```bash
   roslaunch local_planner test_mpcc_framework.launch trajectory_type:=CIRCLE
   # 观察终端输出的误差指标
   ```

3. **记录数据**：
   ```bash
   rosbag record /cmd_vel /localization /local_path /mpcc_pred_path
   ```

### Q8: MPCC 求解失败怎么办？

可能原因：
1. **路径不连续** → 检查 `/local_path` 是否有跳变
2. **约束冲突** → 检查 `s_trust` 参数（默认 1.5）
3. **初始状态远离路径** → 确保机器人初始位置在路径起点附近

---

## 贡献

欢迎贡献代码、报告问题或提出改进建议！

### 报告问题

请在 issue 中提供：
1. 使用的 launch 文件
2. 修改的参数
3. 错误日志
4. ROS 版本和操作系统

### 改进建议

欢迎提出以下方面的改进：
- 新的路径跟踪算法
- 性能优化
- 文档完善
- 测试用例

---

## 许可证

[根据项目实际情况填写]

---

## 联系方式

- **维护者**：[根据项目实际情况填写]
- **邮箱**：[根据项目实际情况填写]

---

## 更新日志

### v1.2.0 (2026-02)
- ✨ 新增 MPCC 跟踪器
- ✨ 新增测试框架
- ✨ 新增预测轨迹可视化
- 📝 完善文档（MPCC 理论、参数说明）
- 🐛 修复航向角累积问题
- 🐛 修复倒车支持

### v1.1.0
- ✨ 新增 Hermite 样条对齐逻辑
- ✨ 支持直线对齐模式（全向底盘）
- 🔧 优化对齐阶段速度控制

### v1.0.0
- 🎉 初始版本
- ✅ Pure Pursuit 路径跟踪
- ✅ 障碍物检测与路径规划
- ✅ 基本对齐功能

---

## 致谢

- HPIPM 求解器：https://github.com/giaf/hpipm
- MPCC 算法参考：Liniger et al., 2015

---

**Happy Navigating! 🚀**
