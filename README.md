# Local Planner with MPCC Unified Controller

全向底盘路径规划与跟踪控制系统，基于模型预测轮廓控制（MPCC）实现高精度路径跟踪。

## 系统架构

### 数据流

```
         /localization (odom)
               │
        ┌──────┴──────────────┐
        │                     │
        v                     v
┌──────────────┐      ┌──────────────────┐
│ localPlanner │      │ mpccfollower     │
│   (规划器)   │      │  _unified        │
└──────┬───────┘      │  (MPCC控制器)    │
       │              └────────┬─────────┘
       │ /local_path           │
       └──────────────>────────┘
                               │
                               │ /cmd_vel
                               v
                     ┌──────────────────┐
                     │  Mobile Base     │
                     │  (全向底盘)      │
                     └──────────────────┘
```

### 关键话题

| 话题 | 类型 | 作用 | 来源/去向 |
|------|------|------|-----------|
| `/localization` | Odometry | 机器人位姿 | 定位系统 → 规划器/控制器 |
| `/local_path` | Path | 规划路径 | 规划器 → 控制器 ⭐ |
| `/cmd_vel` | Twist | 速度命令 | 控制器 → 底盘 ⭐ |
| `/move_base_simple/goal` | PoseStamped | 目标点 | RViz/用户 → 规划器 |
| `/livox_points` | PointCloud2 | 激光点云 | 雷达 → 规划器（避障） |

### 核心组件

#### 1. localPlanner（路径规划器）

生成全局路径并标记路径方向（前进/倒车/旋转），避障处理。

**订阅话题**：
- `/localization` (nav_msgs/Odometry) - 机器人位姿
- `/livox_points` (sensor_msgs/PointCloud2) - 激光雷达点云（障碍物检测）
- `/move_base_simple/goal` (geometry_msgs/PoseStamped) - 目标点
- `/check_obstacle` (std_msgs/Bool) - 启用/禁用障碍物检测

**发布话题**：
- `/local_path` (nav_msgs/Path) - 规划的路径 → 发送给mpccfollower
- `/stop` (std_msgs/Int8) - 停止命令
- `/goal_reached` (std_msgs/Bool) - 目标到达标志
- `/local_scans` (sensor_msgs/PointCloud2) - 处理后的点云

---

#### 2. mpccfollower_unified（MPCC控制器）

统一MPCC控制器，优化求解速度命令实现精确路径跟踪。

**订阅话题**：
- `/localization` (nav_msgs/Odometry) - 机器人位姿（可配置odom_topic）
- `/local_path` (nav_msgs/Path) - 路径输入 ⭐**主要输入**
- `/speed` (std_msgs/Float32) - 外部速度命令（可选）
- `/stop` (std_msgs/Int8) - 停止命令
- `/move_base_simple/goal` (geometry_msgs/PoseStamped) - 目标点（末端对齐用）
- `/goal_reached` (std_msgs/Bool) - 目标到达状态

**发布话题**：
- `/cmd_vel` (geometry_msgs/Twist) - 速度命令 ⭐**主要输出**
- `/goal_reached` (std_msgs/Bool) - 目标到达标志
- `/mpcc_pred_path` (nav_msgs/Path) - MPC预测轨迹（调试可视化）

---

#### 3. mpcc_pred_visualizer（可视化工具）

预测轨迹可视化，显示MPC优化的未来轨迹。

**订阅话题**：
- `/mpcc_pred_path` (nav_msgs/Path) - MPCC预测轨迹
- `/localization` (nav_msgs/Odometry) - 机器人当前位置

**发布话题**：
- `/mpcc_pred_marker` (visualization_msgs/MarkerArray) - RViz可视化标记

---

## 快速开始

### 完整系统启动（规划+控制）

```bash
roslaunch local_planner local_planner_mpcc_unified.launch
```

包含：路径规划器 + MPCC控制器 + 可视化

### 仅启动控制器

```bash
roslaunch local_planner mpccfollower_unified.launch
```

适用于：单独测试MPCC、使用外部路径规划器

**需要的话题输入**：
- ✅ `/localization` - 机器人位姿（必须）
- ✅ `/local_path` - 路径输入（必须）⭐

**输出**：
- `/cmd_vel` - 速度命令到机器人底盘

### 测试倒车功能

```bash
# 1. 启动控制器
roslaunch local_planner mpccfollower_unified.launch

# 2. 运行测试脚本
cd /path/to/gs_robot_world
python test_reverse_path.py
# 选择 2 测试倒车路径
```

---

## 📚 文档

- 📖 [**快速开始**](docs/QUICK_START.md) - 启动方式、基本测试
- 🎛️ [**调参指南**](docs/MPCC_TUNING.md) - 参数配置、故障排查
- 🧮 [**MPCC原理**](docs/MPCC_THEORY.md) - 算法原理、数学模型

---

## 配置文件

```
config/
├── mpccfollower.yaml          # 原版MPCC配置
└── mpccfollower_unified.yaml  # Unified版本配置 ⭐
```

### 关键参数说明

#### 倒车支持（必须配置）

```yaml
vs_min: -0.5                    # ⚠️ 必须负值才能倒车
vs_max: 1.0                     # 前进速度上限
chassis_type: holonomic         # ⚠️ 必须设置为全向底盘
use_path_direction_hint: true   # ⚠️ 启用路径方向推断
```

#### 速度限制

```yaml
autonomySpeed: 0.6    # 自主速度 [m/s]，总体速度参考
vx_max: 1.0           # X方向最大速度
vy_max: 1.0           # Y方向最大速度
w_max: 1.0            # 最大角速度 [rad/s]
```

#### 方向引导权重

```yaml
qVsDir: 5.0           # 方向引导强度（5-10推荐）
                      # 越大越严格遵循路径指定方向
qGearSwitch: 10.0     # 档位切换惩罚，避免频繁换挡
```

#### 跟踪精度

```yaml
qC: 20.0     # 垂直路径误差权重（Contour error）
qL: 5.0      # 沿路径误差权重（Lag error）
qYaw: 10.0   # 姿态误差权重
```

#### 控制平滑

```yaml
rVx: 1.0     # X速度惩罚（越大越平滑）
rVy: 1.0     # Y速度惩罚
rW: 1.0      # 角速度惩罚
```

**配置生效方式**：修改YAML后重启launch即可，无需重新编译。

---

## 功能特性

### ✅ MPCC Unified 核心改进

**1. 统一优化框架**
- vs可在`[-vs_max, vs_max]`全范围优化，不再硬编码档位限制
- 单一QP优化器处理所有运动模式（前进/倒车/旋转）

**2. 自动路径方向推断**
- 从路径点姿态和运动方向自动推断期望行进方向
- 使用向前看策略处理密集路径点
- 自动检测原地旋转段（累积距离 < 阈值 & 姿态变化大）

**3. 软约束方向引导**
- 通过代价函数`qVsDir`引导vs符号，非硬约束
- 允许优化器在必要时偏离引导方向
- 档位切换惩罚`qGearSwitch`避免频繁前进-倒车切换

**4. 原地旋转自适应权重**
- 检测到旋转段时自动调整权重：
  - ↑ Contour error权重（保持位置）
  - ↓ Lag error权重（允许s不变）
  - ↓ 控制惩罚（允许更大角速度）

### 对比原版MPCC

| 特性 | 原版MPCC | Unified MPCC |
|------|----------|--------------|
| **倒车** | ❌ 必须转身前进 | ✅ 直接倒车 |
| **原地旋转** | ⚠️ 位置漂移 | ✅ 原地不动 |
| **档位切换** | ⚠️ 硬编码规则 | ✅ 优化引导 |
| **vs范围** | 单侧约束 | 双向全范围 |
| **旋转处理** | 规则判断 | 权重自适应 |
| **方向选择** | 外部规则 | 代价引导 |

---

## 工作原理

### 路径跟踪流程

1. **路径接收**：订阅`/planning/globalPath`话题获取路径
2. **方向推断**：分析路径点姿态和运动方向，标记每个点为前进(1)/倒车(-1)/旋转(0)
3. **MPC优化**：求解QP优化问题，得到最优控制输入
4. **速度发布**：发布`cmd_vel`到底盘

### MPC优化问题

每个控制周期求解：

```
min  Σ [qC·e_c² + qL·e_l² + qYaw·e_yaw² + qVsDir·(vs·dir_ref)]
     + Σ [rVx·vx² + rVy·vy² + rW·w² + rVs·vs²]

s.t.  x_{k+1} = f(x_k, u_k)          # 运动学模型
      -vx_max ≤ vx ≤ vx_max          # 速度约束
      -vy_max ≤ vy ≤ vy_max
      -w_max ≤ w ≤ w_max
      vs_min ≤ vs ≤ vs_max            # 路径速度（可为负！）
```

**关键改进**：
- `vs_min < 0`允许倒车
- `qVsDir·(vs·dir_ref)`软约束引导方向
- 原地旋转时自动调整权重矩阵

---

## 编译与依赖

### 依赖项

- ROS Melodic/Noetic
- Eigen3
- HPIPM（高性能QP求解器）
- BLASFEO（线性代数库）

### 编译

```bash
cd /path/to/gs_robot_world
catkin build local_planner
source devel/setup.bash
```

---

## 故障排查

### 🔴 倒车不工作，总是转身前进

**检查清单**：
1. ✅ `vs_min: -0.5`（必须负值）
2. ✅ `chassis_type: holonomic`
3. ✅ `use_path_direction_hint: true`
4. ✅ 增加 `qVsDir: 10.0`（加强方向引导）

**验证方法**：
```bash
# 查看日志确认路径方向
rostopic echo /rosout | grep "Path updated"
# 应该看到: dirs: fwd=0 rev=50 rot=0  ✅
```

→ 详见 [调参指南](docs/MPCC_TUNING.md#倒车不工作)

### 🔴 速度太快/末端飞走

**症状**：机器人速度越来越快，末端距离目标越来越远

**解决方案**：
```yaml
# 降低速度上限
autonomySpeed: 0.5    # 从0.6→0.5
vs_max: 0.6           # 从1.0→0.6

# 降低末端对齐速度
align_pos_speed: 0.03  # 从0.1→0.03
align_yaw_speed: 0.15  # 从0.3→0.15

# 增加控制惩罚
rVx: 2.0              # 从1.0→2.0
rVy: 2.5              # 从1.0→2.5
```

### 🔴 超调/振荡

**症状**：路径跟踪左右摆动或超出路径

**解决方案**：
```yaml
# 增加控制平滑
rVx: 2.0
rVy: 2.5
rW: 2.0

# 增加速度平滑
cmd_smooth_tau: 0.8
cmd_max_accel_lin: 0.3
```

### 🔴 原地旋转时漂移

**症状**：原地旋转时位置偏移

**解决方案**：
```yaml
# 减小检测阈值（更早识别旋转）
in_place_rot_threshold: 0.08  # 从0.10→0.08

# 降低旋转时控制惩罚
rInPlaceRot: 0.1             # 允许更大角速度
```

### 🔴 档位频繁切换

**症状**：前进-倒车反复切换

**解决方案**：
```yaml
# 增加换挡惩罚
qGearSwitch: 15.0     # 从10.0→15.0

# 增强方向引导
qVsDir: 8.0           # 从5.0→8.0
```

---

## 更多信息

- 📂 [完整文档目录](docs/)
- 🐛 [GitHub Issues](https://github.com/...)
- 📧 技术支持：参考调参指南自助排查
