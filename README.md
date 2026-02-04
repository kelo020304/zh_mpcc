# Local Planner with MPCC Unified Controller

全向底盘路径规划与跟踪控制系统。

## 组件

- **localPlanner** - 路径规划器
- **mpccfollower_unified** - 统一MPCC控制器（支持前进/倒车/原地旋转）
- **mpcc_pred_visualizer** - 预测轨迹可视化

---

## 快速开始

```bash
# 完整系统启动
roslaunch local_planner local_planner_mpcc_unified.launch

# 仅控制器启动
roslaunch local_planner mpccfollower_unified.launch
```

---

## 文档

- 📖 [**快速开始**](docs/QUICK_START.md) - 基本使用
- 🎛️ [**调参指南**](docs/MPCC_TUNING.md) - 参数配置与故障排查

---

## 配置文件

```
config/
├── mpccfollower.yaml          # 原版MPCC配置
└── mpccfollower_unified.yaml  # Unified版本配置 ⭐
```

**关键参数**：

```yaml
# 倒车支持
vs_min: -0.5              # 必须负值才能倒车
vs_max: 1.0

# 速度控制
autonomySpeed: 0.6
vx_max: 1.0
vy_max: 1.0

# 方向引导
qVsDir: 5.0               # 方向引导强度
use_path_direction_hint: true
```

修改配置后重启launch即可，无需重新编译。

---

## 功能特性

### ✅ MPCC Unified

- **倒车支持** - 自动识别倒车路径
- **原地旋转** - 自适应权重，不漂移
- **档位切换** - 平滑换挡，减少抖动
- **方向引导** - 软约束优化，智能选择方向

### 对比原版

| 特性 | 原版MPCC | Unified MPCC |
|------|----------|--------------|
| 倒车 | ❌ 转身前进 | ✅ 直接倒车 |
| 原地旋转 | ⚠️ 漂移 | ✅ 原地不动 |
| 档位切换 | ⚠️ 硬编码 | ✅ 优化引导 |

---

## 编译

```bash
catkin build local_planner
```

---

## 故障排查

### 倒车不工作
→ 见 [调参指南](docs/MPCC_TUNING.md)

### 速度太快/飞走
→ 降低 `autonomySpeed` 和速度上限

### 超调/振荡
→ 增加控制惩罚 `rVx`, `rVy`, `rW`

---

更多信息见 [docs/](docs/) 目录。
