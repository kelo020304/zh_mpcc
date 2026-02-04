# MPCC Unified 快速开始

## 功能特性

- ✅ **前进/倒车统一控制** - 自动识别路径方向
- ✅ **原地旋转优化** - 不漂移
- ✅ **档位平滑切换** - 减少抖动
- ✅ **全向底盘支持** - Holonomic/Mecanum

---

## 快速启动

### 完整系统（规划器+控制器）

```bash
roslaunch local_planner local_planner_mpcc_unified.launch
```

### 仅控制器

```bash
roslaunch local_planner mpccfollower_unified.launch
```

---

## 测试倒车功能

### 1. 启动控制器
```bash
roslaunch local_planner mpccfollower_unified.launch
```

### 2. 测试脚本
```bash
cd /path/to/workspace
./test_reverse_path.py
# 输入 2 测试倒车
```

### 3. 验证
观察日志应显示：
```
Path updated: dirs: fwd=0 rev=50 rot=0  ✅
mpcc cmd: vs=-0.XX dir=reverse          ✅
```

---

## 配置检查

**必须设置**（否则无法倒车）：

```yaml
vs_min: -0.5                    # ⚠️ 必须负值
chassis_type: holonomic         # ⚠️ 全向底盘
use_path_direction_hint: true   # ⚠️ 启用方向推断
```

**推荐速度**（仿真测试）：

```yaml
autonomySpeed: 0.6
vs_max: 1.0
vs_min: -0.5
```

---

## 常见问题

### 倒车不工作？
→ 参考 `MPCC_TUNING.md` 的故障排查章节

### 速度太快/飞走？
→ 降低 `autonomySpeed` 和 `vs_max`

### 超调/振荡？
→ 增加控制惩罚 `rVx`, `rVy`, `rW`

---

详细调参指南见 [`MPCC_TUNING.md`](MPCC_TUNING.md)
