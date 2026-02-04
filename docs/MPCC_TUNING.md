# MPCC Unified 调参指南

## 快速诊断

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| 倒车不工作，转身前进 | vs_min≥0 | `vs_min: -0.5` |
| 速度太快/飞走 | 速度上限太高 | 降低 `autonomySpeed`, `vs_max` |
| 超调/振荡 | 控制惩罚太低 | 增加 `rVx`, `rVy`, `rW` |
| 频繁换挡抖动 | 换挡惩罚太低 | 增加 `qGearSwitch` |
| 原地旋转漂移 | 检测阈值太大 | 减小 `in_place_rot_threshold` |
| 响应慢/迟钝 | 控制惩罚太高 | 减小 `rVx`, `rVy`, `rW` |

---

## 关键参数

### 速度约束
```yaml
autonomySpeed: 0.6   # 自主速度 [m/s]
vs_max: 1.0          # 前进速度上限
vs_min: -0.5         # 倒车速度上限（必须负值！）
vx_max: 1.0          # X方向速度
vy_max: 1.0          # Y方向速度
w_max: 1.0           # 角速度
```

### 控制平滑
```yaml
rVx: 1.0    # X速度惩罚（越大越平滑）
rVy: 1.0    # Y速度惩罚
rW: 1.0     # 角速度惩罚
rVs: 1.0    # 路径速度惩罚
```

### 倒车控制
```yaml
qVsDir: 5.0         # 方向引导强度（5-10推荐）
qGearSwitch: 10.0   # 换挡平滑度
use_path_direction_hint: true  # 启用方向推断
```

### 跟踪精度
```yaml
qC: 20.0    # 垂直路径误差权重
qL: 5.0     # 沿路径误差权重
qYaw: 10.0  # 姿态误差权重
```

---

## 常见场景调参

### 场景1：仿真测试（保守）
```yaml
autonomySpeed: 0.5
vs_max: 0.6
vs_min: -0.4
rVx: 2.0
rVy: 2.5
cmd_smooth_tau: 0.8
```

### 场景2：真机运行（平衡）
```yaml
autonomySpeed: 0.8
vs_max: 1.0
vs_min: -0.6
rVx: 1.5
rVy: 2.0
cmd_smooth_tau: 0.6
```

### 场景3：性能测试（激进）
```yaml
autonomySpeed: 1.2
vs_max: 1.5
vs_min: -0.8
rVx: 1.0
rVy: 1.5
cmd_smooth_tau: 0.5
```

---

## 调参流程

1. **基础配置**
   - ✅ `vs_min < 0`（允许倒车）
   - ✅ `chassis_type: holonomic`
   - ✅ `use_path_direction_hint: true`

2. **速度限制**
   - 从保守开始（0.5 m/s）
   - 逐步增加

3. **稳定性调整**
   - 有超调 → 增加 `rVx`, `rVy`
   - 有振荡 → 增加 `cmd_smooth_tau`

4. **精度优化**
   - 偏离路径 → 增加 `qC`
   - 超前/滞后 → 增加 `qL`

---

## 故障排查

### Q: 倒车路径还是转身？
**A:**
1. 检查 `vs_min: -0.5`（必须负值）
2. 增加 `qVsDir: 10.0`
3. 检查路径姿态设置

### Q: 末端飞走？
**A:**
```yaml
align_pos_speed: 0.03   # 降低对齐速度
align_yaw_speed: 0.15
autonomySpeed: 0.5      # 降低整体速度
```

### Q: 速度越跑越快？
**A:**
```yaml
qVs: 0.1       # 降低速度奖励
rVs: 1.5       # 增加速度惩罚
```

---

## 配置文件位置

```
src/local_planner/config/mpccfollower_unified.yaml
```

修改后重启launch即可生效，无需重新编译。
