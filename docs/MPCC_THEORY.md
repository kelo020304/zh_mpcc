# MPCC (Model Predictive Contouring Control) 理论与实现

本文档详细解释 mpccfollower 的原理，包括运动学模型建立、MPCC 问题构建、MPC 矩阵构造以及 QP 求解器接口。

---

## 目录

1. [符号与变量定义](#1-符号与变量定义)
2. [MPCC 简介](#2-mpcc-简介)
3. [运动学模型建立](#3-运动学模型建立)
4. [MPCC 问题构建](#4-mpcc-问题构建)
5. [MPC 框架与矩阵构造](#5-mpc-框架与矩阵构造)
6. [批量形式与 G, E, H 矩阵详细推导](#6-批量形式与-g-e-h-矩阵详细推导)
7. [QP 转换与求解](#7-qp-转换与求解)
8. [预测时域设置](#8-预测时域设置)
9. [代码实现细节](#9-代码实现细节)
10. [参数调优指南](#10-参数调优指南)

---

## 1. 符号与变量定义

### 1.1 维度参数

| 符号 | 含义 | 值 | 说明 |
|------|------|-----|------|
| `NX` | 状态向量维度 | 4 | $\mathbf{x} \in \mathbb{R}^{4}$ |
| `NU` | 控制输入维度 | 4 | $\mathbf{u} \in \mathbb{R}^{4}$ |
| `N` | 预测时域步数 | 15 | MPC 预测的未来步数 |
| `dt` | 采样时间 | 0.0333 s | $1 / \text{mpc\_hz}$ |

### 1.2 状态向量

$$
\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \psi_k \\ s_k \end{bmatrix} \in \mathbb{R}^{4}
$$

| 分量 | 符号 | 单位 | 说明 |
|------|------|------|------|
| 位置 X | $x_k$ | m | 世界坐标系下的 X 坐标 |
| 位置 Y | $y_k$ | m | 世界坐标系下的 Y 坐标 |
| 航向角 | $\psi_k$ | rad | 机器人朝向（yaw 角） |
| 路径参数 | $s_k$ | m | 沿路径的弧长参数 |

### 1.3 控制输入向量

$$
\mathbf{u}_k = \begin{bmatrix} v_x \\ v_y \\ \omega \\ v_s \end{bmatrix} \in \mathbb{R}^{4}
$$

| 分量 | 符号 | 单位 | 说明 |
|------|------|------|------|
| 纵向速度 | $v_x$ | m/s | 机体坐标系 X 方向速度（前后） |
| 横向速度 | $v_y$ | m/s | 机体坐标系 Y 方向速度（左右） |
| 角速度 | $\omega$ | rad/s | 绕 Z 轴的旋转速度 |
| 路径速度 | $v_s$ | m/s | 路径参数变化率 $\frac{ds}{dt}$ |

### 1.4 参考轨迹

给定路径参数 $s$，参考轨迹定义为：

$$
\mathbf{x}_{\text{ref}}(s) = \begin{bmatrix} x_{\text{ref}}(s) \\ y_{\text{ref}}(s) \\ \psi_{\text{ref}}(s) \\ s \end{bmatrix}
$$

### 1.5 代价函数权重

| 符号 | 含义 | 默认值 | 单位 |
|------|------|--------|------|
| $w_C$ | 轮廓误差权重 | 20.0 | 无量纲 |
| $w_L$ | 滞后误差权重 | 5.0 | 无量纲 |
| $w_\psi$ | 航向误差权重 | 6.0 | 无量纲 |
| $w_C^f$ | 终端轮廓误差权重 | 40.0 | 无量纲 |
| $w_L^f$ | 终端滞后误差权重 | 10.0 | 无量纲 |
| $w_\psi^f$ | 终端航向误差权重 | 12.0 | 无量纲 |
| $r_{v_x}$ | $v_x$ 控制惩罚 | 1.0 | 无量纲 |
| $r_{v_y}$ | $v_y$ 控制惩罚 | 1.0 | 无量纲 |
| $r_\omega$ | $\omega$ 控制惩罚 | 1.0 | 无量纲 |
| $r_{v_s}$ | $v_s$ 控制惩罚 | 0.5 | 无量纲 |
| $q_{v_s}$ | $v_s$ 奖励权重 | 0.05 | 无量纲 |

---

## 2. MPCC 简介

### 2.1 什么是 MPCC？

**Model Predictive Contouring Control (模型预测轮廓控制)** 是一种先进的路径跟踪算法，特别适用于需要高精度轨迹跟踪的应用场景。

### 2.2 MPCC vs 传统路径跟踪

| 方法 | 误差定义 | 优点 | 缺点 |
|------|---------|------|------|
| **Pure Pursuit** | 到前视点的距离 | 简单，计算量小 | 转弯处切角，精度低 |
| **Stanley Controller** | 横向误差 + 航向误差 | 中等精度 | 参数调优困难 |
| **MPCC** | 轮廓误差 $(e_c)$ + 滞后误差 $(e_l)$ | 高精度，可约束优化 | 计算量大 |

### 2.3 核心思想

MPCC 将路径跟踪问题转化为两个正交误差的最小化：

- **轮廓误差 (Contour Error, $e_c$)**：垂直于路径切线方向的误差（横向偏离）
- **滞后误差 (Lag Error, $e_l$)**：沿着路径切线方向的误差（纵向偏离）

同时优化路径参数 $s$ 的进度，允许机器人在保证跟踪精度的前提下动态调整速度。

---

## 3. 运动学模型建立

### 3.1 全向底盘运动学

我们的机器人是**全向移动底盘**，具有 3 自由度：

- $v_x$：机体坐标系 X 方向速度（前后）
- $v_y$：机体坐标系 Y 方向速度（左右）
- $\omega$：角速度（旋转）

### 3.2 连续时间运动学模型

世界坐标系下的运动学方程：

$$
\begin{aligned}
\dot{x} &= v_x \cos(\psi) - v_y \sin(\psi) \\
\dot{y} &= v_x \sin(\psi) + v_y \cos(\psi) \\
\dot{\psi} &= \omega \\
\dot{s} &= v_s
\end{aligned}
$$

写成状态空间形式：

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})
$$

### 3.3 离散时间运动学模型

采用**一阶欧拉离散化**（简化的积分器模型）：

$$
\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k + \mathbf{b}_k
$$

在我们的实现中：

$$
\mathbf{A}_k = \mathbf{I}_{4 \times 4} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{B}_k = \Delta t \cdot \mathbf{I}_{4 \times 4} = \begin{bmatrix}
\Delta t & 0 & 0 & 0 \\
0 & \Delta t & 0 & 0 \\
0 & 0 & \Delta t & 0 \\
0 & 0 & 0 & \Delta t
\end{bmatrix}, \quad
\mathbf{b}_k = \mathbf{0}
$$

**代码实现**（mpccfollower.cpp:274-282）：
```cpp
// A = I (identity matrix)
for (int i = 0; i < NX; i++)
  A[k][i + i * NX] = 1.0;

// B = dt * I
for (int i = 0; i < NX; i++)
  B[k][i + i * NX] = dt;
```

**物理意义**：

$$
\begin{bmatrix} x_{k+1} \\ y_{k+1} \\ \psi_{k+1} \\ s_{k+1} \end{bmatrix} =
\begin{bmatrix} x_k \\ y_k \\ \psi_k \\ s_k \end{bmatrix} + \Delta t \begin{bmatrix} v_x \\ v_y \\ \omega \\ v_s \end{bmatrix}
$$

注意：这是简化模型，实际中 $x, y$ 的更新需要考虑旋转，但在小时间步长下近似成立。

---

## 4. MPCC 问题构建

### 4.1 轮廓误差与滞后误差

给定参考路径点 $(x_{\text{ref}}, y_{\text{ref}}, \psi_{\text{ref}})$，定义：

#### 4.1.1 轮廓误差 (Contour Error)

垂直于路径切线的偏差：

$$
e_c = -\sin(\psi_{\text{ref}}) \cdot (x - x_{\text{ref}}) + \cos(\psi_{\text{ref}}) \cdot (y - y_{\text{ref}})
$$

写成线性形式：

$$
e_c = a_{c,x} \cdot x + a_{c,y} \cdot y + b_c
$$

其中：
- $a_{c,x} = -\sin(\psi_{\text{ref}})$
- $a_{c,y} = \cos(\psi_{\text{ref}})$
- $b_c = \sin(\psi_{\text{ref}}) \cdot x_{\text{ref}} - \cos(\psi_{\text{ref}}) \cdot y_{\text{ref}}$

**几何解释**：将位置误差 $(x - x_{\text{ref}}, y - y_{\text{ref}})$ 投影到路径法向量 $\mathbf{n} = [-\sin(\psi_{\text{ref}}), \cos(\psi_{\text{ref}})]^T$ 上。

#### 4.1.2 滞后误差 (Lag Error)

沿着路径切线的偏差：

$$
e_l = \cos(\psi_{\text{ref}}) \cdot (x - x_{\text{ref}}) + \sin(\psi_{\text{ref}}) \cdot (y - y_{\text{ref}})
$$

写成线性形式：

$$
e_l = a_{l,x} \cdot x + a_{l,y} \cdot y + b_l
$$

其中：
- $a_{l,x} = \cos(\psi_{\text{ref}})$
- $a_{l,y} = \sin(\psi_{\text{ref}})$
- $b_l = -\cos(\psi_{\text{ref}}) \cdot x_{\text{ref}} - \sin(\psi_{\text{ref}}) \cdot y_{\text{ref}}$

**几何解释**：将位置误差投影到路径切向量 $\mathbf{t} = [\cos(\psi_{\text{ref}}), \sin(\psi_{\text{ref}})]^T$ 上。

**代码实现**（mpccfollower.cpp:305-311）：
```cpp
// Contour error coefficients: e_c = a_cx * x + a_cy * y + b_c
const double a_cx = -sin(yaw_ref);
const double a_cy = cos(yaw_ref);
const double b_c = sin(yaw_ref) * x_ref - cos(yaw_ref) * y_ref;

// Lag error coefficients: e_l = a_lx * x + a_ly * y + b_l
const double a_lx = cos(yaw_ref);
const double a_ly = sin(yaw_ref);
const double b_l = -cos(yaw_ref) * x_ref - sin(yaw_ref) * y_ref;
```

### 4.2 航向误差

除了位置误差，还需要最小化航向偏差：

$$
e_\psi = \psi - \psi_{\text{ref}}
$$

**注意**：代码中使用了 `wrapAngle()` 函数确保 $\psi_{\text{ref}} \in [-\pi, \pi]$，避免角度累积问题。

### 4.3 代价函数设计

MPCC 的目标是最小化以下代价函数：

$$
J = \sum_{k=0}^{N-1} L_k(\mathbf{x}_k, \mathbf{u}_k) + L_N(\mathbf{x}_N)
$$

#### 4.3.1 中间步代价 $(k = 0, \ldots, N-1)$

$$
\begin{aligned}
L_k(\mathbf{x}_k, \mathbf{u}_k) &= w_C \cdot e_c^2 + w_L \cdot e_l^2 + w_\psi \cdot e_\psi^2 \\
&\quad + r_{v_x} \cdot v_x^2 + r_{v_y} \cdot v_y^2 + r_\omega \cdot \omega^2 + r_{v_s} \cdot v_s^2 \\
&\quad - q_{v_s} \cdot v_s
\end{aligned}
$$

#### 4.3.2 终端步代价 $(k = N)$

$$
L_N(\mathbf{x}_N) = w_C^f \cdot e_c^2 + w_L^f \cdot e_l^2 + w_\psi^f \cdot e_\psi^2
$$

其中终端权重通常设为中间权重的 2 倍：$w_C^f = 2 w_C$, $w_L^f = 2 w_L$, $w_\psi^f = 2 w_\psi$。

### 4.4 约束条件

#### 4.4.1 速度约束

$$
\begin{aligned}
-v_x^{\max} &\leq v_x \leq v_x^{\max} \\
-v_y^{\max} &\leq v_y \leq v_y^{\max} \\
-\omega^{\max} &\leq \omega \leq \omega^{\max} \\
v_s^{\min} &\leq v_s \leq v_s^{\max}
\end{aligned}
$$

其中 $v_s^{\min} < 0$ 允许倒车。

#### 4.4.2 路径参数信任范围

$$
s_k - s_{\text{trust}} \leq s \leq s_k + s_{\text{trust}}
$$

防止路径参数 $s$ 在单步内变化过大。

---

## 5. MPC 框架与矩阵构造

### 5.1 标准 MPC 优化问题

MPC 在每个控制周期求解以下有限时域优化问题：

$$
\begin{aligned}
\min_{\mathbf{x}_{0:N}, \mathbf{u}_{0:N-1}} \quad & \sum_{k=0}^{N-1} \left[ \frac{1}{2} \mathbf{x}_k^T \mathbf{Q}_k \mathbf{x}_k + \mathbf{q}_k^T \mathbf{x}_k + \frac{1}{2} \mathbf{u}_k^T \mathbf{R}_k \mathbf{u}_k + \mathbf{r}_k^T \mathbf{u}_k \right] \\
& + \frac{1}{2} \mathbf{x}_N^T \mathbf{Q}_N \mathbf{x}_N + \mathbf{q}_N^T \mathbf{x}_N \\
\text{s.t.} \quad & \mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k + \mathbf{b}_k, \quad k = 0, \ldots, N-1 \\
& \mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max}, \quad k = 0, \ldots, N \\
& \mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max}, \quad k = 0, \ldots, N-1 \\
& \mathbf{x}_0 = \mathbf{x}_{\text{current}}
\end{aligned}
$$

### 5.2 $\mathbf{Q}_k$ 矩阵构造（状态代价矩阵）

$\mathbf{Q}_k$ 是 **$4 \times 4$ 对称半正定矩阵**，编码了状态误差的代价。

#### 5.2.1 从误差到二次型

将轮廓误差和滞后误差代入代价函数：

$$
\begin{aligned}
J_{\text{state}} &= w_C \cdot e_c^2 + w_L \cdot e_l^2 + w_\psi \cdot e_\psi^2 \\
&= w_C \cdot (a_{c,x} x + a_{c,y} y + b_c)^2 \\
&\quad + w_L \cdot (a_{l,x} x + a_{l,y} y + b_l)^2 \\
&\quad + w_\psi \cdot (\psi - \psi_{\text{ref}})^2
\end{aligned}
$$

展开二次项（忽略常数项）：

$$
\begin{aligned}
e_c^2 &= (a_{c,x} x + a_{c,y} y)^2 + 2(a_{c,x} x + a_{c,y} y) b_c + b_c^2 \\
&= a_{c,x}^2 x^2 + 2 a_{c,x} a_{c,y} x y + a_{c,y}^2 y^2 + 2 a_{c,x} b_c x + 2 a_{c,y} b_c y + b_c^2
\end{aligned}
$$

类似地展开 $e_l^2$，合并得到：

$$
J_{\text{state}} = \frac{1}{2} \mathbf{x}_k^T \mathbf{Q}_k \mathbf{x}_k + \mathbf{q}_k^T \mathbf{x}_k + \text{const}
$$

#### 5.2.2 $\mathbf{Q}_k$ 矩阵元素

$$
\mathbf{Q}_k = \begin{bmatrix}
q_{xx} & q_{xy} & 0 & 0 \\
q_{xy} & q_{yy} & 0 & 0 \\
0 & 0 & q_{\psi\psi} & 0 \\
0 & 0 & 0 & 0
\end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

其中：

$$
\begin{aligned}
q_{xx} &= 2 \left( w_C \cdot a_{c,x}^2 + w_L \cdot a_{l,x}^2 \right) \\
&= 2 \left( w_C \cdot \sin^2(\psi_{\text{ref}}) + w_L \cdot \cos^2(\psi_{\text{ref}}) \right) \\[0.5em]
q_{xy} &= 2 \left( w_C \cdot a_{c,x} a_{c,y} + w_L \cdot a_{l,x} a_{l,y} \right) \\
&= 2 \left( -w_C \cdot \sin(\psi_{\text{ref}}) \cos(\psi_{\text{ref}}) + w_L \cdot \cos(\psi_{\text{ref}}) \sin(\psi_{\text{ref}}) \right) \\
&= 2 (w_L - w_C) \sin(\psi_{\text{ref}}) \cos(\psi_{\text{ref}}) \\[0.5em]
q_{yy} &= 2 \left( w_C \cdot a_{c,y}^2 + w_L \cdot a_{l,y}^2 \right) \\
&= 2 \left( w_C \cdot \cos^2(\psi_{\text{ref}}) + w_L \cdot \sin^2(\psi_{\text{ref}}) \right) \\[0.5em]
q_{\psi\psi} &= 2 w_\psi
\end{aligned}
$$

**代码实现**（mpccfollower.cpp:318-332）：
```cpp
// Q 矩阵对角元素和交叉项
const double qxx = 2.0 * (wC * a_cx * a_cx + wL * a_lx * a_lx);
const double qxy = 2.0 * (wC * a_cx * a_cy + wL * a_lx * a_ly);
const double qyy = 2.0 * (wC * a_cy * a_cy + wL * a_ly * a_ly);

Q[k][0 + 0 * NX] += qxx;        // Q(0,0): x 对 x
Q[k][0 + 1 * NX] += qxy;        // Q(0,1): x 对 y
Q[k][1 + 0 * NX] += qxy;        // Q(1,0): y 对 x (对称)
Q[k][1 + 1 * NX] += qyy;        // Q(1,1): y 对 y
Q[k][2 + 2 * NX] += 2.0 * wYaw; // Q(2,2): yaw 对 yaw
Q[k][3 + 3 * NX] = 0.0;         // Q(3,3): s 无二次惩罚
```

#### 5.2.3 $\mathbf{q}_k$ 向量元素（线性项）

$$
\mathbf{q}_k = \begin{bmatrix}
2(w_C a_{c,x} b_c + w_L a_{l,x} b_l) \\
2(w_C a_{c,y} b_c + w_L a_{l,y} b_l) \\
-2 w_\psi \psi_{\text{ref}} \\
0
\end{bmatrix} \in \mathbb{R}^{4}
$$

**代码实现**：
```cpp
q[k][0] += 2.0 * (wC * a_cx * b_c + wL * a_lx * b_l);  // ∂J/∂x
q[k][1] += 2.0 * (wC * a_cy * b_c + wL * a_ly * b_l);  // ∂J/∂y
q[k][2] += -2.0 * wYaw * yaw_ref;                      // ∂J/∂yaw
q[k][3] = 0.0;                                         // ∂J/∂s
```

### 5.3 $\mathbf{F}$ 矩阵（终端代价矩阵 $\mathbf{Q}_N$）

终端代价矩阵 $\mathbf{F} = \mathbf{Q}_N$ 与中间步矩阵结构相同，但权重加倍：

$$
\mathbf{F} = \mathbf{Q}_N = \begin{bmatrix}
f_{xx} & f_{xy} & 0 & 0 \\
f_{xy} & f_{yy} & 0 & 0 \\
0 & 0 & f_{\psi\psi} & 0 \\
0 & 0 & 0 & 0
\end{bmatrix}
$$

其中：

$$
\begin{aligned}
f_{xx} &= 2 \left( w_C^f \cdot \sin^2(\psi_{\text{ref}}) + w_L^f \cdot \cos^2(\psi_{\text{ref}}) \right) \\
f_{xy} &= 2 (w_L^f - w_C^f) \sin(\psi_{\text{ref}}) \cos(\psi_{\text{ref}}) \\
f_{yy} &= 2 \left( w_C^f \cdot \cos^2(\psi_{\text{ref}}) + w_L^f \cdot \sin^2(\psi_{\text{ref}}) \right) \\
f_{\psi\psi} &= 2 w_\psi^f
\end{aligned}
$$

**配置文件中的默认值**：
- $w_C^f = 40.0 = 2 \times 20.0$
- $w_L^f = 10.0 = 2 \times 5.0$
- $w_\psi^f = 12.0 = 2 \times 6.0$

**代码中的构造方式与 $\mathbf{Q}_k$ 完全相同，仅权重参数不同。**

### 5.4 $\mathbf{R}_k$ 矩阵构造（控制代价矩阵）

$\mathbf{R}_k$ 是 **$4 \times 4$ 对角矩阵**，惩罚控制输入的大小：

$$
\mathbf{R}_k = \begin{bmatrix}
2 r_{v_x} & 0 & 0 & 0 \\
0 & 2 r_{v_y} & 0 & 0 \\
0 & 0 & 2 r_\omega & 0 \\
0 & 0 & 0 & 2 r_{v_s}
\end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

**代码实现**：
```cpp
R[k][0 + 0 * NU] = 2.0 * rVx;  // vx 惩罚
R[k][1 + 1 * NU] = 2.0 * rVy;  // vy 惩罚
R[k][2 + 2 * NU] = 2.0 * rW;   // w 惩罚
R[k][3 + 3 * NU] = 2.0 * rVs;  // vs 惩罚
```

### 5.5 $\mathbf{r}_k$ 向量（控制线性项）

$$
\mathbf{r}_k = \begin{bmatrix}
0 \\
0 \\
0 \\
-q_{v_s}
\end{bmatrix} \in \mathbb{R}^{4}
$$

**物理意义**：$-q_{v_s} \cdot v_s$ 项鼓励正向路径速度（最大化 $v_s$）。

**代码实现**：
```cpp
r[k][3] = -qVs;  // 鼓励正向路径速度
```

---

## 6. 批量形式与 G, E, H 矩阵详细推导

### 6.1 批量形式（Batch Formulation）概述

批量形式将整个预测时域的优化变量堆叠成大向量，动力学约束转化为等式约束。

#### 6.1.1 决策变量堆叠

定义堆叠向量：

$$
\begin{aligned}
\mathbf{X} &= \begin{bmatrix} \mathbf{x}_1 \\ \mathbf{x}_2 \\ \vdots \\ \mathbf{x}_N \end{bmatrix} \in \mathbb{R}^{N \cdot NX} \\[1em]
\mathbf{U} &= \begin{bmatrix} \mathbf{u}_0 \\ \mathbf{u}_1 \\ \vdots \\ \mathbf{u}_{N-1} \end{bmatrix} \in \mathbb{R}^{N \cdot NU}
\end{aligned}
$$

其中 $N = 15$ (预测步数), $NX = 4$ (状态维度), $NU = 4$ (控制维度)，因此：
- $\mathbf{X} \in \mathbb{R}^{60}$
- $\mathbf{U} \in \mathbb{R}^{60}$

### 6.2 动力学约束的批量表示

#### 6.2.1 逐步展开

从离散动力学方程 $\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k$ 开始：

$$
\begin{aligned}
\mathbf{x}_1 &= \mathbf{A}_0 \mathbf{x}_0 + \mathbf{B}_0 \mathbf{u}_0 \\
\mathbf{x}_2 &= \mathbf{A}_1 \mathbf{x}_1 + \mathbf{B}_1 \mathbf{u}_1 \\
&= \mathbf{A}_1 (\mathbf{A}_0 \mathbf{x}_0 + \mathbf{B}_0 \mathbf{u}_0) + \mathbf{B}_1 \mathbf{u}_1 \\
&= \mathbf{A}_1 \mathbf{A}_0 \mathbf{x}_0 + \mathbf{A}_1 \mathbf{B}_0 \mathbf{u}_0 + \mathbf{B}_1 \mathbf{u}_1 \\[0.5em]
\mathbf{x}_3 &= \mathbf{A}_2 \mathbf{x}_2 + \mathbf{B}_2 \mathbf{u}_2 \\
&= \mathbf{A}_2 \mathbf{A}_1 \mathbf{A}_0 \mathbf{x}_0 + \mathbf{A}_2 \mathbf{A}_1 \mathbf{B}_0 \mathbf{u}_0 + \mathbf{A}_2 \mathbf{B}_1 \mathbf{u}_1 + \mathbf{B}_2 \mathbf{u}_2 \\
&\vdots \\
\mathbf{x}_N &= \left( \prod_{i=0}^{N-1} \mathbf{A}_i \right) \mathbf{x}_0 + \sum_{j=0}^{N-1} \left( \prod_{i=j+1}^{N-1} \mathbf{A}_i \right) \mathbf{B}_j \mathbf{u}_j
\end{aligned}
$$

#### 6.2.2 矩阵形式

写成矩阵形式：

$$
\mathbf{X} = \mathbf{G} \mathbf{x}_0 + \mathbf{E} \mathbf{U} + \mathbf{H}
$$

### 6.3 $\mathbf{G}$ 矩阵（初始状态影响矩阵）

$\mathbf{G} \in \mathbb{R}^{(N \cdot NX) \times NX}$ 描述初始状态 $\mathbf{x}_0$ 对未来状态的影响。

#### 6.3.1 定义

$$
\mathbf{G} = \begin{bmatrix}
\mathbf{A}_0 \\
\mathbf{A}_1 \mathbf{A}_0 \\
\mathbf{A}_2 \mathbf{A}_1 \mathbf{A}_0 \\
\vdots \\
\mathbf{A}_{N-1} \cdots \mathbf{A}_1 \mathbf{A}_0
\end{bmatrix} \in \mathbb{R}^{60 \times 4}
$$

#### 6.3.2 详细展开（以 $N=3$ 为例）

对于简化示例 $N=3$, $NX=4$：

$$
\mathbf{G} = \begin{bmatrix}
\mathbf{A}_0 \\[0.5em]
\mathbf{A}_1 \mathbf{A}_0 \\[0.5em]
\mathbf{A}_2 \mathbf{A}_1 \mathbf{A}_0
\end{bmatrix} = \begin{bmatrix}
a_{0,11} & a_{0,12} & a_{0,13} & a_{0,14} \\
a_{0,21} & a_{0,22} & a_{0,23} & a_{0,24} \\
a_{0,31} & a_{0,32} & a_{0,33} & a_{0,34} \\
a_{0,41} & a_{0,42} & a_{0,43} & a_{0,44} \\[0.5em]
\multicolumn{4}{c}{\text{(第二行块：} \mathbf{A}_1 \mathbf{A}_0 \text{)}} \\[0.5em]
\multicolumn{4}{c}{\text{(第三行块：} \mathbf{A}_2 \mathbf{A}_1 \mathbf{A}_0 \text{)}}
\end{bmatrix} \in \mathbb{R}^{12 \times 4}
$$

**在我们的实现中**，由于 $\mathbf{A}_k = \mathbf{I}$，有：

$$
\mathbf{G} = \begin{bmatrix}
\mathbf{I} \\
\mathbf{I} \\
\mathbf{I} \\
\vdots \\
\mathbf{I}
\end{bmatrix} \in \mathbb{R}^{60 \times 4}
$$

即每个 $4 \times 4$ 块都是单位矩阵。

### 6.4 $\mathbf{E}$ 矩阵（控制输入影响矩阵）

$\mathbf{E} \in \mathbb{R}^{(N \cdot NX) \times (N \cdot NU)}$ 描述控制输入 $\mathbf{U}$ 对未来状态的影响。

#### 6.4.1 定义

$$
\mathbf{E} = \begin{bmatrix}
\mathbf{B}_0 & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}_1 \mathbf{B}_0 & \mathbf{B}_1 & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}_2 \mathbf{A}_1 \mathbf{B}_0 & \mathbf{A}_2 \mathbf{B}_1 & \mathbf{B}_2 & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\prod_{i=1}^{N-1} \mathbf{A}_i \mathbf{B}_0 & \prod_{i=2}^{N-1} \mathbf{A}_i \mathbf{B}_1 & \cdots & \mathbf{A}_{N-1} \mathbf{B}_{N-2} & \mathbf{B}_{N-1}
\end{bmatrix}
$$

这是一个 **下三角块矩阵**，大小为 $60 \times 60$。

#### 6.4.2 详细展开（以 $N=3$ 为例）

对于简化示例 $N=3$, $NX=NU=4$：

$$
\mathbf{E} = \begin{bmatrix}
\mathbf{B}_0 & \mathbf{0} & \mathbf{0} \\[0.5em]
\mathbf{A}_1 \mathbf{B}_0 & \mathbf{B}_1 & \mathbf{0} \\[0.5em]
\mathbf{A}_2 \mathbf{A}_1 \mathbf{B}_0 & \mathbf{A}_2 \mathbf{B}_1 & \mathbf{B}_2
\end{bmatrix} \in \mathbb{R}^{12 \times 12}
$$

展开每个 $4 \times 4$ 块：

$$
\mathbf{E} = \begin{bmatrix}
b_{0,11} & b_{0,12} & b_{0,13} & b_{0,14} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
b_{0,21} & b_{0,22} & b_{0,23} & b_{0,24} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
b_{0,31} & b_{0,32} & b_{0,33} & b_{0,34} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
b_{0,41} & b_{0,42} & b_{0,43} & b_{0,44} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\[0.5em]
\text{(} \mathbf{A}_1 \mathbf{B}_0 \text{)} & \text{(} \mathbf{B}_1 \text{)} & \multicolumn{4}{c}{\mathbf{0}} \\[0.5em]
\text{(} \mathbf{A}_2 \mathbf{A}_1 \mathbf{B}_0 \text{)} & \text{(} \mathbf{A}_2 \mathbf{B}_1 \text{)} & \text{(} \mathbf{B}_2 \text{)}
\end{bmatrix}
$$

**在我们的实现中**，由于 $\mathbf{A}_k = \mathbf{I}$, $\mathbf{B}_k = \Delta t \mathbf{I}$：

$$
\mathbf{E} = \Delta t \begin{bmatrix}
\mathbf{I} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{I} & \mathbf{I} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{I} & \mathbf{I} & \mathbf{I} & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{I} & \mathbf{I} & \mathbf{I} & \cdots & \mathbf{I}
\end{bmatrix} \in \mathbb{R}^{60 \times 60}
$$

这是一个 **下三角全 1 块矩阵** 乘以 $\Delta t$。

**物理意义**：
- 第 $k$ 个状态 $\mathbf{x}_k$ 受到前 $k$ 个控制输入 $\mathbf{u}_0, \ldots, \mathbf{u}_{k-1}$ 的累积影响
- 因果性：未来的控制不影响过去的状态（上三角为零）

#### 6.4.3 完整 $\mathbf{E}$ 矩阵（$N=15$，实际尺寸）

实际实现中 $N=15$, $NX=NU=4$，因此 $\mathbf{E} \in \mathbb{R}^{60 \times 60}$：

$$
\mathbf{E} = 0.0333 \begin{bmatrix}
\mathbf{I}_4 & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} & \mathbf{0} \\
\mathbf{I}_4 & \mathbf{I}_4 & \mathbf{0} & \cdots & \mathbf{0} & \mathbf{0} \\
\mathbf{I}_4 & \mathbf{I}_4 & \mathbf{I}_4 & \cdots & \mathbf{0} & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
\mathbf{I}_4 & \mathbf{I}_4 & \mathbf{I}_4 & \cdots & \mathbf{I}_4 & \mathbf{I}_4
\end{bmatrix}_{15 \times 15 \text{ blocks}}
$$

### 6.5 $\mathbf{H}$ 矩阵（仿射项）

$\mathbf{H} \in \mathbb{R}^{N \cdot NX}$ 描述系统的仿射项（来自 $\mathbf{b}_k$）。

#### 6.5.1 定义

$$
\mathbf{H} = \begin{bmatrix}
\mathbf{b}_0 \\
\mathbf{A}_1 \mathbf{b}_0 + \mathbf{b}_1 \\
\mathbf{A}_2 (\mathbf{A}_1 \mathbf{b}_0 + \mathbf{b}_1) + \mathbf{b}_2 \\
\vdots \\
\sum_{j=0}^{N-1} \left( \prod_{i=j+1}^{N-1} \mathbf{A}_i \right) \mathbf{b}_j
\end{bmatrix} \in \mathbb{R}^{60}
$$

**在我们的实现中**，由于 $\mathbf{b}_k = \mathbf{0}$（无外部扰动）：

$$
\mathbf{H} = \mathbf{0} \in \mathbb{R}^{60}
$$

### 6.6 批量 QP 形式

将代价函数改写为批量形式：

$$
\begin{aligned}
J(\mathbf{X}, \mathbf{U}) &= \frac{1}{2} \begin{bmatrix} \mathbf{X} \\ \mathbf{U} \end{bmatrix}^T \begin{bmatrix} \bar{\mathbf{Q}} & \mathbf{0} \\ \mathbf{0} & \bar{\mathbf{R}} \end{bmatrix} \begin{bmatrix} \mathbf{X} \\ \mathbf{U} \end{bmatrix} + \begin{bmatrix} \bar{\mathbf{q}} \\ \bar{\mathbf{r}} \end{bmatrix}^T \begin{bmatrix} \mathbf{X} \\ \mathbf{U} \end{bmatrix}
\end{aligned}
$$

其中：

$$
\bar{\mathbf{Q}} = \text{diag}(\mathbf{Q}_0, \mathbf{Q}_1, \ldots, \mathbf{Q}_{N-1}, \mathbf{Q}_N) \in \mathbb{R}^{64 \times 64}
$$

$$
\bar{\mathbf{R}} = \text{diag}(\mathbf{R}_0, \mathbf{R}_1, \ldots, \mathbf{R}_{N-1}) \in \mathbb{R}^{60 \times 60}
$$

**替换约束**：

将 $\mathbf{X} = \mathbf{G} \mathbf{x}_0 + \mathbf{E} \mathbf{U}$ 代入代价函数，得到仅关于 $\mathbf{U}$ 的 QP 问题：

$$
\begin{aligned}
\min_{\mathbf{U}} \quad & \frac{1}{2} \mathbf{U}^T \mathbf{P} \mathbf{U} + \mathbf{c}^T \mathbf{U} \\
\text{s.t.} \quad & \mathbf{U}_{\min} \leq \mathbf{U} \leq \mathbf{U}_{\max}
\end{aligned}
$$

其中：

$$
\begin{aligned}
\mathbf{P} &= \mathbf{E}^T \bar{\mathbf{Q}} \mathbf{E} + \bar{\mathbf{R}} \in \mathbb{R}^{60 \times 60} \\
\mathbf{c} &= \mathbf{E}^T \bar{\mathbf{Q}} \mathbf{G} \mathbf{x}_0 + \mathbf{E}^T \bar{\mathbf{q}} + \bar{\mathbf{r}} \in \mathbb{R}^{60}
\end{aligned}
$$

### 6.7 为何不使用批量形式？

**缺点**：
1. **存储需求大**：$\mathbf{P} \in \mathbb{R}^{60 \times 60}$ 需要存储 3600 个元素
2. **计算量大**：矩阵乘法 $\mathbf{E}^T \bar{\mathbf{Q}} \mathbf{E}$ 的复杂度为 $O(N^3 \cdot NX^3)$
3. **失去稀疏结构**：尽管 $\mathbf{E}$ 是下三角，$\mathbf{P}$ 是稠密矩阵

**HPIPM 的优势**：
- 直接利用 OCP 的块对角稀疏结构
- 使用 Riccati 递推求解，复杂度为 $O(N)$
- 内存占用 $O(N)$ 而非 $O(N^2)$

---

## 7. QP 转换与求解

### 7.1 QP 标准形式

二次规划（Quadratic Programming）问题的标准形式：

$$
\begin{aligned}
\min_{\mathbf{z}} \quad & \frac{1}{2} \mathbf{z}^T \mathbf{P} \mathbf{z} + \mathbf{q}^T \mathbf{z} \\
\text{s.t.} \quad & \mathbf{G} \mathbf{z} \leq \mathbf{h} \\
& \mathbf{A} \mathbf{z} = \mathbf{b}
\end{aligned}
$$

### 7.2 OCP QP 结构化形式

HPIPM 使用的是 **OCP (Optimal Control Problem) QP** 形式，利用了时间维度的稀疏块对角结构：

$$
\begin{aligned}
\min_{\mathbf{x}_{0:N}, \mathbf{u}_{0:N-1}} \quad & \sum_{k=0}^{N-1} \left[ \frac{1}{2} \mathbf{x}_k^T \mathbf{Q}_k \mathbf{x}_k + \mathbf{q}_k^T \mathbf{x}_k + \frac{1}{2} \mathbf{u}_k^T \mathbf{R}_k \mathbf{u}_k + \mathbf{r}_k^T \mathbf{u}_k + \mathbf{u}_k^T \mathbf{S}_k^T \mathbf{x}_k \right] \\
& + \frac{1}{2} \mathbf{x}_N^T \mathbf{Q}_N \mathbf{x}_N + \mathbf{q}_N^T \mathbf{x}_N \\
\text{s.t.} \quad & \mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k + \mathbf{b}_k, \quad k = 0, \ldots, N-1 \\
& \mathbf{lb}_x^k \leq \mathbf{x}_k \leq \mathbf{ub}_x^k, \quad k = 0, \ldots, N \\
& \mathbf{lb}_u^k \leq \mathbf{u}_k \leq \mathbf{ub}_u^k, \quad k = 0, \ldots, N-1
\end{aligned}
$$

**注**：在我们的实现中，$\mathbf{S}_k = \mathbf{0}$（无状态-控制交叉项）。

### 7.3 HPIPM QP 求解器接口

**代码实现**（mpccfollower.cpp:414-421）：

```cpp
// 设置 OCP QP 数据
d_ocp_qp_set_all(
  hA.data(),    // A 矩阵序列 [N]：每个 4×4
  hB.data(),    // B 矩阵序列 [N]：每个 4×4
  hb.data(),    // b 向量序列 [N]：每个长度 4（这里为 0）

  hQ.data(),    // Q 矩阵序列 [N+1]：每个 4×4
  hS.data(),    // S 矩阵序列 [N]：每个 4×4（这里为 0）
  hR.data(),    // R 矩阵序列 [N]：每个 4×4
  hq.data(),    // q 向量序列 [N+1]：每个长度 4
  hr.data(),    // r 向量序列 [N]：每个长度 4

  hidxbx.data(), // 状态约束索引 [N+1]：每个长度 1（仅约束 s）
  hlbx.data(),   // 状态下界 [N+1]
  hubx.data(),   // 状态上界 [N+1]

  hidxbu.data(), // 控制约束索引 [N]：每个长度 4（约束所有控制）
  hlbu.data(),   // 控制下界 [N]
  hubu.data(),   // 控制上界 [N]

  NULL, NULL, NULL, NULL,  // 无一般线性约束（C, D 矩阵）
  NULL, NULL, NULL, NULL,  // 无软约束
  NULL, NULL, NULL, NULL,
  &qp
);
```

**重要修复**（避免段错误）：
早期版本误传递 `hC.data()` 等空指针数组，导致崩溃。正确做法是传递 `NULL`。

### 7.4 求解流程

```cpp
// 1. 设置 QP 数据（上述接口）
d_ocp_qp_set_all(...);

// 2. 求解 QP
d_ocp_qp_ipm_solve(qp_solver, &qp, &qp_sol, &arg, &workspace, &status);

// 3. 提取第一步控制输入
d_ocp_qp_sol_get_u(0, &qp_sol, u0_result.data());
```

### 7.5 内点法优势

| 特性 | 活动集法 | 内点法 (HPIPM) |
|------|---------|----------------|
| 复杂度 | $O(N^3)$ | $O(N)$ (利用稀疏性) |
| 迭代次数 | 不固定 | 几乎固定（~10-20） |
| 实时性 | 差 | 优秀 |
| 对初值敏感度 | 高 | 低 |

**HPIPM 使用 Riccati 递推**，从后向前求解最优控制序列，复杂度为 $O(N \cdot NX^3)$，对于我们的问题 $O(15 \times 4^3) = O(960)$，远小于批量形式的 $O(60^3) = O(216000)$。

---

## 8. 预测时域设置

### 8.1 时域参数

| 参数 | 符号 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| 控制频率 | `mpc_hz` | 30.0 | Hz | 每秒求解 QP 的次数 |
| 预测步数 | `N` | 15 | 步 | 预测未来的离散步数 |
| 采样时间 | $\Delta t$ | 0.0333 | s | $1 / \text{mpc\_hz}$ |
| 预测时长 | $T$ | 0.5 | s | $N \times \Delta t$ |

### 8.2 时域长度的权衡

| 时域长度 | 优点 | 缺点 |
|---------|------|------|
| **短** (0.3s) | 计算快，响应迅速 | 前瞻不足，转弯处性能差 |
| **中** (0.5s) | 平衡性能与计算 | 需要调优权重 |
| **长** (1.0s) | 平滑轨迹，前瞻性好 | 计算慢，可能不稳定 |

**代码设置**（mpccfollower.yaml:14-16）：
```yaml
mpc_hz: 30.0        # 控制频率 [Hz]
mpc_horizon: 15     # 预测步数
# 预测时长 = 15 / 30 = 0.5 秒
```

### 8.3 时域与速度的关系

对于最大速度 $v_s^{\max} = 0.5$ m/s：

$$
\text{预测距离} = v_s^{\max} \times T = 0.5 \times 0.5 = 0.25 \text{ 米}
$$

这意味着 MPC 能"看到"前方 0.25 米的路径，足够处理中低速机动。

### 8.4 时域选择准则

#### 8.4.1 根据系统响应时间

时域应至少为系统主导时间常数的 2-3 倍：

$$
T \geq 2 \tau
$$

对于全向底盘，$\tau \approx 0.1$ s，因此 $T = 0.5$ s 足够。

#### 8.4.2 根据最小转弯半径

对于圆弧路径，半径 $R$，角速度 $\omega_{\max}$：

$$
T \geq \frac{\pi/4}{\omega_{\max}}
$$

对于 $\omega_{\max} = 0.5$ rad/s，需要 $T \geq 1.57$ s 才能"看到" 90° 转弯。但实际中通过增加轮廓误差权重可以补偿。

---

## 9. 代码实现细节

### 9.1 路径参数化

**关键数据结构**：
```cpp
std::vector<double> _path_x;     // 路径点 X 坐标 [m]
std::vector<double> _path_y;     // 路径点 Y 坐标 [m]
std::vector<double> _path_yaw;   // 路径点航向 [rad]
std::vector<double> _path_s;     // 路径弧长参数 [m]
```

**路径点查找**（mpccfollower.cpp:192-207）：
```cpp
int findClosestPoint(double s) {
  // 二分查找最接近 s 的路径索引
  auto it = std::lower_bound(_path_s.begin(), _path_s.end(), s);
  int idx = std::distance(_path_s.begin(), it);
  // 边界处理和线性插值
  return idx;
}
```

### 9.2 参考轨迹生成

对于预测时域内的每一步 $k$：

```cpp
double s_k = s_current + k * dt * vs_guess;  // 预估路径参数
int idx = findClosestPoint(s_k);
double x_ref = _path_x[idx];
double y_ref = _path_y[idx];
double yaw_ref = wrapAngle(_path_yaw[idx]);  // 关键：角度归一化
```

### 9.3 角度归一化

**问题**：航向角在圆形轨迹上累积（90° → 438° → ...），导致 MPC 尝试旋转多圈。

**解决方案**（mpccfollower.cpp:237）：

```cpp
inline double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
```

**数学表达**：

$$
\psi_{\text{wrapped}} = \text{atan2}(\sin(\psi), \cos(\psi)) \in [-\pi, \pi]
$$

应用于所有参考航向角：
```cpp
yaw_ref = wrapAngle(yaw_ref);
```

### 9.4 初始状态设置

```cpp
// 当前机器人状态（来自 /localization 话题）
x_current[0] = robot_x;
x_current[1] = robot_y;
x_current[2] = wrapAngle(robot_yaw);
x_current[3] = s_current;  // 路径参数（从最近点计算）
```

### 9.5 控制输出

求解后提取第一步控制：

```cpp
std::vector<double> u0(NU);
d_ocp_qp_sol_get_u(0, &qp_sol, u0.data());

cmd_vel.linear.x = u0[0];   // vx
cmd_vel.linear.y = u0[1];   // vy
cmd_vel.angular.z = u0[2];  // w
// u0[3] 是 vs，不直接输出
```

---

## 10. 参数调优指南

### 10.1 代价权重调优

#### 10.1.1 提高跟踪精度

```yaml
qC: 20.0 → 50.0   # 增加轮廓误差惩罚
qL: 5.0 → 10.0    # 增加滞后误差惩罚
```

**效果**：机器人更紧密地跟随参考路径。

**副作用**：可能导致控制抖动，特别是在路径曲率变化剧烈处。

#### 10.1.2 提高平滑性

```yaml
rVx: 1.0 → 5.0    # 增加速度惩罚
rW: 1.0 → 3.0     # 增加角速度惩罚
```

**效果**：控制输入更平滑，减少执行器磨损。

**副作用**：响应变慢，跟踪误差增大。

#### 10.1.3 调整速度激进程度

```yaml
qVs: 0.05 → 0.2   # 增加速度奖励 → 更快前进
qVs: 0.05 → 0.01  # 减少速度奖励 → 更谨慎
```

**效果**：控制完成任务的速度。

### 10.2 速度约束调优

#### 10.2.1 降低最大速度（对齐阶段）

```yaml
vs_max: 1.0 → 0.5   # 降低到 0.5 m/s
align_max_speed: 0.3 → 0.1  # 对齐时更慢
```

#### 10.2.2 启用倒车

```yaml
vs_min: 0.0 → -0.5  # 允许最大 -0.5 m/s 倒车
```

### 10.3 时域调优

#### 10.3.1 增加前瞻性（计算量变大）

```yaml
mpc_horizon: 15 → 20
```

**注意**：需要确保实时性，监控求解时间 < 33.3 ms（30 Hz）。

#### 10.3.2 降低计算负载（减少前瞻）

```yaml
mpc_horizon: 15 → 10
```

**权衡**：减少前瞻距离，可能在高速或急转弯时性能下降。

### 10.4 常见问题与解决

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| **振荡/抖动** | $w_C$, $w_L$ 过大 | 降低跟踪权重，增加 $r_{v_x}$, $r_{v_y}$ |
| **切角严重** | $w_C$ 过小 | 增加轮廓误差权重 |
| **转弯慢** | $r_\omega$ 过大 | 降低角速度惩罚 |
| **冲出路径** | $q_{v_s}$ 过大 | 降低速度奖励 |
| **停滞不前** | $v_s^{\max}$ 过小 | 增加最大路径速度 |
| **倒车失败** | $v_s^{\min} \geq 0$ | 设置 $v_s^{\min} < 0$ |
| **最后未对齐** | 时域过短或 $w_\psi^f$ 过小 | 增加 $N$ 或终端航向权重 |
| **求解失败** | 约束冲突或数值问题 | 检查 $s_{\text{trust}}$，确保路径连续 |

---

## 总结

MPCC 通过以下关键技术实现高精度路径跟踪：

1. **轮廓-滞后误差分解**：将 2D 位置误差投影到路径的正交方向
2. **路径参数优化**：动态调整沿路径的进度 $s$
3. **模型预测控制**：利用前瞻信息提前优化控制序列
4. **约束处理**：通过 QP 求解器满足速度和加速度限制
5. **结构化求解**：HPIPM 利用时间维度的稀疏性（通过 G, E, H 矩阵隐式表达），实现 $O(N)$ 实时求解

**矩阵总结**：

| 矩阵 | 大小 | 物理意义 | 在代码中 |
|------|------|----------|----------|
| $\mathbf{Q}_k$ | $4 \times 4$ | 状态代价（轮廓/滞后/航向误差） | `hQ.data()` |
| $\mathbf{F} = \mathbf{Q}_N$ | $4 \times 4$ | 终端代价（权重加倍） | `hQ.data()[N]` |
| $\mathbf{R}_k$ | $4 \times 4$ | 控制代价（速度/角速度惩罚） | `hR.data()` |
| $\mathbf{A}_k$ | $4 \times 4$ | 状态转移矩阵（单位矩阵） | `hA.data()` |
| $\mathbf{B}_k$ | $4 \times 4$ | 控制输入矩阵（$\Delta t \mathbf{I}$） | `hB.data()` |
| $\mathbf{G}$ | $60 \times 4$ | 初始状态影响（批量形式） | 隐式（未显式构造） |
| $\mathbf{E}$ | $60 \times 60$ | 控制输入影响（下三角） | 隐式（未显式构造） |
| $\mathbf{H}$ | $60 \times 1$ | 仿射项（这里为零） | 隐式（未显式构造） |

**参数调优建议**：
- 从默认参数开始
- 优先调整 $w_C$, $w_L$（跟踪精度）
- 再调整 $r_{v_x}$, $r_\omega$（平滑性）
- 最后微调 $q_{v_s}$（速度激进程度）
- 验证时测试直线、转弯、倒车等多种场景

**进一步学习**：
- 阅读原始 MPCC 论文（Liniger et al., 2015）
- 研究 HPIPM 求解器文档：https://github.com/giaf/hpipm
- 实验不同时域长度和采样频率的影响
- 尝试添加加速度约束（需修改为双积分器模型）
