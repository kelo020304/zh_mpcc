#!/usr/bin/env python3
"""
MPC Follower Test Framework
支持多种模式验证 mpccfollower 的正确性，无需真实感知系统
"""
import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


class TrajectoryType(Enum):
    """轨迹类型"""
    FIGURE_EIGHT = "figure_eight"  # 8字形
    CIRCLE = "circle"              # 圆形
    STRAIGHT = "straight"          # 直线
    STRAIGHT_BACKWARD = "straight_backward"  # 倒车直线
    SQUARE = "square"              # 正方形
    SINE_WAVE = "sine_wave"        # 正弦波
    CUSTOM = "custom"              # 自定义


class SimulationMode(Enum):
    """仿真模式"""
    PURE_KINEMATIC = "pure_kinematic"  # 纯运动学模拟
    MUJOCO = "mujoco"                  # MuJoCo物理仿真


@dataclass
class TrackingMetrics:
    """跟踪性能指标"""
    mean_contour_error: float = 0.0  # 平均轮廓误差
    max_contour_error: float = 0.0   # 最大轮廓误差
    mean_lag_error: float = 0.0      # 平均滞后误差
    max_lag_error: float = 0.0       # 最大滞后误差
    mean_yaw_error: float = 0.0      # 平均航向误差 (deg)
    max_yaw_error: float = 0.0       # 最大航向误差 (deg)
    mean_speed: float = 0.0          # 平均速度
    completion_time: float = 0.0     # 完成时间

    def print_summary(self):
        """打印统计摘要"""
        print("=" * 60)
        print("跟踪性能统计:")
        print(f"  轮廓误差: 平均={self.mean_contour_error:.4f}m, 最大={self.max_contour_error:.4f}m")
        print(f"  滞后误差: 平均={self.mean_lag_error:.4f}m, 最大={self.max_lag_error:.4f}m")
        print(f"  航向误差: 平均={self.mean_yaw_error:.2f}°, 最大={self.max_yaw_error:.2f}°")
        print(f"  平均速度: {self.mean_speed:.3f}m/s")
        print(f"  完成时间: {self.completion_time:.2f}s")
        print("=" * 60)


class TrajectoryGenerator(ABC):
    """轨迹生成器基类"""

    @abstractmethod
    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        """生成轨迹点"""
        pass

    @abstractmethod
    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        """获取弧长s处的参考点(x, y, theta)"""
        pass

    def get_total_length(self) -> float:
        """获取轨迹总长度"""
        return 0.0


class FigureEightTrajectory(TrajectoryGenerator):
    """8字形轨迹"""

    def __init__(self, scale: float = 2.0, center_offset: Tuple[float, float] = (3.0, 0.0)):
        self.scale = scale
        self.offset_x, self.offset_y = center_offset

    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        pts = []
        # 生成几乎完整的8字形（97%），控制器已修复航向角 wrap 问题
        # 不完全闭合是为了避免起点终点重合导致的停止判断问题
        max_t = 2.0 * math.pi * 0.97  # 97% ≈ 350度
        for i in range(num_points):
            t = max_t * i / max(1, num_points - 1)
            x = self.offset_x + self.scale * math.sin(t)
            y = self.offset_y + self.scale * math.sin(t) * math.cos(t)
            pts.append((x, y))
        return pts

    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        # 简化版本：直接用参数t
        t = s
        x = self.offset_x + self.scale * math.sin(t)
        y = self.offset_y + self.scale * math.sin(t) * math.cos(t)
        dx = self.scale * math.cos(t)
        dy = self.scale * (math.cos(t)**2 - math.sin(t)**2)
        theta = math.atan2(dy, dx)
        return x, y, theta


class CircleTrajectory(TrajectoryGenerator):
    """圆形轨迹"""

    def __init__(self, radius: float = 2.0, center_offset: Tuple[float, float] = (3.0, 0.0)):
        self.radius = radius
        self.center_x, self.center_y = center_offset

    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        pts = []
        # 生成几乎完整的圆（97%），控制器已修复航向角 wrap 问题
        # 不完全闭合是为了避免起点终点重合导致的停止判断问题
        max_t = 2.0 * math.pi * 0.97  # 97% ≈ 350度
        for i in range(num_points):
            t = max_t * i / max(1, num_points - 1)
            x = self.center_x + self.radius * math.cos(t)
            y = self.center_y + self.radius * math.sin(t)
            pts.append((x, y))
        return pts

    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        t = s / self.radius
        x = self.center_x + self.radius * math.cos(t)
        y = self.center_y + self.radius * math.sin(t)
        theta = math.atan2(math.cos(t), -math.sin(t))  # 切线方向
        return x, y, theta

    def get_total_length(self) -> float:
        return 2.0 * math.pi * self.radius


class StraightTrajectory(TrajectoryGenerator):
    """直线轨迹"""

    def __init__(self, length: float = 5.0, angle: float = 0.0, start_offset: Tuple[float, float] = (0.5, 0.0)):
        self.length = length
        self.angle = angle
        self.start_x, self.start_y = start_offset

    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        pts = []
        # 从 (0.5, 0) 开始，沿着角度方向前进
        for i in range(num_points):
            s = self.length * i / max(1, num_points - 1)
            x = self.start_x + s * math.cos(self.angle)
            y = self.start_y + s * math.sin(self.angle)
            pts.append((x, y))
        return pts

    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        x = self.start_x + s * math.cos(self.angle)
        y = self.start_y + s * math.sin(self.angle)
        return x, y, self.angle

    def get_total_length(self) -> float:
        return self.length


class StraightBackwardTrajectory(TrajectoryGenerator):
    """倒车直线轨迹

    生成一条直线轨迹，但航向角指向运动反方向（相差180°）
    这样 MPC 会输出负的路径速度 vs 来实现倒车
    """

    def __init__(self, length: float = 5.0, angle: float = 0.0, start_offset: Tuple[float, float] = (0.5, 0.0)):
        self.length = length
        self.angle = angle  # 运动方向（前进方向）
        self.start_x, self.start_y = start_offset

    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        pts = []
        # 轨迹点的空间顺序还是正常的（从起点到终点）
        for i in range(num_points):
            s = self.length * i / max(1, num_points - 1)
            x = self.start_x + s * math.cos(self.angle)
            y = self.start_y + s * math.sin(self.angle)
            pts.append((x, y))
        return pts

    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        x = self.start_x + s * math.cos(self.angle)
        y = self.start_y + s * math.sin(self.angle)
        # 关键：航向角指向运动反方向（相差 180°）
        # 这样机器人会倒着走这条轨迹
        theta = self.angle + math.pi  # 反向
        return x, y, theta

    def get_total_length(self) -> float:
        return self.length


class SquareTrajectory(TrajectoryGenerator):
    """正方形轨迹"""

    def __init__(self, side_length: float = 3.0, center_offset: Tuple[float, float] = (3.0, 0.0)):
        self.side = side_length
        self.offset_x, self.offset_y = center_offset

    def generate(self, num_points: int) -> List[Tuple[float, float]]:
        pts = []
        # 生成不完全闭合的正方形（避免起点终点重合导致提前停止）
        # 只生成 3.5 条边而不是 4 条边
        # 偏移到 (3, 0)，远离机器人初始位置 (0, 0)
        total_points = int(num_points * 0.875)  # 87.5% = 3.5/4
        points_per_side = total_points // 4

        # 4条边（但最后一条边只生成一半）
        for i in range(points_per_side):
            t = i / max(1, points_per_side - 1)
            pts.append((self.offset_x + t * self.side - self.side/2,
                       self.offset_y - self.side/2))  # 下边
        for i in range(points_per_side):
            t = i / max(1, points_per_side - 1)
            pts.append((self.offset_x + self.side/2,
                       self.offset_y + t * self.side - self.side/2))  # 右边
        for i in range(points_per_side):
            t = i / max(1, points_per_side - 1)
            pts.append((self.offset_x + self.side/2 - t * self.side,
                       self.offset_y + self.side/2))  # 上边
        # 左边只生成一半
        for i in range(total_points - 3*points_per_side):
            t = i / max(1, points_per_side - 1)
            pts.append((self.offset_x - self.side/2,
                       self.offset_y + self.side/2 - t * self.side))  # 左边（部分）

        return pts

    def get_reference_point(self, s: float) -> Tuple[float, float, float]:
        # 简化：返回最近点（加上中心偏移）
        total_length = 4 * self.side
        s = s % total_length
        if s < self.side:
            return self.offset_x + s - self.side/2, self.offset_y - self.side/2, 0.0
        elif s < 2*self.side:
            return self.offset_x + self.side/2, self.offset_y + (s - self.side) - self.side/2, math.pi/2
        elif s < 3*self.side:
            return self.offset_x + self.side/2 - (s - 2*self.side), self.offset_y + self.side/2, math.pi
        else:
            return self.offset_x - self.side/2, self.offset_y + self.side/2 - (s - 3*self.side), -math.pi/2


class VehicleSimulator(ABC):
    """车辆模拟器基类"""

    @abstractmethod
    def step(self, cmd_vel: Tuple[float, float, float], dt: float):
        """执行一步仿真"""
        pass

    @abstractmethod
    def get_state(self) -> Tuple[float, float, float]:
        """获取当前状态 (x, y, yaw)"""
        pass

    @abstractmethod
    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """重置状态"""
        pass


class KinematicSimulator(VehicleSimulator):
    """纯运动学模拟器（全向运动）"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, cmd_vel: Tuple[float, float, float], dt: float):
        vx, vy, wz = cmd_vel
        # body frame -> world frame
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        vx_w = cy * vx - sy * vy
        vy_w = sy * vx + cy * vy

        self.x += vx_w * dt
        self.y += vy_w * dt
        self.yaw = (self.yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi

    def get_state(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class MuJoCoSimulator(VehicleSimulator):
    """MuJoCo仿真器（集成 SimMobileBaseEnv）"""

    def __init__(self, env):
        self.env = env
        # 从env获取位姿传感器

    def step(self, cmd_vel: Tuple[float, float, float], dt: float):
        # 通过ROS发送cmd_vel，env会自动处理
        pass

    def get_state(self) -> Tuple[float, float, float]:
        # 从env的localization获取
        # TODO: 实现从env读取状态的逻辑
        return 0.0, 0.0, 0.0

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        # TODO: 重置MuJoCo模型位姿
        pass


class MPCCTestFramework:
    """MPC Follower 测试框架"""

    def __init__(
        self,
        trajectory_type: TrajectoryType = TrajectoryType.FIGURE_EIGHT,
        simulation_mode: SimulationMode = SimulationMode.PURE_KINEMATIC,
        enable_visualization: bool = True,
        enable_metrics: bool = True,
    ):
        # 参数
        self.traj_type = trajectory_type
        self.sim_mode = simulation_mode
        self.enable_viz = enable_visualization
        self.enable_metrics = enable_metrics

        # ROS参数
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.path_hz = rospy.get_param("~path_hz", 5.0)
        self.path_pub_once = rospy.get_param("~path_pub_once", False)
        self.path_pub_duration = rospy.get_param("~path_pub_duration", 2.0)
        self.track_scale = rospy.get_param("~track_scale", 2.0)
        self.track_points = rospy.get_param("~track_points", 200)
        self.odom_topic = rospy.get_param("~odom_topic", "/localization")
        self.path_topic = rospy.get_param("~path_topic", "/local_path")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.pred_topic = rospy.get_param("~pred_topic", "/mpcc_pred_path")

        # 数据
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = time.time()
        self._pred_body: List[Tuple[float, float]] = []
        self._trace: List[Tuple[float, float, float]] = []  # (x, y, t)
        self._path_sent = False

        # 轨迹生成器
        self.trajectory = self._create_trajectory()
        # 生成固定在 world frame 的参考轨迹
        self._path_world = self.trajectory.generate(self.track_points)

        # 车辆模拟器
        self.simulator = self._create_simulator()

        # 设置初始位置在轨迹起点附近
        if len(self._path_world) > 0:
            start_x, start_y = self._path_world[0]
            # 稍微偏离起点，让控制器有机会调整
            self.simulator.reset(start_x - 0.2, start_y, 0.0)

        # 误差统计
        self.contour_errors = []
        self.lag_errors = []
        self.yaw_errors = []
        self.speeds = []

        # ROS接口
        self.pub_path = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=1)
        rospy.Subscriber(self.pred_topic, Path, self._pred_cb, queue_size=1)

    def _create_trajectory(self) -> TrajectoryGenerator:
        """创建轨迹生成器"""
        if self.traj_type == TrajectoryType.FIGURE_EIGHT:
            return FigureEightTrajectory(self.track_scale)
        elif self.traj_type == TrajectoryType.CIRCLE:
            return CircleTrajectory(self.track_scale)
        elif self.traj_type == TrajectoryType.STRAIGHT:
            return StraightTrajectory(self.track_scale * 2)
        elif self.traj_type == TrajectoryType.STRAIGHT_BACKWARD:
            return StraightBackwardTrajectory(self.track_scale * 2)
        elif self.traj_type == TrajectoryType.SQUARE:
            return SquareTrajectory(self.track_scale * 2)
        else:
            return FigureEightTrajectory(self.track_scale)

    def _create_simulator(self) -> VehicleSimulator:
        """创建车辆模拟器"""
        if self.sim_mode == SimulationMode.PURE_KINEMATIC:
            return KinematicSimulator()
        else:
            # TODO: 集成MuJoCo
            return KinematicSimulator()

    def _cmd_cb(self, msg: Twist):
        """接收控制指令"""
        with self._lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_cmd_time = time.time()

    def _pred_cb(self, msg: Path):
        """接收预测轨迹"""
        pts = []
        for p in msg.poses:
            pts.append((p.pose.position.x, p.pose.position.y))
        with self._lock:
            self._pred_body = pts

    def _step_simulation(self, dt: float):
        """执行一步仿真"""
        with self._lock:
            vx, vy, wz = self._cmd
            if time.time() - self._last_cmd_time > 0.5:
                vx, vy, wz = 0.0, 0.0, 0.0

        self.simulator.step((vx, vy, wz), dt)
        x, y, yaw = self.simulator.get_state()
        self._trace.append((x, y, time.time()))
        if len(self._trace) > 3000:
            self._trace = self._trace[-3000:]

        # 计算误差
        if self.enable_metrics:
            self._update_metrics(x, y, yaw, vx, vy, wz)

    def _update_metrics(self, x: float, y: float, yaw: float, vx: float, vy: float, wz: float):
        """更新性能指标（基于 world frame 的固定轨迹）"""
        # 找最近的参考点（在 world frame 中）
        min_dist = float('inf')
        closest_idx = 0
        for i, (xr, yr) in enumerate(self._path_world):
            dist = math.sqrt((xr - x)**2 + (yr - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 计算轮廓误差和滞后误差
        if len(self._path_world) > closest_idx + 1:
            xr1, yr1 = self._path_world[closest_idx]
            xr2, yr2 = self._path_world[min(closest_idx + 1, len(self._path_world) - 1)]

            # 参考点切线方向
            theta_ref = math.atan2(yr2 - yr1, xr2 - xr1)

            # 轮廓误差和滞后误差
            dx = x - xr1
            dy = y - yr1
            cos_t = math.cos(theta_ref)
            sin_t = math.sin(theta_ref)

            # 轮廓误差：垂直于路径方向
            e_c = abs(-sin_t * dx + cos_t * dy)
            # 滞后误差：沿路径方向
            e_l = cos_t * dx + sin_t * dy

            self.contour_errors.append(e_c)
            self.lag_errors.append(abs(e_l))

        # 航向误差
        if len(self._path_world) > closest_idx + 1:
            xr1, yr1 = self._path_world[closest_idx]
            xr2, yr2 = self._path_world[min(closest_idx + 1, len(self._path_world) - 1)]
            theta_ref = math.atan2(yr2 - yr1, xr2 - xr1)
            yaw_err = abs((yaw - theta_ref + math.pi) % (2 * math.pi) - math.pi)
            self.yaw_errors.append(math.degrees(yaw_err))

        # 速度
        speed = math.sqrt(vx**2 + vy**2)
        self.speeds.append(speed)

    def _publish_odom(self):
        """发布里程计"""
        x, y, yaw = self.simulator.get_state()
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "body"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_path(self):
        """发布参考路径（将固定的 world frame 轨迹转换到 body frame）"""
        # 获取当前机器人位姿
        x0, y0, yaw0 = self.simulator.get_state()
        cy = math.cos(yaw0)
        sy = math.sin(yaw0)

        # 将 world frame 的轨迹转换到 body frame
        now = rospy.Time.now()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = "body"
        msg.poses = []

        for xw, yw in self._path_world:
            # world -> body transformation
            dx = xw - x0
            dy = yw - y0
            xb = cy * dx + sy * dy
            yb = -sy * dx + cy * dy

            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = xb
            ps.pose.position.y = yb
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _visualize(self, ax):
        """可视化"""
        ax.clear()
        ax.set_title(f"MPC Follower Test - {self.traj_type.value}")
        ax.set_aspect("equal", "box")
        ax.grid(True, alpha=0.3)

        # 参考路径（固定在 world frame）
        xs = [x for x, y in self._path_world]
        ys = [y for x, y in self._path_world]
        ax.plot(xs, ys, "k--", linewidth=1.0, label="Reference Path (World)")

        # 获取当前机器人位姿（用于转换预测轨迹）
        x0, y0, yaw = self.simulator.get_state()
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # 实际轨迹
        if self._trace:
            tx, ty, _ = zip(*self._trace)
            ax.plot(tx, ty, "b-", linewidth=1.5, label="Actual Trajectory")
            ax.plot(tx[-1], ty[-1], "bo", markersize=6)

            # 绘制机器人方向
            arrow_len = 0.3
            dx = arrow_len * math.cos(yaw)
            dy = arrow_len * math.sin(yaw)
            ax.arrow(tx[-1], ty[-1], dx, dy, head_width=0.1,
                    head_length=0.1, fc='blue', ec='blue')

        # 预测轨迹
        with self._lock:
            pred_body = list(self._pred_body)
        if pred_body:
            pxs = []
            pys = []
            for xb, yb in pred_body:
                wx = cy * xb - sy * yb + x0
                wy = sy * xb + cy * yb + y0
                pxs.append(wx)
                pys.append(wy)
            ax.plot(pxs, pys, "r-", linewidth=2.0, label="MPC Prediction")

        ax.legend(loc="upper right")

    def get_metrics(self) -> TrackingMetrics:
        """计算性能指标"""
        metrics = TrackingMetrics()
        if self.contour_errors:
            metrics.mean_contour_error = np.mean(self.contour_errors)
            metrics.max_contour_error = np.max(self.contour_errors)
        if self.lag_errors:
            metrics.mean_lag_error = np.mean(self.lag_errors)
            metrics.max_lag_error = np.max(self.lag_errors)
        if self.yaw_errors:
            metrics.mean_yaw_error = np.mean(self.yaw_errors)
            metrics.max_yaw_error = np.max(self.yaw_errors)
        if self.speeds:
            metrics.mean_speed = np.mean(self.speeds)
        if len(self._trace) > 1:
            metrics.completion_time = self._trace[-1][2] - self._trace[0][2]
        return metrics

    def run(self, duration: Optional[float] = None):
        """运行测试"""
        if self.enable_viz:
            plt.ion()
            fig, ax = plt.subplots(figsize=(10, 8))

        last = time.time()
        start_time = last
        next_path = time.time()

        rate = rospy.Rate(self.rate_hz)

        print(f"启动MPC Follower测试框架")
        print(f"  轨迹类型: {self.traj_type.value}")
        print(f"  仿真模式: {self.sim_mode.value}")
        print(f"  可视化: {'开启' if self.enable_viz else '关闭'}")
        print(f"  误差统计: {'开启' if self.enable_metrics else '关闭'}")
        print("=" * 60)

        try:
            while not rospy.is_shutdown():
                now = time.time()

                # 检查是否超时
                if duration and (now - start_time) > duration:
                    print(f"\n测试完成 (运行时间: {duration}s)")
                    break

                dt = now - last
                last = now
                if dt <= 0.0:
                    dt = 1.0 / max(1.0, self.rate_hz)

                # 执行仿真
                self._step_simulation(dt)
                self._publish_odom()

                # 发布路径
                if self.path_pub_once:
                    if not self._path_sent or (now - start_time) < self.path_pub_duration:
                        self._publish_path()
                        if (now - start_time) >= self.path_pub_duration:
                            self._path_sent = True
                else:
                    if now >= next_path:
                        self._publish_path()
                        next_path = now + 1.0 / max(1.0, self.path_hz)

                # 可视化
                if self.enable_viz and (now - start_time) > 0.5:
                    self._visualize(ax)
                    plt.pause(0.01)

                rate.sleep()

        except KeyboardInterrupt:
            print("\n用户中断测试")

        finally:
            # 打印统计信息
            if self.enable_metrics:
                print("\n")
                metrics = self.get_metrics()
                metrics.print_summary()

            if self.enable_viz:
                plt.ioff()
                plt.show()


def main():
    rospy.init_node("mpcc_test_framework")

    # 从参数服务器读取配置
    traj_type_str = rospy.get_param("~trajectory_type", "figure_eight")
    sim_mode_str = rospy.get_param("~simulation_mode", "pure_kinematic")
    enable_viz = rospy.get_param("~enable_visualization", True)
    enable_metrics = rospy.get_param("~enable_metrics", True)
    duration = rospy.get_param("~duration", None)

    # 转换枚举
    traj_type = TrajectoryType(traj_type_str)
    sim_mode = SimulationMode(sim_mode_str)

    # 创建测试框架
    framework = MPCCTestFramework(
        trajectory_type=traj_type,
        simulation_mode=sim_mode,
        enable_visualization=enable_viz,
        enable_metrics=enable_metrics,
    )

    # 运行
    framework.run(duration=duration)


if __name__ == "__main__":
    main()
