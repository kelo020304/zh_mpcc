#!/usr/bin/env python3
"""
MPC 预测轨迹 RViz 可视化节点
将 /mpcc_pred_path 可视化为有宽度的动态色带
"""
import math
import rospy
import tf
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class MPCCPredVisualizer:
    """MPC 预测轨迹可视化器"""

    def __init__(self):
        # 参数
        self.pred_topic = rospy.get_param("~pred_topic", "/mpcc_pred_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization")
        self.marker_topic = rospy.get_param("~marker_topic", "/mpcc_pred_marker")
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 1.0)  # 标记生命周期(秒)
        self.show_centerline = True

        # 当前位姿
        self.current_pose = None

        # 发布器
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        # 订阅器
        rospy.Subscriber(self.pred_topic, Path, self._pred_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=1)
       

    def _odom_callback(self, msg):
        """接收里程计信息"""
        self.current_pose = msg.pose.pose

    def _pred_callback(self, msg):
        """接收预测轨迹并可视化"""
        if len(msg.poses) < 2:
            # 路径太短，清空可视化
            self._clear_markers()
            return

        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "等待位姿信息...")
            return

        # 转换 body frame 到 world frame
        world_points = self._transform_to_world(msg)

        if len(world_points) < 2:
            self._clear_markers()
            return

        # 创建可视化标记
        markers = self._create_ribbon_markers(world_points, msg.header.stamp)

        # 发布
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    def _transform_to_world(self, path_msg):
        """将 body frame 路径转换到 world frame"""
        # 获取当前位姿
        pos = self.current_pose.position
        quat = self.current_pose.orientation

        # 提取 yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = euler[2]

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        world_points = []
        for pose_stamped in path_msg.poses:
            # body frame 坐标
            xb = pose_stamped.pose.position.x
            yb = pose_stamped.pose.position.y

            # 转换到 world frame
            xw = pos.x + cy * xb - sy * yb
            yw = pos.y + sy * xb + cy * yb
            zw = pos.z

            world_points.append((xw, yw, zw))

        return world_points

    def _create_ribbon_markers(self, points, stamp):
        """创建有宽度的色带标记"""
        markers = []
        n = len(points)

        # 仅保留中心线
        if self.show_centerline:
            centerline = Marker()
            centerline.header.frame_id = "world"
            centerline.header.stamp = stamp
            centerline.ns = "mpcc_pred_centerline"
            centerline.id = 0
            centerline.type = Marker.LINE_STRIP
            centerline.action = Marker.ADD
            centerline.pose.orientation.w = 1.0
            centerline.scale.x = 0.1  # 线宽
            centerline.color = ColorRGBA(1.0, 0, 1.0, 1.0)
            centerline.lifetime = rospy.Duration(self.marker_lifetime)
            centerline.points = [Point(x, y, z) for x, y, z in points]
            markers.append(centerline)

        return markers

    def _clear_markers(self):
        """清空所有标记"""
        marker_array = MarkerArray()
        for ns, id in [("mpcc_pred_centerline", 0)]:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = ns
            marker.id = id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main():
    rospy.init_node("mpcc_pred_visualizer")
    visualizer = MPCCPredVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
