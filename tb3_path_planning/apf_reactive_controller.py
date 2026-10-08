"""Reactive Artificial Potential Field controller for TurtleBot3.

This node implements the second project part: reactive control with Artificial
Potential Fields (APF). It does not subscribe to /map and it does not consume a
precomputed global path. Obstacle repulsion is computed directly from live
sensor_msgs/LaserScan ranges on /scan. The goal attraction is computed from the
current TF pose relative to a fixed goal coordinate.

Potential model used by the controller:

    J(x) = 1/2 (x - x_g)^T K_g (x - x_g)
         + sum_i alpha_i exp(-1/2 (x - x_o^i)^T Sigma_i^-1 (x - x_o^i))

The command direction is the negative gradient of J at the robot position.
The code evaluates it in the robot/base frame, where x = [0, 0]^T, x_g is the
current goal vector in the base frame, and x_o^i are obstacle points from /scan.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Point2D = Tuple[float, float]
class APFReactiveController(Node):
    """Reactive APF controller using /scan for obstacle forces and /cmd_vel output."""

    def __init__(self) -> None:
        super().__init__('apf_reactive_controller')

        # ROS interface parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/apf_markers')
        self.declare_parameter('base_frame', 'base_footprint')
r.
        self.declare_parameter('goal_frame', 'odom')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)


        # Goal and stopping behavior
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('control_frequency', 20.0)

        # Attractive quadratic potential: U_att = 1/2 e^T K_g e.
        # The negative gradient in the base frame is F_att = K_g * goal_vector.
        self.declare_parameter('attractive_kx', 1.20)
        self.declare_parameter('attractive_ky', 1.20)
        self.declare_parameter('attractive_force_max', 1.00)

        # Repulsive RBF obstacle potential.
        # Larger alpha -> stronger push; sigma controls the spatial spread.
        self.declare_parameter('repulsive_alpha', 0.65)
        self.declare_parameter('repulsive_sigma_x', 0.35)
        self.declare_parameter('repulsive_sigma_y', 0.25)
        self.declare_parameter('repulsive_influence_radius', 0.90)
        self.declare_parameter('repulsive_force_max', 1.80)
        self.declare_parameter('scan_stride', 3)

        # Safety and velocity mapping
        self.declare_parameter('max_v', 0.20)
        self.declare_parameter('max_w', 1.80)
        self.declare_parameter('turn_gain', 1.80)
        self.declare_parameter('force_speed_gain', 0.18)
        self.declare_parameter('slowdown_distance', 0.55)
        self.declare_parameter('emergency_stop_distance', 0.18)
        self.declare_parameter('front_sector_deg', 35.0)
        self.declare_parameter('goal_heading_stop_threshold', 1.20)
        self.declare_parameter('min_valid_range', 0.05)

        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10
        )
        self.execution_time_pub = self.create_publisher(Float64, '/apf_execution_time', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan: Optional[LaserScan] = None
        self.execution_start_wall_time: Optional[float] = None
        self.goal_reached = False
        self.last_tf_warning_time = 0.0
        self.last_scan_warning_time = 0.0

        frequency = float(self.get_parameter('control_frequency').value)
        self.timer = self.create_timer(1.0 / max(frequency, 1.0), self.control_loop)

        self.get_logger().info(
            'APF Reactive Controller initialized: '
            f"goal=({float(self.get_parameter('goal_x').value):.2f}, "
            f"{float(self.get_parameter('goal_y').value):.2f}) in "
            f"{str(self.get_parameter('goal_frame').value)}, "
            f"K=({float(self.get_parameter('attractive_kx').value):.2f}, "
            f"{float(self.get_parameter('attractive_ky').value):.2f}), "
            f"alpha={float(self.get_parameter('repulsive_alpha').value):.2f}, "
            f"sigma=({float(self.get_parameter('repulsive_sigma_x').value):.2f}, "
            f"{float(self.get_parameter('repulsive_sigma_y').value):.2f})"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _clip_vector(vector: np.ndarray, max_norm: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm > max_norm > 0.0:
            return vector * (max_norm / norm)
        return vector

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _get_robot_pose_in_goal_frame(self) -> Optional[Tuple[float, float, float]]:
        goal_frame = str(self.get_parameter('goal_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_frame,
                base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            now = time.perf_counter()
            if now - self.last_tf_warning_time > 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {goal_frame} -> {base_frame}: {exc}"
                )
                self.last_tf_warning_time = now
            return None

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        return x, y, yaw

    def _goal_vector_in_base_frame(self, pose: Tuple[float, float, float]) -> np.ndarray:
        robot_x, robot_y, robot_yaw = pose
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)

        # Goal vector in goal_frame/world coordinates.
        dx_world = goal_x - robot_x
        dy_world = goal_y - robot_y

        # Rotate world vector into robot/base coordinates by R(-yaw).
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        dx_base = cos_yaw * dx_world + sin_yaw * dy_world
        dy_base = -sin_yaw * dx_world + cos_yaw * dy_world
        return np.array([dx_base, dy_base], dtype=float)

    def _attractive_force(self, goal_vector_base: np.ndarray) -> np.ndarray:
        kx = float(self.get_parameter('attractive_kx').value)
        ky = float(self.get_parameter('attractive_ky').value)
        force = np.array([kx * goal_vector_base[0], ky * goal_vector_base[1]], dtype=float)
        return self._clip_vector(force, float(self.get_parameter('attractive_force_max').value))

    def _scan_points_in_base_frame(self, scan: LaserScan) -> List[Point2D]:
        stride = max(1, int(self.get_parameter('scan_stride').value))
        min_valid = max(float(self.get_parameter('min_valid_range').value), float(scan.range_min))
        max_valid = min(
            float(self.get_parameter('repulsive_influence_radius').value),
            float(scan.range_max),
        )

        points: List[Point2D] = []
        for i in range(0, len(scan.ranges), stride):
            r = float(scan.ranges[i])
            if not math.isfinite(r):
                continue
            if r < min_valid or r > max_valid:
                continue
            angle = float(scan.angle_min) + float(i) * float(scan.angle_increment)
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def _repulsive_force(self, obstacle_points_base: List[Point2D]) -> np.ndarray:
        if not obstacle_points_base:
            return np.zeros(2, dtype=float)

        alpha = float(self.get_parameter('repulsive_alpha').value)
        sigma_x = max(float(self.get_parameter('repulsive_sigma_x').value), 1e-3)
        sigma_y = max(float(self.get_parameter('repulsive_sigma_y').value), 1e-3)
        inv_sigma = np.diag([1.0 / (sigma_x * sigma_x), 1.0 / (sigma_y * sigma_y)])

        force = np.zeros(2, dtype=float)
        for px, py in obstacle_points_base:
            obstacle = np.array([px, py], dtype=float)
            exponent = -0.5 * float(obstacle.T @ inv_sigma @ obstacle)
            weight = alpha * math.exp(exponent)
            # Negative gradient of the RBF potential at x = [0, 0].
            force += -weight * (inv_sigma @ obstacle)

        # Normalize by the number of contributing rays so the force is not overly
        # dependent on LaserScan angular resolution or scan_stride.
        force /= max(float(len(obstacle_points_base)), 1.0)
        return self._clip_vector(force, float(self.get_parameter('repulsive_force_max').value))

    def _nearest_front_obstacle(self, scan: LaserScan) -> Tuple[float, float, float]:
        front_half_angle = math.radians(float(self.get_parameter('front_sector_deg').value))
        min_front = float('inf')
        left_clearance = float('inf')
        right_clearance = float('inf')

        for i, raw_range in enumerate(scan.ranges):
            r = float(raw_range)
            if not math.isfinite(r):
                continue
            if r < float(scan.range_min) or r > float(scan.range_max):
                continue
            angle = self._wrap_to_pi(
                float(scan.angle_min) + float(i) * float(scan.angle_increment)
            )
            if abs(angle) <= front_half_angle:
                min_front = min(min_front, r)
            if 0.0 < angle <= math.pi / 2.0:
                left_clearance = min(left_clearance, r)
            elif -math.pi / 2.0 <= angle < 0.0:
                right_clearance = min(right_clearance, r)

        return min_front, left_clearance, right_clearance

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _goal_reached(self, goal_vector_base: np.ndarray) -> bool:
        distance = float(np.linalg.norm(goal_vector_base))
        return distance <= float(self.get_parameter('goal_tolerance').value)

    def _make_arrow_marker(
        self,
        marker_id: int,
        name: str,
        vector: np.ndarray,
        rgba: Tuple[float, float, float, float],
        scale: float = 0.45,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = str(self.get_parameter('base_frame').value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.scale.y = 0.060
        marker.scale.z = 0.060
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.08
        end = Point()
        end.x = float(vector[0]) * scale
        end.y = float(vector[1]) * scale
        end.z = 0.08
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def _publish_markers(
        self,
        attractive: np.ndarray,
        repulsive: np.ndarray,
        total: np.ndarray,
        obstacle_points_base: List[Point2D],
    ) -> None:
        markers = MarkerArray()

        # Delete previous obstacle point markers before publishing the current set.
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        markers.markers.append(
            self._make_arrow_marker(
                0, 'apf_attractive_force', attractive, (0.0, 0.8, 0.0, 1.0)
            )
        )
        markers.markers.append(
            self._make_arrow_marker(
                1, 'apf_repulsive_force', repulsive, (0.9, 0.1, 0.1, 1.0)
            )
        )
        markers.markers.append(
            self._make_arrow_marker(
                2, 'apf_total_force', total, (0.1, 0.2, 1.0, 1.0), scale=0.60
            )
        )

        # Show a decimated set of obstacle samples used by the RBF term.
        for j, (px, py) in enumerate(obstacle_points_base[:80]):
            marker = Marker()
            marker.header.frame_id = str(self.get_parameter('base_frame').value)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'apf_lidar_obstacle_samples'
            marker.id = 100 + j
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(px)
            marker.pose.position.y = float(py)
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.scale.y = 0.035
            marker.scale.z = 0.035
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 0.8
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def _velocity_command(
        self,
        total_force: np.ndarray,
        min_front: float,
        left_clearance: float,
        right_clearance: float,
    ) -> Twist:
        cmd = Twist()
        max_v = float(self.get_parameter('max_v').value)
        max_w = float(self.get_parameter('max_w').value)
        turn_gain = float(self.get_parameter('turn_gain').value)
        speed_gain = float(self.get_parameter('force_speed_gain').value)
        emergency_distance = float(self.get_parameter('emergency_stop_distance').value)
        slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        heading_stop_threshold = float(self.get_parameter('goal_heading_stop_threshold').value)

        # Hard safety layer: rotate toward the side with more clearance if the
        # front sector is too close to an obstacle.
        if min_front < emergency_distance:
            cmd.linear.x = 0.0
            turn_direction = 1.0 if left_clearance > right_clearance else -1.0
            cmd.angular.z = turn_direction * min(max_w, 1.2)
            return cmd

        force_norm = float(np.linalg.norm(total_force))
        if force_norm < 1e-6:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        desired_heading = math.atan2(float(total_force[1]), float(total_force[0]))
        cmd.angular.z = float(np.clip(turn_gain * desired_heading, -max_w, max_w))

        # Do not drive forward aggressively while the force direction points far
        # away from the robot's x-axis; rotate first, then translate.
        heading_factor = max(0.0, math.cos(desired_heading))
        if abs(desired_heading) > heading_stop_threshold:
            heading_factor = 0.0

        obstacle_factor = 1.0
        if min_front < slowdown_distance:
            obstacle_factor = max(
                0.05,
                (min_front - emergency_distance)
                / max(slowdown_distance - emergency_distance, 1e-3),
            )

        requested_v = speed_gain * force_norm * heading_factor * obstacle_factor
        cmd.linear.x = float(np.clip(requested_v, 0.0, max_v))
        return cmd

    def control_loop(self) -> None:
        if self.goal_reached:
            self._publish_stop()
            return

        if self.latest_scan is None:
            now = time.perf_counter()
            if now - self.last_scan_warning_time > 2.0:
                self.get_logger().warn('Waiting for LaserScan on /scan.')
                self.last_scan_warning_time = now
            self._publish_stop()
            return

        pose = self._get_robot_pose_in_goal_frame()
        if pose is None:
            self._publish_stop()
            return

        goal_vector = self._goal_vector_in_base_frame(pose)
        if self.execution_start_wall_time is None:
            self.execution_start_wall_time = time.perf_counter()

        if self._goal_reached(goal_vector):
            elapsed = time.perf_counter() - self.execution_start_wall_time
            self.get_logger().info(f'APF goal reached. Execution time: {elapsed:.3f} s')
            self.execution_time_pub.publish(Float64(data=float(elapsed)))
            self.goal_reached = True
            self._publish_stop()
            return

        scan = self.latest_scan
        obstacle_points = self._scan_points_in_base_frame(scan)
        attractive = self._attractive_force(goal_vector)
        repulsive = self._repulsive_force(obstacle_points)
        total = attractive + repulsive
        total = self._clip_vector(total, 2.0)

        min_front, left_clearance, right_clearance = self._nearest_front_obstacle(scan)
        cmd = self._velocity_command(total, min_front, left_clearance, right_clearance)
        self.cmd_vel_pub.publish(cmd)
        self._publish_markers(attractive, repulsive, total, obstacle_points)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = APFReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
