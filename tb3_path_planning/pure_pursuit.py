import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


Point2D = Tuple[float, float]


class PurePursuitTracker(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_v', 0.22)
        self.declare_parameter('max_w', 2.84)
        self.declare_parameter('lookahead_distance', 0.20)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('control_frequency', 20.0)

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.execution_time_pub = self.create_publisher(Float64, '/path_execution_time', 10)
        self.path_sub = self.create_subscription(
            Path, str(self.get_parameter('path_topic').value), self.path_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path: List[Point2D] = []
        self.goal: Optional[Point2D] = None
        self.execution_start_wall_time: Optional[float] = None
        self.active_path_signature = None
        self.completed_path_signatures = set()

        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        control_frequency = float(self.get_parameter('control_frequency').value)
        self.timer = self.create_timer(1.0 / max(control_frequency, 1.0), self.control_loop)
        self.get_logger().info(
            f"Pure Pursuit Tracker initialized: lookahead={self.lookahead_distance:.3f} m, "
            f"goal_tolerance={self.goal_tolerance:.3f} m"
        )

    @staticmethod
    def _path_signature(path: List[Point2D]):
        if not path:
            return (0,)
        # Rounded full-path signature prevents periodic republishes from restarting execution.
        return tuple((round(x, 3), round(y, 3)) for x, y in path)

    def path_callback(self, msg: Path):
        new_path: List[Point2D] = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            new_path.append((x, y))

        if len(new_path) < 2:
            self.get_logger().warn('Received an empty or too-short path; ignoring it.')
            return

        signature = self._path_signature(new_path)

        # Avoid restarting every time the planner republishes the same path for RViz
        # and late-joining subscribers. Also ignore the same path after completion.
        if signature == self.active_path_signature:
            return
        if signature in self.completed_path_signatures:
            return

        self.path = new_path
        self.goal = new_path[-1]
        self.active_path_signature = signature
        self.execution_start_wall_time = time.perf_counter()
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")

    def update_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time()
            )
        except TransformException:
            return False

        self.current_x = transform.transform.translation.x
        self.current_y = transform.transform.translation.y

        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        return True

    @staticmethod
    def get_distance(p1: Point2D, p2: Point2D):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def find_lookahead_point(self):
        if not self.path:
            return None

        robot_pos = (self.current_x, self.current_y)

        min_dist = float('inf')
        closest_idx = 0
        for i, point in enumerate(self.path):
            dist = self.get_distance(robot_pos, point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        self.path = self.path[closest_idx:]

        for point in self.path:
            if self.get_distance(robot_pos, point) >= self.lookahead_distance:
                return point

        return self.path[-1]

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def control_loop(self):
        if not self.path:
            self._publish_stop()
            return

        if not self.update_robot_pose():
            self._publish_stop()
            return

        robot_pos = (self.current_x, self.current_y)
        final_goal = self.goal if self.goal is not None else self.path[-1]

        if self.get_distance(robot_pos, final_goal) < self.goal_tolerance:
            elapsed = 0.0
            if self.execution_start_wall_time is not None:
                elapsed = time.perf_counter() - self.execution_start_wall_time
            self.get_logger().info(
                f"Goal reached. Pure Pursuit execution time: {elapsed:.3f} s"
            )
            self.execution_time_pub.publish(Float64(data=float(elapsed)))
            if self.active_path_signature is not None:
                self.completed_path_signatures.add(self.active_path_signature)
            self.path = []
            self.goal = None
            self.active_path_signature = None
            self.execution_start_wall_time = None
            self._publish_stop()
            return

        target_point = self.find_lookahead_point()
        if target_point is None:
            self._publish_stop()
            return

        alpha = math.atan2(target_point[1] - self.current_y, target_point[0] - self.current_x) - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        target_v = 0.15 * (1.0 - min(abs(alpha) / 1.0, 1.0))
        target_v = max(0.02, target_v)

        if abs(alpha) > 0.8:
            target_v = 0.0
            target_w = math.copysign(1.2, alpha)
        else:
            target_w = (2.0 * target_v * math.sin(alpha)) / max(self.lookahead_distance, 1e-3)

        msg = Twist()
        msg.linear.x = float(np.clip(target_v, -self.max_v, self.max_v))
        msg.angular.z = float(np.clip(target_w, -self.max_w, self.max_w))
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
