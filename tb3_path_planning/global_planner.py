"""Global path planners for the TurtleBot3 path-planning project.

This node subscribes to a static map from nav2_map_server as nav_msgs/OccupancyGrid,
inflates occupied cells for the TurtleBot3 Burger footprint, computes either an A*
or RRT* global path, smooths it, and publishes the final nav_msgs/Path for the
provided Pure Pursuit controller.
"""

from __future__ import annotations

import heapq
import math
import random
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

GridCell = Tuple[int, int]  # (x, y), in map-cell coordinates
Point2D = Tuple[float, float]  # (x, y), in world coordinates


@dataclass
class PlannerResult:
    raw_path: List[Point2D]
    smoothed_path: List[Point2D]
    planning_time_s: float
    raw_length_m: float
    smoothed_length_m: float
    raw_waypoints: int
    smoothed_waypoints: int
    expanded_nodes: int
    smoothing_method: str


class GlobalPlannerNode(Node):
    """ROS 2 node implementing A* and RRT* over an OccupancyGrid map."""

    ENVIRONMENT_START_GOAL: Dict[str, Tuple[Point2D, Point2D]] = {
        "local_minima": ((0.0, 0.0), (4.0, 0.0)),
        "narrow_passage": ((0.0, 0.0), (4.0, 0.0)),
        "sharp_maze": ((-4.5, -4.5), (4.5, 4.5)),
    }

    def __init__(self, forced_algorithm: Optional[str] = None) -> None:
        super().__init__("tb3_global_planner")

        # General configuration
        self.declare_parameter("algorithm", forced_algorithm or "astar")
        self.declare_parameter("environment", "local_minima")
        self.declare_parameter("start_x", float("nan"))
        self.declare_parameter("start_y", float("nan"))
        self.declare_parameter("goal_x", float("nan"))
        self.declare_parameter("goal_y", float("nan"))
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("raw_path_topic", "/raw_path")
        self.declare_parameter("smoothed_path_topic", "/smoothed_path")
        self.declare_parameter("inflated_costmap_topic", "/inflated_costmap")
        self.declare_parameter("rrt_tree_topic", "/rrt_star_tree")
        self.declare_parameter("use_smoothed_path", True)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("treat_unknown_as_occupied", True)
        self.declare_parameter("inflation_radius", 0.12)  # 0.10 m radius + small safety margin
        self.declare_parameter("heuristic", "euclidean")  # euclidean or manhattan

        # RRT* configuration
        self.declare_parameter("rrt_max_iterations", 15000)
        self.declare_parameter("rrt_step_size", 0.25)
        self.declare_parameter("rrt_rewire_radius", 0.60)
        self.declare_parameter("rrt_goal_sample_rate", 0.12)
        self.declare_parameter("rrt_random_seed", 42)
        self.declare_parameter("rrt_tree_publish_stride", 200)

        # Path smoothing configuration
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("spline_resolution", 0.05)
        # The provided Pure Pursuit controller chooses an existing waypoint as
        # its lookahead target. Keep published waypoints dense enough that A*
        # paths with sharp line-of-sight corners remain trackable.
        self.declare_parameter("tracking_waypoint_spacing", 0.05)

        self.algorithm = str(self.get_parameter("algorithm").value).lower().strip()
        self.environment = str(self.get_parameter("environment").value).lower().strip()
        self.start_xy, self.goal_xy = self._resolve_start_goal()

        if self.algorithm not in ("astar", "a_star", "rrtstar", "rrt_star", "rrt*"):
            raise ValueError(
                "algorithm must be one of: astar, a_star, rrtstar, rrt_star, rrt*"
            )

        durable_qos = QoSProfile(depth=1)
        durable_qos.reliability = QoSReliabilityPolicy.RELIABLE
        durable_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        volatile_qos = QoSProfile(depth=10)
        volatile_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            durable_qos,
        )
        self.planned_path_pub = self.create_publisher(
            Path, str(self.get_parameter("planned_path_topic").value), durable_qos
        )
        self.raw_path_pub = self.create_publisher(
            Path, str(self.get_parameter("raw_path_topic").value), durable_qos
        )
        self.smoothed_path_pub = self.create_publisher(
            Path, str(self.get_parameter("smoothed_path_topic").value), durable_qos
        )
        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("inflated_costmap_topic").value),
            durable_qos,
        )
        self.rrt_tree_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("rrt_tree_topic").value), volatile_qos
        )
        self.planning_time_pub = self.create_publisher(
            Float64, "/global_planning_time", durable_qos
        )

        self._original_map_msg: Optional[OccupancyGrid] = None
        self._inflated_map_msg: Optional[OccupancyGrid] = None
        self._raw_path_msg: Optional[Path] = None
        self._smoothed_path_msg: Optional[Path] = None
        self._planned_path_msg: Optional[Path] = None
        self._planned = False

        publish_period = float(self.get_parameter("publish_period_s").value)
        if publish_period > 0.0:
            self.timer = self.create_timer(publish_period, self._publish_cached_outputs)

        self.get_logger().info(
            f"Global planner initialized: algorithm={self.algorithm}, "
            f"environment={self.environment}, start={self.start_xy}, goal={self.goal_xy}"
        )

    def _resolve_start_goal(self) -> Tuple[Point2D, Point2D]:
        default_start, default_goal = self.ENVIRONMENT_START_GOAL.get(
            self.environment, self.ENVIRONMENT_START_GOAL["local_minima"]
        )

        sx = float(self.get_parameter("start_x").value)
        sy = float(self.get_parameter("start_y").value)
        gx = float(self.get_parameter("goal_x").value)
        gy = float(self.get_parameter("goal_y").value)

        start = default_start if math.isnan(sx) or math.isnan(sy) else (sx, sy)
        goal = default_goal if math.isnan(gx) or math.isnan(gy) else (gx, gy)
        return start, goal

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if self._planned:
            return
        self._original_map_msg = msg

        try:
            self._plan_from_map(msg)
        except Exception as exc:  # Keep the node alive so the user can see the error.
            self.get_logger().error(f"Planning failed: {exc}")

    def _plan_from_map(self, msg: OccupancyGrid) -> None:
        occupied = self._occupancy_grid_to_bool(msg)
        inflated = self._inflate_obstacles(
            occupied, msg.info.resolution, float(self.get_parameter("inflation_radius").value)
        )
        self._inflated_map_msg = self._bool_grid_to_occupancy_grid(inflated, msg)
        self.inflated_map_pub.publish(self._inflated_map_msg)

        start_cell = self._world_to_grid(self.start_xy, msg)
        goal_cell = self._world_to_grid(self.goal_xy, msg)

        if not self._is_free(inflated, start_cell):
            raise RuntimeError(
                f"Start point {self.start_xy} maps to inflated/occupied cell {start_cell}."
            )
        if not self._is_free(inflated, goal_cell):
            raise RuntimeError(
                f"Goal point {self.goal_xy} maps to inflated/occupied cell {goal_cell}."
            )

        start_time = time.perf_counter()
        if self.algorithm in ("astar", "a_star"):
            raw_path, expanded_nodes = self._a_star(inflated, msg, start_cell, goal_cell)
        else:
            raw_path, expanded_nodes = self._rrt_star(inflated, msg, start_cell, goal_cell)
        planning_time = time.perf_counter() - start_time

        if not raw_path:
            raise RuntimeError("Planner did not find a path.")

        # Ensure the path matches the assignment's exact start/goal coordinates.
        raw_path[0] = self.start_xy
        raw_path[-1] = self.goal_xy

        smoothing_enabled = bool(self.get_parameter("smoothing_enabled").value)
        if smoothing_enabled:
            smoothed_path, smoothing_method = self._smooth_path_safely(raw_path, inflated, msg)
            smoothed_path[0] = self.start_xy
            smoothed_path[-1] = self.goal_xy
        else:
            smoothed_path = list(raw_path)
            smoothing_method = "disabled"

        final_path = smoothed_path if bool(self.get_parameter("use_smoothed_path").value) else raw_path

        # publish
        # a dense polyline so the existing waypoint-based lookahead sees nearby
        # intermediate targets rather than jumping across maze corners.
        final_path = self._linear_densify(
            final_path, float(self.get_parameter("tracking_waypoint_spacing").value)
        )

        result = PlannerResult(
            raw_path=raw_path,
            smoothed_path=smoothed_path,
            planning_time_s=planning_time,
            raw_length_m=self._path_length(raw_path),
            smoothed_length_m=self._path_length(smoothed_path),
            raw_waypoints=len(raw_path),
            smoothed_waypoints=len(smoothed_path),
            expanded_nodes=expanded_nodes,
            smoothing_method=smoothing_method,
        )

        frame_id = msg.header.frame_id or "map"
        self._raw_path_msg = self._make_path_msg(raw_path, frame_id)
        self._smoothed_path_msg = self._make_path_msg(smoothed_path, frame_id)
        self._planned_path_msg = self._make_path_msg(final_path, frame_id)
        self._planned = True

        self._publish_cached_outputs()
        self.planning_time_pub.publish(Float64(data=planning_time))
        self._log_result(result)

    def _log_result(self, result: PlannerResult) -> None:
        self.get_logger().info("========== GLOBAL PLANNING RESULT ==========")
        self.get_logger().info(f"Algorithm: {self.algorithm}")
        self.get_logger().info(f"Environment: {self.environment}")
        self.get_logger().info(f"Start -> Goal: {self.start_xy} -> {self.goal_xy}")
        self.get_logger().info(f"Inflation radius: {self.get_parameter('inflation_radius').value:.3f} m")
        self.get_logger().info(f"Computation time: {result.planning_time_s:.6f} s")
        self.get_logger().info(f"Expanded/tree nodes: {result.expanded_nodes}")
        self.get_logger().info(
            f"Raw path: {result.raw_waypoints} waypoints, {result.raw_length_m:.3f} m"
        )
        self.get_logger().info(
            f"Smoothed path: {result.smoothed_waypoints} waypoints, "
            f"{result.smoothed_length_m:.3f} m, method={result.smoothing_method}"
        )
        self.get_logger().info(
            "Published topics: /planned_path, /raw_path, /smoothed_path, "
            "/inflated_costmap, /global_planning_time"
        )
        if self.algorithm not in ("astar", "a_star"):
            self.get_logger().info("RRT* tree topic: /rrt_star_tree")
        self.get_logger().info("============================================")

    def _publish_cached_outputs(self) -> None:
        if self._inflated_map_msg is not None:
            self.inflated_map_pub.publish(self._inflated_map_msg)
        if self._raw_path_msg is not None:
            self.raw_path_pub.publish(self._raw_path_msg)
        if self._smoothed_path_msg is not None:
            self.smoothed_path_pub.publish(self._smoothed_path_msg)
        if self._planned_path_msg is not None:
            self.planned_path_pub.publish(self._planned_path_msg)

    # --------------------------- Map utilities ---------------------------

    def _occupancy_grid_to_bool(self, msg: OccupancyGrid) -> np.ndarray:
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        threshold = int(self.get_parameter("occupied_threshold").value)
        occupied = data >= threshold
        if bool(self.get_parameter("treat_unknown_as_occupied").value):
            occupied = np.logical_or(occupied, data < 0)
        return occupied.astype(bool)

    @staticmethod
    def _inflate_obstacles(occupied: np.ndarray, resolution: float, radius_m: float) -> np.ndarray:
        if radius_m <= 0.0:
            return occupied.copy()

        radius_cells = int(math.ceil(radius_m / resolution))
        obstacle_y, obstacle_x = np.where(occupied)
        inflated = occupied.copy()

        offsets: List[Tuple[int, int]] = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) * resolution <= radius_m + 1e-12:
                    offsets.append((dy, dx))

        height, width = occupied.shape
        for dy, dx in offsets:
            yy = obstacle_y + dy
            xx = obstacle_x + dx
            mask = (0 <= xx) & (xx < width) & (0 <= yy) & (yy < height)
            inflated[yy[mask], xx[mask]] = True
        return inflated

    @staticmethod
    def _bool_grid_to_occupancy_grid(grid: np.ndarray, original: OccupancyGrid) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = original.header
        msg.info = original.info
        msg.data = np.where(grid, 100, 0).astype(np.int8).reshape(-1).tolist()
        return msg

    @staticmethod
    def _world_to_grid(point: Point2D, msg: OccupancyGrid) -> GridCell:
        x, y = point
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution
        gx = int(math.floor((x - origin_x) / resolution))
        gy = int(math.floor((y - origin_y) / resolution))
        return gx, gy

    @staticmethod
    def _grid_to_world(cell: GridCell, msg: OccupancyGrid) -> Point2D:
        gx, gy = cell
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution
        return origin_x + (gx + 0.5) * resolution, origin_y + (gy + 0.5) * resolution

    @staticmethod
    def _is_free(grid: np.ndarray, cell: GridCell) -> bool:
        x, y = cell
        height, width = grid.shape
        return 0 <= x < width and 0 <= y < height and not bool(grid[y, x])

    @staticmethod
    def _bresenham(start: GridCell, end: GridCell) -> Iterable[GridCell]:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _line_is_free(self, grid: np.ndarray, start: GridCell, end: GridCell) -> bool:
        return all(self._is_free(grid, cell) for cell in self._bresenham(start, end))

    def _world_line_is_free(self, grid: np.ndarray, msg: OccupancyGrid, a: Point2D, b: Point2D) -> bool:
        return self._line_is_free(grid, self._world_to_grid(a, msg), self._world_to_grid(b, msg))

    # ------------------------------ A* ------------------------------------

    def _heuristic(self, cell: GridCell, goal: GridCell, resolution: float) -> float:
        dx = abs(cell[0] - goal[0])
        dy = abs(cell[1] - goal[1])
        mode = str(self.get_parameter("heuristic").value).lower().strip()
        if mode == "manhattan":
            return (dx + dy) * resolution
        return math.hypot(dx, dy) * resolution

    def _a_star(
        self, grid: np.ndarray, msg: OccupancyGrid, start: GridCell, goal: GridCell
    ) -> Tuple[List[Point2D], int]:
        resolution = msg.info.resolution
        neighbors: Sequence[GridCell] = (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        )

        open_heap: List[Tuple[float, int, GridCell]] = []
        counter = 0
        heapq.heappush(open_heap, (self._heuristic(start, goal, resolution), counter, start))

        g_score: Dict[GridCell, float] = {start: 0.0}
        came_from: Dict[GridCell, GridCell] = {}
        closed: set[GridCell] = set()
        expanded = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                cells = self._reconstruct_grid_path(came_from, current)
                return [self._grid_to_world(cell, msg) for cell in cells], expanded

            closed.add(current)
            expanded += 1

            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._is_free(grid, nxt):
                    continue

                # Avoid diagonal corner cutting through two touching obstacles.
                if dx != 0 and dy != 0:
                    if not self._is_free(grid, (current[0] + dx, current[1])):
                        continue
                    if not self._is_free(grid, (current[0], current[1] + dy)):
                        continue

                tentative_g = g_score[current] + math.hypot(dx, dy) * resolution
                if tentative_g + 1e-12 < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    counter += 1
                    f_score = tentative_g + self._heuristic(nxt, goal, resolution)
                    heapq.heappush(open_heap, (f_score, counter, nxt))

        return [], expanded

    @staticmethod
    def _reconstruct_grid_path(came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ------------------------------ RRT* ----------------------------------

    def _rrt_star(
        self, grid: np.ndarray, msg: OccupancyGrid, start: GridCell, goal: GridCell
    ) -> Tuple[List[Point2D], int]:
        max_iterations = int(self.get_parameter("rrt_max_iterations").value)
        step_size_m = float(self.get_parameter("rrt_step_size").value)
        rewire_radius_m = float(self.get_parameter("rrt_rewire_radius").value)
        goal_sample_rate = float(self.get_parameter("rrt_goal_sample_rate").value)
        random_seed = int(self.get_parameter("rrt_random_seed").value)
        publish_stride = int(self.get_parameter("rrt_tree_publish_stride").value)

        resolution = msg.info.resolution
        step_cells = max(1, int(round(step_size_m / resolution)))
        radius_cells = max(step_cells, int(round(rewire_radius_m / resolution)))

        free_y, free_x = np.where(~grid)
        if len(free_x) == 0:
            return [], 0
        free_cells = np.column_stack((free_x, free_y)).astype(np.int32)

        rng = random.Random(random_seed)
        capacity = max_iterations + 2
        points = np.empty((capacity, 2), dtype=np.int32)
        parents = np.full(capacity, -1, dtype=np.int32)
        costs = np.full(capacity, np.inf, dtype=np.float64)

        points[0] = np.asarray(start, dtype=np.int32)
        parents[0] = -1
        costs[0] = 0.0
        node_count = 1
        best_goal_parent = -1
        best_goal_cost = float("inf")

        for iteration in range(max_iterations):
            if rng.random() < goal_sample_rate:
                sample = np.asarray(goal, dtype=np.int32)
            else:
                sample = free_cells[rng.randrange(len(free_cells))]

            nearest_idx = self._nearest_node(points, node_count, sample)
            new_cell = self._steer(points[nearest_idx], sample, step_cells)
            new_tuple = (int(new_cell[0]), int(new_cell[1]))

            if not self._is_free(grid, new_tuple):
                continue
            if not self._line_is_free(grid, tuple(points[nearest_idx]), new_tuple):
                continue
            if self._cell_already_exists(points, node_count, new_cell):
                continue

            near_indices = self._near_nodes(points, node_count, new_cell, radius_cells)
            parent_idx = nearest_idx
            parent_cost = costs[nearest_idx] + self._grid_distance(points[nearest_idx], new_cell, resolution)

            # Choose the lowest-cost valid parent.
            for idx in near_indices:
                candidate_cost = costs[idx] + self._grid_distance(points[idx], new_cell, resolution)
                if candidate_cost + 1e-12 < parent_cost:
                    if self._line_is_free(grid, tuple(points[idx]), new_tuple):
                        parent_idx = int(idx)
                        parent_cost = float(candidate_cost)

            new_idx = node_count
            points[new_idx] = new_cell
            parents[new_idx] = parent_idx
            costs[new_idx] = parent_cost
            node_count += 1

            # Rewire nearby nodes through the new node when it improves their path cost.
            for idx in near_indices:
                if idx == parent_idx:
                    continue
                candidate_cost = costs[new_idx] + self._grid_distance(points[idx], new_cell, resolution)
                if candidate_cost + 1e-12 < costs[idx]:
                    if self._line_is_free(grid, new_tuple, tuple(points[idx])):
                        parents[idx] = new_idx
                        costs[idx] = float(candidate_cost)

            distance_to_goal = self._grid_distance(new_cell, np.asarray(goal), resolution)
            if distance_to_goal <= step_size_m and self._line_is_free(grid, new_tuple, goal):
                total_cost = costs[new_idx] + distance_to_goal
                if total_cost + 1e-12 < best_goal_cost:
                    best_goal_cost = float(total_cost)
                    best_goal_parent = new_idx

            if publish_stride > 0 and iteration % publish_stride == 0:
                self._publish_rrt_tree(points, parents, node_count, msg.header.frame_id or "map", msg)

        self._publish_rrt_tree(points, parents, node_count, msg.header.frame_id or "map", msg)

        if best_goal_parent < 0:
            return [], node_count

        cells: List[GridCell] = [goal]
        idx = int(best_goal_parent)
        while idx >= 0:
            cells.append((int(points[idx, 0]), int(points[idx, 1])))
            idx = int(parents[idx])
        cells.reverse()
        return [self._grid_to_world(cell, msg) for cell in cells], node_count

    @staticmethod
    def _nearest_node(points: np.ndarray, node_count: int, sample: np.ndarray) -> int:
        diff = points[:node_count].astype(np.float32) - sample.astype(np.float32)
        distances_sq = np.einsum("ij,ij->i", diff, diff)
        return int(np.argmin(distances_sq))

    @staticmethod
    def _steer(from_cell: np.ndarray, to_cell: np.ndarray, step_cells: int) -> np.ndarray:
        vector = to_cell.astype(np.float64) - from_cell.astype(np.float64)
        distance = float(np.hypot(vector[0], vector[1]))
        if distance <= 1e-12:
            return from_cell.copy()
        scale = min(float(step_cells), distance) / distance
        return np.rint(from_cell.astype(np.float64) + vector * scale).astype(np.int32)

    @staticmethod
    def _cell_already_exists(points: np.ndarray, node_count: int, cell: np.ndarray) -> bool:
        diff = points[:node_count] - cell
        return bool(np.any(np.einsum("ij,ij->i", diff, diff) == 0))

    @staticmethod
    def _near_nodes(points: np.ndarray, node_count: int, cell: np.ndarray, radius_cells: int) -> np.ndarray:
        dx = points[:node_count, 0] - int(cell[0])
        dy = points[:node_count, 1] - int(cell[1])
        return np.where(dx * dx + dy * dy <= radius_cells * radius_cells)[0]

    @staticmethod
    def _grid_distance(a: np.ndarray, b: np.ndarray, resolution: float) -> float:
        return float(math.hypot(float(a[0] - b[0]), float(a[1] - b[1])) * resolution)

    def _publish_rrt_tree(
        self,
        points: np.ndarray,
        parents: np.ndarray,
        node_count: int,
        frame_id: str,
        map_msg: OccupancyGrid,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_star_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.8

        for idx in range(1, node_count):
            parent = int(parents[idx])
            if parent < 0:
                continue
            p0 = self._grid_to_world((int(points[parent, 0]), int(points[parent, 1])), map_msg)
            p1 = self._grid_to_world((int(points[idx, 0]), int(points[idx, 1])), map_msg)
            marker.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=0.02))
            marker.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.02))

        self.rrt_tree_pub.publish(MarkerArray(markers=[marker]))

    # ---------------------------- Smoothing -------------------------------

    def _smooth_path_safely(
        self, path: List[Point2D], grid: np.ndarray, msg: OccupancyGrid
    ) -> Tuple[List[Point2D], str]:
        if len(path) <= 2:
            return list(path), "not_needed"

        simplified = self._line_of_sight_simplify(path, grid, msg)
        if len(simplified) <= 2:
            return simplified, "line_of_sight"

        cubic = self._catmull_rom_spline(
            simplified, float(self.get_parameter("spline_resolution").value)
        )
        if self._path_is_safe(cubic, grid, msg):
            return cubic, "catmull_rom_cubic_spline"

        # Safety is more important than a pretty curve. If a cubic segment cuts
        # a corner into the inflated costmap, keep the collision-free simplified
        # path and report that fallback explicitly.
        return simplified, "line_of_sight_safety_fallback"

    def _line_of_sight_simplify(
        self, path: List[Point2D], grid: np.ndarray, msg: OccupancyGrid
    ) -> List[Point2D]:
        if len(path) <= 2:
            return list(path)

        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._world_line_is_free(grid, msg, path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified

    @staticmethod
    def _catmull_rom_spline(path: List[Point2D], resolution_m: float) -> List[Point2D]:
        if len(path) < 4:
            return GlobalPlannerNode._linear_densify(path, resolution_m)

        pts = [path[0]] + list(path) + [path[-1]]
        output: List[Point2D] = [path[0]]

        for i in range(1, len(pts) - 2):
            p0 = np.asarray(pts[i - 1], dtype=np.float64)
            p1 = np.asarray(pts[i], dtype=np.float64)
            p2 = np.asarray(pts[i + 1], dtype=np.float64)
            p3 = np.asarray(pts[i + 2], dtype=np.float64)

            segment_length = float(np.linalg.norm(p2 - p1))
            samples = max(2, int(math.ceil(segment_length / max(resolution_m, 1e-3))))
            for k in range(1, samples + 1):
                t = float(k) / float(samples)
                t2 = t * t
                t3 = t2 * t
                p = 0.5 * (
                    (2.0 * p1)
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
                )
                output.append((float(p[0]), float(p[1])))
        return output

    @staticmethod
    def _linear_densify(path: List[Point2D], resolution_m: float) -> List[Point2D]:
        if len(path) <= 1:
            return list(path)
        output: List[Point2D] = [path[0]]
        for a, b in zip(path[:-1], path[1:]):
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            samples = max(1, int(math.ceil(distance / max(resolution_m, 1e-3))))
            for k in range(1, samples + 1):
                t = float(k) / float(samples)
                output.append((a[0] * (1.0 - t) + b[0] * t, a[1] * (1.0 - t) + b[1] * t))
        return output

    def _path_is_safe(self, path: List[Point2D], grid: np.ndarray, msg: OccupancyGrid) -> bool:
        if len(path) <= 1:
            return True
        for a, b in zip(path[:-1], path[1:]):
            if not self._world_line_is_free(grid, msg, a, b):
                return False
        return True

    @staticmethod
    def _path_length(path: List[Point2D]) -> float:
        if len(path) <= 1:
            return 0.0
        return float(
            sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
        )

    def _make_path_msg(self, path: List[Point2D], frame_id: str) -> Path:
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_astar(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GlobalPlannerNode(forced_algorithm="astar")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_rrt_star(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GlobalPlannerNode(forced_algorithm="rrt_star")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
