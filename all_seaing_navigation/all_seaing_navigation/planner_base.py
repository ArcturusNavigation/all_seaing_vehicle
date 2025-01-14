from abc import ABC, abstractmethod

from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.logging import get_logger

import math


class PlannerBase(ABC):
    def __init__(
        self,
        map: OccupancyGrid,
        start: Point,
        goal: Point,
        obstacle_tol,
        goal_tol,
    ):
        self.map = map
        self.world_start = start
        self.world_goal = goal
        self.grid_start = self.world_to_grid(start)
        self.grid_goal = self.world_to_grid(goal)
        self.obstacle_tol = obstacle_tol
        self.goal_tol = goal_tol
        self.logger = get_logger("path_planner")

    @abstractmethod
    def plan(self) -> PoseArray:
        pass

    def world_to_grid(self, wp: Point) -> Point:
        """Convert world coordinates to grid coordinates"""
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        gx = float((wp.x - origin.x) // resolution)
        gy = float((wp.y - origin.y) // resolution)
        return Point(x=gx, y=gy)

    def grid_to_world(self, gp: Point) -> Point:
        """Convert grid coordinates back to world coordinates"""
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        wx = gp.x * resolution + origin.x
        wy = gp.y * resolution + origin.y
        return Point(x=wx, y=wy)

    def is_grid_occupied(self, gp: Point) -> bool:
        return self.get_grid_val(gp) >= self.obstacle_tol or self.get_grid_val(gp) == -1

    def is_rect_occupied(self, gp1: Point, gp2: Point) -> bool:
        for tx in range(int(min(gp1.x, gp2.x)), int(max(gp1.x, gp2.x) + 1)):
            for ty in range(int(min(gp1.y, gp2.y)), int(max(gp1.y, gp2.y) + 1)):
                if self.is_grid_occupied(Point(x=float(tx), y=float(ty))):
                    return True
        return False

    def is_goal_reached(self, gp: Point) -> bool:
        return (
            math.hypot(gp.x - self.grid_goal.x, gp.y - self.grid_goal.y) < self.goal_tol
        )

    def is_in_bounds(self, gp: Point) -> bool:
        return 0 <= gp.x < self.map.info.width and 0 <= gp.y < self.map.info.height

    def get_grid_val(self, gp: Point) -> int:
        return self.map.data[int(gp.x + gp.y * self.map.info.width)]

    def get_grid_index(self, gp: Point) -> int:
        return int(gp.x + gp.y * self.map.info.width)
