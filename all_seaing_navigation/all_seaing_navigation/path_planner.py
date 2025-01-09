from abc import ABC, abstractmethod

from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData

import math


class PathPlanner(ABC):

    def __init__(self, map_info: MapMetaData, map_grid: OccupancyGrid,
                 start: Pose, goal: Pose, obstacle_tol=50, goal_tol=1.0):
        self.map_info = map_info
        self.map_grid = map_grid
        self.world_start = start
        self.world_goal = goal
        self.grid_start = self.world_to_grid(start)
        self.grid_goal = self.world_to_grid(goal)
        self.obstacle_tol = obstacle_tol
        self.goal_tol = goal_tol

    @abstractmethod
    def plan(self) -> PoseArray:
        pass

    def world_to_grid(self, wp: Point) -> Point:
        """Convert world coordinates to grid coordinates"""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        gx = float((wp.x - origin.x) // resolution)
        gy = float((wp.y - origin.y) // resolution)
        return Point(x=gx, y=gy)

    def grid_to_world(self, gp: Point) -> Point:
        """Convert grid coordinates back to world coordinates"""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        wx = gp.x * resolution + origin.x
        wy = gp.y * resolution + origin.y
        return Point(x=wx, y=wy)

    def is_grid_occupied(self, gp: Point) -> bool:
        grid_val = self.map_grid[gp.x + gp.y * self.map_info.width]
        return grid_val >= self.obstacle_tol or grid_val == -1

    def is_rect_occupied(self, gp1: Point, gp2: Point) -> bool:
        for tx in range(min(gp1.x, gp2.x), max(gp1.x, gp2.x) + 1):
            for ty in range(min(gp1.y, gp2.y), max(gp1.y, gp2.y) + 1):
                if self.is_grid_occupied(Point(x=tx, y=ty)):
                    return True
        return False
    
    def is_goal_reached(self, gp: Point) -> bool:
        return math.hypot(gp.x - self.goal.x, gp.y - self.goal.y) < self.goal_tol

    def is_in_bounds(self, gp: Point) -> bool:
        return 0 <= gp.x < self.map_info.width and 0 <= gp.y < self.map_info.height

    def get_grid_val(self, gp: Point) -> int:
        return self.map_grid[int(gp.x + gp.y * self.map_info.width)]

    def get_grid_index(self, gp: Point) -> int:
        return int(gp.x + gp.y * self.map_info.width)
