#!/usr/bin/env python3
from all_seaing_navigation.path_planner import PathPlanner

from typing import List
from geometry_msgs.msg import Point, Pose, PoseArray

import math
import heapq


class AStar(PathPlanner):

    def heuristic(self, gp: Point) -> float:
        return math.hypot(gp.x - self.grid_goal.x, gp.y - self.grid_goal.y)

    def get_path(self, end: Point, parent: List[Point]) -> PoseArray:
        ans = PoseArray(header=self.map.header)
        curr = Pose(position=end)
        while curr != self.grid_start:
            ans.poses.append(curr)
            curr = Pose(position=parent[self.get_grid_index(curr)])
        ans.poses.append(curr)
        ans.poses.reverse()
        return ans

    def plan(self) -> PoseArray:
        # Neighbor offsets
        dxy = [
            Point(x=1.0, y=0.0),
            Point(x=0.0, y=1.0),
            Point(x=-1.0, y=0.0),
            Point(x=0.0, y=-1.0),
            Point(x=1.0, y=1.0),
            Point(x=-1.0, y=-1.0),
            Point(x=-1.0, y=1.0),
            Point(x=1.0, y=-1.0),
            Point(x=1.0, y=-2.0),
            Point(x=1.0, y=2.0),
            Point(x=-1.0, y=2.0),
            Point(x=-1.0, y=-2.0),
            Point(x=2.0, y=1.0),
            Point(x=-2.0, y=1.0),
            Point(x=2.0, y=-1.0),
            Point(x=-2.0, y=-1.0),
        ]

        # Matrix of scores and parent pointers for each cell
        gscore = [math.inf] * (self.map.info.height * self.map.info.width)
        parent = [Point(x=0.0, y=0.0)] * (self.map.info.height * self.map.info.width)

        # Check if the starting position and/or the ending position is occupied
        if (self.is_grid_occupied(self.grid_start) or self.is_grid_occupied(self.grid_goal)):
            return PoseArray(header=self.map.header)

        gscore[self.get_grid_index(self.grid_start)] = 0
        parent[self.get_grid_index(self.grid_start)] = self.grid_start
        pq = []
        heapq.heappush(pq, (self.heuristic(self.grid_start), self.grid_start))
        while pq:
            curr_score, curr_pos = heapq.heappop(pq)
            #TODO: WHAT IS THIS MAGIC VALUE
            if abs(curr_score - gscore[self.get_grid_index(curr_pos)] + self.heuristic(curr_pos)) > 0.005:
                continue

            if self.is_goal_reached(curr_pos):
                return self.get_path(curr_pos, parent)

            for d in dxy:
                nxt = Point(x=curr_pos.x+d.x, y=curr_pos.y+d.y)

                if not self.is_in_bounds(nxt) or self.is_rect_occupied(curr_pos, nxt):
                    continue
                
                if gscore[self.get_grid_index(curr_pos)] + math.hypot(d.x, d.y) < gscore[self.get_grid_index(nxt)]:
                    gscore[self.get_grid_index(nxt)] = gscore[self.get_grid_index(curr_pos)] + math.hypot(d.x, d.y)
                    parent[self.get_grid_index(nxt)] = curr_pos
                    heapq.heappush(pq, (gscore[self.get_grid_index(nxt)] + self.heuristic(nxt), nxt))

        return PoseArray(header=self.map.header)
