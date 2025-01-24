from all_seaing_navigation.planner_base import PlannerBase

from typing import List
from geometry_msgs.msg import Point, Pose, PoseArray

from collections import defaultdict
import math
import heapq


class PQNode:
    def __init__(self, score: float, point: Point):
        self.score = score
        self.point = point

    def __lt__(self, other):
        return self.score < other.score


class AStar(PlannerBase):

    def heuristic(self, gp: Point) -> float:
        return math.hypot(gp.x - self.grid_goal.x, gp.y - self.grid_goal.y)

    def get_path(self, end: Point, parent: defaultdict[Point]) -> PoseArray:
        ans = PoseArray(header=self.map.header)
        curr = Pose(position=end)
        while curr.position != self.grid_start:
            ans.poses.append(Pose(position=self.grid_to_world(curr.position)))
            curr = Pose(position=parent[self.get_grid_index(curr.position)])
        ans.poses.append(Pose(position=self.grid_to_world(curr.position)))
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
        gscore = [math.inf for _ in range(self.map.info.height * self.map.info.width)]
        parent = [Point() for _ in range(self.map.info.height * self.map.info.width)]
        gscore = defaultdict(lambda: math.inf)
        parent = defaultdict(Point)

        # Check if the starting and end positions are invalid
        if (
            not self.is_in_bounds(self.grid_start)
            or not self.is_in_bounds(self.grid_goal)
            or self.is_grid_occupied(self.grid_start)
        ):
            return PoseArray(header=self.map.header)

        gscore[self.get_grid_index(self.grid_start)] = 0
        parent[self.get_grid_index(self.grid_start)] = self.grid_start
        pq = []
        heapq.heappush(pq, PQNode(self.heuristic(self.grid_start), self.grid_start))
        while pq:
            node = heapq.heappop(pq)
            curr_score = node.score
            curr_pos = node.point

            # Checks if the current node has the best A* heuristic
            epsilon = 0.005
            if abs(curr_score - (gscore[self.get_grid_index(curr_pos)] + self.heuristic(curr_pos))) > epsilon:
                continue

            if self.is_goal_reached(curr_pos):
                return self.get_path(curr_pos, parent)

            for d in dxy:
                nxt = Point(x=curr_pos.x + d.x, y=curr_pos.y + d.y)

                if not self.is_in_bounds(nxt) or self.is_rect_occupied(curr_pos, nxt):
                    continue

                if (
                    gscore[self.get_grid_index(curr_pos)] + math.hypot(d.x, d.y)
                    < gscore[self.get_grid_index(nxt)]
                ):
                    gscore[self.get_grid_index(nxt)] = gscore[
                        self.get_grid_index(curr_pos)
                    ] + math.hypot(d.x, d.y)
                    parent[self.get_grid_index(nxt)] = curr_pos
                    heapq.heappush(
                        pq,
                        PQNode(
                            gscore[self.get_grid_index(nxt)] + self.heuristic(nxt), nxt
                        ),
                    )

        return PoseArray(header=self.map.header)
