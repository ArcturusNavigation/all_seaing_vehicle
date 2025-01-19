from all_seaing_navigation.a_star import AStar
from rclpy.logging import get_logger


class PlannerExecutor:
    def __init__(self, planner_name):
        if planner_name == "astar":
            self.planner = AStar
        elif planner_name == "rrt":
            raise NotImplementedError
        else:
            get_logger("planner_executor").error("Unrecognized planner name")
            raise ValueError

    def plan(self, map, start, goal, obstacle_tol=50, goal_tol=0.5):
        return self.planner(map, start, goal, obstacle_tol, goal_tol).plan()
