from all_seaing_interfaces.srv import PlanPath
from geometry_msgs.msg import PoseArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.logging import get_logger

import time
import rclpy


class PlannerExecutor:
    def __init__(self, node):
        self.node = node
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.client = node.create_client(
            PlanPath, "plan_path", callback_group=self.cb_group
        )

    def plan(self, start, goal, obstacle_tol=50, goal_tol=0.5, should_abort=lambda: False):
        if not self.client.wait_for_service(timeout_sec=2.0):
            get_logger("planner_executor").error("plan_path service not available")
            return PoseArray()

        req = PlanPath.Request()
        req.planner = "astar"
        req.start = start
        req.goal = goal
        req.obstacle_tol = obstacle_tol
        req.goal_tol = float(goal_tol)

        future = self.client.call_async(req)
        while rclpy.ok() and not future.done() and not should_abort():
            time.sleep(0.05)

        if not future.done():
            future.cancel()
            return PoseArray()

        return future.result().path
