#!/usr/bin/env python3
"""
Speed Challenge task server (autonomy onboarding, part 2).

The boat passes through a red/green gate, circles a yellow buoy, and exits
through the same gate. A color indicator buoy says which way to circle:
  - red indicator   -> circle the yellow buoy from the right (counter-clockwise)
  - green indicator -> circle the yellow buoy from the left (clockwise)

Your job: fill in every block marked TODO. Search the file for "TODO".
"""
import os
from enum import Enum

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.report_pb2 import Color, GatePass, GateType, ObjectDetected, ObjectType, TaskType
from all_seaing_common.task_server_base import TaskServerBase
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import ObstacleMap

# Helpers you will probably want (see all_seaing_autonomy/buoy_utils.py):
#   ob_coords(obstacle)               -> np.array([x, y]) of an obstacle in the map frame
#   midpoint_pair_dir(pair, fwd_dist) -> (point, direction) for a buoy pair
from all_seaing_autonomy.buoy_utils import midpoint_pair_dir, ob_coords


# Each task is a small state machine. control_loop() runs the step for the
# current state, then moves self.state to the next one.
class SpeedChallengeState(Enum):
    GATES = 1         # drive through the entry gate
    PROBING_BUOY = 2  # drive forward until the yellow buoy is seen
    CIRCLING = 3      # circle the yellow buoy
    RETURNING = 4     # drive back out through the gate


# Every competition task inherits from TaskServerBase. The base class turns this
# node into a ROS 2 action server and calls these hooks for you, in this order:
#   1. should_accept_task() once, when run_tasks.py asks us to start
#   2. init_setup() repeatedly, until it calls mark_successful()
#   3. control_loop() repeatedly, until it calls mark_successful()/mark_unsuccessful()
# Read all_seaing_common/task_server_base.py to see the helpers you inherit.
class SpeedChallenge(TaskServerBase):
    def __init__(self):
        super().__init__(
            server_name="speed_challenge_server",
            action_name="speed_challenge",
            search_action_name="search_speed",
        )

        # The tracked obstacle map from perception. Each Obstacle has a .label
        # (an int class id), a .id (tracking id), and a position.
        self.obstacles = None
        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        # Parameters are set in all_seaing_bringup/launch/tasks.launch.py (real boat)
        # and sim.launch.py (simulation). The values here are only defaults.
        self.beacon_dist_thres = self.declare_and_get("beacon_dist_thres", 15.0)
        self.probe_distance = self.declare_and_get("probe_distance", 10.0)
        self.adaptive_distance = self.declare_and_get("adaptive_distance", 0.7)
        self.duplicate_dist = self.declare_and_get("duplicate_dist", 0.5)
        self.init_gate_dist = self.declare_and_get("init_gate_dist", 1.0)
        self.gate_dist_back = self.declare_and_get("gate_dist_back", 1.0)

        self.load_labels()
        self.load_location()

        self.state = SpeedChallengeState.GATES
        self.left_first = False  # True = circle clockwise (green indicator)

    def declare_and_get(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def load_labels(self):
        '''
        Fills the label sets used to tell buoys apart.
        Check membership with: obstacle.label in self.red_labels
        '''
        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        mappings_file = self.declare_and_get(
            "color_label_mappings_file",
            os.path.join(bringup_prefix, "config", "perception", "color_label_mappings.yaml"),
        )
        with open(mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)

        self.red_labels = set()
        self.green_labels = set()
        self.yellow_labels = set()
        self.red_indicator_labels = set()
        self.green_indicator_labels = set()

        # The simulator and the real YOLO model use different label names.
        if self.is_sim:
            self.red_labels.add(label_mappings["red"])
            self.green_labels.add(label_mappings["green"])
            self.yellow_labels.add(label_mappings["yellow"])
            return

        for name in ["red_buoy", "red_circle", "red_racquet_ball"]:
            self.red_labels.add(label_mappings[name])
        for name in ["green_buoy", "green_circle"]:
            self.green_labels.add(label_mappings[name])
        for name in ["yellow_buoy", "yellow_racquet_ball"]:
            self.yellow_labels.add(label_mappings[name])
        self.red_indicator_labels.add(label_mappings["red_indicator"])
        self.green_indicator_labels.add(label_mappings["green_indicator"])

    def load_location(self):
        # GPS origin of the course, used to convert map (x, y) to lat/lng for reports.
        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        locations_file = self.declare_and_get(
            "latlng_locations_file",
            os.path.join(bringup_prefix, "config", "localization", "locations.yaml"),
        )
        with open(locations_file, "r") as f:
            latlng_location_mappings = yaml.safe_load(f)
        self.location = self.declare_and_get("location", "nbpark")
        self.latlng_origin = latlng_location_mappings[self.location]

    # ------------------------------------------------------------------
    # TaskServerBase hooks
    # ------------------------------------------------------------------

    def should_accept_task(self, goal_request):
        # Only start if we can already see the entry gate.
        # setup_buoys() (from the base class) finds the closest red/green pair
        # and stores it in self.gate_pair (with .left and .right obstacles).
        if self.obstacles is None:
            return False
        return self.setup_buoys()

    def init_setup(self):
        # Reset per-run state, since the same node can run the task more than once.
        self.get_logger().info("Setup buoys succeeded!")
        self.state = SpeedChallengeState.GATES
        self.circling_buoy_found = False  # used by buoy_detected() in the base class
        self.moved_to_point = False
        self.indicator_color = None
        self.turn_pid.reset()
        self.mark_successful()

    def control_loop(self):
        # Each step blocks until it finishes (busy_wait=True), so one call
        # of control_loop() runs one whole state.
        if self.state == SpeedChallengeState.GATES:
            result = self.pass_entry_gate()
            self.state = SpeedChallengeState.PROBING_BUOY
        elif self.state == SpeedChallengeState.PROBING_BUOY:
            result = self.find_yellow_buoy()
            self.state = SpeedChallengeState.CIRCLING
        elif self.state == SpeedChallengeState.CIRCLING:
            result = self.circle_yellow_buoy()
            self.state = SpeedChallengeState.RETURNING
        elif self.state == SpeedChallengeState.RETURNING:
            result = self.return_to_start()
            if result.success:
                self.mark_successful()

        # If any step fails or the task is cancelled, end the task.
        if not result.success or self.goal_handle.is_cancel_requested:
            self.mark_unsuccessful()

    # ------------------------------------------------------------------
    # Perception callback
    # ------------------------------------------------------------------

    def map_cb(self, msg):
        self.obstacles = msg.obstacles
        if self.paused:
            return

        # TODO (step 3): look for the color indicator while driving toward the yellow buoy.
        #   - Call self.identify_indicator() here.
        #   - Only do it before circling starts (check self.state), so the direction
        #     can't change halfway through the circle.
        
        if self.state not in (SpeedChallengeState.CIRCLING, SpeedChallengeState.RETURNING):
            self.identify_indicator()


    def identify_indicator(self):
        '''
        Sets self.left_first from the color indicator buoy and reports its color once.
        '''
        # TODO (step 3):
        #   1. Loop over self.obstacles. Skip any farther than self.beacon_dist_thres
        #      from self.robot_pos (np.linalg.norm and ob_coords help here).
        #   2. If the label is in self.red_indicator_labels:   circle counter-clockwise.
        #      If the label is in self.green_indicator_labels: circle clockwise.
        #      Set self.left_first to match (True = clockwise).
        #   3. The first time you see a color, report it to the judges:
        #        self.report_data(ObjectDetected(
        #            object_type=ObjectType.OBJECT_LIGHT_BEACON,
        #            color=Color.COLOR_RED,  # or Color.COLOR_GREEN
        #            position=self.pos_to_latlng(self.latlng_origin, ob_coords(obstacle)),
        #            object_id=obstacle.id,
        #            task_context=TaskType.TASK_SPEED_CHALLENGE))
        #      Use self.indicator_color to remember that you already reported.

        for obstacle in self.obstacles:
            dist = np.linalg.norm(self.robot_pos - ob_coords(obstacle))
            if (dist < self.beacon_dist_thres):
                #skips if farther than thres
                if obstacle.label in self.red_indicator_labels:
                    self.left_first, color = False, Color.COLOR_RED
                elif obstacle.label in self.green_indicator_labels:
                    self.left_first, color = True, Color.COLOR_GREEN
                else:
                    continue #not in either 

                if (color != self.indicator_color):
                    #make sure it's first time seeing colour

                    self.indicator_color = color


                    self.report_data(ObjectDetected(
                    object_type=ObjectType.OBJECT_LIGHT_BEACON,
                    color=color,
                    position=self.pos_to_latlng(self.latlng_origin, ob_coords(obstacle)),
                    object_id=obstacle.id,
                    task_context=TaskType.TASK_SPEED_CHALLENGE))


        #pass

    # ------------------------------------------------------------------
    # Task steps: one method per state
    # ------------------------------------------------------------------

    def gate_wpt_updater(self, forward_dist):
        '''
        Returns a goal_update_func for move_to_point() that keeps the gate waypoint
        on the latest buoy positions while the boat drives.
        '''
        # A lambda reads self.obstacles each time it runs, so it always sees the
        # newest map. functools.partial(f, self.obstacles) would freeze the list
        # that existed when partial() was called.
        update_func = lambda: self.update_gate_wpt_pos(
            self.obstacles, self.green_labels, self.red_labels,
            self.duplicate_dist, forward_dist,
        )
        return lambda: self.update_point("gate_wpt", self.adaptive_distance, update_func)

    def pass_entry_gate(self):
        '''
        Drive through the gate: first to a point just before it, then just past it.
        '''
        # midpoint_pair_dir(pair, d) gives a point d meters from the middle of the pair,
        # along the direction the boat should pass through it.
        #   d < 0 -> on the side the boat starts on
        #   d > 0 -> on the far side of the gate
        # It also returns that direction, which we save for probing later.

        # Worked example: drive to a point just before the gate.
        self.gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, -self.init_gate_dist)
        self.get_logger().info("Going to the near side of the gate")
        self.move_to_point(
            self.gate_wpt,
            busy_wait=True,  # block until we arrive
            goal_update_func=self.gate_wpt_updater(-self.init_gate_dist),
        )
        if self.goal_handle.is_cancel_requested:
            return Task.Result(success=False)

        # TODO (step 1): drive to a point just past the gate.
        #   - Follow the worked example above, using +self.init_gate_dist.
        #   - Check for cancellation afterwards, like above.

        #need d>0 so
        self.gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, self.init_gate_dist)
        self.get_logger().info("Going to far side of gate")

        self.move_to_point(
            self.gate_wpt,
            busy_wait = True,
            goal_update_func = self.gate_wpt_updater(self.init_gate_dist)
        )

        if self.goal_handle.is_cancel_requested:
            return Task.Result(success=False)

        # TODO (step 1): report that we entered the gate.
        #   self.report_data(GatePass(type=GateType.GATE_SPEED_START, position=...))
        #   Use self.pos_to_latlng(self.latlng_origin, self.robot_pos) for the position.

        self.report_data(GatePass(type=GateType.GATE_SPEED_START, position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

        return Task.Result(success=True)

    def find_yellow_buoy(self):
        '''
        Drive forward through the gate until the yellow buoy shows up in the map.
        '''
        # TODO (step 2): use the base class helper probe_buoy().
        #   probe_buoy(direction, distance, detected_func)
        #     - direction: self.buoy_direction (saved in pass_entry_gate)
        #     - distance:  self.probe_distance
        #     - detected_func(buoy_front) -> bool. probe_buoy stops driving once it returns True.
        #   The base class already has buoy_detected(obstacles, buoy_labels, duplicate_dist, buoy_front).
        #   Wrap it in a lambda that takes buoy_front and passes self.obstacles, self.yellow_labels,
        #   and self.duplicate_dist.
        #   Return whatever probe_buoy() returns.



        #self.get_logger().warn("find_yellow_buoy() is not implemented yet")
        #return Task.Result(success=False)

        YellowB_detected = lambda buoy_front: self.buoy_detected(
            self.obstacles,
            self.yellow_labels,
            self.duplicate_dist,
            buoy_front
        )

        return self.probe_buoy(self.buoy_direction, self.probe_distance, YellowB_detected)


    def circle_yellow_buoy(self):
        '''
        Circle the yellow buoy once, in the direction the indicator asked for.
        '''
        # TODO (step 4): use the base class helper smooth_circle_buoy().
        #   smooth_circle_buoy(buoy_labels, adaptive_distance, duplicate_dist, target_pt,
        #                      buoy_loop_count, left_first)
        #     - buoy_labels: the yellow labels
        #     - target_pt:   where to point when the loop is done (the gate waypoint)
        #     - buoy_loop_count: 1
        #     - left_first:  set by identify_indicator()
        #   Log which direction you chose. It makes debugging on the water much easier.

        self.get_logger().info(f"Left_first is: {self.left_first}")

        return self.smooth_circle_buoy(
        self.yellow_labels, 
        self.adaptive_distance, 
        self.duplicate_dist,
        self.gate_wpt, 
        1, 
        self.left_first,
        )




        #self.get_logger().warn("circle_yellow_buoy() is not implemented yet")
        #return Task.Result(success=False)

    def return_to_start(self):
        '''
        Drive back out through the entry gate.
        '''
        self.get_logger().info("Returning to start")

        # TODO (step 5): flip the gate so "left" and "right" match the boat's new heading.
        #   - Toggle self.red_left.
        #   - Swap self.gate_pair.left and self.gate_pair.right.

        self.red_left = not self.red_left
        self.gate_pair.left, self.gate_pair.right = self.gate_pair.right, self.gate_pair.left

        # TODO (step 5): look for the gate again on the way back.
        #   - Get a point self.gate_dist_back before the gate with midpoint_pair_dir().
        #   - move_to_point() to it with exit_func=self.setup_buoys. This stops early
        #     once setup_buoys() finds the gate and refreshes self.gate_pair.


        probe_wpt, probe_direction = midpoint_pair_dir(self.gate_pair, -self.gate_dist_back)
        self.move_to_point(
            probe_wpt,
            busy_wait = True,
            exit_func = self.setup_buoys
        )


        # TODO (step 5): pass through the gate, the same way as in pass_entry_gate(),
        #   using -self.gate_dist_back and then +self.gate_dist_back.

        

        self.gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, -self.gate_dist_back)
        self.get_logger().info("Going to the near side of the gate")
        self.move_to_point(
            self.gate_wpt,
            busy_wait=True,  # block until we arrive
            goal_update_func=self.gate_wpt_updater(-self.gate_dist_back),
        )
        if self.goal_handle.is_cancel_requested:
            return Task.Result(success=False)


        #need d>0 so
        self.gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, self.gate_dist_back)
        self.get_logger().info("Going to far side of gate")

        self.move_to_point(
            self.gate_wpt,
            busy_wait = True,
            goal_update_func = self.gate_wpt_updater(self.gate_dist_back)
        )

        if self.goal_handle.is_cancel_requested:
            return Task.Result(success=False)


        # TODO (step 5): report that we exited the gate (GateType.GATE_SPEED_END).

        #self.get_logger().warn("return_to_start() is not implemented yet")
        #return Task.Result(success=False)

        self.report_data(GatePass(
            type=GateType.GATE_SPEED_END,
            position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))
        return Task.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedChallenge()
    # Two threads: one runs the task, the other keeps receiving map/odometry
    # messages while the task is blocked in move_to_point().
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
