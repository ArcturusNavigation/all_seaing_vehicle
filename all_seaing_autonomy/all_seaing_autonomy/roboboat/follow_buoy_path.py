#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase

import math
import os
import yaml
import time

class InternalBuoyPair:
    def __init__(self, left_buoy=None, right_buoy=None):
        if left_buoy is None:
            self.left = Obstacle()
        else:
            self.left = left_buoy

        if right_buoy is None:
            self.right = Obstacle()
        else:
            self.right = right_buoy


class FollowBuoyPath(ActionServerBase):
    def __init__(self):
        super().__init__("follow_path_server")

        self._action_server = ActionServer(
            self,
            Task,
            "follow_buoy_path",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,

        )

        self.map_sub = self.create_subscription(
            ObstacleMap, "/obstacle_map/labeled", self.map_cb, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("xy_threshold", 2.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 5)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        self.robot_pos = (0, 0)

        self.declare_parameter("safe_margin", 0.2)

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.declare_parameter(
            "color_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )

        self.first_map = True

        self.safe_margin = (
            self.get_parameter("safe_margin").get_parameter_value().double_value
        )

        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            self.color_label_mappings = yaml.safe_load(f)

        self.sent_waypoints = set()

        self.red_left = True
        self.result = False
        self.timer_period = 1/60
        self.time_last_seen_buoys = time.time()

        self.obstacles = None

    def ob_coords(self, buoy, local=False):
        if local:
            return (buoy.local_point.point.x, buoy.local_point.point.y)
        else:
            return (buoy.global_point.point.x, buoy.global_point.point.y)

    def get_closest_to(self, source, buoys, local=False):
        return min(
            buoys,
            key=lambda buoy: math.dist(source, self.ob_coords(buoy, local)),
        )

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def midpoint_pair(self, pair):
        return self.midpoint(self.ob_coords(pair.left), self.ob_coords(pair.right))

    def split_buoys(self, obstacles):
        """
        Splits the buoys into red and green based on their labels in the obstacle map
        """
        green_bouy_points = []
        red_bouy_points = []
        for obstacle in obstacles:
            if obstacle.label == self.color_label_mappings["green"]:
                green_bouy_points.append(obstacle)
            elif obstacle.label == self.color_label_mappings["red"]:
                red_bouy_points.append(obstacle)
        return green_bouy_points, red_bouy_points

    def obs_to_pos(self, obs):
        return [self.ob_coords(ob, local=False) for ob in obs]

    def obs_to_pos_label(self, obs):
        return [self.ob_coords(ob, local=False) + (ob.label,) for ob in obs]

    def buoy_pairs_to_markers(self, buoy_pairs):
        """
        Create the markers from an array of buoy pairs to visualize them (and the respective waypoints) in RViz
        """
        marker_array = MarkerArray()
        i = 0
        for buoy_pair in buoy_pairs.pairs:
            marker_array.markers.append(
                Marker(
                    type=Marker.ARROW,
                    pose=buoy_pair.waypoint.point,
                    header=Header(frame_id="odom"),
                    scale=Vector3(x=1.0, y=0.05, z=0.05),
                    color=ColorRGBA(a=1.0),
                    id=(4 * i),
                )
            )
            if self.red_left:
                left_color = ColorRGBA(r=1.0, a=1.0)
                right_color = ColorRGBA(g=1.0, a=1.0)
            else:
                left_color = ColorRGBA(g=1.0, a=1.0)
                right_color = ColorRGBA(r=1.0, a=1.0)

            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(buoy_pair.left)),
                    header=Header(frame_id="odom"),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=left_color,
                    id=(4 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(buoy_pair.right)),
                    header=Header(frame_id="odom"),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=right_color,
                    id=(4 * i) + 2,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.CYLINDER,
                    pose=Pose(
                        position=Point(
                            x=buoy_pair.waypoint.point.position.x,
                            y=buoy_pair.waypoint.point.position.y,
                        )
                    ),
                    header=Header(frame_id="odom"),
                    scale=Vector3(
                        x=buoy_pair.waypoint.radius, y=buoy_pair.waypoint.radius, z=1.0
                    ),
                    color=ColorRGBA(g=1.0, a=0.5),
                    id=(4 * i) + 3,
                )
            )
            i += 1
        return marker_array

    def setup_buoys(self):
        """
        Runs when the first obstacle map is received, filters the buoys that are in front of
        the robot (x>0 in local coordinates) and finds (and stores) the closest green one and
        the closest red one, and because the robot is in the starting position these
        are the front buoys of the robot starting box.
        """
        self.get_logger().debug("Setting up starting buoys!")
        self.get_logger().debug(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )

        # Split all the buoys into red and green
        green_init, red_init = self.split_buoys(self.obstacles)

        # lambda function that filters the buoys that are in front of the robot
        obstacles_in_front = lambda obs: [
            ob for ob in obs if ob.local_point.point.x > 0
        ]
        # take the green and red buoys that are in front of the robot
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().debug(
            f"initial red buoys: {red_buoys}, green buoys: {green_buoys}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            self.get_logger().debug("No starting buoy pairs!")
            return False

        # From the red buoys that are in front of the robot, take the one that is closest to it.
        # And do the same for the green buoys.
        # This pair is the front pair of the starting box of the robot.
        closest_red = self.get_closest_to((0, 0), red_buoys, local=True)
        closest_green = self.get_closest_to((0, 0), green_buoys, local=True)
        if self.ccw(
            (0, 0),
            self.ob_coords(closest_green, local=True),
            self.ob_coords(closest_red, local=True),
        ):
            self.red_left = True
            self.starting_buoys = InternalBuoyPair(
                self.get_closest_to((0, 0), red_buoys, local=True),
                self.get_closest_to((0, 0), green_buoys, local=True),
            )
            self.get_logger().debug("RED BUOYS LEFT, GREEN BUOYS RIGHT")
        else:
            self.red_left = False
            self.starting_buoys = InternalBuoyPair(
                self.get_closest_to((0, 0), green_buoys, local=True),
                self.get_closest_to((0, 0), red_buoys, local=True),
            )
            self.get_logger().debug("GREEN BUOYS LEFT, RED BUOYS RIGHT")
        self.pair_to = self.starting_buoys
        return True

    def ccw(self, a, b, c):
        """Return True if the points a, b, c are counterclockwise, respectively"""
        area = (
            a[0] * b[1]
            + b[0] * c[1]
            + c[0] * a[1]
            - a[1] * b[0]
            - b[1] * c[0]
            - c[1] * a[0]
        )
        return area > 0

    def filter_front_buoys(self, pair, buoys):
        """
        Returns the buoys (from the given array) that are in front of a pair of points,
        considering the forward direction to be the one such that
        the first point of the pair is in the left and the second is in the right
        """
        # (red, green)
        return [
            buoy
            for buoy in buoys
            if self.ccw(
                self.ob_coords(pair.left),
                self.ob_coords(pair.right),
                self.ob_coords(buoy),
            )
        ]

    def next_pair(self, prev_pair, red, green):
        """
        Returns the next buoy pair from the previous pair,
        by checking the closest one to the middle of the previous buoy pair that's in front of the pair
        """

        front_red = self.filter_front_buoys(prev_pair, red)
        front_green = self.filter_front_buoys(prev_pair, green)

        if not front_red or not front_green:
            prev_coords = self.ob_coords(prev_pair.left), self.ob_coords(
                prev_pair.right
            )
            red_coords = self.obs_to_pos(red)
            green_coords = self.obs_to_pos(green)

            self.get_logger().debug(
                f"buoys:  {prev_coords} \nred: {red_coords} \ngreen: {green_coords}"
            )
            self.get_logger().debug(f"robot: {self.robot_pos}")
            self.get_logger().debug("Missing at least one front buoy")
            return None

        if prev_pair is not None:
            prev_pair_midpoint = self.midpoint_pair(prev_pair)
            self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
            if self.red_left:
                return InternalBuoyPair(
                    self.get_closest_to(prev_pair_midpoint, front_red),
                    self.get_closest_to(prev_pair_midpoint, front_green),
                )
            else:
                return InternalBuoyPair(
                    self.get_closest_to(prev_pair_midpoint, front_green),
                    self.get_closest_to(prev_pair_midpoint, front_red),
                )
        else:
            self.get_logger().debug("No previous pair!")
            # TODO: change those to: self.get_closest_to((0,0), red/green, local=True) to not have to use odometry position but just local obstacle positions (wrt the robot)?
            self.get_logger().debug(
                f"next buoys: red: {self.get_closest_to(self.robot_pos, red)}, green: {self.get_closest_to(self.robot_pos, green)}"
            )
            # if there is no previous pair, just take the closest red and green buoys
            if self.red_left:
                return InternalBuoyPair(
                    self.get_closest_to(self.robot_pos, red),
                    self.get_closest_to(self.robot_pos, green),
                )
            else:
                return InternalBuoyPair(
                    self.get_closest_to(self.robot_pos, green),
                    self.get_closest_to(self.robot_pos, red),
                )

    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))

    def generate_waypoints(self):
        """
        Runs every time a new obstacle map is received, keeps
        track of the pair of buoys the robot is heading towards,
        checks if it passed it (the robot is in front of the pair of buoys)
        (#TODO: add a margin of error such that the robot is considered to have passed the buoys if it's
        a bit in front of them) and update the pair accordingly, and afterwards computes
        the sequence of future waypoints based on the first waypoint
        and the next_pair() function to compute the next pair from each one in the sequence,
        as long as there is a next pair from the buoys that are stored in the obstacle map.
        """
        # split the buoys into red and green
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, red buoys: {self.obs_to_pos(red_buoys)}, green buoys: {self.obs_to_pos(green_buoys)}"
        )

        # TODO: Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair

        if self.pair_to is None:
            self.get_logger().debug("No pair to go to.")
            return
        # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
        if self.ccw(
            self.ob_coords(self.pair_to.left),
            self.ob_coords(self.pair_to.right),
            self.robot_pos,
        ):
            new_pair = self.next_pair(self.pair_to, red_buoys, green_buoys)
            if new_pair is not None:
                self.pair_to = new_pair
                self.time_last_seen_buoys = time.time()
            else:
                self.get_logger().debug("No next buoy pair to go to.")
                if time.time() - self.time_last_seen_buoys > 1:
                    self.result = True
                # wait for next spin
                return
            # TODO: there is no longer a case where it is done wiht the task.
            # also, what happens when eg. the green buoy is passed but not the red?

        buoy_pairs = [self.pair_to]
        waypoints = [self.midpoint_pair(self.pair_to)]

        self.get_logger().debug(f"pair to: {len(buoy_pairs)}")

        """
        Form a sequence of buoy pairs (and the respective waypoints) that form a path that the robot can follow
        will terminate if we run out of either green or red buoys
        """

        next_buoy_pair = self.next_pair(buoy_pairs[-1], red_buoys, green_buoys)
        while next_buoy_pair is not None:
            buoy_pairs.append(next_buoy_pair)
            waypoints.append(self.midpoint_pair(next_buoy_pair))
            next_buoy_pair = self.next_pair(buoy_pairs[-1], red_buoys, green_buoys)

        self.get_logger().debug(f"Waypoints: {waypoints}")

        if waypoints:
            waypoint = waypoints[0]
            self.get_logger().debug(
                f"cur_waypoint: {waypoint}, sent_waypoints: {self.sent_waypoints}"
            )
            self.get_logger().debug(f"len(waypoints): {len(waypoints)}")

            # check if waypoint is close enough (check_dist) to some previous waypoint
            passed_waypoint = False
            check_dist = 1  # TODO: FIX magic number T_T
            for sent_waypoint in self.sent_waypoints:
                if math.dist(waypoint, sent_waypoint) < check_dist:
                    passed_waypoint = True

            if not passed_waypoint:
                self.get_logger().info(f"sending waypoint {waypoint} to action server")
                self.follow_path_client.wait_for_server()
                goal_msg = FollowPath.Goal()
                goal_msg.planner = self.get_parameter("planner").value
                goal_msg.x = waypoint[0]
                goal_msg.y = waypoint[1]
                goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
                goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
                goal_msg.goal_tol = self.get_parameter("goal_tol").value
                goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
                goal_msg.choose_every = self.get_parameter("choose_every").value
                goal_msg.is_stationary = True
                self.follow_path_client.wait_for_server()
                self.send_goal_future = self.follow_path_client.send_goal_async(
                    goal_msg
                )
                self.sent_waypoints.add(waypoint)

    def map_cb(self, msg):
        """
        When a new map is received, check if it is the first one (we haven't set up the starting buoys)
        and find the starting pair, and then (if the starting buoys are successfully computed) form
        the buoy pair / waypoint sequence
        """
        self.obstacles = msg.obstacles

    def odometry_cb(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def execute_callback(self, goal_handle):
        self.get_logger().info("FollowBuoyPath request received.")

        self.start_process("Follow buoy path (task 1/2) started.")

        while rclpy.ok() and self.obstacles is None:
            self.get_logger().info("No obstacle map yet; waiting...")
            time.sleep(1.0) # maybe change this
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
        
        self.get_logger().info("received first map")

        success = False
        while not success:
            success = self.setup_buoys()
            time.sleep(1.0)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()

        self.get_logger().info("Setup buoys succeeded!")


        while not self.result:
            # Check if we should abort/cancel if a new goal arrived
            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Cancel requested. Aborting path following.")
                goal_handle.canceled()
                return Task.Result()
            
            self.generate_waypoints()

            time.sleep(self.timer_period)

        self.end_process("Completed FollowBuoyPath request.")
        goal_handle.succeed()
        return Task.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPath()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
