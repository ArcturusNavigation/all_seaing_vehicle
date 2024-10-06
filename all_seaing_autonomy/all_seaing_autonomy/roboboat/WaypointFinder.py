#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3, Quaternion
from all_seaing_interfaces.msg import BuoyPair, BuoyPairArray, Waypoint, WaypointArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import os
from ament_index_python.packages import get_package_share_directory

# TODO: DOCUMENT THE CODE IN THE AUTONOMY WIKI (SEE PERCEPTION WIKI FOR REFERENCE)


class WaypointFinder(Node):
    def __init__(self):
        super().__init__("waypoint_finder")
        self.map_sub = self.create_subscription(
            ObstacleMap, "/obstacle_map/labeled", self.map_cb, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.buoy_pair_pub = self.create_publisher(BuoyPairArray, "buoy_pairs", 10)
        self.waypoint_pub = self.create_publisher(WaypointArray, "waypoints", 10)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.robot_pos = (0, 0)

        self.declare_parameter("safe_margin", 0)

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.declare_parameter("color_label_mappings_file",
            os.path.join(
            bringup_prefix, "config", "perception", "color_label_mappings.yaml")
        )

        self.first_map = True

        self.safe_margin = self.get_parameter("safe_margin").get_parameter_value().double_value
        
        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            self.color_label_mappings = yaml.safe_load(f)

    def norm_squared(self, vec, ref=(0, 0)):
        return vec[0] ** 2 + vec[1] ** 2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))

    def ob_coords(self, buoy, local=False):
        if local:
            return (buoy.local_point.point.x, buoy.local_point.point.y)
        else:
            return (buoy.global_point.point.x, buoy.global_point.point.y)

    def get_closest_to(self, source, buoys, local=False):
        return min(
            buoys,
            key=lambda buoy: self.norm_squared(source, self.ob_coords(buoy, local)),
        )

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def midpoint_pair(self, pair):
        return self.midpoint(self.ob_coords(pair[0]), self.ob_coords(pair[1]))

    def split_buoys(self, obstacles):
        """
        Splits the buoys into red and green based on their labels in the obstacle map (red = 1, green = 2)
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
                    id=(3 * i),
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(buoy_pair.left)),
                    header=Header(frame_id="odom"),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=ColorRGBA(r=1.0, a=1.0),
                    id=(3 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(buoy_pair.right)),
                    header=Header(frame_id="odom"),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=ColorRGBA(g=1.0, a=1.0),
                    id=(3 * i) + 2,
                )
            )
            i += 1
            # TODO: add a cylinder marker to visualize the acceptable area radius and the safety margin
        return marker_array

    def setup_buoys(self):
        """
        Runs when the first obstacle map is received, filters the buoys that are in front of the robot (x>0 in local coordinates)
        and finds (and stores) the closest green one and the closest red one, and because the robot is in the starting position these
        are the front buoys of the robot starting box.
        """
        self.get_logger().info("Setting up starting buoys!")
        self.get_logger().info(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )

        green_init, red_init = self.split_buoys(
            self.obstacles
        )  # split all the buoys into red and green
        # lambda function that filters the buoys that are in front of the robot (using their local coordinates, but provides the global ones as output)
        obstacles_in_front = lambda obs: [
            ob for ob in obs if ob.local_point.point.x > 0
        ]
        # take the green and red buoys that are in front of the robot
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(
            red_init
        )
        self.get_logger().info(
            "initial red buoys: {red_buoys}, green buoys: {green_buoys}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            # didn't find starting buoys
            self.get_logger().warning("No starting buoy pairs!")
            return False
        # from the red buoys that are in front of the robot, take the one that is closest to it, and do the same for the green buoys
        # this pair is the front pair of the starting box of the robot
        self.starting_buoys = (
            self.get_closest_to((0, 0), red_buoys, local=True),
            self.get_closest_to((0, 0), green_buoys, local=True),
        )
        self.pair_to = self.starting_buoys
        self.get_logger().info(f"{self.starting_buoys=}")
        return True

    def ccw(self, a, b, c):
        """Check if the points a, b, c are counterclockwise, in this order, and return True, otherwise return False"""
        # literally just shoelace formula
        area = (
            a[0] * b[1]
            + b[0] * c[1]
            + c[0] * a[1]
            - a[1] * b[0]
            - b[1] * c[0]
            - c[1] * a[0]
        )
        if area > 0:
            return True
        else:
            return False

    # euler <-> quaternion transformation code from: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

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
                self.ob_coords(pair[0]), self.ob_coords(pair[1]), self.ob_coords(buoy)
            )
        ]

    def next_pair(self, prev_pair, red, green):
        """
        Returns the next buoy pair (red left, green right) from the previous pair,
        by checking the closest one to the middle of the previous buoy pair that's in front of the pair
        """
        prev_pair_midpoint = self.midpoint_pair(prev_pair)
        front_red = self.filter_front_buoys(prev_pair, red)
        front_green = self.filter_front_buoys(prev_pair, green)
        return (
            self.get_closest_to(prev_pair_midpoint, front_red),
            self.get_closest_to(prev_pair_midpoint, front_green),
        )

    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))

    def pair_angle_to_pose(self, pair, angle):
        quat = self.quaternion_from_euler(0, angle, 0)
        return Pose(
            position=Point(x=pair[0], y=pair[1]),
            orientation=Quaternion(x=quat[0], y=quat[2], z=quat[2], w=quat[3]),
        )

    def generate_waypoints(self):
        """
        Runs every time a new obstacle map is received, keeps track of the pair of buoys the robot is heading towards,
        checks if it passed it (the robot is in front of the pair of buoys)
        (#TODO: add a margin of error such that the robot is considered to have passed the buoys if it's a bit in front of them)
        and update the pair accordingly, and afterwards computes the sequence of future waypoints based on the first waypoint
        and the next_pair() function to compute the next pair from each one in the sequence,
        as long as there is a next pair from the buoys that are stored in the obstacle map.
        """
        self.get_logger().info(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )
        # split the buoys into red and green
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        # get the positions of the buoys
        # green_buoys = self.obs_to_pos(green_buoys)
        # red_buoys = self.obs_to_pos(red_buoys)
        self.get_logger().info(
            f"red buoys: {self.obs_to_pos(red_buoys)}, green buoys: {self.obs_to_pos(green_buoys)}"
        )
        # RED BUOYS LEFT, GREEN RIGHT

        # TODO: Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair

        # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
        if self.ccw(
            self.ob_coords(self.pair_to[0]),
            self.ob_coords(self.pair_to[1]),
            self.robot_pos,
        ):
            try:
                # keep the next pair as the one the robot is heading to
                self.pair_to = self.next_pair(self.pair_to, red_buoys, green_buoys)
            except Exception as e:
                self.get_logger().warning(repr(e))
                self.get_logger().warning("No next buoy pair to go to!")

        buoy_pairs = [self.pair_to]
        waypoints = [self.midpoint_pair(self.pair_to)]

        # form a sequence of buoy pairs (and the respective waypoints) that form a path that the robot can follow
        # will terminate if we run out of either green or red buoys
        while True:
            try:
                buoy_pairs.append(
                    self.next_pair(buoy_pairs[-1], red_buoys, green_buoys)
                )
                waypoints.append(self.midpoint_pair(buoy_pairs[-1]))
            except Exception as e:
                self.get_logger().warning(repr(e))
                break

        # convert the sequence to a format appropriate to publishing for the path planner to use
        waypoint_arr = WaypointArray(
            waypoints=[
                Waypoint(
                    point=self.pair_angle_to_pose(
                        pair=wpt,
                        angle=(
                            math.atan(self.ob_coords(pair[1])[1] - self.ob_coords(pair[0])[1]) /
                            (self.ob_coords(pair[1])[0] - self.ob_coords(pair[0])[0])
                        ) + (math.pi / 2),
                    ),
                    radius=self.norm(self.ob_coords(pair[0]), self.ob_coords(pair[1])) - self.safe_margin,
                )
                for wpt, pair in zip(waypoints, buoy_pairs)
            ]
        )
        buoy_pair_arr = BuoyPairArray(
            pairs=[
                BuoyPair(left=pair[0], right=pair[1], waypoint=waypoint)
                for pair, waypoint in zip(buoy_pairs, waypoint_arr.waypoints)
            ]
        )

        # publish the waypoints and the buoy pairs (again, including the waypoints)
        self.buoy_pair_pub.publish(buoy_pair_arr)
        self.waypoint_pub.publish(waypoint_arr)
        # publish the markers that show up in RViz
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers(buoy_pair_arr))
        self.get_logger().info(f"{buoy_pairs=}, {waypoints=}")

    def map_cb(self, msg):
        """
        When a new map is received, check if it is the first one (we haven't set up the starting buoys) and find the starting pair,
        and then (if the starting buoys are successfully computed) form the buoy pair / waypoint sequence
        """
        self.obstacles = msg.obstacles
        self.get_logger().info(f"number of obstacles: {len(msg.obstacles)}")
        self.get_logger().info(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )

        success = False
        if self.first_map:
            success = self.setup_buoys()
            self.first_map = not success
        else:
            success = True
        if success:
            self.generate_waypoints()

    def odometry_cb(self, msg):
        """
        Update the stored robot's position based on the odometry messages
        """
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
