from all_seaing_autonomy.roboboat.Task import Task
from enum import Enum
import math
from all_seaing_interfaces.msg import Heartbeat, ASV2State, ControlOption

class DockingState(Enum):
    IDLE = 0
    APPROACHING = 1
    CHECKING_CAMERA = 2
    SHIFTING = 3
    DOCKING = 4
    ORBITING = 5
    DONE = 6
    STOPPED = 7

class BannerShape(Enum):
    CIRCLE = 0
    SQUARE = 1
    TRIANGLE = 2
    PLUS = 3
    NONE = 4

class BannerColor(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    NONE = 3

# CONSTANTS: TODO: SET THESE TO THE ACTUAL VALUES / MAKE THEM EASILY CONFIGURABLE
DOCK_POSITION = (45.697, 32.452, 15) # (x, y, z rotation)
DESIRED_BANNER = (BannerShape.CIRCLE, BannerColor.RED) # (shape, color)
SINGLE_DOCK_LENGTH = 5
DOCK_DEPTH = 2
ORBIT_RADIUS = 10

CURR_BANNER = 0

def minus(a, b):
    # helper function to subtract two tuples
    if type(a) != tuple:
        a = (a.x, a.y, a.z)
    if type(b) != tuple:
        b = (b.x, b.y, b.z)
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

# TODO: MAKE THIS ACTUALLY READ THE CAMERA
def read_camera():
    # read the camera and return the shape and color of the banner
    # fake sequence
    global CURR_BANNER
    return [
        (BannerShape.SQUARE, BannerColor.GREEN),
        (BannerShape.SQUARE, BannerColor.RED),
        (BannerShape.SQUARE, BannerColor.GREEN),
        (BannerShape.NONE, BannerColor.NONE),
        (BannerShape.CIRCLE, BannerColor.RED)
    ][CURR_BANNER]

def point_diff_2d(a, b):
    if type(a) != tuple:
        a = (a.x, a.y)
    if type(b) != tuple:
        b = (b.x, b.y)
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def add_2d(a, b):
    if type(a) != tuple:
        a = (a.x, a.y)
    if type(b) != tuple:
        b = (b.x, b.y)
    return (a[0] + b[0], a[1] + b[1])

class DockingTask(Task):
    def __init__(self, waypoint_action_client, clock, logger):
        self.waypoint_action_client = waypoint_action_client
        self.clock = clock
        self.logger = logger
        
        self.state = DockingState.IDLE
        self.last_state = DockingState.IDLE
        self.current_position = None

        self.has_orbited = False
        self.state_changed = False
        
    def get_name(self):
        return "Docking"
    
    def start(self):
        self.state = DockingState.APPROACHING
        self.logger.info("Starting docking task")
    
    def send_waypoint_goal(self, x, y, theta=None, ignore_theta=True):
        """
        Sends a waypoint goal to the controller server.
        """
        goal_msg = Waypoint.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.ignore_theta = ignore_theta
        if theta is not None:
            goal_msg.theta = theta
        goal_msg.xy_threshold = 1.0  # Adjust threshold as needed
        goal_msg.theta_threshold = 5.0  # Adjust threshold as needed

        self.logger.info(f"Sending waypoint goal: x={x}, y={y}, theta={theta}")
        self.waypoint_action_client.wait_for_server()
        return self.waypoint_action_client.send_goal_async(goal_msg)
    
    def update(self):
        global CURR_BANNER
        
        match self.state:
            case DockingState.APPROACHING:
                # Approach the dock
                if self.current_position is not None:
                    x, y, _ = DOCK_POSITION
                    delta = minus(DOCK_POSITION, self.current_position)
                    deltaMag = math.sqrt(delta[0] ** 2 + delta[1] ** 2)

                    if deltaMag < 2:  # Close enough to dock
                        self.state = DockingState.CHECKING_CAMERA
                        self.logger.info("Approached dock")
                    else:
                        # Send waypoint goal
                        self.send_waypoint_goal(x, y)
                    
            case DockingState.CHECKING_CAMERA:
                # Stop and check the camera
                if self.state_changed:
                    self.logger.info("Checking camera for dock")
                    self.start_checking_camera_time = self.clock.now()

                elif (self.clock.now() - self.start_checking_camera_time).nanoseconds > 2e9:  # Wait 2 seconds
                    banner = read_camera()
                    if banner == DESIRED_BANNER:
                        self.state = DockingState.DOCKING
                    else:
                        self.state = DockingState.SHIFTING if banner[0] != BannerShape.NONE else DockingState.ORBITING
                    CURR_BANNER += 1  # Simulate camera reading progress
                
            case DockingState.SHIFTING:
                # Shift position laterally
                direction = -1
                x, y, _ = DOCK_POSITION
                offset_x = SINGLE_DOCK_LENGTH * direction
                new_x = x + offset_x

                if self.state_changed:
                    self.start_shifting_position = self.current_position
                    self.logger.info("Shifting to new dock position")

                # Send waypoint goal
                self.send_waypoint_goal(new_x, y)
                if point_diff_2d(self.start_shifting_position, self.current_position) > SINGLE_DOCK_LENGTH:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Shifted position")
                
            case DockingState.DOCKING:
                # Docking sequence
                if self.state_changed:
                    self.start_docking_position = self.current_position
                    self.logger.info("Docking in progress")

                # Send waypoint goal for docking
                x, y, _ = DOCK_POSITION
                self.send_waypoint_goal(x, y, ignore_theta=False)
                if point_diff_2d(self.start_docking_position, self.current_position) < DOCK_DEPTH:
                    self.state = DockingState.DONE
                    self.logger.info("Docked successfully")
                
            case DockingState.DONE:
                self.logger.info("Docking task completed")

    def check_finished(self):
        return self.state == DockingState.DONE
    
    def receive_odometry(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_rotation = msg.pose.pose.orientation

    def set_state(self, state):
        self.state = state

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
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw