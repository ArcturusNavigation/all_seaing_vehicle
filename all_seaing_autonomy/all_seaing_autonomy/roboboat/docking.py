from all_seaing_autonomy.roboboat.Task import Task
from all_seaing_interfaces.action import Waypoint
from rclpy.action import ActionClient
from enum import Enum
import math

class DockingState(Enum):
    IDLE = 0
    APPROACHING = 1
    CHECKING_CAMERA = 2
    SHIFTING = 3
    DOCKING = 4
    ORBITING_1 = 5
    ORBITING_2 = 6
    DONE = 7
    STOPPED = 8

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
DOCK_POSITION = (45.697, 32.452, 15)  # (x, y, z rotation)
DESIRED_BANNER = (BannerShape.CIRCLE, BannerColor.RED)  # (shape, color)
SINGLE_DOCK_LENGTH = 5
DOCK_DEPTH = 2
ORBIT_RADIUS = 10

CURR_BANNER = 0

def minus(a, b):
    if type(a) != tuple:
        a = (a.x, a.y, a.z)
    if type(b) != tuple:
        b = (b.x, b.y, b.z)
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def point_diff_2d(a, b):
    if type(a) != tuple:
        a = (a.x, a.y)
    if type(b) != tuple:
        b = (b.x, b.y)
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

class DockingTask(Task):
    def __init__(self, waypoint_client, clock, logger):
        super().__init__()
        self.clock = clock
        self.logger = logger
        
        self.waypoint_client = waypoint_client
        self.state = DockingState.IDLE
        self.last_state = DockingState.IDLE
        self.current_position = None
        self.state_changed = False 
        self.reached_waypoint = False
        self.dock_times = 1
        self.shift_direction = 1

    def get_name(self):
        return "Docking"

    def start(self):
        self.state = DockingState.APPROACHING
        self.logger.info("Starting docking task")

    def send_waypoint(self, x, y, ignore_theta=True, theta=0.0):
        """
        Send a single waypoint to the controller and wait for it to complete.
        """
        goal_msg = Waypoint.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.ignore_theta = ignore_theta
        goal_msg.theta = theta
        goal_msg.xy_threshold = 1.0  # Adjust as needed
        goal_msg.theta_threshold = 5.0  # Degrees, adjust as needed
        self.reached_waypoint = False

        # Wait for action server to be ready
        self.waypoint_client.wait_for_server()

        # Send the goal
        self.logger.info(f"Sending waypoint: x={x}, y={y}, ignore_theta={ignore_theta}")
        self.future = self.waypoint_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.waypoint_response_callback)

    def waypoint_response_callback(self, future):
        """
        Callback for when the waypoint action server responds.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warn("Waypoint request was rejected.")
            return

        self.logger.info("Waypoint accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result_callback)

    def waypoint_result_callback(self, future):
        """
        Callback for when the waypoint action server sends a result.
        """
        result = future.result().result
        if result.is_finished:
            self.reached_waypoint = True
            self.logger.info("Waypoint successfully completed!")
        else:
            self.logger.warn("Waypoint failed or was canceled.")
            self.state = DockingState.STOPPED

    def update(self):
        match self.state:
            case DockingState.APPROACHING:
                # Send waypoint only once when transitioning into APPROACHING
                if self.state_changed:
                    self.send_waypoint(DOCK_POSITION[0], DOCK_POSITION[1], ignore_theta=False, theta=DOCK_POSITION[2])
                    
                if self.reached_waypoint:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Approached dock")
           
            case DockingState.CHECKING_CAMERA:
                # Perform camera-based checks
                if self.state_changed:
                    self.logger.info("Checking camera for dock...")
                    self.dock_times += 1
                    # banner = self.read_camera()
                    banner = (BannerShape.CIRCLE, BannerColor.BLUE)  # Placeholder for WRONG
                    if banner == DESIRED_BANNER:
                        self.state = DockingState.DOCKING
                        self.logger.info("Correct banner detected, docking...")
                    else:
                        if self.dock_times != 3:
                            self.state = DockingState.SHIFTING
                            self.shift_direction = 1 if self.dock_times < 3 else -1
                            self.logger.info("Incorrect banner detected, shifting position...")
                        else 
                            self.state = DockingState.ORBITING_1
                            self.logger.info("Incorrect banner detected, orbiting...")

                        # self.state = DockingState.SHIFTING if self.dock_times 3 else DockingState.ORBITING_1
                        # self.logger.info("Incorrect banner detected, shifting position...")

            case DockingState.SHIFTING:
                # Shift the dock position
                if self.state_changed:
                    self.logger.info("Shifting position...")
                    new_x = DOCK_POSITION[0]
                    new_y = DOCK_POSITION[1] + (SINGLE_DOCK_LENGTH * self.dock_times)
                    self.send_waypoint(new_x, new_y, ignore_theta=False, theta=DOCK_POSITION[2] + (180 if self.shift_direction == -1 else 0))
                
                if self.reached_waypoint:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Shifted position")
            
            case DockingState.ORBITING_1:
                if self.state_changed:
                    self.logger.info("Orbiting dock first half...")
                    new_x = DOCK_POSITION[0] + ORBIT_RADIUS
                    new_y = DOCK_POSITION[1] + (SINGLE_DOCK_LENGTH * self.dock_times) + ORBIT_RADIUS
                    self.send_waypoint(new_x, new_y, ignore_theta=False, theta=DOCK_POSITION[2] + 90)

                if self.reached_waypoint:
                    self.state = DockingState.ORBITING_2
                    self.logger.info("first half of orbit completed")
            
            case DockingState.ORBITING_2:
                if self.state_changed:
                    self.logger.info("Orbiting dock second half...")
                    new_x = DOCK_POSITION[0] + (ORBIT_RADIUS * 2)
                    new_y = DOCK_POSITION[1] + (SINGLE_DOCK_LENGTH * self.dock_times)
                    self.send_waypoint(new_x, new_y, ignore_theta=False, theta=DOCK_POSITION[2] + 180)
                
                if self.reached_waypoint:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Shifted position (via complete orbit)")

            case DockingState.DOCKING:
                if self.state_changed:
                    self.logger.info("Docking at target position...")
                    self.send_waypoint(DOCK_POSITION[0], DOCK_POSITION[1], ignore_theta=False, theta=DOCK_POSITION[2])
                
                if self.reached_waypoint:
                    self.state = DockingState.ORBITING
                    self.logger.info("Docked")

            case DockingState.DONE:
                self.logger.info("Docking completed successfully.")
        
        self.state_changed = self.state != self.last_state

        if (self.state_changed):
            self.logger.info(f"State changed to {self.state}, last state: {self.last_state}")

        self.last_state = self.state

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
