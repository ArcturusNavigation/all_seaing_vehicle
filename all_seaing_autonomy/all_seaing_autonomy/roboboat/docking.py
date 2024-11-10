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
DOCK_POSITION = (45.697, 32.452, 0) # (x, y, z rotation)
DESIRED_BANNER = (BannerShape.CIRCLE, BannerColor.RED) # (shape, color)
SINGLE_DOCK_LENGTH = 1
DOCK_DEPTH = 1
ORBIT_RADIUS = 1

def minus(a, b):
    # helper function to subtract two tuples
    try:
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    except TypeError:
        try:
            return (a.x - b[0], a.y - b[1], a.z - b[2])
        except AttributeError:
            try:
                return (a[0] - b.x, a[1] - b.y, a[2] - b.z)
            except:
                return (a.x - b.x, a.y - b.y, a.z - b.z)

# TODO: MAKE THIS ACTUALLY READ THE CAMERA
def read_camera():
    # read the camera and return the shape and color of the banner
    return (BannerShape.CIRCLE, BannerColor.RED)

def point_diff_2d(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

def add_2d(a, b):
    return (a.x + b.x, a.y + b.y)

class DockingTask(Task):
    def __init__(self, control_message_pub, clock, logger):
        self.control_message_pub = control_message_pub
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
        # maybe verify that we are in a region where docking is possible?
        self.state = DockingState.APPROACHING
        self.logger.info("Starting docking task")
    
    def update(self):
        control_message = ControlOption()
        
        match self.state:
            case DockingState.APPROACHING:
                # move towards the dock
                # TODO: maybe do something more robust than a plan proportional controller
                if self.current_position is not None:
                    delta = minus(DOCK_POSITION, self.current_position)
                    deltaMag = math.sqrt(delta[0] ** 2 + delta[1] ** 2)
                    kp = 0.1

                    vMag = kp * deltaMag
                    vDirection = math.atan2(delta[1], delta[0])

                    _, _, robot_rotation = self.euler_from_quaternion(self.current_rotation)
                    theta = vDirection - robot_rotation

                    vx = vMag * math.cos(theta)
                    vy = vMag * math.sin(theta)
                    vtheta = kp * 30 * theta

                    control_message.twist.linear.x = vx
                    control_message.twist.linear.y = vy
                    control_message.twist.angular.z = vtheta

                    if deltaMag < 0.1:
                        self.state = DockingState.CHECKING_CAMERA
                        self.logger.info("Approached dock")
                    
            case DockingState.CHECKING_CAMERA:
                # check the camera for the dock

                # stop the boat while we do this
                control_message.twist.linear.x = 0.0
                control_message.twist.linear.y = 0.0
                control_message.twist.angular.z = 0.0

                if self.state_changed:
                    self.logger.info("Checking camera for dock")
                    self.start_checking_camera_time = self.clock.now()
                elif self.clock.now() - self.start_checking_camera_time > 2: # wait 2 sec for boat to stop
                    # check camera
                    banner = read_camera()

                    if banner == DESIRED_BANNER:
                        self.state = DockingState.DOCKING
                    else:
                        self.state = DockingState.SHIFTING if banner[0] != BannerShape.NONE else DockingState.ORBITING
            
            case DockingState.SHIFTING:
                # shift to the correct position
                direction = 1 if has_orbited else -1

                if self.state_changed:
                    self.start_shifting_position = self.current_position

                control_message.twist.linear.x = 0
                control_message.twist.linear.y = 0.5 * direction
                control_message.twist.angular.z = 0

                # check if we have shifted by a full dock length
                if point_diff_2d(self.start_shifting_position, self.current_position) > SINGLE_DOCK_LENGTH:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Shifted position")
                
            case DockingState.DOCKING:
                # dock the boat
                direction = 1 if has_orbited else -1

                if self.state_changed:
                    self.start_docking_position = self.current_position

                control_message.twist.linear.x = 0.5
                control_message.twist.linear.y = 0
                control_message.twist.angular.z = 0

                # check if we have docked
                if point_diff_2d(self.start_docking_position, self.current_position) > DOCK_DEPTH:
                    self.state = DockingState.DONE
                    self.logger.info("Docked")
                
            case DockingState.ORBITING:
                has_orbited = True

                if self.state_changed:
                    self.orbit_center = add_2d(self.current_position, (ORBIT_RADIUS, 0))
                    self.start_orbit_position = self.current_position
                    self.orbit_target = (self.current_position[0] + ORBIT_RADIUS, self.current_position[1])
                
                # calculate the robot-relative velocity to orbit the center
                radial_vector = minus(self.orbit_center, self.current_position)
                
                velocity_vector = (radial_vector[1], -radial_vector[0])
                vMag = 0.5
                vDirection = math.atan2(velocity_vector[1], velocity_vector[0])

                angle_to_center = math.atan2(radial_vector[1], radial_vector[0])
                angle_diff = angle_to_center - self.current_position[2]
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                
                angle_kP = 0.1

                control_message.twist.linear.x = vMag * math.cos(vDirection)
                control_message.twist.linear.y = vMag * math.sin(vDirection)
                control_message.twist.angular.z = angle_kP * angle_diff
                
                if point_diff_2d(self.orbit_target, self.current_position) > 1:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Orbited")
                
            case DockingState.DONE:
                # done
                # TODO: leave the dock ?
                control_message.twist.linear.x = 0
                control_message.twist.linear.y = 0
                control_message.twist.angular.z = 0

            case DockingState.STOPPED:
                # stopped
                control_message.twist.linear.x = 0
                control_message.twist.linear.y = 0
                control_message.twist.angular.z = 0
            
        self.control_message_pub.publish(control_message)
        self.state_changed = self.state != self.last_state
        self.last_state = self.state

        if self.state_changed:
            self.logger.info(f"Docking state changed to {self.state}")
        

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