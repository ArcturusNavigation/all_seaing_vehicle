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
        global CURR_BANNER
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
                    if theta > math.pi: theta = -(2*math.pi-theta)

                    vx = 10 * math.cos(theta)
                    vy = 10 * math.sin(theta)
                    vtheta = kp * 30 * theta

                    control_message.twist.linear.x = vx
                    control_message.twist.linear.y = vy
                    control_message.twist.angular.z = vtheta

                    if deltaMag < 2:
                        self.state = DockingState.CHECKING_CAMERA
                        self.logger.info("Approached dock")
                    
            case DockingState.CHECKING_CAMERA:
                # check the camera for the dock

                # stop the boat while we do this
                control_message.twist.linear.x = 0.0
                control_message.twist.linear.y = 0.0
                
                _, _, robot_rotation = self.euler_from_quaternion(self.current_rotation)
                theta = robot_rotation - DOCK_POSITION[2] * 2*math.pi/180.0
                if theta > math.pi: theta = -(2*math.pi-theta)
                control_message.twist.angular.z = -theta * 5.0

                if self.state_changed:
                    self.logger.info("Checking camera for dock")
                    self.start_checking_camera_time = self.clock.now()
                elif (self.clock.now() - self.start_checking_camera_time).nanoseconds > 2e9: # wait 2 sec for boat to stop
                    # check camera
                    banner = read_camera()

                    if banner == DESIRED_BANNER:
                        self.state = DockingState.DOCKING
                    else:
                        self.state = DockingState.SHIFTING if banner[0] != BannerShape.NONE else DockingState.ORBITING
            
                    # TODO REMOVE
                    CURR_BANNER += 1

            case DockingState.SHIFTING:
                # shift to the correct position
                #direction = 1 if self.has_orbited else -1
                direction = -1
                orientation_offset = math.pi if self.has_orbited else 0 

                if self.state_changed:
                    self.start_shifting_position = self.current_position

                control_message.twist.linear.x = 0.0
                control_message.twist.linear.y = -10.0 * direction

                _, _, robot_rotation = self.euler_from_quaternion(self.current_rotation)
                theta = robot_rotation - DOCK_POSITION[2] * 2*math.pi/180.0 - orientation_offset
                if theta > math.pi: theta = -(2*math.pi-theta)
                control_message.twist.angular.z = -theta * 5.0

                # check if we have shifted by a full dock length
                if point_diff_2d(self.start_shifting_position, self.current_position) > SINGLE_DOCK_LENGTH:
                    self.state = DockingState.CHECKING_CAMERA
                    self.logger.info("Shifted position")
                
            case DockingState.DOCKING:
                # dock the boat
                direction = 1 if self.has_orbited else -1

                if self.state_changed:
                    self.start_docking_position = self.current_position

                control_message.twist.linear.x = 10.0
                control_message.twist.linear.y = 0.0
                control_message.twist.angular.z = 0.0

                # check if we have docked
                if point_diff_2d(self.start_docking_position, self.current_position) > DOCK_DEPTH:
                    self.state = DockingState.DONE
                    self.logger.info("Docked")
                
            case DockingState.ORBITING:
                self.has_orbited = True

                if self.state_changed:
                    self.orbit_center = add_2d(self.current_position, (ORBIT_RADIUS, 0))
                    self.start_orbit_position = self.current_position
                    self.orbit_target = (self.current_position.x + 2*ORBIT_RADIUS, self.current_position.y, 0)
                
                '''
                # calculate the robot-relative velocity to orbit the center
                radial_vector = minus(tuple(list(self.orbit_center) + [0]), self.current_position)
                
                velocity_vector = (radial_vector[1], -radial_vector[0])
                vMag = 10.0
                vDirection = math.atan2(velocity_vector[1], velocity_vector[0])

                angle_to_center = math.atan2(radial_vector[1], radial_vector[0])
                angle_diff = angle_to_center - self.current_position.z
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                
                angle_kP = -1.0

                control_message.twist.linear.x = vMag * math.cos(vDirection)
                control_message.twist.linear.y = vMag * math.sin(vDirection)
                control_message.twist.angular.z = angle_kP * angle_diff
                '''

                delta = minus(self.orbit_target, self.current_position)
                deltaMag = math.sqrt(delta[0] ** 2 + delta[1] ** 2)
                kp = 0.1

                vMag = kp * deltaMag
                vDirection = math.atan2(delta[1], delta[0])

                _, _, robot_rotation = self.euler_from_quaternion(self.current_rotation)
                theta = vDirection - robot_rotation
                if theta > math.pi: theta = -(2*math.pi-theta)

                vx = 10 * math.cos(theta)
                vy = 10 * math.sin(theta)
                vtheta = kp * 30 * theta

                control_message.twist.linear.x = vx
                control_message.twist.linear.y = vy
                control_message.twist.angular.z = vtheta

                if point_diff_2d(self.orbit_target, self.current_position) < 1:
                    control_message.twist.linear.x = 0.0
                    control_message.twist.linear.y = 0.0
                    theta = robot_rotation - DOCK_POSITION[2] * 2*math.pi/180.0 + math.pi
                    if theta > math.pi: theta = -(2*math.pi-theta)
                    control_message.twist.angular.z = -theta * 5.0

                    if theta < 0.2*math.pi:
                        self.state = DockingState.CHECKING_CAMERA
                        self.logger.info("Orbited")
                
            case DockingState.DONE:
                # done
                # TODO: leave the dock ?
                control_message.twist.linear.x = 0.0
                control_message.twist.linear.y = 0.0
                control_message.twist.angular.z = 0.0

            case DockingState.STOPPED:
                # stopped
                control_message.twist.linear.x = 0.0
                control_message.twist.linear.y = 0.0
                control_message.twist.angular.z = 0.0
            
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