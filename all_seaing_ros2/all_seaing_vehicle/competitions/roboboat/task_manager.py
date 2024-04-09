#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import Heartbeat, ASV2State, ControlMessage
from nav_msgs.msg import Odometry
from all_seaing_vehicle.competitions.roboboat.Task import Task
from all_seaing_vehicle.competitions.roboboat.FollowThePath import FollowThePath
# from all_seaing_vehicle import Task

def point_diff_2d(a, b):
    return ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5


class NavigationChannel(Task):
    def __init__(self, control_message_pub, logger):
        self.control_message_pub = control_message_pub

        self.start_point = None
        self.current_point = None
        self.logger = logger
    def get_name(self):
        # the idea is that since this is a method, we can edit the name to display some sort of information for debugging
        return "NavigationChannel"
    def start(self):
        # start moos behavior of path following
        control_message = ControlMessage()
        control_message.x = 0.5
        control_message.y = 0.0
        control_message.linear_control_mode = ControlMessage.LOCAL_VELOCITY
        control_message.angular = 0.0
        control_message.angular_control_mode = ControlMessage.WORLD_VELOCITY

        self.control_message_pub.publish(control_message)

        self.start_point = None
        self.current_point = None
    def update(self):
        # pass new waypoints into moos, reading from perception
        pass
    def end(self):
        # remove moos behavior or something
        pass
    def get_next(self):
        # check if moos is done, if so return the next task
        if self.start_point is None or self.current_point is None:
            return False
        self.logger.info(str(point_diff_2d(self.start_point, self.current_point)))
        return point_diff_2d(self.start_point, self.current_point) >= 1
    def receive_odometry(self, msg):
        self.current_point = msg.pose.pose.position
        if self.start_point is None:
            self.start_point = self.current_point
    
class Idling(Task):
    def get_name(self):
        return "Idling"
    def update(self):
        # keep boat in place
        pass

TIMEOUT_TIME = 3 # seconds
UPDATE_RATE = 1/60 # seconds

class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.create_subscription(Heartbeat, "/heartbeat", self.receive_heartbeat, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.receive_odometry, 10)

        self.state_publisher = self.create_publisher(ASV2State, "/boat_state", 10)
        self.state_message = ASV2State()

        self.control_message_publisher = self.create_publisher(ControlMessage, "/control_input", 10)

        self.TASK_LIST = [
            # FollowThePath(self.get_logger()),
            # NavigationChannel(self.control_message_publisher, self.get_logger()), 
            Idling()
        ]

        self.task_index = 0
        self.task = self.TASK_LIST[self.task_index]
        
        self.clock = self.get_clock()
        self.last_heartbeat_timestamp = self.clock.now()

        self.timer = self.create_timer(UPDATE_RATE, self.update)

        self.paused = True
        self.has_started_for_the_first_time = False

        print("starting task manager (paused for now)")

    def update(self):
        if (self.clock.now() - self.last_heartbeat_timestamp).nanoseconds / 1e9 > TIMEOUT_TIME:
            raise Exception("lost heartbeat, turning off task manager")
        
        if self.paused:
            return
        if not self.has_started_for_the_first_time:
            self.has_started_for_the_first_time = True
            self.task.start()

        self.check_transition()
        self.task.update()

        self.state_message.current_state = self.task.get_name()
        self.state_publisher.publish(self.state_message)

    def receive_odometry(self, msg):
        self.task.receive_odometry(msg)

    def check_transition(self):
        if self.task.get_next():
            self.get_logger().info("transitioning to next state")
            self.task.end()
            self.task_index += 1
            if self.task_index >= len(self.TASK_LIST):
                raise Exception("transitioned out of last task!")
            self.task = self.TASK_LIST[self.task_index]
            self.task.start()
            self.check_transition()

    def receive_heartbeat(self, msg):
        if msg.e_stopped:
            raise Exception("received e-stop message, shutting down task manager")
        
        if msg.in_teleop != self.paused:
            self.paused = msg.in_teleop
            if self.paused:
                print("detected teleop mode, pausing task manager")
            else:
                print("unpausing task manager")

        self.last_heartbeat_timestamp = self.clock.now()

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
