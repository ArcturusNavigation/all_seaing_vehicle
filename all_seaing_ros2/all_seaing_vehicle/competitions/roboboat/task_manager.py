#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import Heartbeat
from all_seaing_interfaces.msg import ASV2State

class Task:
    def get_name(self):
        return "uhh... this class was supposed to be abstract lmao"
    def start(self):
        pass
    def update(self):
        pass
    def check_finished(self):
        return False
    def end(self):
        pass
    def get_next(self):
        pass

class NavigationChannel(Task):
    def get_name(self):
        # the idea is that since this is a method, we can edit the name to display some sort of information for debugging
        return "NavigationChannel"
    def start(self):
        # start moos behavior of path following
        pass
    def update(self):
        # pass new waypoints into moos, reading from perception
        pass
    def end(self):
        # remove moos behavior or something
        pass
    def get_next(self):
        # check if moos is done, if so return the next task
        return None
    
class Idling(Task):
    def update():
        # keep boat in place
        pass
    
STARTING_TASK = NavigationChannel()
TIMEOUT_TIME = 3 # seconds
UPDATE_RATE = 1/60 # seconds

class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.task = STARTING_TASK
        self.task.start()
        self.clock = self.get_clock()
        self.last_heartbeat_timestamp = self.clock.now()

        self.create_subscription(Heartbeat, "/heartbeat", self.receive_heartbeat, 10)

        self.state_publisher = self.create_publisher(ASV2State, "/boat_state", 10)
        self.state_message = ASV2State()

        self.timer = self.create_timer(UPDATE_RATE, self.update)

        self.paused = True

        print("starting task manager (paused for now)")

    def update(self):
        if (self.clock.now() - self.last_heartbeat_timestamp).nanoseconds / 1e9 > TIMEOUT_TIME:
            raise Exception("lost heartbeat, turning off task manager")
        
        if self.paused:
            return

        self.check_transition()
        self.task.update()

        self.state_message.current_state = self.task.get_name()
        self.state_publisher.publish(self.state_message)

    def check_transition(self):
        transition = self.task.get_next()
        if transition is not None:
            self.task.end()
            self.task = transition
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
