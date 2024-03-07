#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from all_seaing_interfaces.msg import ASV2State

class Task:
    def get_name():
        return "uhh... this class was supposed to be abstract lmao"
    def start():
        pass
    def update():
        pass
    def check_finished():
        return False
    def end():
        pass
    def get_next():
        pass

class NavigationChannel(Task):
    def get_name():
        # the idea is that since this is a method, we can edit the name to display some sort of information for debugging
        return "NavigationChannel"
    def start():
        # start moos behavior of path following
        pass
    def update():
        # pass new waypoints into moos, reading from perception
        pass
    def end():
        # remove moos behavior or something
        pass
    def get_next():
        # check if moos is done, if so return the next task
        return None
    
class Idling(Task):
    def update():
        # keep boat in place
        pass
    
STARTING_TASK = NavigationChannel()
TIMEOUT_TIME = 3 # seconds
UPDATE_RATE = 1/60 # seconds

class RoboboatDoer(Node):
    def __init__(self):
        self.task = STARTING_TASK
        self.task.start()
        self.clock = self.get_clock()
        self.last_heartbeat_timestamp = self.clock.now()

        self.create_subscription(Header, "/heartbeat", self.receive_heartbeat, 10)

        self.state_publisher = self.create_publisher(ASV2State, "/boat_state", 10)
        self.state_message = ASV2State()

        self.timer = self.create_timer(UPDATE_RATE, self.update)

    def update(self):
        if (self.clock.now() - self.last_heartbeat_timestamp).nanoseconds / 1e9 > TIMEOUT_TIME:
            raise Exception("lost heartbeat, turning off roboboat doer")
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

    def receive_heartbeat(self, _):
        self.last_heartbeat_timestamp = self.clock.now()

def main(args=None):
    rclpy.init(args=args)
    roboboat_doer = RoboboatDoer()
    rclpy.spin(roboboat_doer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
