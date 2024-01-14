from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec

class TaskNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.task_name = ""
        self.state = ""
        self.subscription = self.create_subscription(
            ParamVec,
            '/vrx/task/info',
            self.task_callback,
            10)

    def task_callback(self, msg: ParamVec):
        for elem in msg.params:
            if elem.name == "name":
                self.task_name = elem.value.string_value
            if elem.name == "state":
                self.state = elem.value.string_value
