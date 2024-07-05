#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry


class OdomPrinter(Node):

    def __init__(self):
        super().__init__("odom_printer")
        self.create_subscription(Odometry, "/odometry/filtered", self.printer, 10)

    def printer(self, data):
        pos = data.pose.pose.position
        ori = data.pose.pose.orientation
        yaw = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler("xyz")[2]
        print(
            "POSE     ",
            ("%.3f" % pos.x),
            "    ",
            ("%.3f" % pos.y),
            "   ",
            ("%.3f" % pos.z),
            "     HEADING     ",
            ("%.4f" % yaw),
        )


def main(args=None):
    rclpy.init(args=args)
    pub = OdomPrinter()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
