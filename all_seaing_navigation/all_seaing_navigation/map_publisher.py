import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped, Pose
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory, Map, PriorityQueue

class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Map()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: map ' )


def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
