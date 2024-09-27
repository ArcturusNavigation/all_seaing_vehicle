import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from geometry_msgs.msg import PointStamped, Pose, PoseArray

class WaypointFinder(Node):
    def __init__(self):
        super().__init__("waypoint_finder")
        self.map_sub = self.create_subscription(
            ObstacleMap, "labeled_map", self.map_cb, 10
        )

        self.waypoint_pub = self.create_publisher(PoseArray, "waypoints", 10)

    def norm_squared(self, vec, ref=(0, 0)):
        return vec[0] ** 2 + vec[1] ** 2

    def get_closest_waypoint_to(self, point, waypoints):
        return min(
            waypoints,
            key=lambda waypoint: self.norm_squared(waypoint, point)
        )

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def generate_waypoints(self):
        green_bouy_points = []
        red_bouy_points = []
        for obstacle in self.obstacles:
            if obstacle.label == 2:
                green_bouy_points.append((obstacle.local_point.point.x, obstacle.local_point.point.y))
            elif obstacle.label == 1:
                red_bouy_points.append((obstacle.local_point.point.x, obstacle.local_point.point.y))
            
        if len(green_bouy_points) == 0 or len(red_bouy_points) == 0:
            return
        
        closest_green = get_closest_waypoint_to((0, 0), green_bouy_points)
        closest_red = get_closest_waypoint_to((0, 0), red_bouy_points)

        green_bouy_points.remove(closest_green)
        red_bouy_points.remove(closest_red)

        waypoints = [midpoint(closest_green, closest_red)]

        # will terminate if we run out of either of the points
        while len(green_bouy_points) > 0 and len(red_bouy_points) > 0:
            last_waypoint = waypoints[-1]
            
            closest_green = get_closest_waypoint_to(last_waypoint, green_bouy_points)
            closest_red = get_closest_waypoint_to(last_waypoint, red_bouy_points)

            green_bouy_points.remove(closest_green)
            red_bouy_points.remove(closest_red)

            waypoints.append(midpoint(closest_green, closest_red))
        
        pose_array = PoseArray()
        pose_list = [Pose(position=PointStamped(x=waypoint[0], y=waypoint[1])) for waypoint in waypoints]
        pose_array.poses = pose_list
        self.waypoint_pub.publish(pose_array)

    def map_cb(self, msg):
        self.obstacles = msg.obstacles

        

        
        
def main(args=None):
    rclpy.init(args=args)
    node = WaypointFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()