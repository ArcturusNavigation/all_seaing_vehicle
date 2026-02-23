#!/usr/bin/env python3
import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor

from all_seaing_interfaces.action import Task
from ament_index_python.packages import get_package_share_directory
from all_seaing_common.task_server_base import TaskServerBase
from all_seaing_interfaces.msg import KeyboardButton 
from enum import Enum
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class ReturnState(Enum):
    SETTING_UP = 1
    STATIONKEEPING = 2

class HarborAlert(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "harbor_alert_server", action_name = "harbor_alert", search_action_name = "search_harbor_alert")

        self.declare_parameter("buoy_dist_thres", 40.0)
        self.buoy_dist_thres = self.get_parameter("buoy_dist_thres").get_parameter_value().double_value

        self.declare_parameter("stationkeep_dist", 2.0)
        self.stationkeep_dist = self.get_parameter("stationkeep_dist").get_parameter_value().double_value

        self.declare_parameter("probe_distance", 10)
        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("is_sim", False)

        self.blue_buoy_pos = None

        
        self.key_presses = 0
        # self.return_pos = None
        # self.return_dir = None
        
        # self.buoy_found = False

        # self.obstacles = None

        self.state = ReturnState.SETTING_UP

        if self.is_sim: 
            self.get_logger().info("Running in simulation mode. Listening to joystick input.")
            self.keyboard_sub = self.create_subscription(
                Joy, "/joy", self.real_keyboard_callback, 10
            )
        else: 
            self.keyboard_sub = self.create_subscription(
                KeyboardButton, "/keyboard_button", self.real_keyboard_callback, 10
            )

        # bringup_prefix = get_package_share_directory("all_seaing_bringup")

        # self.blue_labels = set()

        # self.declare_parameter(
        #     "color_label_mappings_file",
        #     os.path.join(
        #         bringup_prefix, "config", "perception", "color_label_mappings.yaml"
        #     ),
        # )

        # color_label_mappings_file = self.get_parameter(
        #     "color_label_mappings_file"
        # ).value
        # with open(color_label_mappings_file, "r") as f:
        #     label_mappings = yaml.safe_load(f)


        # if self.get_parameter("is_sim").get_parameter_value().bool_value:
        #     # hardcoded from reading YAML
        #     self.blue_labels.add(label_mappings["yellow"])
        # else:
        #     for buoy_label in ["yellow_buoy", "yellow_racquet_ball"]:
        #         self.blue_labels.add(label_mappings[buoy_label])
    
    # def should_accept_task(self, goal_request=None):
    #     if self.obstacles is None:
    #         return False
    #     return self.blue_buoy_detected()
    
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Task Server [{self.server_name}] received task')
        return GoalResponse.ACCEPT
    
    # def search_goal_callback(self, goal_request):
    #     self.return_pos = self.robot_pos
    #     self.return_dir = self.robot_dir
    #     self.get_logger().info(f'Searching Server for [{self.server_name}] called')
    #     return GoalResponse.ACCEPT

    def init_setup(self):
        self.blue_buoy_pos = None
        self.mark_successful()

    def control_loop(self):
        if self.state == ReturnState.STATIONKEEPING:
            self.get_logger().info(f'Going to waypoint at {self.blue_buoy_pos}')
            self.station_keep_blue_buoy()
            self.mark_successful()
        elif self.state == ReturnState.SETTING_UP:
            if self.blue_buoy_pos is None:
                self.blue_buoy_pos = self.robot_pos
                self.get_logger().info(f'Finished setting up waypoint, ready to enter station keeping at {self.blue_buoy_pos}')
            if self.key_presses == 2:
                self.state = ReturnState.STATIONKEEPING
                self.mark_successful()
                self.get_logger().info(f'Key pressed, moving on to next task')

        
    # def map_cb(self, msg):
    #     '''
    #     Gets the labeled map from all_seaing_perception.
    #     '''
    #     self.obstacles = msg.obstacles

    def real_keyboard_callback(self, msg):
        if msg.key == "p":
            self.key_presses += 1

    def station_keep_blue_buoy(self):
        self.get_logger().info("Going to the preset waypoint")

        # robot_x, robot_y = self.robot_pos
        # robot_buoy_vector = (self.blue_buoy_pos[0]-robot_x, self.blue_buoy_pos[1]-robot_y)
        # robot_buoy_dist = self.norm(robot_buoy_vector)
        # buoy_direction = (robot_buoy_vector[0]/robot_buoy_dist, robot_buoy_vector[1]/robot_buoy_dist)

        # add_tuple = lambda a,b: tuple(sum(x) for x in zip(a, b))
        # times_tuple = lambda a,b: tuple(a*x for x in b)
        # minus_tuple = lambda a,b: add_tuple(a, times_tuple(-1, b))

        # def update_current_point():
        #     self.blue_buoy_detected()
            
        #     return minus_tuple(self.blue_buoy_pos, times_tuple(self.stationkeep_dist, buoy_direction))

        self.move_to_point(self.blue_buoy_pos, busy_wait=True)
        self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)


        # self.get_logger().info("")
        # self.move_to_point(self.return_pos, busy_wait=True)

        # if self.return_dir is None:
        #     return Task.Result(success=True)

        # self.get_logger().info('Fixing orientation')
        # self.move_to_waypoint((self.return_pos[0], self.return_pos[1], math.atan2(self.return_dir[1], self.return_dir[0])), busy_wait=True)
        
        # return Task.Result(success=True)

    # def norm_squared(self, vec, ref=(0, 0)):
    #     return (vec[0] - ref[0])**2 + (vec[1]-ref[1])**2

    # def norm(self, vec, ref=(0, 0)):
    #     return math.sqrt(self.norm_squared(vec, ref))
    
    # def ob_coords(self, buoy, local=False):
    #     if local:
    #         return (buoy.local_point.point.x, buoy.local_point.point.y)
    #     else:
    #         return (buoy.global_point.point.x, buoy.global_point.point.y)

    # def blue_buoy_detected(self, buoy_front=False):
    #     '''
    #     Check if the blue buoy for turning is detected (returns boolean).
    #     Also sets the position of the blue buoy if it is found.
    #     '''
    #     backup_buoy = None
    #     updated_pos = False
    #     for obstacle in self.obstacles:
    #         if obstacle.label in self.blue_labels:
    #             if self.norm(self.robot_pos, self.ob_coords(obstacle)) > self.buoy_dist_thres:
    #                 continue
    #             buoy_dir = (obstacle.global_point.point.x-self.robot_pos[0], 
    #                         obstacle.global_point.point.y-self.robot_pos[1])
    #             dot_prod = buoy_dir[0] * self.robot_dir[0] + buoy_dir[1] * self.robot_dir[1]
    #             buoy_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
    #             if (backup_buoy is None) or (self.buoy_found and (self.norm(self.blue_buoy_pos, buoy_pos) < self.norm(self.blue_buoy_pos, backup_buoy))):
    #                 backup_buoy = buoy_pos
    #             if ((not buoy_front) or (dot_prod > 0)) and ((not self.buoy_found) or (self.norm(self.blue_buoy_pos, buoy_pos) < self.duplicate_dist)): #check if buoy position is behind robot i.e. dot product is negative
    #                 self.buoy_found = True
    #                 updated_pos = True
    #                 self.blue_buoy_pos = buoy_pos
    #                 break
    #     if (not updated_pos) and (backup_buoy is not None):
    #         self.get_logger().info('SWITCHING TO BACKUP BUOY')
    #         self.blue_buoy_pos = backup_buoy
    #     return self.buoy_found



def main(args=None):
    rclpy.init(args=args)
    node = HarborAlert()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
