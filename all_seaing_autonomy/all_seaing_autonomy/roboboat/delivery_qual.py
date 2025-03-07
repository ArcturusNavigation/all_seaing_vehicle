# #!/usr/bin/env python3
# import rclpy
# from rclpy.action import ActionServer, ActionClient
# from rclpy.executors import MultiThreadedExecutor


# from all_seaing_interfaces.action import Task
# from all_seaing_controller.pid_controller import PIDController
# from ament_index_python.packages import get_package_share_directory
# from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption
# from all_seaing_common.action_server_base import ActionServerBase
# from sensor_msgs.msg import CameraInfo

# import os
# import yaml
# import time

# class DeliveryQual(ActionServerBase):
#     def __init__(self):
#         super().__init__("delivery_qual_server")

#         self._action_server = ActionServer(
#             self,
#             Task,
#             "delivery_qual",
#             execute_callback=self.execute_callback,
#             cancel_callback=self.default_cancel_callback,
#         )

#         self.bbox_sub = self.create_subscription(
#             LabeledBoundingBox2DArray, 
#             "bounding_boxes", 
#             self.bbox_callback, 
#             10
#         )

#         self.intrinsics_sub = self.create_subscription(
#             CameraInfo, 
#             "camera_info",
#             self.intrinsics_callback,
#             10
#         )

#         self.control_pub = self.create_publisher(
#             ControlOption, 
#             "control_options", 
#             10
#         )

#         bringup_prefix = get_package_share_directory("all_seaing_bringup")
#         self.declare_parameter("is_sim", False)
        
#         self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

#         # update from subs
#         self.height = None
#         self.width = None
#         self.bboxes = []

#         self.timer_period = 1 / 30.0

#         # change this to shapes
#         self.declare_parameter(
#             "shape_label_mappings_file", 
#             os.path.join(
#                 bringup_prefix, "config", "perception", "shape_label_mappings.yaml"
#             ),
#         )
#         shape_label_mappings_file = self.get_parameter(
#             "shape_label_mappings_file"
#         ).value
#         with open(shape_label_mappings_file, "r") as f:
#             label_mappings = yaml.safe_load(f)
#         self.cross_label = label_mappings["black_cross"]
#         self.triangle_label = label_mappings["black_triangle"]

#         self.declare_parameter("forward_speed", 5.0)
#         self.declare_parameter("max_yaw", 1.0)
#         self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
#         self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

#         pid_vals = (
#             self.declare_parameter("pid_vals", [0.003, 0.0, 0.0])
#             .get_parameter_value()
#             .double_array_value
#         )
#         self.pid = PIDController(*pid_vals)
#         self.pid.set_effort_min(-self.max_yaw_rate)
#         self.pid.set_effort_max(self.max_yaw_rate)
    

#     def intrinsics_callback(self, msg):
#         self.height = msg.height
#         self.width = msg.width

#     def bbox_callback(self, msg):
#         self.bboxes = msg.boxes

#     def control_loop(self):

#         box_center_x = None
#         box_area = 0

#         for box in self.bboxes:
#             if not box.label == self.triangle_label:
#                 continue
#             area = (box.max_x - box.min_x) * (box.max_y - box.min_y)
#             midpt = (box.max_x + box.min_x) / 2.0
#             if area > box_area:
#                 box_area = area
#                 box_center_x = midpt
        
#         # rotate very very very slowly and explore
#         if box_center_x is None:
#             control_msg = ControlOption()
#             control_msg.priority = 1
#             control_msg.twist.linear.x = 0.0
#             control_msg.twist.linear.y = 0.0
#             control_msg.twist.angular.z = float(self.max_yaw_rate * 0.1)
#             self.control_pub.publish(control_msg)
#             return 

#         img_ctr = self.width / 2.0
#         error = img_ctr - box_center_x

        
        

        

#         # publish control at the end 
#         control_msg = ControlOption()
#         control_msg.priority = 1
#         control_msg.twist.linear.x = float(self.forward_speed)
#         control_msg.twist.linear.y = 0.0
#         control_msg.twist.angular.z = float(yaw_rate)
#         self.control_pub.publish(control_msg)

#     def execute_callback(self, goal_handle):
#         self.start_process("starting moving to delivery vessels")

#         while not self.result:

#             if self.should_abort():
#                 self.end_process("aborting moving to delivery")
#                 goal_handle.abort()
#                 return Task.Result()

#             if goal_handle.is_cancel_requested:
#                 self.end_process("cancelling moving to delivery")
#                 goal_handle.canceled()
#                 return Task.Result()

#             self.control_loop()
#             time.sleep(self.timer_period)
        
#         self.end_process("moving to delivery completed!")
#         goal_handle.succeed()
#         return Task.Result(success=True)



# def main(args=None):
#     rclpy.init(args=args)
#     node = DeliveryQual()
#     executor = MultiThreadedExecutor(num_threads=2)
#     executor.add_node(node)
#     executor.spin()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

    