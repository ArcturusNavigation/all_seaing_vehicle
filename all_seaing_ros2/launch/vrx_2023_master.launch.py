from launch import LaunchDescription
import launch_ros

def generate_launch_description():

    return LaunchDescription([
        # controller
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="simple_controller.py",
            output="screen",
            remappings=[
                ("/left_thrust", "/wamv/thrusters/left/thrust"),
                ("/right_thrust", "/wamv/thrusters/right/thrust")],
            parameters=[
                {"linear_scaling": 25.0},
                {"angular_scaling": 15.0},
                {"lower_thrust_limit": -1000.0},
                {"upper_thrust_limit": 1000.0}]),

        # state reporter
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="state_reporter", 
            output="screen",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")]),

        # acoustic perception/tracking
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="vrx_2023_acoustic_sub.py",
            output="screen"),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="vrx_2023_acoustic_sender.py",
            output="screen"),

        # wayfinding
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="vrx_2023_waypoints.py", 
            output="screen"),

        # stationkeeping
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="vrx_2023_stationkeeping.py", 
            output="screen"),

        # task sender
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="vrx_2023_task_sender.py", 
            output="screen"),

        # MOOS-ROS bridge
        launch_ros.actions.Node(
	        package="protobuf_client", executable="protobuf_client_node", output="screen"),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", executable="message_parser", output="screen"),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", executable="message_sender", output="screen"),
    ])
