import rclpy
from rclpy.node import Node
import rclpy.qos


def add_qos_parameter(node: Node, default_qos="SYSTEM_DEFAULT", parameter_name="qos"):
    return parse_qos_string(
        node.declare_parameter(parameter_name, default_qos)
        .get_parameter_value()
        .string_value
    )


def parse_qos_string(qos_str: str):
    profile = qos_str.upper()

    if profile == "SYSTEM_DEFAULT":
        return rclpy.qos.qos_profile_system_default
    if profile == "DEFAULT":
        return rclpy.qos.QoSProfile(depth=10)
    if profile == "PARAMETER_EVENTS":
        return rclpy.qos.qos_profile_parameter_events
    if profile == "SERVICES_DEFAULT":
        return rclpy.qos.qos_profile_services_default
    if profile == "PARAMETERS":
        return rclpy.qos.qos_profile_parameters
    if profile == "SENSOR_DATA":
        return rclpy.qos.qos_profile_sensor_data

    Node("parseQoSString").get_logger().warn(
        f"Unknown QoS profile: {profile}. Returning profile: DEFAULT"
    )
    return rclpy.qos.QoSProfile(depth=10)
