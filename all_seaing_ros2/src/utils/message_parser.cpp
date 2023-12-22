#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "all_seaing_interfaces/msg/moos_command.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"

class MessageParser : public rclcpp::Node {
    public:
        MessageParser() : Node("message_parser") {
            m_command_publisher = this->create_publisher<all_seaing_interfaces::msg::MOOSCommand>("/moos/command", 10);
            m_coord_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("/moos/local_utm", 10);

            m_subscription = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
                "/gateway_msg",
                10,
                std::bind(&MessageParser::topic_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_subscription;
        rclcpp::Publisher<all_seaing_interfaces::msg::MOOSCommand>::SharedPtr m_command_publisher;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_coord_publisher;

        all_seaing_interfaces::msg::MOOSCommand command_msg = all_seaing_interfaces::msg::MOOSCommand();
        geometry_msgs::msg::PointStamped point_msg = geometry_msgs::msg::PointStamped();

        void topic_callback(const protobuf_client_interfaces::msg::Gateway & msg) {
            
            // Publish desired rudder and thrust (when DESIRED_THRUST is received)
            if (msg.gateway_key == "DESIRED_RUDDER") command_msg.desired_rudder = msg.gateway_double;
            if (msg.gateway_key == "DESIRED_THRUST") {
                command_msg.desired_thrust = msg.gateway_double;
                m_command_publisher->publish(command_msg);
            }

            // Publisher local utm coordinates (when NAV_Y is received)
            if (msg.gateway_key == "NAV_X") point_msg.point.x = msg.gateway_double;
            if (msg.gateway_key == "NAV_Y") {
                point_msg.header.stamp = msg.gateway_time;
                point_msg.point.y = msg.gateway_double;
                point_msg.point.z = 0;
                m_coord_publisher->publish(point_msg);
            }

        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageParser>());
    rclcpp::shutdown();
    return 0;
}
