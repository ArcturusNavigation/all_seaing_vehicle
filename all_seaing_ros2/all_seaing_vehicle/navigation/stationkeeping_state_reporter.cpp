#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"
#include "all_seaing_interfaces/msg/goal_state.hpp"
#include "all_seaing_interfaces/msg/control_message.hpp"

using namespace std::chrono_literals;

class StationkeepingStateReporter : public rclcpp::Node {
    public:
        StationkeepingStateReporter() : Node("stationkeeping_state_reporter") {
            m_state_sub = this->create_subscription<all_seaing_interfaces::msg::GoalState>(
                "/goal_state",
                10,
                std::bind(&StationkeepingStateReporter::state_callback, this, std::placeholders::_1)
            );

            m_gateway_sub = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
                "/gateway_msg",
                10,
                std::bind(&StationkeepingStateReporter::vel_callback, this, std::placeholders::_1)
            );

            m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
            m_control_pub = this->create_publisher<all_seaing_interfaces::msg::ControlMessage>("/control_input", 10);
        }

    private:

        //Defining Pubs/Subs
        rclcpp::Subscription<all_seaing_interfaces::msg::GoalState>::SharedPtr m_state_sub;
        rclcpp::Publisher<all_seaing_interfaces::msg::ControlMessage>::SharedPtr m_control_pub;
        rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_pub;
        rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_sub;

        //Setting message type to ControlMessage
        all_seaing_interfaces::msg::ControlMessage m_control = all_seaing_interfaces::msg::ControlMessage();


        //Publisher to MOOS
        void state_callback(const all_seaing_interfaces::msg::GoalState & msg) {
            auto nav_msg = protobuf_client_interfaces::msg::Gateway();
            nav_msg.gateway_key = "ROS_REPORT_GOAL";
            nav_msg.gateway_string = "GOAL_LAT=" + std::to_string(msg.goal_lat) + ", GOAL_LON=" + std::to_string(msg.goal_lon) +
                                     ", GOAL_HEADING=" + std::to_string(msg.goal_heading);
            m_gateway_pub->publish(nav_msg);
        }

        //Parser from MOOS
        void vel_callback(const protobuf_client_interfaces::msg::Gateway & msg) {
            if (msg.gateway_key == "VEL_X") m_control.x = msg.gateway_double;
            if (msg.gateway_key == "VEL_Y") m_control.y = msg.gateway_double;
            if (msg.gateway_key == "GOAL_HEADING") {m_control.angular = msg.gateway_double; m_control.angular_control_mode = all_seaing_interfaces::msg::ControlMessage::WORLD_POSITION; }
            m_control_pub->publish(m_control);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StationkeepingStateReporter>());
    rclcpp::shutdown();
    return 0;
}
