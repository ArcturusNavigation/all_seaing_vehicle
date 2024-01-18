#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"
#include "all_seaing_interfaces/msg/goal_state.hpp"

using namespace std::chrono_literals;

class StationkeepingGoalSender : public rclcpp::Node {
    public:
        StationkeepingGoalSender() : Node("stationkeeping_goal_sender") {
            m_state_sub = this->create_subscription<all_seaing_interfaces::msg::GoalState>(
                "/goal_state",
                10,
                std::bind(&StationkeepingGoalSender::state_callback, this, std::placeholders::_1)
            );

            m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
        }

    private:
        rclcpp::Subscription<all_seaing_interfaces::msg::GoalState>::SharedPtr m_state_sub;
        rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_pub;

        void state_callback(const all_seaing_interfaces::msg::GoalState & msg) {
            auto nav_msg = protobuf_client_interfaces::msg::Gateway();
            nav_msg.gateway_key = "ROS_REPORT_GOAL";
            nav_msg.gateway_string = "GOAL_LAT=" + std::to_string(msg.goal_lat) + ", GOAL_LON=" + std::to_string(msg.goal_long) + 
                                     ", GOAL_HEADING=" + std::to_string(msg.goal_heading);
            m_gateway_pub->publish(nav_msg);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StationkeepingGoalSender>());
    rclcpp::shutdown();
    return 0;
}
