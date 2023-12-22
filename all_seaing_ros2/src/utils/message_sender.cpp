#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"
#include "all_seaing_interfaces/msg/asv_state.hpp"

using namespace std::chrono_literals;

class MessageSender : public rclcpp::Node {
    public:
        MessageSender() : Node("message_sender") {
            m_state_sub = this->create_subscription<all_seaing_interfaces::msg::ASVState>(
                "/allseaing_main/state",
                10,
                std::bind(&MessageSender::state_callback, this, std::placeholders::_1)
            );

            m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
        }

    private:
        rclcpp::Subscription<all_seaing_interfaces::msg::ASVState>::SharedPtr m_state_sub;
        rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_pub;

        void state_callback(const all_seaing_interfaces::msg::ASVState & msg) {
            auto nav_msg = protobuf_client_interfaces::msg::Gateway();
            nav_msg.gateway_key = "ROS_REPORT";
            nav_msg.gateway_string = "NAV_LAT=" + std::to_string(msg.nav_lat) + ", NAV_LON=" + std::to_string(msg.nav_long) + 
                                     ", NAV_HEADING=" + std::to_string(msg.nav_heading) + ", NAV_SPEED=" + std::to_string(msg.nav_speed);
            m_gateway_pub->publish(nav_msg);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageSender>());
    rclcpp::shutdown();
    return 0;
}
