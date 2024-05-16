#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"
#include "all_seaing_interfaces/msg/control_message.hpp"

using namespace std::chrono_literals;

class MoosToController : public rclcpp::Node
{
public:
    MoosToController() : Node("moos_to_controller")
    {
        m_gateway_sub = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
            "/gateway_msg",
            10,
            std::bind(&MoosToController::moos_callback, this, std::placeholders::_1));

        m_control_pub = this->create_publisher<all_seaing_interfaces::msg::ControlMessage>("/control_input", 10);
    }

private:
    // Defining Pubs/Subs
    rclcpp::Publisher<all_seaing_interfaces::msg::ControlMessage>::SharedPtr m_control_pub;
    rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_sub;

    double moos_thrust;
    double moos_heading;

    // MOOS to Controller Parser
    void moos_callback(const protobuf_client_interfaces::msg::Gateway &msg)
    {

        // Setting message type to ControlMessage
        all_seaing_interfaces::msg::ControlMessage control = all_seaing_interfaces::msg::ControlMessage();

        control.linear_control_mode = all_seaing_interfaces::msg::ControlMessage::LOCAL_VELOCITY;
        control.angular_control_mode = all_seaing_interfaces::msg::ControlMessage::WORLD_POSITION;

        if (msg.gateway_key == "DESIRED_THRUST")
        {
            moos_thrust = msg.gateway_double;
        } // mapping = 100:5 = thrust:speed (m/s)

        if (msg.gateway_key == "DESIRED_HEADING")
        {
            moos_heading = ((msg.gateway_double - 90) * (3.14159 / 180));
        }

        double speed = moos_thrust * .05;

        // control.x = moos_thrust;
        control.x = speed;
        control.angular = -moos_heading;

        m_control_pub->publish(control);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoosToController>());
    rclcpp::shutdown();
    return 0;
}
