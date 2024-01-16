#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "all_seaing_interfaces/msg/asv_state.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"

using namespace std::chrono_literals;

class NavStateReporter : public rclcpp::Node {
    public:
        NavStateReporter() : Node("nav_state_reporter") {
            // Timer
            m_timer = this->create_wall_timer(
                200ms,
                std::bind(&NavStateReporter::timer_callback, this)
            );

            // Subscribers
            m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data",
                10,
                std::bind(&NavStateReporter::imu_callback, this, std::placeholders::_1)
            );
            m_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix",
                10,
                std::bind(&NavStateReporter::gps_callback, this, std::placeholders::_1)
            );
            m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered",
                10,
                std::bind(&NavStateReporter::odom_callback, this, std::placeholders::_1)
            );
            m_gateway_sub = this->create_subscription<protobuf_client_interfaces::msg::Gateway>(
                "/gateway_msg",
                10,
                std::bind(&NavStateReporter::gateway_callback, this, std::placeholders::_1)
            );

            // Publishers
            m_state_pub = this->create_publisher<all_seaing_interfaces::msg::ASVState>("/asv_state", 10);
            m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
        }
    
    private:
        // Timer
        rclcpp::TimerBase::SharedPtr m_timer;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_local_utm_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
        rclcpp::Subscription<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_sub;

        // Publishers
        rclcpp::Publisher<all_seaing_interfaces::msg::ASVState>::SharedPtr m_state_pub;
        rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr m_gateway_pub;

        // State variable to keep track of current ASV state
        all_seaing_interfaces::msg::ASVState m_state = all_seaing_interfaces::msg::ASVState();

        //-------------------- Callbacks --------------------//

        void imu_callback(const sensor_msgs::msg::Imu & msg) {
            tf2::Quaternion q;
            q.setW(msg.orientation.w);
            q.setX(msg.orientation.x);
            q.setY(msg.orientation.y);
            q.setZ(msg.orientation.z);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            // TODO: make NED/ENU IMU difference a ROSParam
            m_state.nav_heading = -y * 180 / M_PI + 90; // Offset and negative sign due to NED/ENU IMU difference
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            double vel_x = msg.twist.twist.linear.x;
            double vel_y = msg.twist.twist.linear.y;
            m_state.nav_speed = sqrt(vel_x * vel_x + vel_y * vel_y);
        }

        void gps_callback(const sensor_msgs::msg::NavSatFix & msg) {
            m_state.nav_lat = msg.latitude;
            m_state.nav_long = msg.longitude;
        }

        void gateway_callback(const protobuf_client_interfaces::msg::Gateway & msg) {
            if (msg.gateway_key == "DESIRED_RUDDER") m_state.desired_rudder = msg.gateway_double;
            if (msg.gateway_key == "DESIRED_THRUST") m_state.desired_thrust = msg.gateway_double;
            if (msg.gateway_key == "NAV_X") m_state.nav_x = msg.gateway_double;
            if (msg.gateway_key == "NAV_Y") m_state.nav_y = msg.gateway_double;
        }

        void timer_callback() {
            // Publish current state
            m_state.header.stamp = this->get_clock()->now();
            m_state_pub->publish(m_state);

            // Send latitude, longitude, and heading to MOOS
            auto gateway_msg = protobuf_client_interfaces::msg::Gateway();
            gateway_msg.gateway_key = "ROS_REPORT";
            gateway_msg.gateway_string = "NAV_LAT=" + std::to_string(m_state.nav_lat) + ", NAV_LON=" + std::to_string(m_state.nav_long) + 
                    ", NAV_HEADING=" + std::to_string(m_state.nav_heading) + ", NAV_SPEED=" + std::to_string(m_state.nav_speed);
            m_gateway_pub->publish(gateway_msg);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavStateReporter>());
    rclcpp::shutdown();
    return 0;
}
