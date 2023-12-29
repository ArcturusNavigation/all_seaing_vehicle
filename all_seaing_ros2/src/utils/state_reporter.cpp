#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "all_seaing_interfaces/msg/asv_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class StateReporter : public rclcpp::Node {
    public:
        StateReporter() : Node("state_reporter") {

            m_timer = this->create_wall_timer(
                200ms,
                std::bind(&StateReporter::timer_callback, this)
            );

            m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data",
                10,
                std::bind(&StateReporter::imu_callback, this, std::placeholders::_1)
            );

            m_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix",
                10,
                std::bind(&StateReporter::gps_callback, this, std::placeholders::_1)
            );

            m_local_utm_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/moos/local_utm",
                10,
                std::bind(&StateReporter::local_utm_callback, this, std::placeholders::_1)
            );

            m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered",
                10,
                std::bind(&StateReporter::odom_callback, this, std::placeholders::_1)
            );

            m_state_pub = this->create_publisher<all_seaing_interfaces::msg::ASVState>("/allseaing_main/state", 10);

        }
    
    private:
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_local_utm_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
        rclcpp::Publisher<all_seaing_interfaces::msg::ASVState>::SharedPtr m_state_pub;

        all_seaing_interfaces::msg::ASVState m_state = all_seaing_interfaces::msg::ASVState();

        void imu_callback(const sensor_msgs::msg::Imu & msg) {
            tf2::Quaternion q;
            q.setW(msg.orientation.w);
            q.setX(msg.orientation.x);
            q.setY(msg.orientation.y);
            q.setZ(msg.orientation.z);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            //std::cout << "Roll: " << r << ", Pitch: " << p << ", Yaw: " << y << std::endl;
            // TODO: make NED/ENU IMU difference a ROSParam
            m_state.nav_heading = -y * 180 / M_PI + 90; // Offset and negative sign due to NED/ENU IMU difference
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            double vel_x = msg.twist.twist.linear.x;
            double vel_y = msg.twist.twist.linear.y;
            m_state.nav_speed = sqrt(vel_x * vel_x + vel_y * vel_y);
        }

        void local_utm_callback(const geometry_msgs::msg::PointStamped & msg) {
            m_state.nav_x = msg.point.x;
            m_state.nav_y = msg.point.y;
        }

        void gps_callback(const sensor_msgs::msg::NavSatFix & msg) {
            m_state.nav_lat = msg.latitude;
            m_state.nav_long = msg.longitude;
        }

        void timer_callback() {
            m_state.header.stamp = this->get_clock()->now();
            m_state_pub->publish(m_state);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateReporter>());
    rclcpp::shutdown();
    return 0;
}
