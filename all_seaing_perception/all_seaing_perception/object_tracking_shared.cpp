#include "all_seaing_perception/object_tracking_shared.hpp"

namespace all_seaing_perception{

    template<typename PointT>
    ObjectCloud<PointT>::ObjectCloud(rclcpp::Time t, int l, typename all_seaing_perception::Obstacle<PointT> obs) : obstacle(obs) {
        label = l;
        time_seen = t;
        last_dead = rclcpp::Time(0);
        time_dead = rclcpp::Duration(0, 0);
        is_dead = false;
    }

    Banner::Banner(rclcpp::Time t, int l, all_seaing_interfaces::msg::LabeledObjectPlane msg){
        label = l;
        time_seen = t;
        last_dead = rclcpp::Time(0);
        time_dead = rclcpp::Duration(0, 0);
        is_dead = false;
        plane_msg = msg;
    }

    template<typename PointT>
    std::shared_ptr<ObjectCloud<PointT>> clone(typename std::shared_ptr<ObjectCloud<PointT>> orig){
        std::shared_ptr<ObjectCloud<PointT>> new_ocl = std::make_shared<ObjectCloud<PointT>>(*orig);
        return new_ocl;
    }

    template<typename PointT>
    std::vector<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> orig){
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> new_vec;
        for(auto ocl : orig){
            new_vec.push_back(clone(ocl));
        }
        return new_vec;
    }

    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta){
        double dx = cos(from_theta)*(to_x-from_x)+sin(from_theta)*(to_y-from_y);
        double dy = -sin(from_theta)*(to_x-from_x)+cos(from_theta)*(to_y-from_y);
        double dtheta = to_theta - from_theta;
        return std::make_tuple(dx, dy, dtheta);
    }

    // multiply left to right
    std::tuple<double, double, double> compose_transforms(std::tuple<double, double, double> t1, std::tuple<double, double, double> t2){
        double t1_dx, t1_dy, t1_dtheta, t2_dx, t2_dy, t2_dtheta;
        std::tie(t1_dx, t1_dy, t1_dtheta) = t1;
        std::tie(t2_dx, t2_dy, t2_dtheta) = t2;
        double t_dx = t1_dx+cos(t1_dtheta)*t2_dx-sin(t1_dtheta)*t2_dy;
        double t_dy = t1_dy+sin(t1_dtheta)*t2_dx+cos(t1_dtheta)*t2_dy;
        double t_dtheta = t1_dtheta+t2_dtheta;
        return std::make_tuple(t_dx, t_dy, t_dtheta);
    }

    // given the starting and ending robot positions (wrt to world) and object pos wrt world given first robot position being true, compute world to object given second robot position being true (robot to object should be the same)
    // useful to overlay maps with two different robot positions
    std::tuple<double, double, double> apply_transform_from_to(double x, double y, double theta, double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta){
        std::tuple<double, double, double> local_rel = compute_transform_from_to(from_x, from_y, from_theta, x, y, theta);// from starting pos to object
        return compose_transforms(std::make_tuple(to_x, to_y, to_theta), local_rel);
    }

    double mod_2pi(double angle){ // pushes angle to [0,2pi) range
        return angle >= 2*M_PI?angle-floor(angle/(2*M_PI))*2*M_PI:angle+ceil(-angle/(2*M_PI))*2*M_PI;
    }

    double angle_to_pi_range(double angle){ // pushes angle to [-pi, pi) range
        return mod_2pi(angle+M_PI)-M_PI;
    }

    double bidirectional_angle_to_pi_range(double angle){
        double pi_range_angle = angle_to_pi_range(angle);
        return abs(pi_range_angle) < M_PI/((float)2) ? pi_range_angle : angle_to_pi_range(angle+M_PI);
    }

    //(range, bearing, signature)
    template<typename PointT>
    std::tuple<float, float, int> local_to_range_bearing_signature(PointT point, int label) {
        double range = std::hypot(point.x, point.y);
        double bearing = mod_2pi(std::atan2(point.y, point.x));
        return std::make_tuple(range, bearing, label);
    }

    //(range, bearing, phi, signature)
    std::tuple<float, float, float, int> local_banner_to_range_bearing_signature(geometry_msgs::msg::Pose pose, int label){
        double range = std::hypot(pose.position.x, pose.position.y);
        double bearing = mod_2pi(std::atan2(pose.position.y, pose.position.x));
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 mat(q);
        double r, p, phi;
        mat.getRPY(r, p, phi);
        if(r > M_PI/2 || p > M_PI/2){ // to discard weird RPY solutions
            mat.getRPY(r, p, phi, 2);
        }
        return std::make_tuple(range, bearing, phi, label);
    }

    geometry_msgs::msg::TransformStamped get_tf(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer, const std::string &in_target_frame,
                                                                const std::string &in_src_frame) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            // RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        }
        return tf;
    }

    template class ObjectCloud<pcl::PointXYZ>;
    template class ObjectCloud<pcl::PointXYZI>;
    template class ObjectCloud<pcl::PointXYZHSV>;

    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZ point, int label);
    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZI point, int label);
    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZHSV point, int label);
}