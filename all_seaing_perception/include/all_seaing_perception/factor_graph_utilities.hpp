#ifndef ALL_SEAING_PERCEPTION__FACTOR_GRAPH_UTILITIES_HPP
#define ALL_SEAING_PERCEPTION__FACTOR_GRAPH_UTILITIES_HPP


#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <tuple>
#include <chrono>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/distances.h>
#include "pcl_conversions/pcl_conversions.h"

#include "cv_bridge/cv_bridge.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "all_seaing_interfaces/msg/labeled_object_point_cloud_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud.hpp"
#include "all_seaing_interfaces/msg/labeled_object_plane_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_plane.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"

#include "all_seaing_perception/obstacle.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "all_seaing_perception/Hungarian.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

namespace all_seaing_perception {
    //custom struct to also keep the points themselves with the obstacle (Obstacle doesn't do that and don't want to mess with it)    
    template<typename PointT>
    struct ObjectCloud{
        int label;
        rclcpp::Time time_seen;
        rclcpp::Time last_dead;
        rclcpp::Duration time_dead = rclcpp::Duration(0,0);
        bool is_dead;
        all_seaing_perception::Obstacle<PointT> obstacle;
        Eigen::Vector2d mean_pred;
        Eigen::Matrix2d cov;
        gtsam::Key node_key;

        ObjectCloud(): obstacle(all_seaing_perception::Obstacle<PointT>(std_msgs::msg::Header(), std_msgs::msg::Header(), all_seaing_interfaces::msg::Obstacle())){
        
        }
        ObjectCloud(rclcpp::Time t, int l, all_seaing_perception::Obstacle<PointT> obs);
    };

    struct Banner{
        int label;
        rclcpp::Time time_seen;
        rclcpp::Time last_dead;
        rclcpp::Duration time_dead = rclcpp::Duration(0,0);
        bool is_dead;
        Eigen::Vector3d mean_pred;
        Eigen::Matrix3d cov;
        all_seaing_interfaces::msg::LabeledObjectPlane plane_msg;
        gtsam::Key node_key;

        Banner();
        Banner(rclcpp::Time t, int l, all_seaing_interfaces::msg::LabeledObjectPlane msg);
    };

    class UnaryPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
        double mx_, my_, mtheta_;
        bool incl_theta_, only_theta_;
    public:
        UnaryPoseFactor(gtsam::Key key, double x, double y, double theta, const gtsam::SharedNoiseModel& noise_model, 
            bool include_theta=true, bool only_theta=false):
            gtsam::NoiseModelFactor1<gtsam::Pose2>(noise_model, key), mx_(x), my_(y), mtheta_(theta), incl_theta_(include_theta), only_theta_(only_theta) {}
        gtsam::Vector evaluateError(const gtsam::Pose2& pose, boost::optional<gtsam::Matrix&> H = boost::none) const {
            gtsam::Vector error = gtsam::Vector3(only_theta_? 0 : pose.x() - mx_, only_theta_? 0 : pose.y() - my_, incl_theta_? pose.theta() - mtheta_ : 0);
            // Same dimensions for measurement & error regardless of have/no theta so that we can use the same noise model
            if (H) {
                (*H) = gtsam::Matrix::Zero(3, 3);
                if (incl_theta_) {
                    (*H)(2, 2) = 1; // exp map for theta is just addition of angles
                }
                if (!only_theta_) {
                    (*H).block<2, 2>(0, 0) = pose.rotation().matrix(); // because of exp map
                }
            }
            return error;
        }
    };

    double Pose2Theta(const gtsam::Pose2& pose, gtsam::OptionalJacobian<1, 3> H) {
    if (H) *H = (gtsam::Matrix13() << 0, 0, 1).finished();
        return pose.theta();
    }

    double Rot2Theta(const gtsam::Rot2& rot, gtsam::OptionalJacobian<1, 1> H) {
        if (H) *H = gtsam::Matrix11::Identity();
        return rot.theta();
    }

    double BearingRangeRange(const gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>& br,
                            gtsam::OptionalJacobian<1, 2> H) {
        if (H) *H = (gtsam::Matrix12() << 0, 1).finished();
        return br.range();
    }

    gtsam::Rot2 BearingRangeBearing(const gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>& br,
                                    gtsam::OptionalJacobian<1, 2> H) {
        if (H) *H = (gtsam::Matrix12() << 1, 0).finished();
        return br.bearing();
    }

    gtsam::Vector3 makeVector3(double a, double b, double c,
        gtsam::OptionalJacobian<3, 1> Ha,
        gtsam::OptionalJacobian<3, 1> Hb,
        gtsam::OptionalJacobian<3, 1> Hc) {
        if (Ha) *Ha = (gtsam::Vector3() << 1, 0, 0).finished();
        if (Hb) *Hb = (gtsam::Vector3() << 0, 1, 0).finished();
        if (Hc) *Hc = (gtsam::Vector3() << 0, 0, 1).finished();
        return gtsam::Vector3(a, b, c);
    }

    class BearingRangePhiFactor : public gtsam::ExpressionFactor2<gtsam::Vector3, gtsam::Pose2, gtsam::Pose2> {
    public:
        BearingRangePhiFactor(gtsam::Key key1, gtsam::Key key2, double bearing, double range, double phi, const gtsam::SharedNoiseModel& noise_model):
            gtsam::ExpressionFactor2<gtsam::Vector3, gtsam::Pose2, gtsam::Pose2>(key1, key2, noise_model, gtsam::Vector3(bearing, range, phi)) {
                this->initialize(expression(key1, key2));
            }
        // gtsam::Vector evaluateError(const gtsam::Pose2& robot_pose, const gtsam::Pose2& banner_pose, boost::optional<gtsam::Matrix&> H_robot = boost::none, boost::optional<gtsam::Matrix&> H_banner = boost::none) const {
        //     double pred_bearing = gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>::MeasureBearing(robot_pose, banner_pose).theta();
        //     double pred_range = gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>::MeasureRange(robot_pose, banner_pose);
        //     double dx = banner_pose.x() - robot_pose.x();
        //     double dy = banner_pose.y() - robot_pose.y();
        //     double dtheta = banner_pose.theta() - robot_pose.theta();
        //     gtsam::Vector error = gtsam::Vector3(pred_bearing, pred_range, dtheta) - gtsam::Vector3(mbearing_, mrange_, mphi_);
        //     if (H_robot) {
        //         (*H_robot) = gtsam::Matrix::Zero(3, 3);
        //         (*H_robot)(0, 2) = -1;
        //         (*H_robot)(1, 0) = ()/pred_range
        //         (*H_robot)(2,2) = -1;
        //     }
        //     if (H_banner) {
        //         (*H_banner) = gtsam::Matrix::Zero(3, 3);
        //         (*H_robot)(0, 2) = 1;
        //         (*H_banner).block<2, 2>(0, 0) = -robot_pose.rotation().matrix(); // because of exp map
        //     }
        //     return error;
        // }
        gtsam::Expression<gtsam::Vector3> expression(gtsam::Key key1, gtsam::Key key2) const override{
            gtsam::Expression<gtsam::Pose2> robot_pose(key1);
            gtsam::Expression<gtsam::Pose2> banner_pose(key2);
            gtsam::Expression<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>> pred_bearing_range(gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>::Measure, robot_pose, banner_pose);
            gtsam::Expression<gtsam::Rot2> pred_bearing(BearingRangeBearing, pred_bearing_range);
            gtsam::Expression<double> pred_range(BearingRangeRange, pred_bearing_range);
            gtsam::Expression<double> robot_theta(Pose2Theta, robot_pose);
            gtsam::Expression<double> banner_theta(Pose2Theta, banner_pose);
            gtsam::Expression<double> bearing_theta(Rot2Theta, pred_bearing);
            gtsam::Expression<double> pred_phi = banner_theta - robot_theta;
            // TODO change to use a combination of Rot2 and double for both the measurement and the expression
            // to not have issues with angle jumps, Rot2 automatically handles those by converting to local frame
            return gtsam::Expression<gtsam::Vector3>(makeVector3, bearing_theta, pred_range, pred_phi);
        }
    };

    template<typename PointT>
    std::shared_ptr<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::shared_ptr<ObjectCloud<PointT>> orig);

    template<typename PointT>
    std::vector<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> orig);

    double mod_2pi(double angle);
    double angle_to_pi_range(double angle);
    double bidirectional_angle_to_pi_range(double angle);

    template<typename PointT>
    std::tuple<float, float, int> local_to_range_bearing_signature(PointT point, int label);

    std::tuple<float, float, float, int> local_banner_to_range_bearing_signature(geometry_msgs::msg::Pose pose, int label);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped get_tf(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
        const std::string &in_target_frame, const std::string &in_src_frame);
                                                
    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);
    std::tuple<double, double, double> compose_transforms(std::tuple<double, double, double> t1, std::tuple<double, double, double> t2);
    std::tuple<double, double, double> apply_transform_from_to(double x, double y, double theta, double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);

    // Returns the matchings from detections to map and the sets of indices of chosen tracked and detected obstacles
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_indiv_var_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<float> v_indiv, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_meas_var_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> v_meas, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt=false);

    // Similar to the greedy_data_association function but returning the computed weight as well
    template<typename PointT>
    std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres);

    // for banners
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_banner_data_association(std::vector<std::shared_ptr<Banner>> tracked_obstacles,
        std::vector<std::shared_ptr<Banner>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres,
        std::map<int, int> banner_label_number = std::map<int,int>());

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__FACTOR_GRAPH_UTILITIES_HPP