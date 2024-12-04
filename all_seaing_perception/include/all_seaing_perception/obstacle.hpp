#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "all_seaing_interfaces/msg/obstacle.hpp"

namespace all_seaing_perception {

class Obstacle {
public:
    void to_ros_msg(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                    all_seaing_interfaces::msg::Obstacle &out_obstacle_msg);

    Obstacle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
             const std::vector<int> &in_cluster_indices, int in_id,
             builtin_interfaces::msg::Time last_seen, double nav_x, double nav_y,
             double nav_heading);

    virtual ~Obstacle();

    builtin_interfaces::msg::Time get_last_seen();
    int get_id();
    void set_id(int id);

    pcl::PointCloud<pcl::PointXYZI>::Ptr get_cloud();
    pcl::PointXYZI get_local_point();
    pcl::PointXYZI get_global_point();

    geometry_msgs::msg::PolygonStamped get_local_chull();
    geometry_msgs::msg::PolygonStamped get_global_chull();
    float get_polygon_area();

    geometry_msgs::msg::Point get_bbox_min();
    geometry_msgs::msg::Point get_bbox_max();

    geometry_msgs::msg::Point get_global_bbox_min();
    geometry_msgs::msg::Point get_global_bbox_max();

    geometry_msgs::msg::Pose get_pose();
    void set_pose(geometry_msgs::msg::Pose pose);

    template <typename T>
    T convert_to_global(double nav_x, double nav_y, double nav_heading, T point);

private:
    builtin_interfaces::msg::Time m_last_seen;
    int m_id;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;
    pcl::PointXYZI m_local_point;
    pcl::PointXYZI m_global_point;
    geometry_msgs::msg::PolygonStamped m_local_chull;
    geometry_msgs::msg::PolygonStamped m_global_chull;
    geometry_msgs::msg::Point m_bbox_min;
    geometry_msgs::msg::Point m_bbox_max;
    geometry_msgs::msg::Point m_global_bbox_min;
    geometry_msgs::msg::Point m_global_bbox_max;
    geometry_msgs::msg::Pose m_pose;
    float m_area;
};

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_HPP
