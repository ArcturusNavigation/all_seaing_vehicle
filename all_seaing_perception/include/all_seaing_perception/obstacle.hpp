#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "all_seaing_interfaces/msg/obstacle.hpp"

namespace all_seaing_perception {

class Obstacle {
public:
    void to_ros_msg(all_seaing_interfaces::msg::Obstacle &out_obstacle_msg);

    Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
             const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
             const std::vector<int> &in_cluster_indices, int in_id,
             geometry_msgs::msg::TransformStamped lidar_map_tf);

    Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                    const typename pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcloud,
                    const typename pcl::PointCloud<pcl::PointXYZI>::Ptr global_pcloud,
                    int in_id);

    virtual ~Obstacle();

    int get_id();
    void set_id(int id);

    pcl::PointXYZI get_local_point();
    pcl::PointXYZI get_global_point();
    pcl::PointXYZI get_bbox_min();
    pcl::PointXYZI get_bbox_max();
    pcl::PointXYZI get_global_bbox_min();
    pcl::PointXYZI get_global_bbox_max();

    geometry_msgs::msg::PolygonStamped get_local_chull();
    geometry_msgs::msg::PolygonStamped get_global_chull();
    float get_polygon_area();

private:
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_id;
    pcl::PointXYZI m_local_point;
    pcl::PointXYZI m_global_point;
    pcl::PointXYZI m_bbox_min;
    pcl::PointXYZI m_bbox_max;
    pcl::PointXYZI m_global_bbox_min;
    pcl::PointXYZI m_global_bbox_max;
    geometry_msgs::msg::PolygonStamped m_local_chull;
    geometry_msgs::msg::PolygonStamped m_global_chull;
    geometry_msgs::msg::TransformStamped m_lidar_map_tf;
    float m_area;
};

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_HPP
