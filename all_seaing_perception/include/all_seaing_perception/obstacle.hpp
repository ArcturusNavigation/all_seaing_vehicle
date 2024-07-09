#include <pcl/PCLPointCloud2.h>
#include <pcl/common/distances.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include "all_seaing_interfaces/msg/obstacle.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include <limits>

class Obstacle {
    std_msgs::msg::Header m_ros_header;
    builtin_interfaces::msg::Time m_last_seen;
    int m_id;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;

    pcl::PointXYZI m_local_point;
    pcl::PointXYZI m_global_point;

    geometry_msgs::msg::Polygon m_local_chull;
    geometry_msgs::msg::Polygon m_global_chull;
    float m_area;

public:
    void to_ros_msg(std_msgs::msg::Header in_ros_header,
                    all_seaing_interfaces::msg::Obstacle &out_obstacle_msg);

    Obstacle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
             const std::vector<int> &in_cluster_indices,
             std_msgs::msg::Header in_ros_header, int in_id,
             builtin_interfaces::msg::Time last_seen, double nav_x, double nav_y,
             double nav_heading);

    virtual ~Obstacle();

    std_msgs::msg::Header get_ros_header();
    builtin_interfaces::msg::Time get_last_seen();
    int get_id();
    void set_id(int id);

    pcl::PointCloud<pcl::PointXYZI>::Ptr get_cloud();
    pcl::PointXYZI get_local_point();
    pcl::PointXYZI get_global_point();

    geometry_msgs::msg::Polygon get_local_chull();
    geometry_msgs::msg::Polygon get_global_chull();
    float get_polygon_area();

    pcl::PointXYZI convert_to_global(double nav_x, double nav_y, double nav_heading,
                                     pcl::PointXYZI point);
};