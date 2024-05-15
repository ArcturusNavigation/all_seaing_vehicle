#ifndef CLUSTER_HPP_
#define CLUSTER_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/impl/common.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "all_seaing_interfaces/msg/cloud_cluster.hpp"
#include "all_seaing_interfaces/msg/cloud_cluster_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <limits>

class Cluster
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_;
    pcl::PointXYZI min_point_;
    pcl::PointXYZI max_point_;
    pcl::PointXYZI average_point_;
    float area_;

    geometry_msgs::msg::Polygon polygon_;
    std_msgs::msg::Header ros_header_;

    int id_;
    builtin_interfaces::msg::Time last_seen_;

public:
    /**
     * @brief Constructor. Creates a Cluster object using the specified points in a point cloud
     * @param[in] in_origin_cloud_ptr Origin PC
     * @param[in] in_cluster_indices  Inices of the Origin point cloud  to create the Cluster object
     * @param[in] in_id               ID of the cluster
     * @param[in] in_label            label to identify the cluster (optional)
     */

    void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                  const std::vector<int> &in_cluster_indices,
                  std_msgs::msg::Header in_ros_header,
                  int in_id, builtin_interfaces::msg::Time last_seen);

    /**
     * @brief Constructs the all_seaing_interfaces::CloudCluster message associated to this Cluster
     * @param[in] in_ros_header sensor header message
     * @param[in] out_cluster_message CloudCluster ROS message output
     */

    void ToROSMessage(std_msgs::msg::Header in_ros_header,
                      all_seaing_interfaces::msg::CloudCluster &out_cluster_message);

    /**
     * @brief Constructor
     */

    Cluster();
    virtual ~Cluster();

    /** @brief Returns the pointer to the pointcloud containing the points in this Cluster */
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloud();
    /** @brief Returns the minimum point in the cluster */
    pcl::PointXYZI GetMinPoint();
    /** @brief Returns the maximum point in the cluster */
    pcl::PointXYZI GetMaxPoint();
    /** @brief Returns the average point in the cluster */
    pcl::PointXYZI GetAveragePoint();
    /** @brief Returns the convex hull (polygon) of the cluster */
    geometry_msgs::msg::Polygon GetPolygon();
    /** @brief Returns the area of the polygon (convex hull) */
    float GetPolygonArea();
    /** @brief Returns the ID of the cluster */
    int GetID();
    /** @brief Returns the ROS header of the cluster */
    std_msgs::msg::Header GetROSHeader();
    /** @brief Returns message header */
    std_msgs::msg::Header GetHeader();
    /** @brief Returns last scene tick*/
    builtin_interfaces::msg::Time GetLastSeen();
    /** @brief Sets cluster id */
    void SetID(int id);
};

#endif /* CLUSTER_HPP_ */
