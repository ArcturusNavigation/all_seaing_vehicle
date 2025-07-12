#include "all_seaing_perception/obstacle.hpp"

#include <limits>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/surface/convex_hull.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace all_seaing_perception {

template<typename PointT>
int Obstacle<PointT>::get_id() { return m_id; }

template<typename PointT>
void Obstacle<PointT>::set_id(int id) { m_id = id; }

template<typename PointT>
PointT Obstacle<PointT>::get_local_point() { return m_local_point; }

template<typename PointT>
PointT Obstacle<PointT>::get_global_point() { return m_global_point; }

template<typename PointT>
PointT Obstacle<PointT>::get_bbox_min() { return m_bbox_min; }

template<typename PointT>
PointT Obstacle<PointT>::get_bbox_max() { return m_bbox_max; }

template<typename PointT>
PointT Obstacle<PointT>::get_global_bbox_min() { return m_global_bbox_min; }

template<typename PointT>
PointT Obstacle<PointT>::get_global_bbox_max() { return m_global_bbox_max; }

template<typename PointT>
geometry_msgs::msg::PolygonStamped Obstacle<PointT>::get_local_chull() { return m_local_chull; }

template<typename PointT>
geometry_msgs::msg::PolygonStamped Obstacle<PointT>::get_global_chull() { return m_global_chull; }

template<typename PointT>
float Obstacle<PointT>::get_polygon_area() { return m_area; }

template<typename PointT>
void Obstacle<PointT>::to_ros_msg(all_seaing_interfaces::msg::Obstacle &out_obstacle_msg) {

    out_obstacle_msg.id = this->get_id();

    out_obstacle_msg.local_point.point.x = this->get_local_point().x;
    out_obstacle_msg.local_point.point.y = this->get_local_point().y;
    out_obstacle_msg.local_point.point.z = this->get_local_point().z;
    out_obstacle_msg.local_point.header = m_local_header;

    out_obstacle_msg.global_point.point.x = this->get_global_point().x;
    out_obstacle_msg.global_point.point.y = this->get_global_point().y;
    out_obstacle_msg.global_point.point.z = this->get_global_point().z;
    out_obstacle_msg.global_point.header = m_global_header;

    out_obstacle_msg.local_chull = this->get_local_chull();
    out_obstacle_msg.local_chull.header = m_local_header;

    out_obstacle_msg.global_chull = this->get_global_chull();
    out_obstacle_msg.global_chull.header = m_global_header;

    out_obstacle_msg.polygon_area = this->get_polygon_area();

    out_obstacle_msg.bbox_min.x = this->get_bbox_min().x;
    out_obstacle_msg.bbox_min.y = this->get_bbox_min().y;
    out_obstacle_msg.bbox_min.z = this->get_bbox_min().z;

    out_obstacle_msg.bbox_max.x = this->get_bbox_max().x;
    out_obstacle_msg.bbox_max.y = this->get_bbox_max().y;
    out_obstacle_msg.bbox_max.z = this->get_bbox_max().z;

    out_obstacle_msg.global_bbox_min.x = this->get_global_bbox_min().x;
    out_obstacle_msg.global_bbox_min.y = this->get_global_bbox_min().y;
    out_obstacle_msg.global_bbox_min.z = this->get_global_bbox_min().z;

    out_obstacle_msg.global_bbox_max.x = this->get_global_bbox_max().x;
    out_obstacle_msg.global_bbox_max.y = this->get_global_bbox_max().y;
    out_obstacle_msg.global_bbox_max.z = this->get_global_bbox_max().z;
}

template<typename PointT>
Obstacle<PointT>::Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                   const typename pcl::PointCloud<PointT>::Ptr in_cloud_ptr,
                   const std::vector<int> &in_cluster_indices, int in_id,
                   geometry_msgs::msg::TransformStamped lidar_map_tf) {

    // Set id, header, and tf
    m_local_header = local_header;
    m_global_header = global_header;
    m_id = in_id;
    m_lidar_map_tf = lidar_map_tf;

    // Calculate min, max, and average point
    PointT local_min, local_max, local_avg, global_min, global_max, global_avg;
    typename pcl::PointCloud<PointT>::Ptr local_cloud_ptr(new typename pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*in_cloud_ptr, in_cluster_indices, *local_cloud_ptr);
    pcl::getMinMax3D(*local_cloud_ptr, local_min, local_max);
    pcl::computeCentroid(*local_cloud_ptr, local_avg);

    // Transform local to global point cloud
    sensor_msgs::msg::PointCloud2 local_pcl_msg, global_pcl_msg;
    typename pcl::PointCloud<PointT>::Ptr global_cloud_ptr(new typename pcl::PointCloud<PointT>);
    pcl::toROSMsg(*local_cloud_ptr, local_pcl_msg);
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(local_pcl_msg, global_pcl_msg, m_lidar_map_tf);
    pcl::fromROSMsg(global_pcl_msg, *global_cloud_ptr);

    // Calculate global min, max, and average point
    pcl::getMinMax3D(*global_cloud_ptr, global_min, global_max);
    pcl::computeCentroid(*global_cloud_ptr, global_avg);

    // Specify that all points are finite
    local_cloud_ptr->is_dense = true;
    global_cloud_ptr->is_dense = true;

    m_local_point = local_avg;
    m_global_point = global_avg;
    m_bbox_min = local_min;
    m_bbox_max = local_max;
    m_global_bbox_min = global_min;
    m_global_bbox_max = global_max;

    // Skip chull calculation if less than 3 points
    if (local_cloud_ptr->points.size() < 3) return;

    // Flatten cluster point cloud to 2D
    typename pcl::PointCloud<PointT>::Ptr cloud_2d(new typename pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*local_cloud_ptr, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = local_min.z;

    // Calculate convex hull polygon
    typename pcl::PointCloud<PointT>::Ptr hull_cloud(new typename pcl::PointCloud<PointT>);
    typename pcl::ConvexHull<PointT> chull;
    chull.setInputCloud(cloud_2d);
    chull.reconstruct(*hull_cloud);
    m_area = pcl::calculatePolygonArea(*hull_cloud);

    // Add each point in convex hull
    for (size_t i = 0; i < hull_cloud->points.size(); i++) {
        geometry_msgs::msg::Point32 local_p, global_p;
        local_p.x = hull_cloud->points[i].x;
        local_p.y = hull_cloud->points[i].y;
        local_p.z = local_min.z;
        m_local_chull.polygon.points.push_back(local_p);

        tf2::doTransform<geometry_msgs::msg::Point32>(local_p, global_p, m_lidar_map_tf);
        m_global_chull.polygon.points.push_back(global_p);
    }
}

template<typename PointT>
Obstacle<PointT>::Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                   const typename pcl::PointCloud<PointT>::Ptr local_pcloud,
                   const typename pcl::PointCloud<PointT>::Ptr global_pcloud,
                   int in_id) {

    // Set id, header, and tf
    m_local_header = local_header;
    m_global_header = global_header;
    m_id = in_id;

    // Calculate min, max, and average point
    PointT local_min, local_max, local_avg, global_min, global_max, global_avg;
    pcl::getMinMax3D(*global_pcloud, global_min, global_max);
    pcl::computeCentroid(*global_pcloud, global_avg);
    pcl::getMinMax3D(*local_pcloud, local_min, local_max);
    pcl::computeCentroid(*local_pcloud, local_avg);

    // Specify that all points are finite
    local_pcloud->is_dense = true;
    global_pcloud->is_dense = true;


    m_local_point = local_avg;
    m_global_point = global_avg;
    m_bbox_min = local_min;
    m_bbox_max = local_max;
    m_global_bbox_min = global_min;
    m_global_bbox_max = global_max;

    typename pcl::PointCloud<PointT>::Ptr local_hull_cloud(new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr global_hull_cloud(new typename pcl::PointCloud<PointT>);

    // Skip chull calculation if less than 3 points
    if (local_pcloud->points.size() >= 3){
        // Flatten cluster point cloud to 2D
        typename pcl::PointCloud<PointT>::Ptr cloud_2d(new typename pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*local_pcloud, *cloud_2d);
        for (size_t i = 0; i < cloud_2d->points.size(); i++)
            cloud_2d->points[i].z = local_min.z;

        // Calculate convex hull polygon
        typename pcl::ConvexHull<PointT> chull;
        chull.setInputCloud(cloud_2d);
        chull.reconstruct(*local_hull_cloud);
        m_area = pcl::calculatePolygonArea(*local_hull_cloud);
    }

    if (global_pcloud->points.size() >= 3){
        // Flatten cluster point cloud to 2D
        typename pcl::PointCloud<PointT>::Ptr cloud_2d(new typename pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*global_pcloud, *cloud_2d);
        for (size_t i = 0; i < cloud_2d->points.size(); i++)
            cloud_2d->points[i].z = global_min.z;

        // Calculate convex hull polygon
        typename pcl::ConvexHull<PointT> chull;
        chull.setInputCloud(cloud_2d);
        chull.reconstruct(*global_hull_cloud);
    }

    // Add each point in convex hull
    for (size_t i = 0; i < local_hull_cloud->points.size(); i++) {
        geometry_msgs::msg::Point32 local_p;
        local_p.x = local_hull_cloud->points[i].x;
        local_p.y = local_hull_cloud->points[i].y;
        local_p.z = local_min.z;
        m_local_chull.polygon.points.push_back(local_p);
    }

    for (size_t i = 0; i < global_hull_cloud->points.size(); i++) {
        geometry_msgs::msg::Point32 global_p;
        global_p.x = global_hull_cloud->points[i].x;
        global_p.y = global_hull_cloud->points[i].y;
        global_p.z = global_min.z;
        m_global_chull.polygon.points.push_back(global_p);
    }
}

template<typename PointT>
Obstacle<PointT>::Obstacle(std_msgs::msg::Header header,
                    const typename pcl::PointCloud<PointT>::Ptr pcloud,
                    int in_id,
                    bool global) {

    // Set id, header, and tf
    if (global){
        m_global_header = header;
        m_id = in_id;

        // Calculate min, max, and average point
        PointT global_min, global_max, global_avg;
        pcl::getMinMax3D(*pcloud, global_min, global_max);
        pcl::computeCentroid(*pcloud, global_avg);

        // Specify that all points are finite
        pcloud->is_dense = true;

        m_global_point = global_avg;
        m_global_bbox_min = global_min;
        m_global_bbox_max = global_max;

        typename pcl::PointCloud<PointT>::Ptr global_hull_cloud(new typename pcl::PointCloud<PointT>);

        // Skip chull calculation if less than 3 points
        if (pcloud->points.size() >= 3){
            // Flatten cluster point cloud to 2D
            typename pcl::PointCloud<PointT>::Ptr cloud_2d(new typename pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*pcloud, *cloud_2d);
            for (size_t i = 0; i < cloud_2d->points.size(); i++)
                cloud_2d->points[i].z = global_min.z;

            // Calculate convex hull polygon
            typename pcl::ConvexHull<PointT> chull;
            chull.setInputCloud(cloud_2d);
            chull.reconstruct(*global_hull_cloud);
        }

        for (size_t i = 0; i < global_hull_cloud->points.size(); i++) {
            geometry_msgs::msg::Point32 global_p;
            global_p.x = global_hull_cloud->points[i].x;
            global_p.y = global_hull_cloud->points[i].y;
            global_p.z = global_min.z;
            m_global_chull.polygon.points.push_back(global_p);
        }
    }else{
        m_local_header = header;
        m_id = in_id;

        // Calculate min, max, and average point
        PointT local_min, local_max, local_avg;
        pcl::getMinMax3D(*pcloud, local_min, local_max);
        pcl::computeCentroid(*pcloud, local_avg);

        // Specify that all points are finite
        pcloud->is_dense = true;

        m_local_point = local_avg;
        m_bbox_min = local_min;
        m_bbox_max = local_max;

        typename pcl::PointCloud<PointT>::Ptr local_hull_cloud(new typename pcl::PointCloud<PointT>);

        // Skip chull calculation if less than 3 points
        if (pcloud->points.size() >= 3){
            // Flatten cluster point cloud to 2D
            typename pcl::PointCloud<PointT>::Ptr cloud_2d(new typename pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*pcloud, *cloud_2d);
            for (size_t i = 0; i < cloud_2d->points.size(); i++)
                cloud_2d->points[i].z = local_min.z;

            // Calculate convex hull polygon
            typename pcl::ConvexHull<PointT> chull;
            chull.setInputCloud(cloud_2d);
            chull.reconstruct(*local_hull_cloud);
        }

        for (size_t i = 0; i < local_hull_cloud->points.size(); i++) {
            geometry_msgs::msg::Point32 local_p;
            local_p.x = local_hull_cloud->points[i].x;
            local_p.y = local_hull_cloud->points[i].y;
            local_p.z = local_min.z;
            m_local_chull.polygon.points.push_back(local_p);
        }
    }
}

template<typename PointT>
Obstacle<PointT>::~Obstacle() {}

template class Obstacle<pcl::PointXYZ>;
template class Obstacle<pcl::PointXYZI>;
template class Obstacle<pcl::PointXYZHSV>;

} // namespace all_seaing_perception
