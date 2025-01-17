#include "all_seaing_perception/obstacle.hpp"

#include <limits>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/surface/convex_hull.h>

#include "geometry_msgs/msg/point32.hpp"

namespace all_seaing_perception {

builtin_interfaces::msg::Time Obstacle::get_last_seen() { return m_last_seen; }

int Obstacle::get_id() { return m_id; }

void Obstacle::set_id(int id) { m_id = id; }

pcl::PointCloud<pcl::PointXYZI>::Ptr Obstacle::get_cloud() { return m_cloud; }

pcl::PointXYZI Obstacle::get_local_point() { return m_local_point; }

pcl::PointXYZI Obstacle::get_global_point() { return m_global_point; }

geometry_msgs::msg::PolygonStamped Obstacle::get_local_chull() { return m_local_chull; }

geometry_msgs::msg::PolygonStamped Obstacle::get_global_chull() { return m_global_chull; }

geometry_msgs::msg::Point Obstacle::get_bbox_min() { return m_bbox_min; }

geometry_msgs::msg::Point Obstacle::get_bbox_max() { return m_bbox_max; }

geometry_msgs::msg::Point Obstacle::get_global_bbox_min() { return m_global_bbox_min; }

geometry_msgs::msg::Point Obstacle::get_global_bbox_max() { return m_global_bbox_max; }

float Obstacle::get_polygon_area() { return m_area; }

// TODO: do this using tf and not manually
template <typename T> // this allows for both pcl::PointXYZI and geometry_msgs::msg::Point
T Obstacle::convert_to_global(double nav_x, double nav_y, double nav_heading, T point) {
    T new_point;
    double magnitude = std::hypot(point.x, point.y);
    double point_angle = std::atan2(point.y, point.x);
    new_point.x = nav_x + std::cos(nav_heading + point_angle) * magnitude;
    new_point.y = nav_y + std::sin(nav_heading + point_angle) * magnitude;
    new_point.z = 0;
    return new_point;
}

//TODO: add the cluster and the image segments (respective frames should be correct in the header of both) to the published quantities, change the class attributes & methods to do that

void Obstacle::to_ros_msg(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                          all_seaing_interfaces::msg::Obstacle &out_obstacle_msg) {
    out_obstacle_msg.id = this->get_id();

    out_obstacle_msg.local_point.point.x = this->get_local_point().x;
    out_obstacle_msg.local_point.point.y = this->get_local_point().y;
    out_obstacle_msg.local_point.point.z = this->get_local_point().z;
    out_obstacle_msg.local_point.header = local_header;

    out_obstacle_msg.global_point.point.x = this->get_global_point().x;
    out_obstacle_msg.global_point.point.y = this->get_global_point().y;
    out_obstacle_msg.global_point.point.z = this->get_global_point().z;
    out_obstacle_msg.global_point.header = global_header;

    out_obstacle_msg.local_chull = this->get_local_chull();
    out_obstacle_msg.local_chull.header = local_header;

    out_obstacle_msg.global_chull = this->get_global_chull();
    out_obstacle_msg.global_chull.header = global_header;

    out_obstacle_msg.polygon_area = this->get_polygon_area();

    out_obstacle_msg.last_seen = this->get_last_seen();
    
    out_obstacle_msg.bbox_min = this->get_bbox_min();
    out_obstacle_msg.bbox_max = this->get_bbox_max();

    out_obstacle_msg.global_bbox_min = this->get_global_bbox_min();
    out_obstacle_msg.global_bbox_max = this->get_global_bbox_max();

}

Obstacle::Obstacle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                   const std::vector<int> &in_cluster_indices, int in_id,
                   builtin_interfaces::msg::Time in_last_seen, double nav_x, double nav_y,
                   double nav_heading) {
    // Set id and header
    m_id = in_id;
    m_last_seen = in_last_seen;

    // Fill cluster point by point
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    float average_x = 0, average_y = 0, average_z = 0;
    for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); pit++) {
        pcl::PointXYZI p;
        p.x = in_origin_cloud_ptr->points[*pit].x;
        p.y = in_origin_cloud_ptr->points[*pit].y;
        p.z = in_origin_cloud_ptr->points[*pit].z;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        current_cluster->points.push_back(p);

        min_x = std::min(p.x, min_x);
        min_y = std::min(p.y, min_y);
        min_z = std::min(p.z, min_z);

        max_x = std::max(p.x, max_x);
        max_y = std::max(p.y, max_y);
        max_z = std::max(p.z, max_z);
    }

    // Speicfy that all points are finite
    current_cluster->is_dense = true;

    // Add pointcloud to member variable
    m_cloud = current_cluster;

    // Calculate average local point
    if (in_cluster_indices.size() > 0) {
        average_x /= in_cluster_indices.size();
        average_y /= in_cluster_indices.size();
        average_z /= in_cluster_indices.size();
    }
    m_local_point.x = average_x;
    m_local_point.y = average_y;
    m_local_point.z = average_z;

    m_bbox_min.x = min_x;
    m_bbox_min.y = min_y;
    m_bbox_min.z = min_z;

    m_bbox_max.x = max_x;
    m_bbox_max.y = max_y;
    m_bbox_max.z = max_z;

    // Calculate global point
    m_global_point = convert_to_global(nav_x, nav_y, nav_heading, m_local_point);
    m_global_bbox_min = convert_to_global(nav_x, nav_y, nav_heading, m_bbox_min);
    m_global_bbox_max = convert_to_global(nav_x, nav_y, nav_heading, m_bbox_max);

    // Skip chull calculation if less than 3 points
    if (current_cluster->points.size() < 3) return;

    // Flatten cluster point cloud to 2D
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*current_cluster, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = min_z;

    // Calculate convex hull polygon
    pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(cloud_2d);
    chull.reconstruct(*hull_cloud);
    m_area = pcl::calculatePolygonArea(*hull_cloud);

    // Add each point in convex hull
    for (size_t i = 0; i < hull_cloud->points.size(); i++) {
        // Push local point
        geometry_msgs::msg::Point32 p;
        p.x = hull_cloud->points[i].x;
        p.y = hull_cloud->points[i].y;
        p.z = min_z;
        m_local_chull.polygon.points.push_back(p);

        // Push global point
        pcl::PointXYZI p_glob = convert_to_global(nav_x, nav_y, nav_heading, hull_cloud->points[i]);
        geometry_msgs::msg::Point32 p_glob_msg;
        p_glob_msg.x = p_glob.x;
        p_glob_msg.y = p_glob.y;
        m_global_chull.polygon.points.push_back(p_glob_msg);
    }
}

Obstacle::~Obstacle() {}

} // namespace all_seaing_perception
