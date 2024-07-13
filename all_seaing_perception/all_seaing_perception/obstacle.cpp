#include "all_seaing_perception/obstacle.hpp"

#include <limits>

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include "geometry_msgs/msg/point32.hpp"

std_msgs::msg::Header Obstacle::get_ros_header() { return m_ros_header; }

builtin_interfaces::msg::Time Obstacle::get_last_seen() { return m_last_seen; }

int Obstacle::get_id() { return m_id; }

void Obstacle::set_id(int id) { m_id = id; }

pcl::PointCloud<pcl::PointXYZI>::Ptr Obstacle::get_cloud() { return m_cloud; }

pcl::PointXYZI Obstacle::get_local_point() { return m_local_point; }

pcl::PointXYZI Obstacle::get_global_point() { return m_global_point; }

geometry_msgs::msg::Polygon Obstacle::get_local_chull() { return m_local_chull; }

geometry_msgs::msg::Polygon Obstacle::get_global_chull() { return m_global_chull; }

float Obstacle::get_polygon_area() { return m_area; }

pcl::PointXYZI Obstacle::convert_to_global(double nav_x, double nav_y,
                                           double nav_heading, pcl::PointXYZI point) {
    pcl::PointXYZI new_point;
    double magnitude = std::hypot(point.x, point.y);
    double point_angle = std::atan2(point.y, point.x);
    new_point.x = nav_x + std::cos(nav_heading + point_angle) * magnitude;
    new_point.y = nav_y + std::sin(nav_heading + point_angle) * magnitude;
    new_point.z = 0;
    return new_point;
}

void Obstacle::to_ros_msg(std_msgs::msg::Header in_ros_header,
                          all_seaing_interfaces::msg::Obstacle &out_obstacle_msg) {
    out_obstacle_msg.header = in_ros_header;
    out_obstacle_msg.id = this->get_id();

    out_obstacle_msg.local_point.x = this->get_local_point().x;
    out_obstacle_msg.local_point.y = this->get_local_point().y;
    out_obstacle_msg.local_point.z = this->get_local_point().z;

    out_obstacle_msg.global_point.x = this->get_global_point().x;
    out_obstacle_msg.global_point.y = this->get_global_point().y;
    out_obstacle_msg.global_point.z = this->get_global_point().z;

    out_obstacle_msg.local_chull = this->get_local_chull();
    out_obstacle_msg.global_chull = this->get_global_chull();

    out_obstacle_msg.polygon_area = this->get_polygon_area();

    out_obstacle_msg.last_seen = this->get_last_seen();
}

Obstacle::Obstacle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                   const std::vector<int> &in_cluster_indices,
                   std_msgs::msg::Header in_ros_header, int in_id,
                   builtin_interfaces::msg::Time in_last_seen, double nav_x,
                   double nav_y, double nav_heading) {
    // Set id and header
    m_id = in_id;
    m_ros_header = in_ros_header;
    m_last_seen = in_last_seen;

    // Fill cluster point by point
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster(
        new pcl::PointCloud<pcl::PointXYZI>);
    float min_z = std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0;
    for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end();
         pit++) {
        pcl::PointXYZI p;
        p.x = in_origin_cloud_ptr->points[*pit].x;
        p.y = in_origin_cloud_ptr->points[*pit].y;
        p.z = in_origin_cloud_ptr->points[*pit].z;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        current_cluster->points.push_back(p);

        if (p.z < min_z)
            min_z = p.z;
    }

    // Calculate average local point
    if (in_cluster_indices.size() > 0) {
        average_x /= in_cluster_indices.size();
        average_y /= in_cluster_indices.size();
        average_z /= in_cluster_indices.size();
    }
    m_local_point.x = average_x;
    m_local_point.y = average_y;
    m_local_point.z = average_z;

    // Calculate global point
    m_global_point = convert_to_global(nav_x, nav_y, nav_heading, m_local_point);

    // Calculate convex hull polygon
    pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(current_cluster);
    chull.reconstruct(*hull_cloud);
    m_area = pcl::calculatePolygonArea(*hull_cloud);

    // Add each point in convex hull
    for (size_t i = 0; i < hull_cloud->points.size(); i++) {
        // Push local point
        geometry_msgs::msg::Point32 p;
        p.x = hull_cloud->points[i].x;
        p.y = hull_cloud->points[i].y;
        p.z = min_z;
        m_local_chull.points.push_back(p);

        // Push global point
        pcl::PointXYZI p_glob =
            convert_to_global(nav_x, nav_y, nav_heading, hull_cloud->points[i]);
        geometry_msgs::msg::Point32 p_glob_msg;
        p_glob_msg.x = p_glob.x;
        p_glob_msg.y = p_glob.y;
        m_global_chull.points.push_back(p_glob_msg);
    }

    // Speicfy that all points are finite
    current_cluster->is_dense = true;

    // Add pointcloud to member variable
    m_cloud = current_cluster;
}

Obstacle::~Obstacle() {}
