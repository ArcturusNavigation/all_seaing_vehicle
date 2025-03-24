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

int Obstacle::get_id() { return m_id; }

void Obstacle::set_id(int id) { m_id = id; }

pcl::PointXYZI Obstacle::get_local_point() { return m_local_point; }

pcl::PointXYZI Obstacle::get_global_point() { return m_global_point; }

pcl::PointXYZI Obstacle::get_bbox_min() { return m_bbox_min; }

pcl::PointXYZI Obstacle::get_bbox_max() { return m_bbox_max; }

pcl::PointXYZI Obstacle::get_global_bbox_min() { return m_global_bbox_min; }

pcl::PointXYZI Obstacle::get_global_bbox_max() { return m_global_bbox_max; }

geometry_msgs::msg::PolygonStamped Obstacle::get_local_chull() { return m_local_chull; }

geometry_msgs::msg::PolygonStamped Obstacle::get_global_chull() { return m_global_chull; }

float Obstacle::get_polygon_area() { return m_area; }

void Obstacle::to_ros_msg(all_seaing_interfaces::msg::Obstacle &out_obstacle_msg) {

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

// TODO: overload the below function to allow creating obstacles with global coordinates directly, with local coords being optional (e.g. for global map)

Obstacle::Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                   const std::vector<int> &in_cluster_indices, int in_id,
                   geometry_msgs::msg::TransformStamped lidar_map_tf) {

    // Set id, header, and tf
    m_local_header = local_header;
    m_global_header = global_header;
    m_id = in_id;
    m_lidar_map_tf = lidar_map_tf;

    // Fill cluster point by point
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_cluster_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI local_min, local_max, local_avg, global_min, global_max, global_avg;
    local_min.x = std::numeric_limits<float>::max();
    local_min.y = std::numeric_limits<float>::max();
    local_min.z = std::numeric_limits<float>::max();
    local_max.x = std::numeric_limits<float>::lowest();
    local_max.y = std::numeric_limits<float>::lowest();
    local_max.z = std::numeric_limits<float>::lowest();
    global_min.x = std::numeric_limits<float>::max();
    global_min.y = std::numeric_limits<float>::max();
    global_min.z = std::numeric_limits<float>::max();
    global_max.x = std::numeric_limits<float>::lowest();
    global_max.y = std::numeric_limits<float>::lowest();
    global_max.z = std::numeric_limits<float>::lowest();
    for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); pit++) {
        pcl::PointXYZI local_p, global_p;
        local_p = in_cloud_ptr->points[*pit];

        local_min.x = std::min(local_p.x, local_min.x);
        local_min.y = std::min(local_p.y, local_min.y);
        local_min.z = std::min(local_p.z, local_min.z);
        local_max.x = std::max(local_p.x, local_max.x);
        local_max.y = std::max(local_p.y, local_max.y);
        local_max.z = std::max(local_p.z, local_max.z);

        local_avg.x += local_p.x;
        local_avg.y += local_p.y;
        local_avg.z += local_p.z;
        local_cluster_pc->points.push_back(local_p);

        // Convert point to global frame
        geometry_msgs::msg::Point p_msg;
        geometry_msgs::msg::Point p_tf;
        p_msg.x = local_p.x;
        p_msg.y = local_p.y;
        p_msg.z = local_p.z;
        tf2::doTransform<geometry_msgs::msg::Point>(p_msg, p_tf, m_lidar_map_tf);
        global_p.x = p_tf.x;
        global_p.y = p_tf.y;
        global_p.z = p_tf.z;

        global_min.x = std::min(global_p.x, global_min.x);
        global_min.y = std::min(global_p.y, global_min.y);
        global_min.z = std::min(global_p.z, global_min.z);
        global_max.x = std::max(global_p.x, global_max.x);
        global_max.y = std::max(global_p.y, global_max.y);
        global_max.z = std::max(global_p.z, global_max.z);

        global_avg.x += global_p.x;
        global_avg.y += global_p.y;
        global_avg.z += global_p.z;
    }

    // Specify that all points are finite
    local_cluster_pc->is_dense = true;

    // Calculate average local point
    if (in_cluster_indices.size() > 0) {
        local_avg.x /= in_cluster_indices.size();
        local_avg.y /= in_cluster_indices.size();
        local_avg.z /= in_cluster_indices.size();
        global_avg.x /= in_cluster_indices.size();
        global_avg.y /= in_cluster_indices.size();
        global_avg.z /= in_cluster_indices.size();
    }
    m_local_point = local_avg;
    m_global_point = global_avg;
    m_bbox_min = local_min;
    m_bbox_max = local_max;
    m_global_bbox_min = global_min;
    m_global_bbox_max = global_max;

    // Skip chull calculation if less than 3 points
    if (local_cluster_pc->points.size() < 3) return;

    // Flatten cluster point cloud to 2D
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*local_cluster_pc, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = local_min.z;

    // Calculate convex hull polygon
    pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConvexHull<pcl::PointXYZI> chull;
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

Obstacle::~Obstacle() {}

} // namespace all_seaing_perception
