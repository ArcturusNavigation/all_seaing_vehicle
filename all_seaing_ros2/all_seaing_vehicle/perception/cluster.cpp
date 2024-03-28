#include "all_seaing_vehicle/cluster.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr Cluster::GetCloud()
{
    return pointcloud_;
}

pcl::PointXYZI Cluster::GetMinPoint()
{
    return min_point_;
}

pcl::PointXYZI Cluster::GetMaxPoint()
{
    return max_point_;
}

pcl::PointXYZI Cluster::GetAveragePoint()
{
    return average_point_;
}

all_seaing_interfaces::msg::BoundingBox Cluster::GetBoundingBox()
{
    return bounding_box_;
}

geometry_msgs::msg::Polygon Cluster::GetPolygon()
{
    return polygon_;
}

float Cluster::GetPolygonArea()
{
    return area_;
}

int Cluster::GetID()
{
    return id_;
}

std_msgs::msg::Header Cluster::GetROSHeader()
{
    return ros_header_;
}

builtin_interfaces::msg::Time Cluster::GetLastSeen()
{
    return last_seen_;
}

void Cluster::SetID(int id)
{
    id_ = id;
}

void Cluster::SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                       const std::vector<int> &in_cluster_indices, std_msgs::msg::Header in_ros_header,
                       int in_id, builtin_interfaces::msg::Time in_last_seen)
{
    id_ = in_id;
    ros_header_ = in_ros_header;
    last_seen_ = in_last_seen;

    // extract point cloud using the indices
    // calculate min and max points
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0;

    for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); ++pit)
    {
        // fill cluster point by point
        pcl::PointXYZI p;
        p.x = in_origin_cloud_ptr->points[*pit].x;
        p.y = in_origin_cloud_ptr->points[*pit].y;
        p.z = in_origin_cloud_ptr->points[*pit].z;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        current_cluster->points.push_back(p);

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;
    }

    // min, max points
    min_point_.x = min_x;
    min_point_.y = min_y;
    min_point_.z = min_z;
    max_point_.x = max_x;
    max_point_.y = max_y;
    max_point_.z = max_z;

    // calculate average
    if (in_cluster_indices.size() > 0)
    {
        average_x /= in_cluster_indices.size();
        average_y /= in_cluster_indices.size();
        average_z /= in_cluster_indices.size();
    }
    average_point_.x = average_x;
    average_point_.y = average_y;
    average_point_.z = average_z;

    // length, width, height
    length_ = max_point_.x - min_point_.x;
    width_ = max_point_.y - min_point_.y;
    height_ = max_point_.z - min_point_.z;

    // calculate bounding box
    bounding_box_.header = in_ros_header;
    bounding_box_.pose.position.x = min_point_.x + length_ / 2;
    bounding_box_.pose.position.y = min_point_.y + width_ / 2;
    bounding_box_.pose.position.z = min_point_.z + height_ / 2;
    bounding_box_.dimensions.x = length_;
    bounding_box_.dimensions.y = width_;
    bounding_box_.dimensions.z = height_;

    // calculate convex hull polygon
    pcl::PointCloud<pcl::PointXYZI>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(current_cluster);
    chull.reconstruct(*hull_cloud);

    for (size_t i = 1; i < hull_cloud->points.size(); i++)
    {
        geometry_msgs::msg::Point32 p;
        p.x = hull_cloud->points[i].x;
        p.y = hull_cloud->points[i].y;
        p.z = min_point_.z;
        polygon_.points.push_back(p);
    }

    // get area of polygon
    area_ = pcl::calculatePolygonArea(*hull_cloud);

    current_cluster->width = current_cluster->points.size();
    current_cluster->height = 1;
    current_cluster->is_dense = true;

    pointcloud_ = current_cluster;
}

void Cluster::ToROSMessage(std_msgs::msg::Header in_ros_header,
                           all_seaing_interfaces::msg::CloudCluster &out_cluster_message)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(*(this->GetCloud()), cloud_msg);
    cloud_msg.header = in_ros_header;

    out_cluster_message.header = in_ros_header;

    out_cluster_message.min_point.x = this->GetMinPoint().x;
    out_cluster_message.min_point.y = this->GetMinPoint().y;
    out_cluster_message.min_point.z = this->GetMinPoint().z;

    out_cluster_message.max_point.x = this->GetMaxPoint().x;
    out_cluster_message.max_point.y = this->GetMaxPoint().y;
    out_cluster_message.max_point.z = this->GetMaxPoint().z;

    out_cluster_message.avg_point.x = this->GetAveragePoint().x;
    out_cluster_message.avg_point.y = this->GetAveragePoint().y;
    out_cluster_message.avg_point.z = this->GetAveragePoint().z;

    out_cluster_message.convex_hull = this->GetPolygon();

    out_cluster_message.polygon_area = this->GetPolygonArea();

    out_cluster_message.last_seen = this->GetLastSeen();

    out_cluster_message.id = this->GetID();

    out_cluster_message.bb_msg = this->GetBoundingBox();
}

Cluster::Cluster(){}

Cluster::~Cluster(){}
