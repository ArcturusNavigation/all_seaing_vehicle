#include "all_seaing_vehicle/cluster.hpp"

Cluster::Cluster()
{
    valid_cluster_ = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Cluster::GetCloud()
{
    return pointcloud_;
}

pcl::PointXYZ Cluster::GetMinPoint()
{
    return min_point_;
}

pcl::PointXYZ Cluster::GetMaxPoint()
{
    return max_point_;
}

pcl::PointXYZ Cluster::GetCentroidPoint()
{
    return centroid_;
}

pcl::PointXYZ Cluster::GetAveragePoint()
{
    return average_point_;
}

double Cluster::GetOrientationAngle()
{
    return orientation_angle_;
}

Eigen::Matrix3f Cluster::GetEigenVectors()
{
    return eigen_vectors_;
}

Eigen::Vector3f Cluster::GetEigenValues()
{
    return eigen_values_;
}

all_seaing_interfaces::msg::BoundingBox Cluster::GetBoundingBox()
{
    return bounding_box_;
}

geometry_msgs::msg::PolygonStamped Cluster::GetPolygon()
{
    return polygon_;
}

float Cluster::GetPolygonArea()
{
    return area_;
}

bool Cluster::IsValid()
{
    return valid_cluster_;
}

int Cluster::GetID()
{
    return id_;
}

std_msgs::msg::Header Cluster::GetROSHeader()
{
    return ros_header_;
}

builtin_interfaces::msg::Time Cluster::SetLastSeen(builtin_interfaces::msg::Time value)
{
    return last_seen_ = value;
}
builtin_interfaces::msg::Time Cluster::GetLastSeen()
{
    return last_seen_;
}
void Cluster::SetValidity(bool in_valid)
{
    valid_cluster_ = in_valid;
}

int Cluster::SetID(int id)
{
    id_ = id;
    return id_;
}

void Cluster::SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
                       const std::vector<int> &in_cluster_indices, std_msgs::msg::Header in_ros_header,
                       int in_id, std::string in_label, builtin_interfaces::msg::Time in_last_seen)
{
    label_ = in_label;
    id_ = in_id;
    ros_header_ = in_ros_header;
    last_seen_ = in_last_seen;

    // extract point cloud using the indices
    // calculate min and max points
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZ>);
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
        pcl::PointXYZ p;
        p.x = in_origin_cloud_ptr->points[*pit].x;
        p.y = in_origin_cloud_ptr->points[*pit].y;
        p.z = in_origin_cloud_ptr->points[*pit].z;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        centroid_.x += p.x;
        centroid_.y += p.y;
        centroid_.z += p.z;
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

    // calculate centroid, average
    if (in_cluster_indices.size() > 0)
    {
        centroid_.x /= in_cluster_indices.size();
        centroid_.y /= in_cluster_indices.size();
        centroid_.z /= in_cluster_indices.size();

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

    bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
    bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
    bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

    // calculate convex hull polygon
    polygon_.header = in_ros_header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(current_cluster);
    chull.reconstruct(*hull_cloud);

    for (size_t i = 1; i < hull_cloud->points.size(); i++)
    {
        geometry_msgs::msg::Point32 p;
        p.x = hull_cloud->points[i].x;
        p.y = hull_cloud->points[i].y;
        p.z = min_point_.z;
        polygon_.polygon.points.push_back(p);
    }

    // get area of polygon
    area_ = pcl::calculatePolygonArea(*hull_cloud);

    current_cluster->width = current_cluster->points.size();
    current_cluster->height = 1;
    current_cluster->is_dense = true;

    valid_cluster_ = true;
    pointcloud_ = current_cluster;
}

void Cluster::ToROSMessage(std_msgs::msg::Header in_ros_header,
                           all_seaing_interfaces::msg::CloudCluster &out_cluster_message)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(*(this->GetCloud()), cloud_msg);
    cloud_msg.header = in_ros_header;

    out_cluster_message.cloud = cloud_msg;
    out_cluster_message.min_point.header = in_ros_header;
    out_cluster_message.min_point.point.x = this->GetMinPoint().x;
    out_cluster_message.min_point.point.y = this->GetMinPoint().y;
    out_cluster_message.min_point.point.z = this->GetMinPoint().z;

    out_cluster_message.max_point.header = in_ros_header;
    out_cluster_message.max_point.point.x = this->GetMaxPoint().x;
    out_cluster_message.max_point.point.y = this->GetMaxPoint().y;
    out_cluster_message.max_point.point.z = this->GetMaxPoint().z;

    out_cluster_message.avg_point.header = in_ros_header;
    out_cluster_message.avg_point.point.x = this->GetAveragePoint().x;
    out_cluster_message.avg_point.point.y = this->GetAveragePoint().y;
    out_cluster_message.avg_point.point.z = this->GetAveragePoint().z;

    out_cluster_message.centroid_point.header = in_ros_header;
    out_cluster_message.centroid_point.point.x = this->GetCentroidPoint().x;
    out_cluster_message.centroid_point.point.y = this->GetCentroidPoint().y;
    out_cluster_message.centroid_point.point.z = this->GetCentroidPoint().z;

    out_cluster_message.convex_hull.header = in_ros_header;
    out_cluster_message.convex_hull = this->GetPolygon();

    out_cluster_message.polygon_area = this->GetPolygonArea();

    out_cluster_message.last_seen = this->GetLastSeen();

    out_cluster_message.id = this->GetID();

    out_cluster_message.bb_msg = this->GetBoundingBox();
}

Cluster::~Cluster()
{
    // Destructor
}
