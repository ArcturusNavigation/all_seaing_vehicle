#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/impl/centroid.hpp>
#include "pcl_conversions/pcl_conversions.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "all_seaing_interfaces/msg/obstacle.hpp"

#include "all_seaing_perception/perception_utilities.hpp"

namespace all_seaing_perception {

template<typename PointT>
class Obstacle {
public:
    void to_ros_msg(all_seaing_interfaces::msg::Obstacle &out_obstacle_msg);

    Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header, all_seaing_interfaces::msg::Obstacle in_obstacle_msg);

    Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
             const typename pcl::PointCloud<PointT>::Ptr in_cloud_ptr,
             const std::vector<int> &in_cluster_indices, int in_id,
             geometry_msgs::msg::TransformStamped lidar_map_tf);

    Obstacle(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header,
            const typename pcl::PointCloud<PointT>::Ptr local_pcloud,
            const typename pcl::PointCloud<PointT>::Ptr global_pcloud,
            int in_id);

    Obstacle(std_msgs::msg::Header header,
            const typename pcl::PointCloud<PointT>::Ptr pcloud,
            int in_id,
            bool global);

    virtual ~Obstacle();

    int get_id();
    void set_id(int id);

    PointT get_local_point();
    void set_local_point(PointT local_point){
        m_local_point = local_point;
    }
    PointT get_global_point();
    void set_global_point(PointT global_point){
        m_global_point = global_point;
    }
    PointT get_bbox_min();
    void set_bbox_min(PointT bbox_min){
        m_bbox_min = bbox_min;
    }
    PointT get_bbox_max();
    void set_bbox_max(PointT bbox_max){
        m_bbox_max = bbox_max;
    }
    PointT get_global_bbox_min();
    void set_global_bbox_min(PointT global_bbox_min){
        m_global_bbox_min = global_bbox_min;
    }
    PointT get_global_bbox_max();
    void set_global_bbox_max(PointT global_bbox_max){
        m_global_bbox_max = global_bbox_max;
    }

    geometry_msgs::msg::PolygonStamped get_local_chull();
    void set_local_chull(geometry_msgs::msg::PolygonStamped local_chull){
        m_local_chull = local_chull;
    }
    geometry_msgs::msg::PolygonStamped get_global_chull();
    void set_global_chull(geometry_msgs::msg::PolygonStamped global_chull){
        m_global_chull = global_chull;
    }
    float get_polygon_area();
    void set_polygon_area(float polygon_area){
        m_area = polygon_area;
    }

    std_msgs::msg::Header get_local_header(){
        return m_local_header;
    }

    std_msgs::msg::Header get_global_header(){
        return m_global_header;
    }
    
    void set_local_header(std_msgs::msg::Header local_header){
        m_local_header = local_header;
    }

    void set_global_header(std_msgs::msg::Header global_header){
        m_global_header = global_header;
    }

    void transform_pcl_pt(PointT pt_in, PointT& pt_tf, geometry_msgs::msg::TransformStamped tf);

    void local_to_global(std_msgs::msg::Header global_header, geometry_msgs::msg::TransformStamped lidar_map_tf);
    void global_to_local(std_msgs::msg::Header local_header, geometry_msgs::msg::TransformStamped map_lidar_tf);

    void local_to_global(std_msgs::msg::Header global_header, double dx, double dy, double dtheta);
    void global_to_local(std_msgs::msg::Header local_header, double dx, double dy, double dtheta);

    void global_transform(geometry_msgs::msg::TransformStamped tf);
    void global_transform(double dx, double dy, double dtheta);

private:
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_id;
    PointT m_local_point;
    PointT m_global_point;
    PointT m_bbox_min;
    PointT m_bbox_max;
    PointT m_global_bbox_min;
    PointT m_global_bbox_max;
    geometry_msgs::msg::PolygonStamped m_local_chull;
    geometry_msgs::msg::PolygonStamped m_global_chull;
    geometry_msgs::msg::TransformStamped m_lidar_map_tf;
    geometry_msgs::msg::TransformStamped m_map_lidar_tf;
    float m_area;
};

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_HPP
