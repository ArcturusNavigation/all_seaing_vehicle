#include <string>
#include <vector>
#include <unordered_set>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class ObstacleBboxOverlay : public rclcpp::Node
{
private:
    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_text_marker_array_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> m_bbox_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::ObstacleMap> m_map_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_viz;
    bool m_pc_cam_tf_ok;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    // Obstacle-bbox sync policies
    typedef message_filters::sync_policies::ApproximateTime<
        all_seaing_interfaces::msg::LabeledBoundingBox2DArray, all_seaing_interfaces::msg::ObstacleMap>
        ObstacleBboxPolicy;
    typedef message_filters::Synchronizer<ObstacleBboxPolicy> ObstacleBboxSync;
    std::shared_ptr<ObstacleBboxSync> m_obstacle_bbox_sync;

    // Fuse image and obstacle by projecting 3D points onto 2D image.
    void obstacle_bbox_fusion_cb(
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
        const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg);

    // Get intrinsic camera model information needed for projection
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped
    get_tf(const std::string &in_target_frame,
           const std::string &in_src_frame);

    // Publish markers for visualizing matched obstacles
    void markers(const all_seaing_interfaces::msg::ObstacleMap &in_map_msg);

public:
    ObstacleBboxOverlay();
    virtual ~ObstacleBboxOverlay();
};