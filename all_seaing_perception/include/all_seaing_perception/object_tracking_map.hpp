#ifndef ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP
#define ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h" 

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"

#include "cv_bridge/cv_bridge.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "all_seaing_interfaces/msg/labeled_object_point_cloud_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class ObjectTrackingMap : public rclcpp::Node{
private:

public:
    ObjectTrackingMap();
    virtual ~ObjectTrackingMap();
};

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP