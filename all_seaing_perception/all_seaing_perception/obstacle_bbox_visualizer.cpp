#include "all_seaing_perception/obstacle_bbox_visualizer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ObstacleBboxVisualizer::ObstacleBboxVisualizer() : Node("obstacle_bbox_visualizer") {
    // Initialize subscribers
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10,
        std::bind(&ObstacleBboxVisualizer::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "image", rmw_qos_profile_sensor_data);
    m_obstacle_map_sub.subscribe(this, "obstacle_map/labeled", rmw_qos_profile_default);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_default);

    // Initialize publisher
    m_image_pub = this->create_publisher<sensor_msgs::msg::Image>("image/obstacle_visualized", 10);

    // Initialize synchronizer
    m_sync = std::make_shared<Synchronizer>(SyncPolicy(10), m_image_sub, m_obstacle_map_sub, m_bbox_sub);
    m_sync->registerCallback(std::bind(&ObstacleBboxVisualizer::image_obstacle_cb, this,
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

ObstacleBboxVisualizer::~ObstacleBboxVisualizer() {}
void ObstacleBboxVisualizer::image_obstacle_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& in_img_msg,
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr& in_map_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr& in_bbox_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    for (const auto& obstacle : in_map_msg->obstacles) {

        // find the centroid and display it.
        cv::Point3d centroid(obstacle.local_point.point.y,
                             obstacle.local_point.point.z,
                             -obstacle.local_point.point.x);
        cv::Point2d pixel_centroid = m_cam_model.project3dToPixel(centroid);

        if (pixel_centroid.x >= 0 && pixel_centroid.x < cv_ptr->image.cols &&
            pixel_centroid.y >= 0 && pixel_centroid.y < cv_ptr->image.rows) {
            cv::Scalar color = get_color_for_label(obstacle.label);
            
            // Draw centroid
            cv::circle(cv_ptr->image, pixel_centroid, 5, color, -1);

        }
    }

    // draw the bounding boxes
    for (const auto& bbox : in_bbox_msg->bounding_boxes) {
        cv::Scalar color = get_color_for_label(bbox.label);
        cv::rectangle(cv_ptr->image, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), color, 2);
    }

    m_image_pub->publish(*cv_ptr->toImageMsg());
}

void ObstacleBboxVisualizer::intrinsics_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

// label -> color hardcoded for now, parametrize later 
cv::Scalar ObstacleBboxVisualizer::get_color_for_label(const int& label) {
    switch (label) {
        case 0: // orange
            return cv::Scalar(0, 165, 255);
        case 1: // red
            return cv::Scalar(0, 0, 255);
        case 2: // green
            return cv::Scalar(0, 255, 0);
        case 3: // black
            return cv::Scalar(0, 0, 0);
        case 4: // white
            return cv::Scalar(255, 255, 255);
        default: // blue
            return cv::Scalar(255, 0, 0);

    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleBboxVisualizer>());
    rclcpp::shutdown();
    return 0;
}