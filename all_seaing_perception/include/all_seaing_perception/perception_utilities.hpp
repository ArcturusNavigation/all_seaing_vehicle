#ifndef ALL_SEAING_PERCEPTION__PERCEPTION_UTILITIES_HPP
#define ALL_SEAING_PERCEPTION__PERCEPTION_UTILITIES_HPP

#include "image_geometry/pinhole_camera_model.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/extract_indices.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include "all_seaing_interfaces/msg/labeled_bounding_box2_d.hpp"

namespace all_seaing_perception{
    // To project OpenCV 3d points to the camera frame
    cv::Point2d project3dToPixel(image_geometry::PinholeCameraModel& cmodel, const cv::Point3d& xyz);

    // To project PCL 3D points to the camera frame, uses project3dToPixel
    template<typename PointT>
    cv::Point2d projectPCLPtToPixel(image_geometry::PinholeCameraModel& cmodel, const PointT& point_tf, bool is_sim){
        cv::Point2d xy_rect;
        try{
            // Project 3D point onto the image plane using the intrinsic matrix.
            // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
            xy_rect = is_sim
                ? all_seaing_perception::project3dToPixel(cmodel, cv::Point3d(point_tf.y, point_tf.z, -point_tf.x))
                : all_seaing_perception::project3dToPixel(cmodel, cv::Point3d(point_tf.x, point_tf.y, point_tf.z));
        }catch(image_geometry::Exception &e){
            // ignore
        }
        return xy_rect;
    }

    // Check if within bounds
    bool inBounds(image_geometry::PinholeCameraModel& cmodel, const cv::Point2d& xy_rect);

    // Check if in bbox
    bool inBBox(const cv::Point2d& xy_rect, all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox);

    // Add padding to bounding box
    void addBBoxPadding(all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox, int margin, int height, int width);

    // Convert OpenCV HSV to PCL (OpenCV max HSV (int): (180, 255, 255), PCL (float): (360, 1, 1))
    template<typename numType>
    std::vector<numType> HSVOpenCVToPCL(cv::Vec3b hsv_vec3b){
        return std::vector<numType>({((float)hsv_vec3b[0])*2, ((float)hsv_vec3b[1])/((float)255.0), ((float)hsv_vec3b[2])/((float)255.0)});
    }

    // Convert PCL HSV to OpenCV (OpenCV max HSV (int): (180, 255, 255), PCL (float): (360, 1, 1))
    template<typename numType>
    std::vector<numType> HSVPCLToOpenCV(pcl::PointXYZHSV point){
        return std::vector<numType>({point.h/2, point.s*((float)255.0), point.v*((float)255.0)});
    }

    // Invert HSV in PCL points
    void invertHSVPCL(pcl::PointXYZHSV& pt);

    // Invert HSV in OpenCV points
    template<typename numType>
    void invertHSVOpenCV(std::vector<numType>& pt){
        pt[0]=(pt[0]+90)%180;
    }

    // Color segmentation in HSV, handling red as needed, limits are (h_min,s_min,v_min)->(h_max,s_max,v_max)
    //TODO: change red handling to have one pair of limits for red and just invert and do color segmentation in cyan essentially
    std::pair<cv::Mat, std::vector<std::vector<cv::Point>>> colorSegmentationHSV(cv::Mat& img, std::vector<int> lims, int erode_sz = 5, int dilate_sz = 7, bool red = false, std::vector<int> lims2 = std::vector<int>());

    // Find points inside a contour
    std::vector<cv::Point> inContour(std::vector<cv::Point>& contour);

    // Euclidean clustering
    void euclideanClustering(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance = 0.0f, int obstacle_sz_min = 1, int obstacle_sz_max = std::numeric_limits<int>::max (), bool conditional = false, std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)> cond_func = std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)>());

    // @overload
    void euclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance = 0.0f, int obstacle_sz_min = 1, int obstacle_sz_max = std::numeric_limits<int>::max (), bool conditional = false, std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)> cond_func = std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)>());

    // project bbox to point cloud and assign HSV colors
    void PCLInBBoxHSV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr, all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox, cv::Mat& img, image_geometry::PinholeCameraModel& cmodel, bool is_sim = false);

    // transform pcl::PointCloud using TF2
    template<typename PointT>
    void transformPCLCloud(typename pcl::PointCloud<PointT> pcl_in, typename pcl::PointCloud<PointT> &pcl_out, geometry_msgs::msg::TransformStamped tf);

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__PERCEPTION_UTILITIES_HPP