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
#include <pcl/common/impl/common.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Geometry>
// #include "tf2_eigen/tf2_eigen.h"

#include "pcl_ros/transforms.hpp"
#include "pcl_ros/impl/transforms.hpp"

#include "all_seaing_interfaces/msg/labeled_bounding_box2_d.hpp"
#include "all_seaing_perception/obstacle.hpp"

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
    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance = 0.0f, int obstacle_sz_min = 1, int obstacle_sz_max = std::numeric_limits<int>::max (), bool conditional = false, std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)> cond_func = std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)>());

    // @overload
    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance = 0.0f, int obstacle_sz_min = 1, int obstacle_sz_max = std::numeric_limits<int>::max (), bool conditional = false, std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)> cond_func = std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)>());

    // @overload
    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance = 0.0f, int obstacle_sz_min = 1, int obstacle_sz_max = std::numeric_limits<int>::max (), bool conditional = false, std::function<bool(const pcl::PointXYZ&, const pcl::PointXYZ&, float)> cond_func = std::function<bool(const pcl::PointXYZ&, const pcl::PointXYZ&, float)>());

    // project bbox to point cloud and assign HSV colors
    void PCLInBBoxHSV(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr, all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox, cv::Mat& img, image_geometry::PinholeCameraModel& cmodel, bool is_sim = false);

    // transform pcl::PointCloud using TF2
    template<typename PointT>
    void transformPCLCloud(const typename pcl::PointCloud<PointT> pcl_in, typename pcl::PointCloud<PointT> &pcl_out, geometry_msgs::msg::TransformStamped tf);

    // compute the minimum oriented bounding box for a point cloud -> (centroid, eigenvectors, axes lengths)
    template<typename PointT>
    std::tuple<Eigen::Vector4d, Eigen::Matrix3d, Eigen::Vector3d> minimumOrientedBBox(const typename pcl::PointCloud<PointT> pcl_in){
        Eigen::Matrix3d cov_mat;
        Eigen::Vector4d centr;
        pcl::computeMeanAndCovarianceMatrix(pcl_in, cov_mat, centr);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov_mat, Eigen::ComputeEigenvectors);
        Eigen::Matrix3d eigenvecs = eigen_solver.eigenvectors();
        Eigen::Vector3d eigenvals = eigen_solver.eigenvalues();
        // from https://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4d projectionTransform(Eigen::Matrix4d::Identity());
        projectionTransform.block<3,3>(0,0) = eigenvecs.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centr.head<3>());
        typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(pcl_in, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        PointT minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        Eigen::Vector3d axes_length = Eigen::Vector3d(maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);
        return std::make_tuple(centr, eigenvecs, axes_length);
    }

    // pick largest cluster
    template<typename PointT>
    pcl::PointCloud<pcl::PointXYZ> pickLargestCluster(typename pcl::PointCloud<PointT> pcl_in, double clust_dist);

    // given axes & pcloud, compute centroid (in original frame) & dimensions (in given axes)
    template<typename PointT>
    std::pair<Eigen::Vector3d, Eigen::Vector3d> centroidDims(typename pcl::PointCloud<PointT> pcl_in, Eigen::Matrix3d axes);

    // given pcl, and centroid, and normal of fitted plane, pick the inliers
    template<typename PointT>
    pcl::PointCloud<pcl::PointXYZ> pickInliers(typename pcl::PointCloud<PointT> pcl_in, Eigen::Vector3d ctr, Eigen::Matrix3d normal, double dist_thres);

    // given a PCL point cloud, perform RANSAC to find the plane the points are in and compute the size of that plane excluding outliers
    // (centroid, normal, size, num_inliers)
    // normal x axis is the plane normal, y axis is left horizontal to the ground, and z is up
    template<typename PointT>
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, int> PCLRANSAC(typename pcl::PointCloud<PointT> pcl_in, double dist_thres, int max_iter, bool cluster = false, double clust_dist = 0.0f);

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__PERCEPTION_UTILITIES_HPP