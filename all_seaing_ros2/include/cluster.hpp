// 2021-03
// Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
// License: MIT

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/common/impl/common.hpp>

#include "auvlab_msgs/Centroids.h"
#include "auvlab_msgs/CloudCluster.h"
#include "auvlab_msgs/CloudClusterArray.h"
#include "auvlab_msgs/BoundingBox.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nmea_msgs/Sentence.h"
#include "std_msgs/Time.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <limits>
#include <cmath>
#include <sstream>

class Cluster {
    private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_;
        pcl::PointXYZI min_point_;
        pcl::PointXYZI max_point_;
        pcl::PointXYZI average_point_;
        pcl::PointXYZI centroid_;
        float length_, width_, height_, area_;
        double orientation_angle_;

        Eigen::Matrix3f eigen_vectors_;
        Eigen::Vector3f eigen_values_;
        auvlab_msgs::BoundingBox bounding_box_;

        geometry_msgs::PolygonStamped polygon_;
        std_msgs::Header ros_header_;
        std::string nmea_header_;

        std::string label_;
        int id_;
        std_msgs::Time last_seen_;

        bool valid_cluster_;

    public:
        /**
         * @brief Constructor. Creates a Cluster object using the specified points in a point cloud
         * @param in_origin_cloud_ptr Origin PC
         * @param in_cluster_indices  Inices of the Origin point cloud  to create the Cluster object
         * @param in_id               ID of the cluster
         * @param in_label            label to identify the cluster (optional)
         * @param nmea_msg_header     nmea obstacle msg sensor type, i.e. PYROB, PYLOB
         */
        void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                const std::vector<int>& in_cluster_indices,
                std_msgs::Header in_ros_header,
                int in_id,
                std::string in_label,
                std::string nmea_msg_header,
                std_msgs::Time last_seen);

        /**
         * @brief Constructs the auvlab_msgs::CloudCluster message associated to this Cluster 
         * @param in_ros_header sensor header message 
         * @param out_cluster_message CloudCluster ROS message output
         */
        void ToROSMessage(std_msgs::Header in_ros_header,
                auvlab_msgs::CloudCluster& out_cluster_message);


        /** @brief Constructor */
        Cluster();
        virtual ~Cluster();

        /** @brief Returns the pointer to the pointcloud containing the points in this Cluster */
        pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloud();
        /** @brief Returns the minimum point in the cluster */
        pcl::PointXYZI GetMinPoint();
        /** @brief Returns the maximum point in the cluster */
        pcl::PointXYZI GetMaxPoint();
        /** @brief Returns the average point in the cluster */
        pcl::PointXYZI GetAveragePoint();
        /** @brief Returns the centroid point in the cluster */
        pcl::PointXYZI GetCentroidPoint();
        /** @brief Returns the convex hull (polygon) of the cluster */
        geometry_msgs::PolygonStamped GetPolygon();
        /** @brief Returns the area of the polygon (convex hull) */
        float GetPolygonArea();
        /** @brief Returns the length of the cluster */
        float GetLength();
        /** @brief Returns the width of the cluster */
        float GetWidth();
        /** @brief Returns the height of the cluster */
        float GetHeight();
        /** @brief Returns the orientation angle in radians of the Bounding Box */
        double GetOrientationAngle();
        /** @brief Returns the ID of the cluster */
        int GetID();
        /** @brief Returns the Bounding Box of the cluster */
        auvlab_msgs::BoundingBox GetBoundingBox();
        /** @brief Returns the Eigen Vectors of the cluster */
        Eigen::Matrix3f GetEigenVectors();
        /** @brief Returns the Eigen Values of the cluster */
        Eigen::Vector3f GetEigenValues();
        /** @brief Returns the NEMA style message to be sent to MOOS-IvP for reporting Cluster convex hull */
        std::string GetNMEA();
        /** @brief Returns the NMEA header of the cluster */
        std::string GetNMEAHeader();
        /** @brief Returns the ROS header of the cluster */
        std_msgs::Header GetROSHeader();
        /** @brief Returns if the cluster is valid or not */
        bool IsValid();
        /** @brief Sets whether cluster is valid or not */
        void SetValidity(bool in_valid);
        /** @brief Returns message header */
        std_msgs::Header GetHeader();
        /** @brief Returns last scene tick*/
        std_msgs::Time GetLastSeen();
        /** @brief Sets last scene tick*/
        std_msgs::Time SetLastSeen(std_msgs::Time value);
        /** @brief Sets cluster id */
        int SetID(int id);
};

#endif /* CLUSTER_H_ */
