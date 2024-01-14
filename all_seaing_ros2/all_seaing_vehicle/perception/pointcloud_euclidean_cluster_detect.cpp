// 2021-03
// Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
// License: MIT
//

/************************************************************
// Usage
// roslaunch pointcloud_cluster_detect pointcloud_cluster_detect.launch pc_topic:="/broaand_radar/channel_0/pointcloud/revolution" filter_cloud:="true" remove_points:="3.0" downsampled_cloud:="true" leaf_size:="1.1" clustering_distance:="5.0" cluster_size_min:="20" cluster_size_max:="1000" cluster_seg_thresh:="5.0" nmea_msg_header:="PYROB"
 ************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <pcl/common/distances.h>
#include <pcl/filters/frustum_culling.h>

#include "cluster.h"

#include "auvlab_msgs/Centroids.h"
#include "auvlab_msgs/CloudCluster.h"
#include "auvlab_msgs/CloudClusterArray.h"

// TODO: Need to make this file a class (OOP style in ROS2) with the node name below
// Maybe we should define a header file as well
#define __APP_NAME__ "euclidean_cluster_detect"

ros::Publisher _pub_cluster_cloud;
ros::Publisher _centroid_pub;
ros::Publisher _pub_clusters_message;
ros::Publisher _pub_nmea_message;
ros::Publisher _pub_matched_clusters_msg;
ros::Publisher _pub_matched_clusters_cloud;
ros::Publisher _marker_array_pub;
ros::Publisher _text_marker_array_pub;
ros::Publisher _bb_marker_pub;
std_msgs::Header _sensor_header;
std_msgs::Time _current_time;

static string _output_frame;
static string _nmea_msg_header;

static bool _using_cloud = false;
static bool _filter_cloud = false;
static bool _downsampled_cloud = false;
static bool _use_multiple_thres = false;
static bool _viz = false;
static bool _filter_camera_view = false;

static double _camera_theta;
static int _camera_hfov;
static double _cull_min;
static double _cull_max;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;
static double _clustering_distance;
static double _cluster_seg_thresh;
static int _drop_cluster_count;
static double _drop_cluster_thresh;
static double _polygon_area_thresh;

static auvlab_msgs::CloudClusterArray prev_clusters;     // Previous clusters not yet fully matched
static auvlab_msgs::CloudClusterArray matched_clusters;  // Matching clusters from prev segments
static auvlab_msgs::CloudClusterArray tracking_clusters; // Active clusters

static std::vector<std::shared_ptr<Cluster>> tracked_clusters; // Active clusters
static int cluster_id = 0; 
static int sweep_count = 0;

/**
 * @brief publishes clusters in point cloud format 
 * @param in_publisher ros publisher
 * @param in_clusters  array of clusters to publish point cloud clusters
 */
void publishCloudClusters(const ros::Publisher *in_publisher,
        const auvlab_msgs::CloudClusterArray &in_clusters,
        const string &in_target_frame, const std_msgs::Header &in_header) {
    // Transform target frame if needed (probably not tho, that's done in the other node)
    // TODO: Honestly this doesn't need to be a separate function probably
    in_publisher->publish(in_clusters);
}

/**
 * @brief publishes centroids of all tracked clusters
 * @param in_publisher ros publisher
 * @param in_centroids centroids to publish
 */
void publishCentroids(const ros::Publisher *in_publisher, const auvlab_msgs::Centroids &in_centroids,
        const string &in_target_frame, const std_msgs::Header &in_header) {
    // TODO: What's going on with these super extra functions we'll delete them later
    in_publisher->publish(in_centroids);
}

/**
 * @brief publishes nmea messeages for each tracked cluster to report to MOOS
 * @param in_publisher ros publisher
 * @param in_clusters  array of clusters to extract and publish nmea msg
 */
void publishNMEA(const ros::Publisher *in_publisher,
        const auvlab_msgs::CloudClusterArray &in_clusters) {
    // TODO: NMEA stuff is outdated
    for (auto cluster : in_clusters.clusters)
        in_publisher->publish(cluster.nmea_msg);
}

// TODO: finish this docstring
/**
 * @brief clusters current point cloud using Euclidean cluster extraction
 * @param in_cloud_ptr            Point cloud to cluster
 * @param out_cloud_ptr
 * @param in_out_centroids        
 * @param in_max_cluster_distance Euclidean cluster tolerance
 * @param nmea_msg_header         NMEA message header to track which type of sensor the clustered object has been created from. Radar: PYROB, Lidar: PYLOB
 *
 * @return vector of Cluster objects determined by the cluster extraction algorithm
 */
std::vector<std::shared_ptr<Cluster>> clusterCloud(const PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
        auvlab_msgs::Centroids &in_out_centroids,
        double in_max_cluster_distance = 0.5,
        std::string nmea_msg_header = "$PYXOB") {
    search::KdTree<pcl::PointXYZ>::Ptr tree(new search::KdTree<pcl::PointXYZ>);

    // Create 2d pointcloud
    PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new PointCloud<pcl::PointXYZ>);
    copyPointCloud(*in_cloud_ptr, *cloud_2d);

    // Flatten cloud
    for (size_t i=0; i<cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;
    if (cloud_2d->points.size() > 0) 
        tree->setInputCloud(cloud_2d);

    // Extract clusters from point cloud and save indices in cluster_indices
    std::vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(in_max_cluster_distance);
    ec.setMinClusterSize(_cluster_size_min);
    ec.setMaxClusterSize(_cluster_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);

    // Cluster Points
    std::vector<std::shared_ptr<Cluster>> clusters;

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        std::shared_ptr<Cluster> cluster(new Cluster());
        cluster->SetCloud(in_cloud_ptr, it->indices, _sensor_header, cluster_id, "",
                nmea_msg_header, _current_time);
        clusters.push_back(cluster);

        cluster_id++;
    }

    return (clusters);
}

/** 
 * @brief matches tracked clusters to current time step clusters. 
 * If cluster is complete pushes full cluster to auvlab_msgs::CloudClusterArray matched_clusters
 * @param in_clusters     current sensor sweep clusters (all_clusters) 
 * @param in_out_clusters tracked clusters (tracked_clusters)
 */
void matchClusters(std::vector<std::shared_ptr<Cluster>> &in_clusters,
        std::vector<std::shared_ptr<Cluster>> &in_out_clusters) {
    // Vector of Cluster object type returned from clusterCloud
    std::vector<std::shared_ptr<Cluster>> track_clusters;  

    // Check in_out_clusters for aged out clusters
    for (int i = 0; i < in_out_clusters.size(); i++) {
        float last_seen;
        std_msgs::Time cluster_last_seen = in_out_clusters[i]->GetLastSeen();
        last_seen = _current_time.data.toSec() - cluster_last_seen.data.toSec();

        if (last_seen >= _drop_cluster_thresh) {
            if (i > 0) {
                int del = i - 1;
                in_out_clusters.erase(in_out_clusters.begin()+del);
            } else {
                in_out_clusters.erase(in_out_clusters.begin());
            }
        }
    }

    // Find closest clusters and combine if below threshold
    for (int i = 0; i < in_clusters.size(); i++) {
        // Check if Area is to large to be a valid clustered object
        if (in_clusters[i]->GetPolygonArea() > _polygon_area_thresh) {
            in_clusters.erase(in_clusters.begin()+i);
            continue;
        }
        float dist = _cluster_seg_thresh;
        int best_match = -1;
        for (int j = 0; j < in_out_clusters.size(); j++) {
            float this_dist = euclideanDistance(in_out_clusters[j]->GetCentroidPoint(),
                    in_clusters[i]->GetCentroidPoint());
            if (this_dist < dist) {
                best_match = j;
                dist = this_dist;
            }
        }

        if (best_match >= 0) {
            // Change cluster new cluster id to matched cluster id
            in_clusters[i]->SetID(in_out_clusters[best_match]->GetID());
            track_clusters.push_back(in_clusters[i]);
        } else {
            track_clusters.push_back(in_clusters[i]);
        }
    }

    // If no tracked clusters copy current clusters into tracked
    if (in_out_clusters.size() < 1) {
        in_out_clusters = in_clusters;
    } else {
        // Push all new tracked clusters into in_out_clusters
        for (int i=0; i<track_clusters.size(); i++)
            in_out_clusters.push_back(track_clusters[i]);
    }
}

/**
 * @brief point cloud segmentation
 * @param in_cloud_ptr     input point cloud
 * @param out_cloud_ptr    segmented and clustered point cloud
 * @param in_out_centroids auvlab_msgs::Centroids, custom message to track centroid of each clustered object 
 * @param in_out_clusters  auvlab_msgs::CloudClusterArray, custom message to track objects individually
 * @param nmea_msg_header  nmea message header to track which type of sensor the clustered object has been created from. Radar: PYROB, Lidar: PYLOB
 */
void segmentCloud(const PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
        auvlab_msgs::Centroids &in_out_centroids,
        auvlab_msgs::CloudClusterArray &in_out_clusters,
        std::string nmea_msg_header) {
    // Cluster the pointcloud by the distance of the points using different thresholds
    // This helps with clustering at larger distances. See adaptive clustering for more info.
    // Thresholds are based off empirical evidence, further study required for larger distances

    // 0 => 0-20m   d=0.5
    // 1 => 20-50   d=1.0
    // 2 => 50-100  d=1.5
    // 3 => 100-300 d=2.0
    // 4 => >300    d=3.0

    std::vector<std::shared_ptr<Cluster>> all_clusters;
    if (!_use_multiple_thres) {

        PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new PointCloud<pcl::PointXYZ>);
        for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
            pcl::PointXYZ current_point;
            current_point.x = in_cloud_ptr->points[i].x;
            current_point.y = in_cloud_ptr->points[i].y;
            current_point.z = in_cloud_ptr->points[i].z;
            cloud_ptr->points.push_back(current_point);
        }

        all_clusters = clusterCloud(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance, nmea_msg_header);
    }

    // Push raw clusters (all_clusters) to CloudClusterArray and publish
    auvlab_msgs::CloudClusterArray raw_cloud_clusters;
    for (unsigned int i = 0; i < all_clusters.size(); i++) {
        if (all_clusters[i]->IsValid()) {
            auvlab_msgs::CloudCluster raw_cloud_cluster;
            all_clusters[i]->ToROSMessage(_sensor_header, raw_cloud_cluster);
            raw_cloud_clusters.clusters.push_back(raw_cloud_cluster);
        }
    }
    publishCloudClusters(&_pub_clusters_message, raw_cloud_clusters, _output_frame, _sensor_header);

    // Current sensor clusters (all_clusters), tracked_clusters
    matchClusters(all_clusters, tracked_clusters);

    // Final pointcloud to be published  // Should be for the current sweep
    for (unsigned int i = 0; i < all_clusters.size(); i++) {
        *out_cloud_ptr += *all_clusters[i]->GetCloud();

        pcl::PointXYZ center_point = all_clusters[i]->GetCentroidPoint();
        geometry_msgs::Point centroid;
        centroid.x = center_point.x;
        centroid.y = center_point.y;
        centroid.z = center_point.z;

        // Push matched clusters to CloudClusterArray
        if (all_clusters[i]->IsValid()) {
            in_out_centroids.points.push_back(centroid);

            auvlab_msgs::CloudCluster cloud_cluster;
            all_clusters[i]->ToROSMessage(_sensor_header, cloud_cluster);
            in_out_clusters.clusters.push_back(cloud_cluster);
        }
    }
}

/**
 * @brief point cloud publisher
 * @param in_pubisher             ros publisher
 * @param in_cloud_to_publish_ptr point cloud to publish 
 */
void publishCloud(const ros::Publisher *in_publisher,
        const PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr) {
    sensor_msgs::PointCloud2 cloud_msg;
    toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _sensor_header;
    in_publisher->publish(cloud_msg);
}

/**
 * @brief filter points around vessel to remove self reflections
 * @param in_cloud_ptr  input point cloud to filter
 * @param out_cloud_ptr output point cloud to store filtered cloud
 * @param in_distance   min distance allowed to keep point
 */
void filterCloud(const PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double min_distance, const double max_distance) {
    out_cloud_ptr->points.clear();
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
        float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) +
                pow(in_cloud_ptr->points[i].y, 2));
        if (origin_distance > min_distance && origin_distance < max_distance)
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
}

/**
 * @brief filter out points that are not in the view of the camera. Try using Frustum Culling if it is supported by ROS
 * @param in_cloud_ptr  input pointcloud to filter
 * @param out_cloud_ptr output pointcloud to store filtered cloud
 * @param fov           horizontal fov of the camera
 * @param theta         pose of the camera w.r.t the camera origin
//                   Frustum and the vectors a, b, c and d. T is the position of the camera
//                        http://docs.ros.org/en/hydro/api/pcl/html/frustum__culling_8hpp_source.html
//                             _________
//                           /|       . |
//                       d  / |   c .   |
//                         /  | __._____| 
//                        /  /  .      .
//                 a <---/-/  .    .
//                      / / .   .  b
//                     /   .
//                     . 
//                   T
*/
void filterCameraView(const PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const int fov, const double theta) {
    float half_fov = float (fov * M_PI / 180) / 2;
    float camera_theta = float (theta * M_PI / 180);
    out_cloud_ptr->points.clear();
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
        float current_theta = atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x);
        if (current_theta > camera_theta - half_fov && current_theta < camera_theta + half_fov)
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
}

/**
 * @brief voxel filter to downsample point cloud
 * @param in_cloud_ptr  input point cloud to downsample
 * @param out_cloud_ptr output point cloud to store downsampled cloud
 * @param in_leaf_size  leaf size to downsample cloud 
 */
void downsampledCloud(const PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2) {
    VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in_cloud_ptr);
    vg.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);
    vg.filter(*out_cloud_ptr);
}

/**
 * @brief function to add convex hull markers and obstacle id for visualization in RVIZ
 * @param in_cluster_array current tracked cluster array
 */
void markers(auvlab_msgs::CloudClusterArray in_cluster_array) {
    visualization_msgs::MarkerArray markers_array;
    visualization_msgs::MarkerArray text_markers_array;
    visualization_msgs::MarkerArray bb_markers_array;

    for (int i = 0; i < in_cluster_array.clusters.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.stamp = _sensor_header.stamp;
        marker.header.frame_id = _sensor_header.frame_id;
        marker.header.seq = _sensor_header.seq;
        marker.ns = "clustering";
        marker.id = in_cluster_array.clusters[i].id;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 1.0;
        marker.lifetime = ros::Duration(1.3);
        marker.scale.z = 1.0;

        visualization_msgs::Marker text_marker;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.header = _sensor_header;
        text_marker.ns = "text_clustering";
        text_marker.id = in_cluster_array.clusters[i].id; 
        text_marker.scale.z = 10.0; // Change for text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = ros::Duration(1.3);
        text_marker.text = to_string(in_cluster_array.clusters[i].id);

        text_marker.pose.position.x = in_cluster_array.clusters[i].max_point.point.x + 3;
        text_marker.pose.position.y = in_cluster_array.clusters[i].centroid_point.point.y;
        text_marker.pose.position.z = in_cluster_array.clusters[i].centroid_point.point.z;
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_markers_array.markers.push_back(text_marker);

        for (int j = 0; j < in_cluster_array.clusters[i].convex_hull.polygon.points.size(); j++) {
            geometry_msgs::Point p;
            p.x = in_cluster_array.clusters[i].convex_hull.polygon.points[j].x;
            p.y = in_cluster_array.clusters[i].convex_hull.polygon.points[j].y;
            p.z = in_cluster_array.clusters[i].convex_hull.polygon.points[j].z;
            marker.points.push_back(p);
        }
        geometry_msgs::Point p;
        p.x = in_cluster_array.clusters[i].convex_hull.polygon.points[0].x;
        p.y = in_cluster_array.clusters[i].convex_hull.polygon.points[0].y;
        p.z = in_cluster_array.clusters[i].convex_hull.polygon.points[0].z;

        marker.points.push_back(p);
        markers_array.markers.push_back(marker);
    }
    _marker_array_pub.publish(markers_array);
    _text_marker_array_pub.publish(text_markers_array);
}

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud) {
    if (!_using_cloud) {
        _using_cloud = true;
        _current_time.data = in_cloud->header.stamp;

        PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new PointCloud<pcl::PointXYZ>);
        PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new PointCloud<pcl::PointXYZ>);
        PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new PointCloud<pcl::PointXYZ>);
        PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_ptr(new PointCloud<pcl::PointXYZ>);
        PointCloud<pcl::PointXYZ>::Ptr camera_cloud_ptr(new PointCloud<pcl::PointXYZ>);

        auvlab_msgs::Centroids centroids;              // Current segment centroids
        auvlab_msgs::CloudClusterArray cloud_clusters; // Current segments clusters

        fromROSMsg(*in_cloud, *current_cloud_ptr);
        _sensor_header = in_cloud->header;

        if (_filter_cloud)
            filterCloud(current_cloud_ptr, filtered_cloud_ptr, _cull_min, _cull_max);
        else
            filtered_cloud_ptr = current_cloud_ptr;

        if (_downsampled_cloud)
            downsampledCloud(filtered_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
        else
            downsampled_cloud_ptr = filtered_cloud_ptr;

        if (_filter_camera_view)
            filterCameraView(downsampled_cloud_ptr, camera_cloud_ptr, _camera_hfov, _camera_theta);
        else
            camera_cloud_ptr = downsampled_cloud_ptr;

        segmentCloud(camera_cloud_ptr, clustered_cloud_ptr, centroids, cloud_clusters, _nmea_msg_header);

        sweep_count++;

        // Cluster_cloud publisher (raw cloud publisher)
        publishCloud(&_pub_cluster_cloud, clustered_cloud_ptr);

        centroids.header = _sensor_header;
        publishCentroids(&_centroid_pub, centroids, _output_frame, _sensor_header);

        // Cloud clusters message publisher (matched clusters)
        cloud_clusters.header = _sensor_header;
        publishCloudClusters(&_pub_matched_clusters_msg, cloud_clusters, _output_frame, _sensor_header);
        publishNMEA(&_pub_nmea_message, cloud_clusters);

        // Visualization
        if (_viz) markers(cloud_clusters);

        _using_cloud = false;
    }
}


int main(int argc, char **argv) {
    // TODO: all this is deprecated ROS1 stuff, need to update!
    // Init ROS
    ros::init (argc, argv, "cluster");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string pc_topic, sensor, app;

    if (private_nh.getParam("pc_topic", pc_topic)) {
        ROS_INFO("euclidean_cluster > Setting pc_topic to %s", pc_topic.c_str());
    } else {
        ROS_INFO("euclidean_cluster > No pointcloud received, defaulting to velodyne_points, use _pc_topic = /topic to set your points topic");
        pc_topic = "/velodyne_points";
    }
    private_nh.getParam("sensor", sensor);

    _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>(pc_topic + "/cluster_cloud", 100);
    _centroid_pub = nh.advertise<auvlab_msgs::Centroids>(pc_topic + "/cluster_centroids", 1);
    _pub_clusters_message = nh.advertise<auvlab_msgs::CloudClusterArray>(pc_topic + "/detection/raw_cloud_clusters", 1);
    _pub_matched_clusters_msg = nh.advertise<auvlab_msgs::CloudClusterArray>(pc_topic + "/detection/matched_cloud_clusters", 1);
    _pub_nmea_message = nh.advertise<nmea_msgs::Sentence>("/nmea_to_send", 100);
    _marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(pc_topic + "/chull_markers", 100);
    _text_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(pc_topic + "/text_markers", 100);

    _using_cloud = false;

    // Init tuning parameters
    private_nh.param("filter_camera_view", _filter_camera_view, false);
    ROS_INFO("filter_camera_view: %d", _filter_camera_view);
    private_nh.param("camera_hfov", _camera_hfov, 80);
    ROS_INFO("camera_hfov: %d", _camera_hfov);
    private_nh.param("camera_theta", _camera_theta, 0.0);
    ROS_INFO("camera_theta: %f", _camera_theta);
    private_nh.param("filter_cloud", _filter_cloud, false);
    ROS_INFO("filter_cloud: %d", _filter_cloud);
    private_nh.param("cull_min", _cull_min, 0.0);
    ROS_INFO("cull_min: %f", _cull_min);
    private_nh.param("cull_max", _cull_max, 0.0);
    ROS_INFO("cull_max: %f", _cull_max);
    private_nh.param("downsampled_cloud", _downsampled_cloud, false);
    ROS_INFO("downsampled_cloud %d", _downsampled_cloud);
    private_nh.param("leaf_size", _leaf_size, 0.0);
    ROS_INFO("leaf_size %f", _leaf_size);
    private_nh.param("cluster_size_min", _cluster_size_min, 20);
    ROS_INFO("cluster_size_min %d", _cluster_size_min);
    private_nh.param("cluster_size_max", _cluster_size_max, 100000);
    ROS_INFO("cluster_size_max %d", _cluster_size_max);
    private_nh.param("clustering_distance", _clustering_distance, 0.75);
    ROS_INFO("clustering_distance %f", _clustering_distance);
    private_nh.param("use_multiple_thres", _use_multiple_thres, false);
    ROS_INFO("use_multiple_thres %d", _use_multiple_thres);
    private_nh.param<string>("output_frame", _output_frame, "velodyne");
    ROS_INFO("output_frame: %s", _output_frame.c_str());
    private_nh.param("cluster_seg_thresh", _cluster_seg_thresh, 1.0);
    ROS_INFO("cluster_seg_thresh: %f", _cluster_seg_thresh);
    private_nh.param("drop_cluster_count", _drop_cluster_count, 5);
    ROS_INFO("drop_cluster_count: %d", _drop_cluster_count);
    private_nh.param("drop_cluster_thresh", _drop_cluster_thresh, 1.0);
    ROS_INFO("%s drop_cluster_thresh: %f", sensor.c_str(), _drop_cluster_thresh);
    private_nh.param("polygon_area_thresh", _polygon_area_thresh, 100000.0);
    ROS_INFO("polygon_area_thresh: %f", _polygon_area_thresh);
    private_nh.param<string>("nmea_msg_header", _nmea_msg_header, "PYXOB");
    _nmea_msg_header = "$" + _nmea_msg_header;
    ROS_INFO("nmea_msg_header: %s", _nmea_msg_header.c_str());
    private_nh.param("viz", _viz, false);
    ROS_INFO("viz: %d", _viz);

    // Subscribe to the input point cloud 'pc_topic'
    ros::Subscriber sub = nh.subscribe(pc_topic, 100, pc_callback);

    // Spin
    ros::spin();
}
