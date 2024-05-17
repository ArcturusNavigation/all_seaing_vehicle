#include "all_seaing_vehicle/cluster_bbox_overlay.hpp"
#include <iostream>

void ClusterBboxOverlay::ClusterBboxFusionCb(
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
		const all_seaing_interfaces::msg::CloudClusterArray::ConstSharedPtr &in_cluster_msg)
{
	// Get transform the first iteration
	if (!pc_cam_tf_ok_)
		pc_cam_tf_ = GetTransform(in_bbox_msg->header.frame_id, in_cluster_msg->header.frame_id);

    // Match clusters and bounding boxes
    all_seaing_interfaces::msg::CloudClusterArray new_cluster_array;
    new_cluster_array.header = in_cluster_msg->header;
    std::unordered_set<int> chosen_indices;
	for (const all_seaing_interfaces::msg::CloudCluster &cluster : in_cluster_msg->clusters)
	{
        // Transform from LiDAR to camera coordinate systems
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = cluster.avg_point.x;
        lidar_point.y = cluster.avg_point.y;
        lidar_point.z = cluster.avg_point.z;
        geometry_msgs::msg::Point tf_point;
        tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, tf_point, pc_cam_tf_);

		// Project 3D point onto the image plane using the intrinsic matrix.
		// Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
	    cv::Point2d xy_rect = cam_model_.project3dToPixel(cv::Point3d(tf_point.y, tf_point.z, -tf_point.x));

		// Match clusters if within bounds and in front of the boat
		if ((xy_rect.x >= 0) && (xy_rect.x < cam_model_.cameraInfo().width) &&
            (xy_rect.y >= 0) && (xy_rect.y < cam_model_.cameraInfo().height) && (cluster.avg_point.x >= 0)) {

		    // RCLCPP_INFO(this->get_logger(), "Projected 2D point: x: %f, y: %f", xy_rect.x, xy_rect.y);

            // Iterate through bounding boxes
            double best_dist = 1e9;
            int best_match = -1;
            for (unsigned long i = 0; i < in_bbox_msg->boxes.size(); i++)
            {
                // Skip indices already chosen
                if (chosen_indices.find(i) != chosen_indices.end())
                    continue;

                const all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = in_bbox_msg->boxes[i];

                double center_x = (bbox.max_x + bbox.min_x) / 2;
                double center_y = (bbox.max_y + bbox.min_y) / 2;

                double curr_dist = std::hypot(center_x - xy_rect.x, center_y - xy_rect.y);
                if (curr_dist < best_dist)
                {
                    best_match = i;
                    best_dist = curr_dist;
                }
            }

            // If best_match was never assigned, then skip
            if (best_match == -1) continue;

            // Add best match index to chosen indices and add label to cluster
            all_seaing_interfaces::msg::CloudCluster new_cluster = cluster;
            new_cluster.label = in_bbox_msg->boxes[best_match].label;
            new_cluster_array.clusters.push_back(new_cluster);
            chosen_indices.insert(best_match);
        }
    }
    cluster_pub_->publish(new_cluster_array);
    if (viz_) markers(new_cluster_array);
}

geometry_msgs::msg::TransformStamped ClusterBboxOverlay::GetTransform(const std::string &in_target_frame,
																      const std::string &in_src_frame)
{
	geometry_msgs::msg::TransformStamped tf;
	pc_cam_tf_ok_ = false;
	try
	{
		tf = tf_buffer_->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
		pc_cam_tf_ok_ = true;
		RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good");
		RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s", in_target_frame.c_str(), in_src_frame.c_str());
	}
	catch (tf2::TransformException &ex)
	{
		RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
	}
	return tf;
}

void ClusterBboxOverlay::IntrinsicsCb(const sensor_msgs::msg::CameraInfo &info_msg)
{
	cam_model_.fromCameraInfo(info_msg);
}

void ClusterBboxOverlay::markers(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array)
{
    visualization_msgs::msg::MarkerArray markers_array;
    visualization_msgs::msg::MarkerArray text_markers_array;

    for (const auto &cluster : in_cluster_array.clusters)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = cluster.header.frame_id;
        marker.ns = "clustering";
        marker.id = cluster.id;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = 0.05;
        marker.lifetime = rclcpp::Duration::from_seconds(1.3);

        visualization_msgs::msg::Marker text_marker;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.header = cluster.header;
        text_marker.ns = "text_clustering";
        text_marker.id = cluster.id;
        text_marker.scale.z = 0.7; // Text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        text_marker.text = std::to_string(cluster.label);

        text_marker.pose.position.x = cluster.max_point.x;
        text_marker.pose.position.y = cluster.avg_point.y;
        text_marker.pose.position.z = cluster.avg_point.z + 1.0;
        text_markers_array.markers.push_back(text_marker);

        for (const auto &p : cluster.convex_hull.points)
        {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            marker.points.push_back(point);
        }

        // Close the loop for LINE_STRIP
        if (!cluster.convex_hull.points.empty())
        {
            geometry_msgs::msg::Point first_point;
            first_point.x = cluster.convex_hull.points.front().x;
            first_point.y = cluster.convex_hull.points.front().y;
            first_point.z = cluster.convex_hull.points.front().z;
            marker.points.push_back(first_point);
        }

        markers_array.markers.push_back(marker);
    }

    marker_array_pub_->publish(markers_array);
    text_marker_array_pub_->publish(text_markers_array);
}

ClusterBboxOverlay::ClusterBboxOverlay() : Node("cluster_bbox_overlay")
{
    // Initialize parameters
    this->declare_parameter<bool>("viz", true);
    viz_ = this->get_parameter("viz").as_bool();

	// Initialize tf_listener pointer
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Subscriptions
	image_intrinsics_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/img_info_src",
		1,
		std::bind(&ClusterBboxOverlay::IntrinsicsCb, this, std::placeholders::_1));
	bbox_sub_.subscribe(this, "/bounding_boxes", rmw_qos_profile_system_default);
	cluster_sub_.subscribe(this, "/matched_cloud_clusters", rmw_qos_profile_system_default);

    // Publisher
	cluster_pub_ = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/labeled_cloud_clusters", 1);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/overlay_chull_markers", 10);
    text_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/overlay_text_markers", 10);

	// Send cluster msg and bbox msg to ClusterBboxFusionCb
	cluster_bbox_sync_ = std::make_shared<Cluster_Bbox_Sync>(Cluster_Bbox_Policy(10), bbox_sub_, cluster_sub_);
	cluster_bbox_sync_->registerCallback(std::bind(&ClusterBboxOverlay::ClusterBboxFusionCb, this, std::placeholders::_1, std::placeholders::_2));
}

ClusterBboxOverlay::~ClusterBboxOverlay() {}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ClusterBboxOverlay>());
	rclcpp::shutdown();
	return 0;
}
