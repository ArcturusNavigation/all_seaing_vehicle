#ifndef BUOY_LAYER_HPP_
#define BUOY_LAYER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

namespace nav2_buoy_costmap_plugin
{
	class BuoyLayer : public nav2_costmap_2d::Layer
	{
	public:
		BuoyLayer();

		virtual void onInitialize();
		virtual void updateBounds(
			double robot_x, double robot_y, double robot_yaw, double *min_x,
			double *min_y, double *max_x, double *max_y);
		virtual void updateCosts(
			nav2_costmap_2d::Costmap2D &master_grid,
			int min_i, int min_j, int max_i, int max_j);

		virtual void reset() { return; }

		virtual void onFootprintChanged();

		virtual bool isClearable() { return false; }

	private:
		void obstacleCallback(const geometry_msgs::msg::PoseArray &msg);
		double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
		bool need_recalculation_;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
		std::vector<std::pair<int, int>> obstacles_;
	};
}

#endif // BUOY_LAYER_HPP_
