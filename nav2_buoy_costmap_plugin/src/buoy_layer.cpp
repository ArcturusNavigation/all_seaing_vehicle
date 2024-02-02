#include "nav2_buoy_costmap_plugin/buoy_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_buoy_costmap_plugin
{
    BuoyLayer::BuoyLayer()
        : last_min_x_(-std::numeric_limits<float>::max()),
          last_min_y_(-std::numeric_limits<float>::max()),
          last_max_x_(std::numeric_limits<float>::max()),
          last_max_y_(std::numeric_limits<float>::max())
    {
    }

    void BuoyLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);
        // TODO: Change this to use actual data
        obstacle_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "/obstacles",
            10,
            std::bind(&BuoyLayer::obstacleCallback, this, std::placeholders::_1));

        need_recalculation_ = false;
        current_ = true;
    }

    void BuoyLayer::obstacleCallback(const geometry_msgs::msg::PoseArray &msg)
    {
        obstacles_.clear();
        for (geometry_msgs::msg::Pose pose : msg.poses)
            obstacles_.push_back(std::make_pair((int)pose.position.x, (int)pose.position.y));
    }

    void BuoyLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;

            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        }
        else
        {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }

    void BuoyLayer::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
                     "BuoyLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

    void BuoyLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
        int max_i,
        int max_j)
    {
        if (!enabled_)
            return;

        unsigned char *master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        for (std::pair<int, int> obstacle : obstacles_)
        {
            int x = obstacle.first + (size_x / 2);
            int y = obstacle.second + (size_y / 2);
            if (x > min_i && x < max_i && y > min_j && y < max_j)
            {
                int index = master_grid.getIndex(x, y);
                master_array[index] = LETHAL_OBSTACLE;
            }
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_buoy_costmap_plugin::BuoyLayer, nav2_costmap_2d::Layer)
