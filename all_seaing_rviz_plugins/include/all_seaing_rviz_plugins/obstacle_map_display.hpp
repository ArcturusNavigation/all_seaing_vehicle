#ifndef ALL_SEAING_RVIZ_PLUGINS__OBSTACLE_MAP_DISPLAY_HPP_
#define ALL_SEAING_RVIZ_PLUGINS__OBSTACLE_MAP_DISPLAY_HPP_

#include <memory>

#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "rviz_common/display.hpp"
#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker_array/marker_array_display.hpp"

namespace all_seaing_rviz_plugins {

class ObstacleMapDisplay
    : public rviz_common::RosTopicDisplay<all_seaing_interfaces::msg::ObstacleMap> {

public:
    ObstacleMapDisplay();
    void onInitialize() override;
    void load(const rviz_common::Config &config) override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

private:
    void processMessage(const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr msg) override;
    visualization_msgs::msg::Marker::SharedPtr
    get_centroid_marker(const all_seaing_interfaces::msg::Obstacle &obstacle) const;
    visualization_msgs::msg::Marker::SharedPtr
    get_vertex_marker(const all_seaing_interfaces::msg::Obstacle &obstacle) const;
    visualization_msgs::msg::Marker::SharedPtr
    get_text(bool is_labeled, const all_seaing_interfaces::msg::Obstacle &obstacle) const;
    std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> m_marker_common;
};

} // namespace all_seaing_rviz_plugins

#endif // ALL_SEAING_RVIZ_PLUGINS__OBSTACLE_MAP_DISPLAY_HPP_