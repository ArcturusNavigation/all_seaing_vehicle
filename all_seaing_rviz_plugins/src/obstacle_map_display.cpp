#include "all_seaing_rviz_plugins/obstacle_map_display.hpp"
#include "rviz_common/logging.hpp"

namespace all_seaing_rviz_plugins {

ObstacleMapDisplay::ObstacleMapDisplay()
    : rviz_common::RosTopicDisplay<all_seaing_interfaces::msg::ObstacleMap>(),
      m_marker_common(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this)) {}

void ObstacleMapDisplay::onInitialize() {
    RTDClass::onInitialize();
    m_marker_common->initialize(context_, scene_node_);
}

void ObstacleMapDisplay::load(const rviz_common::Config &config) {
    Display::load(config);
    m_marker_common->load(config);
}

void ObstacleMapDisplay::processMessage(
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr msg) {

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
        RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '"
                                     << msg->header.frame_id << "' to frame '"
                                     << qPrintable(fixed_frame_) << "'");
    }
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    for (size_t i = 0; i < msg->obstacles.size(); i++) {
        const auto marker_ptr = get_marker(msg->obstacles[i]);
        marker_ptr->ns = msg->ns;
        const auto text_marker_ptr = get_text(msg->is_labeled, msg->obstacles[i]);
        text_marker_ptr->ns = msg->ns + "_text";

        m_marker_common->addMessage(marker_ptr);
        m_marker_common->addMessage(text_marker_ptr);
    }
}

visualization_msgs::msg::Marker::SharedPtr
ObstacleMapDisplay::get_marker(const all_seaing_interfaces::msg::Obstacle &obstacle) const {
    auto marker = std::make_shared<visualization_msgs::msg::Marker>();

    marker->type = visualization_msgs::msg::Marker::SPHERE;
    marker->action = visualization_msgs::msg::Marker::ADD;

    marker->id = obstacle.id;
    marker->header = obstacle.global_point.header;

    marker->color.a = 1.0;
    marker->color.r = 1.0;
    marker->scale.x = 0.3;
    marker->scale.y = 0.3;
    marker->scale.z = 0.3;

    marker->pose.position.x = obstacle.global_point.point.x;
    marker->pose.position.y = obstacle.global_point.point.y;

    return marker;
}

visualization_msgs::msg::Marker::SharedPtr
ObstacleMapDisplay::get_text(bool is_labeled, const all_seaing_interfaces::msg::Obstacle &obstacle) const {
    auto marker = std::make_shared<visualization_msgs::msg::Marker>();

    marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker->action = visualization_msgs::msg::Marker::ADD;

    marker->id = obstacle.id;
    marker->header = obstacle.global_point.header;

    marker->scale.z = 0.7;
    marker->color.a = 1.0;
    marker->color.g = 1.0;

    if (is_labeled)
        marker->text = std::to_string(obstacle.label);
    else
        marker->text = std::to_string(obstacle.id);

    marker->pose.position.x = obstacle.global_point.point.x;
    marker->pose.position.y = obstacle.global_point.point.y;
    marker->pose.position.z = 1.0;

    return marker;
}

void ObstacleMapDisplay::update(float wall_dt, float ros_dt) {
    m_marker_common->update(wall_dt, ros_dt);
}

void ObstacleMapDisplay::reset() {
    RosTopicDisplay::reset();
    m_marker_common->clearMarkers();
}

} // namespace all_seaing_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(all_seaing_rviz_plugins::ObstacleMapDisplay, rviz_common::Display)