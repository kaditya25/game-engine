#include "balloon_position_publisher_node.h"

#include <chrono>

namespace game_engine {
BalloonPositionPublisherNode::BalloonPositionPublisherNode(
    const std::string& topic) {
  this->publisher_guard_ =
      std::make_shared<PublisherGuard<geometry_msgs::Point>>(topic);
}

void BalloonPositionPublisherNode::Publish(
    const Eigen::Vector3d& balloon_position) {
  geometry_msgs::Point msg;
  msg.x = balloon_position.x();
  msg.y = balloon_position.y();
  msg.z = balloon_position.z();

  this->publisher_guard_->Publish(msg);
}
}  // namespace game_engine
