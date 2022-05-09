#include "balloon_position_subscriber_node.h"

namespace game_engine {
BalloonPositionSubscriberNode::BalloonPositionSubscriberNode(
    const std::string& topic,
    std::shared_ptr<Eigen::Vector3d> balloon_position) {
  this->balloon_position_ = balloon_position;
  this->node_handle_ = ros::NodeHandle("/game_engine/");
  this->subscriber_ = node_handle_.subscribe(
      topic, 1, &BalloonPositionSubscriberNode::SubscriberCallback, this);
}

void BalloonPositionSubscriberNode::SubscriberCallback(
    const geometry_msgs::Point& msg) {
  Eigen::Vector3d position;
  position[0] = msg.x;
  position[1] = msg.y;
  position[2] = msg.z;

  *(this->balloon_position_) = position;
}
}  // namespace game_engine
