// Author: Dan LaChapelle

#include "goal_status_publisher_node.h"

#include <chrono>

namespace game_engine {
GoalStatusPublisherNode::GoalStatusPublisherNode(const std::string& topic) {
  this->publisher_guard_ =
      std::make_shared<PublisherGuard<mg_msgs::GoalStatus>>(topic);
}

void GoalStatusPublisherNode::Publish(const GoalStatus& goal_status) {
  mg_msgs::GoalStatus msg;
  msg.header.frame_id = "world";
  msg.header.stamp.sec = 0;
  msg.header.stamp.nsec = 0;
  msg.active.data = goal_status.active;
  msg.reached.data = goal_status.reached;
  msg.scorer.data = goal_status.scorer;
  msg.reach_time.data = goal_status.reach_time;
  msg.pos.x = goal_status.position.x();
  msg.pos.y = goal_status.position.y();
  msg.pos.z = goal_status.position.z();
  msg.set_start.data = goal_status.set_start;

  this->publisher_guard_->Publish(msg);
}
}  // namespace game_engine
