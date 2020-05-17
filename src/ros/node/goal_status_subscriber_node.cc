// Author: Dan LaChapelle

#include "goal_status_subscriber_node.h"

namespace game_engine {
  GoalStatusSubscriberNode::GoalStatusSubscriberNode(
      const std::string& topic, 
      std::shared_ptr<GoalStatus> goal_status) {
    this->goal_status_ = goal_status;
    this->node_handle_ = ros::NodeHandle("/game_engine/");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &GoalStatusSubscriberNode::SubscriberCallback, 
        this);
  }

  void GoalStatusSubscriberNode::SubscriberCallback(const mg_msgs::GoalStatus& msg) {
    Eigen::Vector3d position;
    position[0] = msg.pos.x;
    position[1] = msg.pos.y;
    position[2] = msg.pos.z;

    GoalStatus goal_status {
      .active = msg.active.data,
      .reached = msg.reached.data,
      .scorer = msg.scorer.data,
      .reach_time = msg.reach_time.data,
      .position = position,
      .set_start = msg.set_start.data
    };
    *(this->goal_status_) = goal_status;
  }
}
