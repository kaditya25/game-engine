#include "balloon_status_subscriber_node.h"

namespace game_engine {
  BalloonStatusSubscriberNode::
  BalloonStatusSubscriberNode(const std::string& topic, 
                              std::shared_ptr<BalloonStatus> balloon_status) {
    this->balloon_status_ = balloon_status;
    this->node_handle_ = ros::NodeHandle("/game_engine/");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &BalloonStatusSubscriberNode::SubscriberCallback, 
        this);
  }

  void BalloonStatusSubscriberNode::
  SubscriberCallback(const mg_msgs::BalloonStatus& msg) {
    Eigen::Vector3d position;
    position[0] = msg.pos.x;
    position[1] = msg.pos.y;
    position[2] = msg.pos.z;

    BalloonStatus balloon_status {
      .popped = static_cast<bool>(msg.popped.data),
      .popper = msg.popper.data,
      .pop_time = msg.pop_time.data,
      .position = position,
      .set_start = static_cast<bool>(msg.set_start.data)
    };
    *(this->balloon_status_) = balloon_status;
  }
}
