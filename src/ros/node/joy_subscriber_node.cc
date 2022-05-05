#include "joy_subscriber_node.h"

namespace game_engine {
JoySubscriberNode::JoySubscriberNode(const std::string& topic,
                                     std::shared_ptr<joyStruct> joy_status) {
  this->joy_input_ = joy_status;
  this->node_handle_ = ros::NodeHandle("/game_engine/");
  this->subscriber_ = node_handle_.subscribe(
      topic, 1, &JoySubscriberNode::SubscriberCallback, this);
}

void JoySubscriberNode::SubscriberCallback(const sensor_msgs::Joy& msg) {
  *(this->joy_input_) = driverXbox360Wired(msg);
}
}  // namespace game_engine
