

#pragma once

#include <ros/ros.h>

#include <functional>
#include <memory>
#include <string>

#include "balloon_status.h"
#include "mg_msgs/BalloonStatus.h"

namespace game_engine {
class BalloonStatusSubscriberNode {
 private:
  // ROS node handle
  ros::NodeHandle node_handle_;

  // ROS subscriber
  ros::Subscriber subscriber_;

  // Subscriber callback function. Converts ROS message into local
  // BalloonStatus message
  void SubscriberCallback(const mg_msgs::BalloonStatus& msg);

 public:
  // Pointer to balloon status. No guarantees on read/write threading access
  std::shared_ptr<BalloonStatus> balloon_status_;
  // Constructor.
  //
  // Note parameters are intentionally copied.
  BalloonStatusSubscriberNode(const std::string& topic,
                              std::shared_ptr<BalloonStatus> balloon_status);
};
}  // namespace game_engine
