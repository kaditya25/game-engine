#ifndef BALLOON_POSITION_SUBSCRIBER_NODE_H
#define BALLOON_POSITION_SUBSCRIBER_NODE_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <Eigen/StdVector>
#include <functional>
#include <memory>
#include <string>

namespace game_engine {
class BalloonPositionSubscriberNode {
 private:
  // ROS node handle
  ros::NodeHandle node_handle_;

  // ROS subscriber
  ros::Subscriber subscriber_;

  // Subscriber callback function. Converts ROS message into local
  // Eigen::Vector3d message

  void SubscriberCallback(const geometry_msgs::Point& msg);

 public:
  // Pointer to balloon status. No guarantees on read/write threading access
  std::shared_ptr<Eigen::Vector3d> balloon_position_;
  // Constructor.
  //
  // Note parameters are intentionally copied.
  BalloonPositionSubscriberNode(
      const std::string& topic,
      std::shared_ptr<Eigen::Vector3d> balloon_position);
};
}  // namespace game_engine

#endif