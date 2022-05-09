#ifndef BALLOON_POSITION_PUBLISHER_NODE_H
#define BALLOON_POSITION_PUBLISHER_NODE_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <Eigen/StdVector>
#include <functional>
#include <memory>
#include <string>

#include "publisher_guard.h"
#include "quad_state.h"

namespace game_engine {
class BalloonPositionPublisherNode {
 private:
  // A publisher guard ensures that the Publish() function may be called in
  // a thread-safe manner
  std::shared_ptr<PublisherGuard<geometry_msgs::Point>> publisher_guard_;

 public:
  // Constructor.
  BalloonPositionPublisherNode(const std::string& topic);

  // Publishes the message
  void Publish(const Eigen::Vector3d& balloon_position);
};
}  // namespace game_engine

#endif
