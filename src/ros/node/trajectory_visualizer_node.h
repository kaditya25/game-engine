#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "mg_msgs/PVAYStampedTrajectory.h"
#include "nav_msgs/Path.h"

#include "publisher_guard.h"

namespace game_engine {
  // Subscribes to a trajectory, converts this into a nav_msgs/Path message,
  // and publishes this for display in RVIZ.
  class TrajectoryVisualizerNode {
  private:

    // ROS node handle
    ros::NodeHandle node_handle_;

    // ROS subscriebr
    ros::Subscriber subscriber_;

    // Path
    nav_msgs::Path path_;

    // Subscriber callback. Receives trajectory and stores it in Path
    void SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg);

    // A publisher guard ensures that the Publish() function may be called in
    // a thread-safe manner
    std::shared_ptr<PublisherGuard<nav_msgs::Path>> publisher_guard_;
      
  public:
    // Constructor.
    TrajectoryVisualizerNode(const std::string& topic_subscribe,
                             const std::string& topic_publish);

    // Publishes the message
    void Publish();
  };
}
