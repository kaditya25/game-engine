#pragma once

#include <ros/ros.h>

#include <functional>
#include <memory>
#include <string>

#include "mg_msgs/PVAYT.h"
#include "trajectory.h"
#include "trajectory_code.h"

namespace game_engine {
class TrajectoryClientNode {
 private:
  // ROS node handle
  ros::NodeHandle node_handle_;

  // ROS subscriber
  ros::ServiceClient client_;

 public:
  // Constructor
  TrajectoryClientNode(const std::string& topic);

  // Publishes the message
  TrajectoryCode Request(const Trajectory& trajectory);
};
}  // namespace game_engine
