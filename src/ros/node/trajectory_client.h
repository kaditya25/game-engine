#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory.h"
#include "mg_msgs/PVAYT.h"
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
    TrajectoryCode Call(const Trajectory& trajectory);
  };
}
