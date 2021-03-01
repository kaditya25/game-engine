
#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory.h"
//#include "publisher_guard.h"
#include "mg_msgs/PVAYT.h"


namespace game_engine {
  class TrajectoryClientNode {
    private:
      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriebr
      ros::ServiceClient client_;

    public:
      // Constructor.
      TrajectoryClientNode(const std::string& topic);

      // Publishes the message
      void Publish(const Trajectory& trajectory);
  };
}
