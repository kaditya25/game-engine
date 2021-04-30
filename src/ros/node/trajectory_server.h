#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory.h"
#include "warden.h"
#include "mg_msgs/PVAYT.h"
#include "trajectory_code.h"

namespace game_engine {
  class TrajectoryServerNode {
    private:
      // Trajectory warden manages multi-threaded access to trajectory data
      std::shared_ptr<TrajectoryWardenServer> warden_;

      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriber
      ros::ServiceServer service_;

      // Key to be passed on to the trajectory warden
      std::string key_;

      // Service callback. Extracts ROS data and converts it into a
      // Trajectory instance to be passed on to the trajectory warden
      bool ServiceCallback(mg_msgs::PVAYT::Request &req, mg_msgs::PVAYT::Response &res);

    public:
      // Constructor.

      // Note parameters are intentionally copied.
      TrajectoryServerNode(
          const std::string& topic,
          const std::string& key,
          std::shared_ptr<TrajectoryWardenServer> warden);
  };
}
