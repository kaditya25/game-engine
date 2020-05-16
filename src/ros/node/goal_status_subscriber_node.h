// Author: Dan LaChapelle

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "goal_status.h"
#include "mg_msgs/GoalStatus.h"

namespace game_engine {
  class GoalStatusSubscriberNode {
    private:
      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriber
      ros::Subscriber subscriber_;

      // Subscriber callback function. Converts ROS message into local
      // GoalStatus message
      void SubscriberCallback(const mg_msgs::GoalStatus& msg);
      
    public:
      // Pointer to goal status. No guarantees on read/write threading access
      std::shared_ptr<GoalStatus> goal_status_;
      // Constructor.
      //
      // Note parameters are intentionally copied.
      GoalStatusSubscriberNode(
          const std::string& topic, 
          std::shared_ptr<GoalStatus> goal_status
          );
  };
}
