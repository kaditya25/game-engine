

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>
#include "sensor_msgs/Joy.h"
#include "joyDrivers.h"
#include "joyMapper.h"
#include "joyStructs.h"

namespace game_engine {
  class JoySubscriberNode {
    private:
      // ROS node handle
      ros::NodeHandle node_handle_;

      // ROS subscriber
      ros::Subscriber subscriber_;

      // Subscriber callback function. Converts ROS message into local
      // Joy message
      void SubscriberCallback(const sensor_msgs::Joy& msg);
      
    public:

      std::shared_ptr<joyStruct> joy_input_;
      // Constructor.
      //
      // Note parameters are intentionally copied.
      JoySubscriberNode(
          const std::string& topic, 
          std::shared_ptr<joyStruct> joy_status
          );
  };
}
