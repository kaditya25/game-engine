// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory_warden.h"
#include "trajectory.h"

namespace mediation_layer {
  // Trajectory subscriber acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms incoming ROS data into a
  // trajectory and then passes it to the TrajectoryWarden to manage.
  template <size_t T>
  class TrajectorySubscriberNode {
    private:
      std::shared_ptr<TrajectoryWarden<T>> warden_;
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_;
      std::string key_;

      void SubscriberCallback(const std_msgs::String& msg);
      
    public:
      // Constructor.
      //
      // Note parameters are intentionally copied.
      TrajectorySubscriberNode(
          const std::string& topic, 
          const std::string& key,
          std::shared_ptr<TrajectoryWarden<T>> warden);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  TrajectorySubscriberNode<T>::TrajectorySubscriberNode(
      const std::string& topic, 
      const std::string& key,
      std::shared_ptr<TrajectoryWarden<T>> warden) {
    this->key_ = key;
    this->warden_ = warden;
    this->node_handle_ = ros::NodeHandle("~");
    this->subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &TrajectorySubscriberNode<T>::SubscriberCallback, 
        this);
  }

  template <size_t T>
  void TrajectorySubscriberNode<T>::SubscriberCallback(const std_msgs::String& msg) {
    Trajectory<T> trajectory;
    this->warden_->Write(this->key_, trajectory);
  }

  using TrajectorySubscriberNode2D = TrajectorySubscriberNode<2>;
  using TrajectorySubscriberNode3D = TrajectorySubscriberNode<3>;
};
