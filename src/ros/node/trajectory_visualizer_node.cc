#include "trajectory_visualizer_node.h"

namespace game_engine {
  TrajectoryVisualizerNode::
  TrajectoryVisualizerNode(const std::string& topic_subscribe,
                           const std::string& topic_publish) {
    this->node_handle_ = ros::NodeHandle("/game_engine/");
    this->subscriber_ =
      node_handle_.subscribe(topic_subscribe, 
                             1, 
                             &TrajectoryVisualizerNode::SubscriberCallback, 
                             this);
    this->publisher_guard_ =
      std::make_shared<PublisherGuard<nav_msgs::Path>>(topic_publish);
  }

  void TrajectoryVisualizerNode::
  SubscriberCallback(const mg_msgs::PVAYStampedTrajectory& msg) {

    this->path_.poses.clear();
    for(const mg_msgs::PVAYStamped& instant : msg.trajectory) {
      if(!(std::isnan(instant.pos.x) || std::isnan(instant.pos.y) ||
           std::isnan(instant.pos.z || std::isnan(instant.header.seq)) ||
           std::isnan(instant.header.stamp.sec) ||
           std::isnan(instant.header.stamp.nsec))) {
        geometry_msgs::PoseStamped ps;
        ps.header = instant.header;
        ps.pose.position.x = instant.pos.x;
        ps.pose.position.y = instant.pos.y;
        ps.pose.position.z = instant.pos.z;
        ps.pose.orientation.x = 0;
        ps.pose.orientation.y = 0;
        ps.pose.orientation.z = 0;
        ps.pose.orientation.w = 1;
        this->path_.poses.push_back(ps);
        this->path_.header = ps.header;
      }
    }
  }

  void TrajectoryVisualizerNode::Publish() {
    if(!this->path_.poses.empty()) {
      this->publisher_guard_->Publish(path_);
    }
  }  
}

