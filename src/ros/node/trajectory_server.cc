

#include "trajectory_server.h"

namespace game_engine {
  TrajectoryServerNode::TrajectoryServerNode(
      const std::string& topic,
      const std::string& key,
      std::shared_ptr<TrajectoryWardenIn> warden) {
        this->key_ = key;
        this->warden_ = warden;
        this->node_handle_ = ros::NodeHandle("/game_engine/");
        this->service_ = node_handle_.advertiseService(
          topic,
          &TrajectoryServerNode::ServiceCallback,
          this);
  }

  bool TrajectoryServerNode::ServiceCallback(mg_msgs::PVAYT::Request &req, mg_msgs::PVAYT::Response &res)
    {
    // Required data structure. Formatted as follows:
    //   [ pos(3), vel(3), acc(3), yaw(1), time(1)]
    std::vector<
      Eigen::Matrix<double, 11, 1>,
      Eigen::aligned_allocator<Eigen::Matrix<double, 11, 1>>> data;
  for(const mg_msgs::PVAYStamped& instant: req.trajectory) {
    Eigen::Matrix<double, 11, 1> local_instant;
    local_instant(0) = instant.pos.x;
    local_instant(1) = instant.pos.y;
    local_instant(2) = instant.pos.z;
    local_instant(3) = instant.vel.x;
    local_instant(4) = instant.vel.y;
    local_instant(5) = instant.vel.z;
    local_instant(6) = instant.acc.x;
    local_instant(7) = instant.acc.y;
    local_instant(8) = instant.acc.z;
    local_instant(9) = instant.yaw;
    local_instant(10) = instant.header.stamp.sec + (double)instant.header.stamp.nsec / 1e9;
    data.push_back(local_instant);
  }
  // blocking = true
  StatusCode status = this->warden_->Write(this->key_, Trajectory(data), true);
  res.status = status;
  return true;
  }
}
