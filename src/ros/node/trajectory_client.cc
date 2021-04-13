
 #include "trajectory_client.h"


 namespace game_engine {
   TrajectoryClientNode::TrajectoryClientNode(
       const std::string& topic) {
     this->client_
         = node_handle_.serviceClient<mg_msgs::PVAYT>(topic);
   }

   unsigned int TrajectoryClientNode::Request(const Trajectory& trajectory) {
     mg_msgs::PVAYT srv;
     for(size_t idx = 0; idx < trajectory.Size(); ++idx) {
       mg_msgs::PVAYStamped instant;
       instant.pos.x = trajectory.Position(idx).x();
       instant.pos.y = trajectory.Position(idx).y();
       instant.pos.z = trajectory.Position(idx).z();
       instant.vel.x = trajectory.Velocity(idx).x();
       instant.vel.y = trajectory.Velocity(idx).y();
       instant.vel.z = trajectory.Velocity(idx).z();
       instant.acc.x = trajectory.Acceleration(idx).x();
       instant.acc.y = trajectory.Acceleration(idx).y();
       instant.acc.z = trajectory.Acceleration(idx).z();

       instant.yaw = trajectory.Yaw(idx);
       instant.header.frame_id = "world";
       instant.header.stamp.sec = std::floor(trajectory.Time(idx));
       instant.header.stamp.nsec
         = (trajectory.Time(idx) - std::floor(trajectory.Time(idx))) * 1e9;

       srv.request.trajectory.push_back(instant);
       }
       if (client_.call(srv)) {
         //ROS_INFO("Client: Successfully called service. Response: %d ", srv.response.status);
         return srv.response.status;
       } else {
         ROS_ERROR("Client: Failed to call service.");
         return FailedToCallService;
       }
    }
 }
