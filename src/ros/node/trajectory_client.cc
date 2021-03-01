
 #include "trajectory_client.h"


 namespace game_engine {
   TrajectoryClientNode::TrajectoryClientNode(
       const std::string& topic) {
     this->client_
       = node_handle_.serviceClient<mg_msgs::PVAYT>(topic);
   }

   void TrajectoryClientNode::Publish(const Trajectory& trajectory) {
     mg_msgs::PVAYT srv;
     if (client_.call(srv)) {
       mg_msgs::PVAYT::Request msg;
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

         msg.trajectory.push_back(instant);
       }

       // Need to add publishing functionality here
       //this->publisher_node_->Publish(msg);
       ROS_INFO("Accepted? %d", srv.response.accept);
     }
     else {
       ROS_ERROR("Failed to call service");
     }

   }
 }
