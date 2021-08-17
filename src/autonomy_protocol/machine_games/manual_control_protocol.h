#pragma once

#include "autonomy_protocol.h"

#include "joyMapper.h"
#include "joyStructs.h"
#include "joyDrivers.h"

namespace game_engine {
  class ManualControlProtocol : public AutonomyProtocol {
    private:
    	std::shared_ptr<joyStruct> joy_input_;
    	
    	Eigen::Vector3d vel_output_;
    	Eigen::Vector3d rpy_output_;
    	JoyParams joy_params_;
    	
    public:
       ManualControlProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWardenClient> trajectory_warden_client,
          const std::shared_ptr<PreSubmissionTrajectoryVetter> prevetter,
          const Map3D& map3d,
          const std::shared_ptr<BalloonStatus> red_balloon_status,
          const std::shared_ptr<Eigen::Vector3d> red_balloon_position,
          const std::shared_ptr<BalloonStatus> blue_balloon_status,
          const std::shared_ptr<Eigen::Vector3d> blue_balloon_position,
          const Eigen::Vector3d& goal_position,
          const WindIntensity& wind_intensity,
          const std::shared_ptr<joyStruct> joy_input)
        : AutonomyProtocol(
            friendly_names,
            enemy_names,
            snapshot,
            trajectory_warden_client,
            prevetter,
            map3d,
            red_balloon_status,
            red_balloon_position,
            blue_balloon_status,
            blue_balloon_position,
            goal_position,
            wind_intensity),
            joy_input_(joy_input)
            {}
      
      std::unordered_map<std::string, Trajectory> UpdateTrajectories() override;
      
      TrajectoryVector3D joyToTrajectory(Eigen::Vector3d current_position, Eigen::Vector3d current_velocity, 
      	std::vector<Eigen::Vector3d> path_points, double yaw, std::vector<double> times);
      	
      void loadJoyParams();
  };
}
