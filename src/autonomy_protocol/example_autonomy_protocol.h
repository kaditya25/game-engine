#pragma once

#include "autonomy_protocol.h"

namespace game_engine {
  // The ExampleAutonomyProtocol is a class that demonstrates how to create a
  // trajectory for a quadcopter to follow. The example protocol instructs a
  // quadcopter to fly in a circle of radius two at an altitude of one meter.
  //
  // The trajectory with be specified by parameterizing the circle with time.
  // The quadcopter must complete a revolution every five seconds.
  class ExampleAutonomyProtocol : public AutonomyProtocol {
    private:

    public:
      ExampleAutonomyProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWardenClient> trajectory_warden_client,
          const std::shared_ptr<PreSubmissionTrajectoryVetter> prevetter,
          const Map3D& map3d,
          const std::shared_ptr<BalloonStatus> red_balloon_status,
          const std::shared_ptr<BalloonStatus> blue_balloon_status,
          const Eigen::Vector3d& goal_position,
          const WindIntensity& wind_intensity)
        : AutonomyProtocol(
            friendly_names,
            enemy_names,
            snapshot,
            trajectory_warden_client,
            prevetter,
            map3d,
            red_balloon_status,
            blue_balloon_status,
            goal_position,
            wind_intensity) {}
            
      std::unordered_map<std::string, Trajectory> UpdateTrajectories() override;
  };
}
