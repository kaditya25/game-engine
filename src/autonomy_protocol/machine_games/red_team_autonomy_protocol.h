#pragma once

#include "autonomy_protocol.h"

namespace game_engine {
    class RedTeamAutonomyProtocol : public AutonomyProtocol {
    private:

    public:
        RedTeamAutonomyProtocol(
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
                const WindIntensity& wind_intensity)
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
                wind_intensity) {}

        std::unordered_map<std::string, Trajectory> UpdateTrajectories() override;
        
        TrajectoryVector3D targetToTrajectory(Eigen::Vector3d current_position, Eigen::Vector3d current_velocity, 
      		std::vector<Eigen::Vector3d> path_points, double yaw, std::vector<double> times);
    };
}
