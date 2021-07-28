#include "trajectory_watchdog.h"

#include <Eigen/Core>
#include <iostream>

namespace game_engine {

    void TrajectoryWatchdog::Run(const std::vector<std::string>& quad_names,
                                 const std::shared_ptr<QuadStateWarden> quad_state_warden,
                                 const std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
                                 const std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
                                 const std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status) {

      while(this->ok_) {
        for (const std::string &quad_name: quad_names) {
          // CHECK TRAJECTORIES OF THE QUADS TO SEE IF THEY INTERSECT
          // Grab trajectory of first quad
          Trajectory main_trajectory;
          trajectory_warden_pub->Read(quad_name, main_trajectory);
          size_t lookahead_index_main;
          for(size_t idx = 0; idx < main_trajectory.Size(); ++idx) {
            double time = main_trajectory.Time(idx);
            if(time <= this->options_.simulation_forward_time) {
              lookahead_index_main++;
            } else {
              break;
            }
          }

          // Check if current quad too close to another quad
          if (quad_names.size() > 1) {
            for (const std::string &other_quad_name: quad_names) {
              if (quad_name != other_quad_name) {
                // Grab trajectory of other quad
                Trajectory secondary_trajectory;
                trajectory_warden_pub->Read(other_quad_name, secondary_trajectory);
                size_t lookahead_index_second;
                for(size_t idx = 0; idx < secondary_trajectory.Size(); ++idx) {
                  double time = secondary_trajectory.Time(idx);
                  if(time <= this->options_.simulation_forward_time) {
                    lookahead_index_second++;
                  } else {
                    break;
                  }
                }

                TrajectoryCode future_collision;
                for(size_t idx = 0; idx < lookahead_index_main; ++idx) {
                  const Eigen::Vector3d main_position = main_trajectory.Position(idx);
                  for(size_t idx2 = 0; idx2 < lookahead_index_second; ++idx2) {
                    const Eigen::Vector3d secondary_position = secondary_trajectory.Position(idx2);
                    if((main_position - secondary_position).norm() < this->options_.collision_distance) {
                      future_collision.code = MediationLayerCode::QuadTrajectoryCollidesWithAnotherQuad;
                      future_collision.value = (main_position - secondary_position).norm();
                      future_collision.index = idx;
                      trajectory_watchdog_status->Write(quad_name, future_collision);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    void TrajectoryWatchdog::Stop() {
      this->ok_ = false;
    }
}