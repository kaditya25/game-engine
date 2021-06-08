#include "trajectory_watchdog.h"

#include <Eigen/Core>
#include <iostream>

namespace game_engine {

    void TrajectoryWatchdog::Run(const Trajectory& trajectory,
                                 const Map3D& map,
                                 const std::shared_ptr<QuadStateWarden> quad_state_warden,
                                 const std::vector<std::string>& quad_names) {

      // TODO: fill out with intersecting trajectories!!

    }

    void TrajectoryWatchdog::Stop() {
      this->ok_ = false;
    }
}