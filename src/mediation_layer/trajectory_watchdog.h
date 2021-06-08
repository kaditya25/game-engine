#pragma once

#include <memory>
#include <string>

#include "trajectory.h"
#include "map3d.h"
#include "warden.h"
#include "trajectory_code.h"

namespace game_engine {
    // The trajectory watchdog watches the trajectories of the quadcopters and determines
    // if any of them are intersecting within a certain lookahead time. If they will intersect, a
    // violation is reported.
    class TrajectoryWatchdog {
    public:
        struct Options {
            // Lookahead time for simulation
            double simulation_forward_time = 5; //seconds

            Options() {}
        };

        TrajectoryWatchdog(const Options& options = Options())
            : options_(options) {}

        // Main thread function
        void Run(const Trajectory& trajectory,
                 const Map3D& map,
                 const std::shared_ptr<QuadStateWarden> quad_state_warden,
                 const std::vector<std::string>& quad_names) ;

        // Stop this thread
        void Stop();

    private:
        volatile std::atomic_bool ok_{true};
        Options options_;
    };
}
