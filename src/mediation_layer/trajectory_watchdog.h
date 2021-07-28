#pragma once

#include <memory>
#include <string>

#include "trajectory.h"
#include "trajectory_publisher_node.h"
#include "trajectory_watchdog_status.h"
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
            double collision_distance = 0.4; //meters
            Options() {}
        };

        TrajectoryWatchdog(const Options& options = Options())
            : options_(options) {}

        // Main thread function
        void Run(const std::vector<std::string>& quad_names,
                 const std::shared_ptr<QuadStateWarden> quad_state_warden,
                 const std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
                 const std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
                 const std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status);

        // Stop this thread
        void Stop();

    private:
        volatile std::atomic_bool ok_{true};
        Options options_;
    };
}
