#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <atomic>
#include <string>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <iostream>

#include "warden.h"
#include "safety_monitor_status.h"
#include "quad_state_watchdog_status.h"
#include "trajectory_watchdog_status.h"
#include "map3d.h"

namespace game_engine {

    class SafetyMonitor {
    private:
        volatile std::atomic_bool ok_{true};
        int revision_mode_;
    public:
        SafetyMonitor(const int& revision_mode)
            : revision_mode_(revision_mode) {}

        void Run(
            const std::string& key,
            const Map3D& map,
            std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
            std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
            std::shared_ptr<QuadStateWarden> quad_state_warden,
            std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
            std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status);

        void TimeRevision();

        void ShapeRevision();

        void WaypointRevision();

        // Stop this thread
        void Stop();

    };
}

#endif