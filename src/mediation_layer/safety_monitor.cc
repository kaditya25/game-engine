#include "safety_monitor.h"

#include <thread>

namespace game_engine {

    void SafetyMonitor::Run(
        const std::string& key,
        const Map3D& map,
        std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
        std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
        std::shared_ptr<QuadStateWarden> quad_state_warden,
        std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
        std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status) {

      bool finished = false;

      while(this->ok_) {
        Trajectory trajectory;
        trajectory_warden_pub->Read(key, trajectory);

        if (revision_mode_ == 0) {
          // do nothing
        } else if (revision_mode_ == 1) {
          TimeRevision();

        } else if (revision_mode_ == 2) {
          ShapeRevision();

        } else if (revision_mode_ == 3) {
          // Waypoint mode
          WaypointRevision();

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }

    void SafetyMonitor::TimeRevision() {};

    void SafetyMonitor::ShapeRevision() {};

    void SafetyMonitor::WaypointRevision() {};

    void SafetyMonitor::Stop() {
      this->ok_ = false;
    }
}
