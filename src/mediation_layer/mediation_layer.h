#pragma once

#include <atomic>
#include <random>
#include <string>

#include "map3d.h"
#include "polyhedron.h"
#include "quad_safety_status.h"
#include "quad_state_watchdog_status.h"
#include "safety_monitor_status.h"
#include "trajectory_client.h"
#include "trajectory_code.h"
#include "trajectory_publisher_node.h"
#include "trajectory_watchdog_status.h"
#include "types.h"
#include "warden.h"

namespace game_engine {
// The mediation layer is a software layer that mediates user input to ensure
// that the trajectories provided to quadcopters are safe. During the
// machine games, two user are allowed to specify trajectories for
// quadcopters. The user-provided trajectories may not be safe --- the
// trajectories might cause quads to fly into each other, walls, or other
// obstacles in the environment.
//
class MediationLayer {
 private:
  int quad_safety_limits_ = 0;
  bool joy_mode_ = false;
  double inflation_distance_ = 0;
  volatile std::atomic_bool ok_{true};

  // Transfers data associated with key from trajectory_warden_srv to
  // trajectory_warden_pub
  void TransferData(
      const std::string& key, const Map3D& map,
      std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
      std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
      std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status,
      std::shared_ptr<SafetyMonitorStatus> safety_monitor_status,
      std::shared_ptr<TrajectoryPublisherNode> publisher);

  TrajectoryVector3D FreezeQuad(const std::string& key,
                                const Eigen::Vector3d freeze_quad_position);
  bool IsQuadMovingAwayFromOtherQuad(const Trajectory main_trajectory,
                                     const Polyhedron violation_space);
  bool IsQuadMovingAwayFromObstacle(const Trajectory main_trajectory,
                                    const Map3D inflated_map);
  Polyhedron ExpandQuad(const Eigen::Vector3d cm,
                        const double inflation_distance);

 public:
  MediationLayer(const int& quad_safety_limits, const bool& joy_mode)
      : quad_safety_limits_(quad_safety_limits), joy_mode_(joy_mode) {
    if (quad_safety_limits_ == 2) {
      // extreme mode
      inflation_distance_ =
          1.25 *
          0.5;  // tested value on the real quads for unfreezing capabilities.
    } else if (quad_safety_limits_ == 1) {
      // sport mode
      inflation_distance_ = 1.0 * 0.75;
    } else {
      // leisure mode and default
      inflation_distance_ = 0.4 * 0.75;
    }
  }

  // Run the mediation layer.
  //
  // Note: These values are intentionally copied
  void Run(
      const Map3D& map,
      std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
      std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
      std::shared_ptr<QuadStateWarden> state_warden,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
      std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status,
      std::shared_ptr<SafetyMonitorStatus> safety_monitor_status,
      std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>>
          trajectory_publishers);

  // Stop this thread and all sub-threads
  void Stop();
};
}  // namespace game_engine
