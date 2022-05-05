#pragma once

#include <memory>
#include <string>
#include <vector>

#include "map3d.h"
#include "quad_state_watchdog_status.h"
#include "trajectory_code.h"
#include "warden.h"

namespace game_engine {
// The state watchdog watches the positions of the quadcopters and determines
// if any of them have flown too close to any obstacles. If they have, a
// StateWatchdogStatus instance is set to report this violation
class QuadStateWatchdog {
 public:
  struct Options {
    // Minimum l-infinity distance from all obstacles that a quad may fly
    double min_distance;
    // Minimum l-infinity distance from all other quads that a quad may fly
    double min_distance_btwn_quads = 1.0;

    Options() {}
  };

  QuadStateWatchdog(const int& quad_safety_limits, const bool& joy_mode,
                    const Options& options = Options())
      : quad_safety_limits_(quad_safety_limits),
        joy_mode_(joy_mode),
        options_(options) {
    if (quad_safety_limits_ == 2) {
      // extreme mode
      options_.min_distance = 1.25;
    } else if (quad_safety_limits_ == 1) {
      // sport mode
      options_.min_distance = 1.0;
    } else {
      // leisure mode and default
      options_.min_distance = 0.4;
    }
  }

  // Main thread function
  void Run(std::shared_ptr<QuadStateWarden> quad_state_warden,
           const std::vector<std::string>& quad_names,
           std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
           const Map3D map);

  Polyhedron CreateQuad(const Eigen::Vector3d cm);

  // Stop this thread
  void Stop();

 private:
  volatile std::atomic_bool ok_{true};
  Options options_;
  int quad_safety_limits_;
  bool joy_mode_;
  std::unordered_map<std::string, bool> locked_freeze_;
};
}  // namespace game_engine
