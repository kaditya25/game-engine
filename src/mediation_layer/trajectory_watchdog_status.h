#pragma once

#include <atomic>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "trajectory_code.h"

namespace game_engine {
// Struct that contains information about the status of the TrajectoryWatchdog.
// If the TrajectoryWatchdog determines that a quadcopter has flown too close to
// an obstacle, an instance of this status is updated to reflect that.
class TrajectoryWatchdogStatus {
 private:
  // Mutex to manage multi-threaded access
  mutable std::mutex mtx_;

  // Map from quad name to whether or not an infraction has occurred
  std::unordered_map<std::string, TrajectoryCode> infraction_map_;

 public:
  TrajectoryWatchdogStatus() {}
  void Register(const std::string& quad_name);
  TrajectoryCode Read(const std::string& quad_name) const;
  void Write(const std::string& quad_name,
             const TrajectoryCode infraction_occurs);
};
}  // namespace game_engine
