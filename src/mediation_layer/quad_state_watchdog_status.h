#pragma once

#include <atomic>
#include <string>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include "trajectory_code.h"

namespace game_engine {
    struct InfractionInfo {
        MediationLayerCode code;
        long int time;
    };
  // Struct that contains information about the status of the QuadStateWatchdog. If
  // the QuadStateWatchdog determines that a quadcopter has flown too close to an
  // obstacle, an instance of this status is updated to reflect that.
  class QuadStateWatchdogStatus {
    public:

      QuadStateWatchdogStatus() {}
      void Register(const std::string& quad_name);
      InfractionInfo Read(const std::string& quad_name) const;
      void Write(const std::string& quad_name, const MediationLayerCode infraction_occurred);
      void SetExecution(const std::string& quad_name, const bool execution);
      bool ReadExecution(const std::string& quad_name);

    private:
      // Mutex to manage multi-threaded access
      mutable std::mutex mtx_;

      // Map from quad name to whether or not an infraction has occurred
      std::unordered_map<std::string, InfractionInfo> infraction_map_;

      std::unordered_map<std::string, bool> allow_execution_;

  };
}
