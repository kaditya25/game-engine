#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <iostream>
#include <condition_variable>
#include <atomic>

#include "trajectory.h"
#include "trajectory_code.h"

namespace game_engine {
  // TrajectoryWarden is a thread-safe abstraction around an unordered map
  // [trajectory_name -> trajectory]. TrajectoryWarden provides thread-safe access,
  // modification, and await-modification of the underlying trajectory.
  class TrajectoryWarden {
  private:
    // Wraps a Trajectory with local mutexes and condition variables that
    // ensure thread-safe access
    struct TrajectoryContainer {
      Trajectory trajectory_;
      std::mutex access_mtx_;

      std::mutex modified_mtx_;
      bool modified_{false};
      std::condition_variable modified_cv_;

      TrajectoryContainer(const Trajectory& trajectory)
        : trajectory_(trajectory) {}
    };

    std::unordered_map<std::string, std::shared_ptr<TrajectoryContainer>> map_;
    std::set<std::string> keys_;
    volatile std::atomic<bool> ok_{true};
    volatile std::atomic<bool> statusUpdated_{false};
    TrajectoryCode trajectoryStatus_{TrajectoryCode::Success};
    TrajectoryCode GetLastTrajectoryStatus(bool blocking);

  public:
    TrajectoryWarden(){};
    // Adds a key-value pair to the map
    TrajectoryCode Register(const std::string& key);
    // Writes a new trajectory
    TrajectoryCode Write(const std::string& key,
                         const Trajectory& trajectory,
                         bool blocking = false);
    // Copies the latest trajectory associated with a key
    TrajectoryCode Read(const std::string& key, Trajectory& trajectory);
    // Awaits a change to the trajectory associated with the key
    TrajectoryCode Await(const std::string& key, Trajectory& trajectory);
    const std::set<std::string>& Keys() const;
    void SetTrajectoryStatus(TrajectoryCode status);
    void Stop();
  };
}
