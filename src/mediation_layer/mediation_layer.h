// Author: Tucker Haydon

#pragma once

#include <atomic>
#include <string>
#include <random>

#include "trajectory_warden.h"
#include "quad_state_warden.h"
#include "quad_state_watchdog_status.h"
#include "trajectory_client.h"
#include "map3d.h"

namespace game_engine {
  // The mediation layer is a software layer that mediates user input to ensure
  // that the trajectories provided to quadcopters are safe. During the
  // machine games, two user are allowed to specify trajectories for
  // quadcopters. The user-provided trajectories may not be safe --- the
  // trajectories might cause quads to fly into each other, walls, or other
  // obstacles in the environment.
  //
  // TODO: Modify trajectories to prevent quads from flying into obstacles
  // TODO: Vet trajectories and ensure that they do not exceed prescribed bounds
  class MediationLayer {
    private:
      volatile std::atomic_bool ok_{true};

      // Transfers data associated with key from trajectory_warden_in to
      // trajectory_warden_out
      void TransferData(
          const std::string& key,
          const Map3D& map,
          std::shared_ptr<TrajectoryWardenIn> trajectory_warden_in,
          std::shared_ptr<TrajectoryWardenOut> trajectory_warden_out,
          std::shared_ptr<QuadStateWarden> quad_state_warden,
          std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
          std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> trajectory_clients);

    public:
      MediationLayer() {}

      // Run the mediation layer.
      //
      // Note: These values are intentionally copied
      void Run(
          const Map3D& map,
          std::shared_ptr<TrajectoryWardenIn> trajectory_warden_in,
          std::shared_ptr<TrajectoryWardenOut> trajectory_warden_out,
          std::shared_ptr<QuadStateWarden> state_warden,
          std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
          std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> trajectory_clients);

      // Stop this thread and all sub-threads
      void Stop();

  };
}
