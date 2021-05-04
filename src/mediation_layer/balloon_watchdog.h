#pragma once

#include <memory>
#include <string>
#include <Eigen/Core>
#include <vector>
#include <random>

#include "warden.h"
#include "balloon_status_publisher_node.h"
#include "balloon_status_subscriber_node.h"

namespace game_engine {
  // The BalloonWatchdog watches the position of quadcopters and determines if
  // the quadcopter has popped the balloon. If it has, update the status of the
  // balloons over ROS.
  //
  // Should be run as its own thread
  class BalloonWatchdog {
    public:
      struct Options {
        // Distance from the center of the balloon that a quad must achieve to
        // 'pop' a balloon in meters
        double pop_distance = 0.30;

        Options() {}
      };

      BalloonWatchdog(const Options& options = Options())
        : options_(options) {}

      // Main thread function
      void Run(
          std::shared_ptr<BalloonStatusPublisherNode> balloon_status_publisher,
          std::shared_ptr<BalloonStatusSubscriberNode> balloon_status_subscriber,
          std::shared_ptr<QuadStateWarden> quad_state_warden,
          const std::vector<std::string>& quad_names,
          Eigen::Vector3d& balloon_position,
          Eigen::Vector3d& new_balloon_position,
          double max_move_time,
          std::mt19937& gen);

      // Stop this thread
      void Stop();

    private:
      volatile std::atomic_bool ok_{true};
      Options options_;

  };
}
