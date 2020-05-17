// Author: Dan LaChapelle

#pragma once

#include <memory>
#include <string>
#include <Eigen/Core>
#include <vector>
#include <random>

#include "quad_state_warden.h"
#include "goal_status_publisher_node.h"
#include "goal_status_subscriber_node.h"

namespace game_engine {
  // The GoalWatchdog watches the position of quadcopters and determines if
  // the quadcopter has reached the goal. If it has, update the status of the
  // goal over ROS.
  //
  // Should be run as its own thread
  class GoalWatchdog {
    public:
      struct Options {
        // Distance from the center of the goal that a quad must achieve to
        // reach a goal, in meters
        double reach_distance = 0.20;

        // quad speed must be less than this amount to reach goal
        double reach_speed = 0.1;

        // time that must elapse before goal can be reached
        double time_fuze = 20.0;

        Options() {}
      };

      GoalWatchdog(const Options& options = Options())
        : options_(options) {}

      // Main thread function
      void Run(
          std::shared_ptr<GoalStatusPublisherNode> goal_status_publisher,
          std::shared_ptr<GoalStatusSubscriberNode> goal_status_subscriber,
          std::shared_ptr<QuadStateWarden> quad_state_warden,
          const std::vector<std::string>& quad_names,
          Eigen::Vector3d& goal_position);

      // Stop this thread
      void Stop();

    private:
      volatile std::atomic_bool ok_{true};
      Options options_;

  };
}
