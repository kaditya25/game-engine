#pragma once

#include <string>
#include <chrono>
#include <Eigen/Dense>

namespace game_engine {
  // Plain-old-data structure containing information regarding a goal point
  // status
  struct GoalStatus {
    // Whether there is a quad at the goal or not
    bool active = false;

    // Whether the goal has been reached or not
    bool reached = false;

    // Which quad reached the goal
    std::string scorer = "null";

    // The time the goal was reached
    double reach_time;

    Eigen::Vector3d position;

    bool set_start;
  };
}
