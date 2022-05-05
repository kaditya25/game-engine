#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <string>

namespace game_engine {
// Plain-old-data structure containing information regarding a balloon's
// status
struct BalloonStatus {
  // Whether the balloon has been popped or not
  bool popped = false;

  // Which quad popped the balloon
  std::string popper = "null";

  // The time the balloon was popped at
  double pop_time;

  bool set_start;
};
}  // namespace game_engine
