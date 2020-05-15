// Author: Tucker Haydon

#pragma once

#include <string>
#include <chrono>
#include <Eigen/Dense>

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

    //Eigen::Vector3d position = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Vector3d position;

    bool set_start;
  };
}
