#pragma once

#include <memory>
#include <string>

#include "map3d.h"
#include "trajectory.h"
#include "trajectory_code.h"
#include "warden.h"

namespace game_engine {
// The TrajectoryVetter determines if a Trajectory complies with a set of
// specified requirements. The requirements are:
//   1) A quad following the trajectory will not exceed the boundaries of the
//      map
//   2) A quad following the trajectory will not pass through or touch any
//      obstacles
//   3) A quad following the trajectory will not exceed the maximum specified
//      velocity.
//   4) A quad following the trajectory will not exceed the maximum
//      specified acceleration.
//   5) The quad following the trajectory is no more that a specified maximum
//      distance from the first point in the trajectory
//   6) The time between trajectory samples is no more than a specified
//      maximum delta-time
//
class TrajectoryVetter {
 public:
  struct Options {
    // Maximum allowed velocity in any direction in m/s
    double max_velocity_magnitude;

    // Maximum allowed l2 norm of acceleration, in m/s^2
    double max_acceleration_magnitude;

    // The maximum distance from a quad's current position that a
    // trajectory's first point may deviate in meters
    double max_distance_from_current_position = 1.0;

    // The maximum time between trajectory samples in seconds
    double max_delta_t = 0.020;

    // Minimum l-infinity distance from all obstacles that a quad may fly
    double min_distance;

    Options() {}
  };

  int quad_safety_limits_;
  Options options_;

  TrajectoryVetter(const int& quad_safety_limits,
                   const Options& options = Options())
      : quad_safety_limits_(quad_safety_limits), options_(options) {
    if (quad_safety_limits_ == 1) {
      // sport mode
      options_.max_velocity_magnitude = 3.0;
      options_.max_acceleration_magnitude = 1.5;
      options_.min_distance = 1.25;
    } else if (quad_safety_limits_ == 1.25) {
      // extreme mode
      options_.max_velocity_magnitude = 3.0;
      options_.max_acceleration_magnitude = 3.0;
      options_.min_distance = 1.0;
    } else {
      // safe and default
      options_.max_velocity_magnitude = 2.0;
      options_.max_acceleration_magnitude = 0.4;
      options_.min_distance = .40;
    }
  }

  // Determines if a trajectory meets the trajectory requirements laid out
  // in the documentation
  TrajectoryCode Vet(const Trajectory& trajectory, const Map3D& map,
                     const std::shared_ptr<QuadStateWarden> quad_state_warden,
                     const std::string& quad_name) const;
};
}  // namespace game_engine
