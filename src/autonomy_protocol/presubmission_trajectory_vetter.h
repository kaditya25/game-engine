#ifndef PRESUBMISSION_TRAJECTORY_VETTER_H_
#define PRESUBMISSION_TRAJECTORY_VETTER_H_

#include "trajectory_vetter.h"

namespace game_engine {
class PreSubmissionTrajectoryVetter : public TrajectoryVetter {
 private:
  std::shared_ptr<QuadStateWarden> quad_state_warden_;

 public:
  PreSubmissionTrajectoryVetter(
      const int& quad_safety_limits,
      const std::shared_ptr<QuadStateWarden> quad_state_warden)
      : TrajectoryVetter(quad_safety_limits),
        quad_state_warden_(quad_state_warden) {}

  // Determines if a trajectory meets the trajectory requirements
  TrajectoryCode PreVet(const std::string& quad_name,
                        const Trajectory& trajectory, const Map3D& map);
};

//  ******************
//  * IMPLEMENTATION *
//  ******************
// The presubmission trajectory vetter returns exactly the response from the
// mediation layer. This interface is available for student use to pass in a
// trajectory and check its validity before sending to the mediation layer for
// submission. This tool should minimize erroneous submissions to the mediation
// layer. The TrajectoryCode returns the Code (found in util/trajectory_code.h),
// as well as the value and index of the violation if it exists.
inline TrajectoryCode PreSubmissionTrajectoryVetter::PreVet(
    const std::string& quad_name, const Trajectory& trajectory,
    const Map3D& map) {
  // Vet is part of trajectory_vetter.h in the mediation layer.
  return Vet(trajectory, map, quad_state_warden_, quad_name);
}
}  // namespace game_engine

#endif
