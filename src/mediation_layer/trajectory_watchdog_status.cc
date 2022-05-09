#include "trajectory_watchdog_status.h"

namespace game_engine {
TrajectoryCode trajectory_code;
void TrajectoryWatchdogStatus::Register(const std::string& quad_name) {
  std::lock_guard<std::mutex> lock(mtx_);
  trajectory_code.code = MediationLayerCode::RegisterQuadWithWatchdog;
  this->infraction_map_[quad_name] = trajectory_code;
}

TrajectoryCode TrajectoryWatchdogStatus::Read(
    const std::string& quad_name) const {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    return this->infraction_map_.at(quad_name);
  } catch (std::out_of_range e) {
    std::cerr << "Quad with name " << quad_name
              << " is not registered with the TrajectoryWatchdog." << std::endl;
    trajectory_code.code = MediationLayerCode::QuadNotRegistered;
    return trajectory_code;
  }
}

void TrajectoryWatchdogStatus::Write(const std::string& quad_name,
                                     const TrajectoryCode infraction_occurs) {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    this->infraction_map_.at(quad_name) = infraction_occurs;
  } catch (std::out_of_range e) {
    std::cerr << "Quad with name " << quad_name
              << " is not registered with the TrajectoryWatchdog." << std::endl;
    return;
  }
}
}  // namespace game_engine
