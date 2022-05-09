#include "safety_monitor_status.h"

namespace game_engine {
void SafetyMonitorStatus::Register(const std::string& quad_name) {
  std::lock_guard<std::mutex> lock(mtx_);
  this->safety_map_[quad_name] = QuadSafetyStatus::OK;
}

QuadSafetyStatus SafetyMonitorStatus::Read(const std::string& quad_name) const {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    return this->safety_map_.at(quad_name);
  } catch (std::out_of_range e) {
    std::cerr << "Quad with name " << quad_name
              << " is not registered with the SafetyMonitor." << std::endl;
    return QuadSafetyStatus::OK;
  }
}

void SafetyMonitorStatus::Write(const std::string& quad_name,
                                const QuadSafetyStatus current_safety_level) {
  std::lock_guard<std::mutex> lock(mtx_);
  try {
    this->safety_map_.at(quad_name) = current_safety_level;
  } catch (std::out_of_range e) {
    std::cerr << "Quad with name " << quad_name
              << " is not registered with the SafetyMonitor." << std::endl;
    return;
  }
}
}  // namespace game_engine
