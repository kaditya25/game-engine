#include "quad_state_watchdog_status.h"

namespace game_engine {
  void QuadStateWatchdogStatus::Register(const std::string& quad_name) {
    std::lock_guard<std::mutex> lock(mtx_);
    InfractionInfo info = {MediationLayerCode::RegisterQuadWithWatchdog,
                           0};
    this->infraction_map_[quad_name] = info;
    this->allow_execution_[quad_name] = false;
  }

    InfractionInfo QuadStateWatchdogStatus::Read(const std::string& quad_name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      return this->infraction_map_.at(quad_name);
    } catch(std::out_of_range e) {
      std::cerr << "Quad with name " << quad_name 
        << " is not registered with the QuadStateWatchdog." << std::endl;
      return {MediationLayerCode::QuadNotRegistered, 0};
    }
  }
  
  void QuadStateWatchdogStatus::Write(const std::string& quad_name, const MediationLayerCode infraction_occurred) {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

      InfractionInfo info = {infraction_occurred,
                             time};
      this->infraction_map_.at(quad_name) = info;
    } catch(std::out_of_range e) {
      std::cerr << "Quad with name " << quad_name 
        << " is not registered with the QuadStateWatchdog." << std::endl;
      return;
    }
  }

  void QuadStateWatchdogStatus::AllowExecution(const std::string& quad_name) {
    std::lock_guard<std::mutex> lock(mtx_);
    this->allow_execution_[quad_name] = true;
  }

  bool QuadStateWatchdogStatus::ReadExecution(const std::string& quad_name) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool allow = false;
    try {
      allow = this->allow_execution_.at(quad_name);
      this->allow_execution_[quad_name] = false;
      return allow;
    } catch(std::out_of_range e) {
      std::cerr << "Quad with name " << quad_name
                << " is not registered with the QuadStateWatchdog." << std::endl;
      return allow;
    }
  }
}
