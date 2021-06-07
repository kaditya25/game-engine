#include "quad_state_watchdog.h"

#include <thread>
#include <chrono>
#include <Eigen/Core>

namespace game_engine {

  void QuadStateWatchdog::Run(
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::vector<std::string>& quad_names,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
      const Map3D map) {

    // Inflate the map by min_distance. This creates a new map whose obstacles
    // have been expanded by min_distance and whose boundaries have been shrunk
    // by min_distance. After inflating the map, the quadcopter may be treated
    // as a point-particle. To determine if the quadcopter is within an
    // l-infinity distance of min_distance from any obstacle, simply check
    // whether the quad's center point is intersecting any of the inflate
    // obstacles. The distance between quads is also determined and a violation
    // is reported if the quads get too close.
    const Map3D inflated_map = map.Inflate(this->options_.min_distance);

    while(this->ok_) {
      for(const std::string& quad_name: quad_names) {
        // Read in the current state
        QuadState current_state;
        quad_state_warden->Read(quad_name, current_state);

        // Get the current position of the quad
        Eigen::Vector3d current_position = current_state.Position();

        // Evaluate whether the current position intersects an obstacle
        bool infraction_occurred =
            !inflated_map.IsFreeSpace(current_position)
            || !inflated_map.Contains(current_position);

        if(true == infraction_occurred) {
          quad_state_watchdog_status->Write(quad_name, true);
        }

        // Check if current quad too close to another quad
        if (quad_names.size() > 1) {
          for(const std::string& other_quad_name: quad_names) {
            if(quad_name != other_quad_name) {
              // Grab position of other quad
              QuadState other_quad_current_state;
              quad_state_warden->Read(other_quad_name, other_quad_current_state);
              Eigen::Vector3d other_quad_current_position = other_quad_current_state.Position();
              // Check if distance between quads is less than the limit
              if((other_quad_current_position - current_position).norm() < this->options_.min_distance_btwn_quads) {
                quad_state_watchdog_status->Write(quad_name, true);
              }
            }
          }
        }
      }
      // 20 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

  }

  void QuadStateWatchdog::Stop() {
    this->ok_ = false;
  }
}
