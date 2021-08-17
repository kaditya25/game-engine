#include "quad_state_watchdog.h"

#include <thread>
#include <chrono>
#include <Eigen/Core>

namespace game_engine {

    Polyhedron QuadStateWatchdog::CreateQuad(const Eigen::Vector3d cm) {
      // Build a Polyhedron around the quad and inflate it by inflation distance
      // half size of each direction of the quads
      constexpr float size_x = 0.15; //0.3/2
      constexpr float size_y = 0.15; //0.3/2
      constexpr float size_z = 0.05; //0.1/2
      const double x_coord = cm(0);
      const double y_coord = cm(1);
      const double z_coord = cm(2);

      // Eight corners of the bounding box around the quad
      Point3D p1{x_coord - size_x, y_coord - size_y, z_coord + size_z};
      Point3D p2{x_coord - size_x, y_coord + size_y, z_coord + size_z};
      Point3D p3{x_coord + size_x, y_coord + size_y, z_coord + size_z};
      Point3D p4{x_coord + size_x, y_coord - size_y, z_coord + size_z};
      Point3D p5{x_coord - size_x, y_coord - size_y, z_coord - size_z};
      Point3D p6{x_coord - size_x, y_coord + size_y, z_coord - size_z};
      Point3D p7{x_coord + size_x, y_coord + size_y, z_coord - size_z};
      Point3D p8{x_coord + size_x, y_coord - size_y, z_coord - size_z};

      // Six sides of the bounding box of the quad
      Plane3D bottom{{{p5, p6}, {p6, p7}, {p7, p8}, {p8, p5}}};
      Plane3D top{{{p1, p2}, {p2, p3}, {p3, p4}, {p4, p1}}};
      Plane3D side1{{{p1, p2}, {p2, p6}, {p6, p5}, {p5, p1}}};
      Plane3D side2{{{p2, p3}, {p3, p7}, {p7, p6}, {p6, p2}}};
      Plane3D side3{{{p3, p4}, {p4, p8}, {p8, p7}, {p7, p3}}};
      Plane3D side4{{{p4, p1}, {p1, p5}, {p5, p8}, {p8, p4}}};

      Polyhedron poly{{bottom, top, side1, side2, side3, side4}};
      return poly;
    }

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
    for(const std::string& quad_name: quad_names) {
      this->locked_freeze_[quad_name] = false;
    }

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

        if (infraction_occurred || locked_freeze_.at(quad_name)) {
          quad_state_watchdog_status->Write(quad_name, MediationLayerCode::QuadViolatesMapBoundaries);
          if(!joy_mode_) {
            locked_freeze_[quad_name] = true;
          } else {
            if(quad_state_watchdog_status->ReadExecution(quad_name)) {
              quad_state_watchdog_status->Write(quad_name, MediationLayerCode::Success);
              std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            }
          }
        }

        else if (quad_state_watchdog_status->ReadExecution(quad_name)) {
          quad_state_watchdog_status->Write(quad_name, MediationLayerCode::Success);
          std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }

          // Check if current quad too close to another quad
        else if (quad_names.size() > 1) {
//          inflated_map.ClearDynamicObstacles();
          for (const std::string &other_quad_name: quad_names) {
            if (other_quad_name != quad_name) {
              // Grab position of other quad
              QuadState other_quad_current_state;
              quad_state_warden->Read(other_quad_name, other_quad_current_state);
              Eigen::Vector3d other_quad_current_position = other_quad_current_state.Position();

              quad_state_warden->Read(quad_name, current_state);
              // Get the current position of the quad
              current_position = current_state.Position();
//              inflated_map.AddInflatedDynamicObstacle(other_quad_name, CreateQuad(other_quad_current_position), this->options_.min_distance_btwn_quads);
//
//              if(!inflated_map.IsFreeDynamicSpace(other_quad_name, current_position)) {
//                quad_state_watchdog_status->Write(quad_name, MediationLayerCode::QuadTooCloseToAnotherQuad);
//              } else {
//                quad_state_watchdog_status->Write(quad_name, MediationLayerCode::Success);
//              }
//              // Check if distance between quads is less than the limit
              if ((other_quad_current_position - current_position).norm() < this->options_.min_distance_btwn_quads) {
                quad_state_watchdog_status->Write(quad_name, MediationLayerCode::QuadTooCloseToAnotherQuad);
              } else {
                quad_state_watchdog_status->Write(quad_name, MediationLayerCode::Success);
              }
            }
          }
        } else {
          quad_state_watchdog_status->Write(quad_name, MediationLayerCode::Success);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

  void QuadStateWatchdog::Stop() {
    this->ok_ = false;
  }
}
