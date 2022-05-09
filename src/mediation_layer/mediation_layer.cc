#include "mediation_layer.h"

#include <chrono>
#include <thread>

#include "trajectory_vetter.h"

namespace game_engine {
TrajectoryVector3D MediationLayer::FreezeQuad(
    const std::string& key, const Eigen::Vector3d freeze_quad_position) {
  std::cout << key
            << " has violated the obstacle/boundary constraints.  Freezing at "
               "current position: "
            << freeze_quad_position.transpose() << std::endl;

  const double current_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  TrajectoryVector3D freeze_trajectory_vector;
  for (size_t idx = 0; idx < 150; ++idx) {
    freeze_trajectory_vector.push_back(
        (Eigen::Matrix<double, 11, 1>() << freeze_quad_position.x(),
         freeze_quad_position.y(), freeze_quad_position.z(), 0, 0, 0, 0, 0, 0,
         0, current_time + idx * 0.01)
            .finished());
  }
  const Trajectory freeze_trajectory(freeze_trajectory_vector);
  return freeze_trajectory_vector;
}

bool MediationLayer::IsQuadMovingAwayFromOtherQuad(
    const Trajectory main_trajectory, const Polyhedron violation_space) {
  // lookahead 15 trajectory points
  size_t look = 15;
  // or pick the total number of points if that is smaller
  size_t lookahead_index = std::min(main_trajectory.Size(), look);
  if (lookahead_index == 0) {
    return false;
  } else {
    const Eigen::Vector3d cm = violation_space.InteriorPoint();
    const double min_dist = (main_trajectory.Position(0) - cm).norm();
    for (size_t idx = 1; idx < lookahead_index; ++idx) {
      // Polyhedron contains sees if a point lies interior in the space
      //          if(violation_space.Contains(main_trajectory.Position(idx))) {
      //            return false;
      //          }
      if ((main_trajectory.Position(idx) - cm).norm() < min_dist) {
        return false;
      }
    }
  }
  return true;
}

bool MediationLayer::IsQuadMovingAwayFromObstacle(
    const Trajectory main_trajectory, const Map3D inflated_map) {
  // lookahead 15 trajectory points
  size_t look = 15;
  // or pick the total number of points if that is smaller
  size_t lookahead_index = std::min(main_trajectory.Size(), look);
  if (lookahead_index == 0) {
    return false;
  } else {
    for (size_t idx = 0; idx < lookahead_index; ++idx) {
      // Evaluate whether the current position intersects an obstacle
      bool infraction_occurred =
          !inflated_map.IsFreeSpace(main_trajectory.Position(idx)) ||
          !inflated_map.Contains(main_trajectory.Position(idx));

      if (infraction_occurred) {
        return false;
      }
    }
  }
  return true;
}

Polyhedron MediationLayer::ExpandQuad(const Eigen::Vector3d cm,
                                      const double inflation_distance) {
  // Build a Polyhedron around the quad and inflate it by inflation distance
  // half size of each direction of the quads
  constexpr float size_x = 0.15;  // 0.3/2
  constexpr float size_y = 0.15;  // 0.3/2
  constexpr float size_z = 0.05;  // 0.1/2
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
  // Expand by the inflation distance
  Polyhedron new_poly = poly.Expand(inflation_distance);
  return new_poly;
}

void MediationLayer::TransferData(
    const std::string& key, const Map3D& map,
    std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
    std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
    std::shared_ptr<QuadStateWarden> quad_state_warden,
    std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
    std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status,
    std::shared_ptr<SafetyMonitorStatus> safety_monitor_status,
    std::shared_ptr<TrajectoryPublisherNode> publisher) {
  TrajectoryVetter trajectory_vetter(quad_safety_limits_);
  const Map3D inflated_map = map.Inflate(inflation_distance_);

  while (this->ok_) {
    // 1) Determine if trajectory has violated constraints
    // 2) Grab lock
    // 3) Check if modified
    // 4) if true, grab trajectory, vet, and then publish
    // 5) else continue

    // Determine if quad has violated state constraints. If it has, freeze it in
    // place
    QuadState current_state;
    quad_state_warden->Read(key, current_state);
    // Get the current position of the quad
    Eigen::Vector3d current_position = current_state.Position();

    if ((quad_state_watchdog_status->Read(key)).code ==
        MediationLayerCode::QuadViolatesMapBoundaries) {
      //          std::cout << key << " Outside boundaries." << std::endl;
      TrajectoryCode trajectoryCode;
      trajectoryCode.code = MediationLayerCode::QuadViolatesMapBoundaries;
      trajectory_warden_srv->SetTrajectoryStatus(key, trajectoryCode);

      TrajectoryVector3D freeze_trajectory_vector =
          FreezeQuad(key, current_position);
      trajectory_warden_pub->Write(key, freeze_trajectory_vector, publisher);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      if (joy_mode_) {
        // if joy mode is true we want the walls of the arena to act as "padded
        // walls", meaning we don't have a permanent game over freeze as we do
        // with regular autonomy protocols
        if (trajectory_warden_srv->ModifiedStatus(key)) {
          Trajectory trajectory;
          trajectory_warden_srv->Await(key, trajectory);

          bool safe_to_move =
              IsQuadMovingAwayFromObstacle(trajectory, inflated_map);
          if (safe_to_move) {
            TrajectoryCode trajectoryCode =
                trajectory_vetter.Vet(trajectory, map, quad_state_warden, key);
            trajectory_warden_srv->SetTrajectoryStatus(key, trajectoryCode);
            if (trajectoryCode.code != MediationLayerCode::Success) {
              std::cout
                  << "Trajectory did not pass vetting: rejected with code "
                  << static_cast<unsigned int>(trajectoryCode.code) << "."
                  << std::endl;
              continue;
            }
            quad_state_watchdog_status->SetExecution(key, true);
            trajectory_warden_pub->Write(key, trajectory, publisher);
          }
        }
      }
    } else if ((quad_state_watchdog_status->Read(key)).code ==
               MediationLayerCode::QuadTooCloseToAnotherQuad) {
      //          std::cout << key << " Quad too close to another quad." <<
      //          std::endl;
      TrajectoryCode trajectoryCode;
      trajectoryCode.code = MediationLayerCode::QuadTooCloseToAnotherQuad;
      trajectory_warden_srv->SetTrajectoryStatus(key, trajectoryCode);

      TrajectoryVector3D freeze_trajectory_vector =
          FreezeQuad(key, current_position);
      trajectory_warden_pub->Write(key, freeze_trajectory_vector, publisher);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      if (trajectory_warden_srv->ModifiedStatus(key)) {
        Trajectory trajectory;
        trajectory_warden_srv->Await(key, trajectory);

        const std::set<std::string> quad_names = quad_state_warden->Keys();
        // Check if current quad too close to another quad
        for (const std::string& other_quad_name : quad_names) {
          if (other_quad_name != key) {
            // Grab position of other quad
            QuadState other_quad_current_state;
            quad_state_warden->Read(other_quad_name, other_quad_current_state);
            const Eigen::Vector3d other_quad_current_position =
                other_quad_current_state.Position();
            const Polyhedron expanded_quad =
                ExpandQuad(other_quad_current_position, 1.0);

            bool safe_to_move =
                IsQuadMovingAwayFromOtherQuad(trajectory, expanded_quad);
            if (safe_to_move) {
              TrajectoryCode trajectoryCode = trajectory_vetter.Vet(
                  trajectory, map, quad_state_warden, key);
              trajectory_warden_srv->SetTrajectoryStatus(key, trajectoryCode);
              if (trajectoryCode.code != MediationLayerCode::Success) {
                std::cout
                    << "Trajectory did not pass vetting: rejected with code "
                    << static_cast<unsigned int>(trajectoryCode.code) << "."
                    << std::endl;
                continue;
              }
              quad_state_watchdog_status->SetExecution(key, true);
              trajectory_warden_pub->Write(key, trajectory, publisher);
            }
          }
        }
      }
    } else if ((trajectory_watchdog_status->Read(key)).code ==
               MediationLayerCode::QuadTrajectoryCollidesWithAnotherQuad) {
      // Figure out where it hits the other quad
      Trajectory main_trajectory;
      trajectory_warden_pub->Read(key, main_trajectory);
      double time =
          main_trajectory.Time((trajectory_watchdog_status->Read(key)).index);
      std::cout << key << " Trajectory collides with another quad in " << time
                << " seconds." << std::endl;
      if (time < 5.0) {
        // intervene in some way
      }
    }

    if (trajectory_warden_srv->ModifiedStatus(key)) {
      Trajectory trajectory;
      trajectory_warden_srv->Await(key, trajectory);
      TrajectoryCode trajectoryCode =
          trajectory_vetter.Vet(trajectory, map, quad_state_warden, key);
      trajectory_warden_srv->SetTrajectoryStatus(key, trajectoryCode);
      if (trajectoryCode.code != MediationLayerCode::Success) {
        std::cerr << "Trajectory did not pass vetting: rejected with code "
                  << static_cast<unsigned int>(trajectoryCode.code) << "."
                  << std::endl;
        continue;
      }
      trajectory_warden_pub->Write(key, trajectory, publisher);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MediationLayer::Run(
    const Map3D& map,
    std::shared_ptr<TrajectoryWardenServer> trajectory_warden_srv,
    std::shared_ptr<TrajectoryWardenPublisher> trajectory_warden_pub,
    std::shared_ptr<QuadStateWarden> quad_state_warden,
    std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
    std::shared_ptr<TrajectoryWatchdogStatus> trajectory_watchdog_status,
    std::shared_ptr<SafetyMonitorStatus> safety_monitor_status,
    std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>>
        trajectory_publishers) {
  // Local thread pool
  std::vector<std::thread> thread_pool;

  // Get all registered trajectories
  const std::set<std::string> state_keys = quad_state_warden->Keys();

  // Assign a thread to await changes in each trajectory
  for (const std::string& key : state_keys) {
    thread_pool.push_back(std::move(std::thread([&]() {
      TransferData(key, map, trajectory_warden_srv, trajectory_warden_pub,
                   quad_state_warden, quad_state_watchdog_status,
                   trajectory_watchdog_status, safety_monitor_status,
                   trajectory_publishers[key]);
    })));
  }

  // Wait for this thread to receive a stop command
  std::thread kill_thread([&, this]() {
    while (true) {
      if (false == ok_) {
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
  });

  kill_thread.join();

  // Wait for thread pool to terminate
  for (std::thread& t : thread_pool) {
    t.join();
  }
}

void MediationLayer::Stop() { ok_ = false; }
}  // namespace game_engine
