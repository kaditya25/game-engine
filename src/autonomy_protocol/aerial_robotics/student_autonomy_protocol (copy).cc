#include "student_autonomy_protocol.h"

#include <chrono>

#include "a_star3d.h"
#include "graph.h"
#include "occupancy_grid3d.h"
#include "path_info.h"
#include "polynomial_sampler.h"
#include "polynomial_solver.h"
#include "student_game_engine_visualizer.h"

namespace game_engine {

PathInfo runAStar(const Graph3D& graph,
                  const std::shared_ptr<Node3D>& start_node,
                  const std::shared_ptr<Node3D>& end_node) {
  AStar3D a_star;
  PathInfo ret = a_star.Run(graph, start_node, end_node);

  return ret;
}

// std::shared_ptr<Node3D> ptr_Phy2Com(Eigen::Vector3d phy_pos,
//                                     OccupancyGrid3D occupancy_grid) {
//   std::tuple<int, int, int> pos_ind =
//       occupancy_grid.mapToGridCoordinates(phy_pos);
//   // std::shared_ptr<Node3D> require an Eigen::Vector3d, but
//   // mapToGridCoordinates produces int
//   Eigen::Vector3d com_pos = {std::get<0>(pos_ind), std::get<1>(pos_ind),
//                              std::get<2>(pos_ind)};
//   std::shared_ptr<Node3D> pos_ptr = std::make_shared<Node3D>(com_pos);
//   return pos_ptr;
// }

// Eigen::MatrixXd ptr_Com2Phy(const PathInfo& path_info,
//                             OccupancyGrid3D occupancy_grid) {
//   Eigen::MatrixXd xyz_Phy;
//   for (int i = 0; i < path_info.details.path_length; i++) {
//     int x_ind = lround(path_info.path[i]->Data().x());
//     int y_ind = lround(path_info.path[i]->Data().y());
//     int z_ind = lround(path_info.path[i]->Data().z());
//     xyz_Phy.row(i) = occupancy_grid.boxCenter(x_ind, y_ind, z_ind);
//   }
//   return xyz_Phy;
// }

TrajectoryVector3D generateTrajectoryVector(
    const int d_order, double dt_factor,
    std::chrono::time_point<std::chrono::system_clock> current_chrono_time,
    const Eigen::MatrixXd xyz_Phy, const Eigen::Vector3d current_vel) {
  // Generates Time Vector for P4 solver
  std::vector<double> times = {};

  // Establishes Constraints for P4 Solver
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
      // Constrains velocity and acceleration for first node
      p4::NodeEqualityBound(0, 0, 1, current_vel(0)),
      p4::NodeEqualityBound(1, 0, 1, current_vel(1)),
      p4::NodeEqualityBound(2, 0, 1, current_vel(2)),
      p4::NodeEqualityBound(0, 0, 2, 0),
      p4::NodeEqualityBound(1, 0, 2, 0),
      p4::NodeEqualityBound(2, 0, 2, 0)};
  // Generates positional constraints
  for (int i = 0; i < xyz_Phy.cols(); i++) {
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(0, i, 0, xyz_Phy(0, i)));
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(1, i, 0, xyz_Phy(1, i)));
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(2, i, 0, xyz_Phy(2, i)));
    times.push_back(dt_factor * i);
  }
  // node_equality_bounds.push_back(p4::NodeEqualityBound(0, xyz_Phy.cols()-1,
  // 1, xyz_Phy(0, i)));
  node_equality_bounds.push_back(
      p4::NodeEqualityBound(1, xyz_Phy.cols() - 1, 1, 0));
  // node_equality_bounds.push_back(p4::NodeEqualityBound(2, xyz_Phy.cols()-1,
  // 1, xyz_Phy(2, i)));

  // Various options for P4 Solver
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 3;
  solver_options.polynomial_order = 8;
  solver_options.continuity_order = 4;
  solver_options.derivative_order = d_order;

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;
  solver_options.osqp_settings.verbose = false;

  // Runs P4 Solver
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path =
      solver.Run(times, node_equality_bounds, {}, {});

  // Sets up P4 Position Sampler options
  p4::PolynomialSampler::Options sampler_p_options;
  sampler_p_options.frequency = 200;
  sampler_p_options.derivative_order = 0;
  // Sets up P4 Velocity Sampler options
  p4::PolynomialSampler::Options sampler_v_options;
  sampler_v_options.frequency = 200;
  sampler_v_options.derivative_order = 1;
  // Sets up P4 Acceleration Sampler options
  p4::PolynomialSampler::Options sampler_a_options;
  sampler_a_options.frequency = 200;
  sampler_a_options.derivative_order = 2;

  // Runs P4 Sampler
  p4::PolynomialSampler sampler_p(sampler_p_options);
  Eigen::MatrixXd samples_p = sampler_p.Run(times, path);
  p4::PolynomialSampler sampler_v(sampler_v_options);
  Eigen::MatrixXd samples_v = sampler_v.Run(times, path);
  p4::PolynomialSampler sampler_a(sampler_a_options);
  Eigen::MatrixXd samples_a = sampler_a.Run(times, path);

  // Generates trajectory vector to return
  TrajectoryVector3D ret;
  for (int i = 0; i < samples_p.cols(); i++) {
    const std::chrono::duration<double> flight_chrono_time =
        current_chrono_time.time_since_epoch() +
        std::chrono::duration<double>(samples_p(0, i));
    ret.push_back((Eigen::Matrix<double, 11, 1>() << samples_p(1, i),
                   samples_p(2, i), samples_p(3, i), samples_v(1, i),
                   samples_v(2, i), samples_v(3, i), samples_a(1, i),
                   samples_a(2, i), samples_a(3, i), 0,
                   flight_chrono_time.count())
                      .finished());
  }

  return ret;
}

std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(100);
constexpr double DISCRETE_LENGTH = 0.2;
constexpr double SAFETY_BOUNDS = 0.5;

std::unordered_map<std::string, Trajectory>
StudentAutonomyProtocol::UpdateTrajectories() {
  // Holds static values needed for autonomy protocol
  static OccupancyGrid3D occupancy_grid;
  static Graph3D graph_of_arena;
  static Student_game_engine_visualizer visualizer;
  static bool first_time = true;
  static bool halt = false;
  static const std::string& quad_name = friendly_names_[0];
  static const std::chrono::time_point<std::chrono::system_clock>
      start_chrono_time = std::chrono::system_clock::now();
  static double dt_factor = 1.2;
  static Eigen::Vector3d start_pos;
  static Eigen::Vector3d prev_pos;
  static int
      target;  // 0 - red balloon || 1 - blue balloon || 2 - return to home

  std::unordered_map<std::string, Trajectory> quad_to_trajectory_map;
  Eigen::Vector3d current_pos;
  Eigen::Vector3d current_vel;
  const std::chrono::time_point<std::chrono::system_clock> current_chrono_time =
      std::chrono::system_clock::now();
  double dt = std::chrono::duration<double>(dt_chrono).count() / 1000;

  snapshot_->Position(quad_name, current_pos);
  snapshot_->Velocity(quad_name, current_vel);

  const bool red_balloon_popped = red_balloon_status_->popped;
  const bool blue_balloon_popped = blue_balloon_status_->popped;
  const Eigen::Vector3d red_balloon_pos = *red_balloon_position_;
  const Eigen::Vector3d blue_balloon_pos = *blue_balloon_position_;

  if (first_time) {
    // Sets up variables for first run of updateTrajectories
    occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
    // minXYZ = occupancy_grid.Origin();
    graph_of_arena = occupancy_grid.AsGraph();
    visualizer.startVisualizing("/game_engine/environment");
    snapshot_->Position(quad_name, current_pos);
    start_pos = current_pos;
    Eigen::Vector3d dist_red = current_pos - *red_balloon_position_;
    Eigen::Vector3d dist_blue = current_pos - *blue_balloon_position_;

    // Chooses an initial target depending on which balloon is closer
    if (dist_red.norm() < dist_blue.norm()) {
      target = 0;
    } else {
      target = 1;
    }
    first_time = false;
  }

  Eigen::Vector3d target_pos;
  switch (target) {
    case 0:
      if (!red_balloon_popped && !blue_balloon_popped) {
        target_pos = red_balloon_pos;
        dt_factor = 1.5;
      } else if (red_balloon_popped && !blue_balloon_popped) {
        target_pos = blue_balloon_pos;
        dt_factor = 1.5;
      } else {
        target_pos = start_pos;
        dt_factor = 1.5;
      }
      break;
    case 1:
      if (!red_balloon_popped && !blue_balloon_popped) {
        target_pos = blue_balloon_pos;
        dt_factor = 1.5;

      } else if (!red_balloon_popped && blue_balloon_popped) {
        target_pos = red_balloon_pos;
        dt_factor = 1.5;
      } else {
        target_pos = start_pos;
        dt_factor = 1.5;
      }
      break;
  }

  std::cout << "target_pos: " << target_pos << std::endl;
  // std::shared_ptr<Node3D> com_cur_pos =
  //     ptr_Phy2Com(current_pos, occupancy_grid);
  // not sure why this function gives errors but work well outside
  // function start here~~~~~~~~
  std::tuple<int, int, int> pos_ind1 =
      occupancy_grid.mapToGridCoordinates(current_pos);
  // std::shared_ptr<Node3D> require an Eigen::Vector3d, but
  // mapToGridCoordinates produces int
  Eigen::Vector3d com_pos1 = {std::get<0>(pos_ind1), std::get<1>(pos_ind1),
                              std::get<2>(pos_ind1)};
  std::shared_ptr<Node3D> pos_ptr1 = std::make_shared<Node3D>(com_pos1);
  // end here~~~~~~~~~~~~~~~~~

  // function start here~~~~~~~~
  std::tuple<int, int, int> pos_ind2 =
      occupancy_grid.mapToGridCoordinates(target_pos);
  // std::shared_ptr<Node3D> require an Eigen::Vector3d, but
  // mapToGridCoordinates produces int
  Eigen::Vector3d com_pos2 = {std::get<0>(pos_ind2), std::get<1>(pos_ind2),
                              std::get<2>(pos_ind2)};
  std::shared_ptr<Node3D> pos_ptr2 = std::make_shared<Node3D>(com_pos2);
  // end here~~~~~~~~~~~~~~~~~

  PathInfo path_info = runAStar(graph_of_arena, pos_ptr1, pos_ptr2);
  // std::cout << path_info.details.path_length << std::endl;

  // Eigen::MatrixXd xyz_Phy = ptr_Com2Phy(path_info, occupancy_grid);
  // convert from computational pos to physical pos
  // function starts here~~~~~~~~~~~~~~~~~~~
  Eigen::MatrixXd xyz_Phy(3, path_info.details.path_length);
  for (int i = 0; i < path_info.details.path_length; i++) {
    int x_ind = lround(path_info.path[i]->Data().x());
    int y_ind = lround(path_info.path[i]->Data().y());
    int z_ind = lround(path_info.path[i]->Data().z());

    xyz_Phy.col(i) = occupancy_grid.boxCenter(x_ind, y_ind, z_ind);
  }
  // end here~~~~~~~~~~~~~~~~~~~~~~
  TrajectoryVector3D trajectory_vector = generateTrajectoryVector(
      2, dt_factor, current_chrono_time, xyz_Phy, current_vel);

  Trajectory trajectory(trajectory_vector);
  std::unordered_map<std::string, Trajectory> empty_quad_to_trajectory_map;
  TrajectoryCode prevetter_response =
      prevetter_->PreVet(quad_name, trajectory, map3d_);

  switch (prevetter_response.code) {
    case MediationLayerCode::Success:
      // Invoke the visualizer to see the proposed trajectory, which will be
      // displayed in violet. See student_game_engine_visualizer.h for other
      // visualization options: you can visualize a short path, a single point,
      // etc. It will be helpful to get such visual feedback on candidate
      // trajectories. Note that there is a built-in visualizer called
      // "ViewManager" implemented elsewhere in the game-engine code, but you
      // don't have full control over what it displays like you do with the
      // Student_game_engine_visualizer invoked below.
      visualizer.drawTrajectory(trajectory);
      quad_to_trajectory_map[quad_name] = trajectory;
      dt_factor = dt_factor - 0.06;
      std::cout << "speed up! :" << dt_factor << std::endl;
      return quad_to_trajectory_map;
      break;
    case MediationLayerCode::NotEnoughTrajectoryPoints: {
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::StartPointFarFromCurrentPosition: {
      dt_factor = 1.5;
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      break;
    }
    case MediationLayerCode::PointExceedsMapBounds: {
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::PointWithinObstacle: {
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::ExceedsMaxVelocity: {
      std::cout << "too fast! :" << dt_factor << std::endl;
      dt_factor = dt_factor + 0.05;
      // std::cout << "slow down! :" << seg_part[seg] << std::endl;
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::MeanValueExceedsMaxVelocity: {
      std::cout << "too fast! :" << dt_factor << std::endl;
      dt_factor = dt_factor + 0.05;
      //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::ExceedsMaxAcceleration: {
      std::cout << "too fast! :" << dt_factor << std::endl;
      dt_factor = dt_factor + 0.05;
      //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::MeanValueExceedsMaxAcceleration: {
      std::cout << "too fast! :" << dt_factor << std::endl;
      dt_factor = dt_factor + 0.05;
      //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::TimestampsNotIncreasing: {
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
    case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP intends to submit a trajectory with a time that exceeds
      // the maximum allowed time between points. The prevetter would catch this
      // before you submit to the mediation layer.
      std::cout << "Prevet: Shorten time between trajectory points."
                << std::endl;
      std::cout << "Prevet: Time violation: " << prevetter_response.value
                << std::endl;
      return empty_quad_to_trajectory_map;
      break;
    }
  }
}
}  // namespace game_engine
