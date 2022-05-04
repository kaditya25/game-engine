// #include <cstdlib>
// #include <vector>

// #include "a_star3d.h"
// // #include "gnuplot-iostream.h"
// #include "occupancy_grid3d.h"
// #include "path_info.h"
// #include "polynomial_sampler.h"
// #include "polynomial_solver.h"
// // #include "gui2d.h"
// #include <fstream>
// #include <iostream>
// #include "RunAstarPlusNumWaypointPolySolver.h"

// using namespace game_engine;

// game_engine::TrajectoryVector3D RunAstarPlusNumWaypointPolySolver(
//     const Graph3D &graph, const std::shared_ptr<Node3D> start_ptr,
//     const std::shared_ptr<Node3D> end_ptr, int derivative_order, int seg) {
//   // A star algo
//   game_engine::AStar3D a_star;
//   game_engine::PathInfo path_info = a_star.Run(graph, *start_ptr, *end_ptr);

//   // Time in seconds
//   std::vector<double> times = {};  // aaaaaa
//   // The parameter order for p4::NodeEqualityBound is:
//   // (dimension_index, node_idx, derivative_idx, value)
//   switch (seg) {
//     case seg == 0:
//       std::vector<p4::NodeEqualityBound> node_equality_bounds = {
//           p4::NodeEqualityBound(0, 0, 1, 0), p4::NodeEqualityBound(1, 0, 1, 0),
//           p4::NodeEqualityBound(2, 0, 1, 0), p4::NodeEqualityBound(0, 0, 2, 0),
//           p4::NodeEqualityBound(1, 0, 2, 0), p4::NodeEqualityBound(2, 0, 2, 0),
//       };

//       for (int i = 0; i < path_info.path.size(); i++) {
//         times.push_back(i);
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
//       };

//     case seg == 1:

//       for (int i = 0; i < path_info.path.size(); i++) {
//         times.push_back(i);
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
//       };

//     case seg == 2:

//       for (int i = 0; i < path_info.path.size(); i++) {
//         times.push_back(i);
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
//         node_equality_bounds.push_back(
//             p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
//       };

//       std::vector<p4::NodeEqualityBound> node_equality_bounds = {
//           p4::NodeEqualityBound(0, path_info.size() - 1, 1, 0),
//           p4::NodeEqualityBound(1, path_info.size() - 1, 1, 0),
//           p4::NodeEqualityBound(2, path_info.size() - 1, 1, 0),
//           p4::NodeEqualityBound(0, path_info.size() - 1, 2, 0),
//           p4::NodeEqualityBound(1, path_info.size() - 1, 2, 0),
//           p4::NodeEqualityBound(2, path_info.size() - 1, 2, 0),
//       };
//   }

//   // Options to configure the polynomial solver with
//   p4::PolynomialSolver::Options solver_options;
//   solver_options.num_dimensions = 3;    // 3D
//   solver_options.polynomial_order = 8;  // Fit an 8th-order polynomial
//   solver_options.continuity_order = 4;  // Require continuity to the 4th order
//   solver_options.derivative_order = derivative_order;  // Minimize something (2 for accel)

//   osqp_set_default_settings(&solver_options.osqp_settings);
//   solver_options.osqp_settings.polish =
//       true;  // Polish the solution, getting the best answer possible
//   solver_options.osqp_settings.verbose = false;  // Suppress the printout

//   // Use p4::PolynomialSolver object to solve for polynomial trajectories
//   p4::PolynomialSolver solver(solver_options);
//   const p4::PolynomialSolver::Solution path =
//       solver.Run(times, node_equality_bounds, {}, {});

//   game_engine::TrajectoryVector3D trajectory_vector;
//   std::vector<eigen::MatrixXd> trajectory_state;

//   for (int o = 0; o < 3; o++) {
//     // Options to configure the polynomial sampler with
//     p4::PolynomialSampler::Options sampler_options;
//     sampler_options.frequency = 200;       // Number of samples per second
//     sampler_options.derivative_order = o;  // Derivative to sample (0 = pos)

//     // Use this object to sample a trajectory
//     p4::PolynomialSampler sampler(sampler_options);
//     Eigen::MatrixXd samples = sampler.Run(times, path);

//     if (o == 0) {
//       trajectory_state.push_back(samples.row(0));
//     }
//     trajectory_state.push_back(samples.row(1));
//     trajectory_state.push_back(samples.row(2));
//     trajectory_state.push_back(samples.row(3));
//   }

//   for (int idx = 0; idx < trajectory_state[0].size() + 1; idx++) {

//     // using TrajectoryVector3D = std::vector<
//     //   Eigen::Matrix<double, 11, 1>,
//     //   Eigen::aligned_allocator<Eigen::Matrix<double, 11, 1>>>;
//     double flight_time = trajectory_state[0][idx];
//     double x = trajectory_state[1][idx];
//     double y = trajectory_state[2][idx];
//     double z = trajectory_state[3][idx];
//     double vx = trajectory_state[4][idx];
//     double vy = trajectory_state[5][idx];
//     double vz = trajectory_state[6][idx];
//     double ax = trajectory_state[7][idx];
//     double ay = trajectory_state[8][idx];
//     double az = trajectory_state[9][idx];
//     double yaw = 0;

//     trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() << x, y, z, vx,
//                                  vy, vz, ax, ay, az, yaw, flight_time)
//                                     .finished());
//   }

//   return trajectory_vector;
// }
