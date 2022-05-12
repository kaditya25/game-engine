// #include "student_autonomy_protocol.h"

// #include <chrono>
// #include <cstdlib>
// #include <fstream>
// #include <iostream>
// #include <vector>

// // #include "a_star3d.h"
// #include <stdio.h>

// #include <algorithm>
// #include <memory>
// #include <queue>

// #include "graph.h"
// #include "occupancy_grid3d.h"
// #include "path_info.h"
// #include "polynomial_sampler.h"
// #include "polynomial_solver.h"
// #include "student_game_engine_visualizer.h"
// #include "timer.h"

// // The length of one side of a cube in meters in the occupancy grid
// constexpr double DISCRETE_LENGTH = 0.2;
// // How big the "inflation" bubble around obstacles will be, in meters
// constexpr double SAFETY_BOUNDS = 0.34;

// namespace game_engine {

// const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
//                                        Eigen::DontAlignCols, ", ", "\n");
// // writing functions taking Eigen types as parameters,
// // see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
// template <typename Derived>
// void writeToCSVfile(std::string name,
//                     const Eigen::MatrixBase<Derived>& matrix) {
//   std::ofstream file(name.c_str());
//   file << matrix.format(CSVFormat);
//   // file.close() is not necessary,
//   // desctructur closes file, see
//   // https://en.cppreference.com/w/cpp/io/basic_ofstream
// }

// std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(15);

// std::unordered_map<std::string, Trajectory>
// StudentAutonomyProtocol::UpdateTrajectories() {
//   static int kkk = 0;
//   // STUDENTS: FILL OUT THIS FUNCTION
//   // Set the duration of the example trajectory
//   constexpr int duration_sec = 80;
//   const std::chrono::milliseconds T_chrono =
//       std::chrono::seconds(duration_sec);  // seconds() takes only int!!!!!

//   // 'static' variables act like Matlab persistent variables, maintaining their
//   // value between function calls. Their initializer is only called once, on the
//   // first pass through the code.  If you prefer not to include static
//   // variables, you could instead make these data members of your
//   // StudentAutonomyProtocol class, which is a more standard C++ code design
//   // pattern.
//   static OccupancyGrid3D occupancy_grid;
//   static Graph3D graph_of_arena;
//   // Student_game_engine_visualizer is a class that supports visualizing paths,
//   // curves, points, and whole trajectories in the RVIZ display of the arena to
//   // aid in your algorithm development.
//   static Student_game_engine_visualizer visualizer;
//   static bool first_time = true;
//   static bool halt = false;
//   static Eigen::Vector3d start_pos;
//   static const std::chrono::time_point<std::chrono::system_clock>
//       start_chrono_time = std::chrono::system_clock::now();
//   static const std::chrono::time_point<std::chrono::system_clock>
//       end_chrono_time = start_chrono_time + T_chrono;
//   static std::vector<double> seg_part;
//   static int seg;
//   static int check1;
//   static int check2;
//   static int check22;
//   static int check3;
//   static int check33;
//   // Load the current quad position
//   const std::string& quad_name = friendly_names_[0];
//   Eigen::Vector3d current_pos;
//   Eigen::Vector3d current_vel;
//   snapshot_->Position(quad_name, current_pos);
//   snapshot_->Velocity(quad_name, current_vel);

//   // Set some static variables the first time this function is called
//   if (first_time) {
//     first_time = false;
//     occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
//     // You can run A* on graph_of_arena once you created a 3D version of A*
//     graph_of_arena = occupancy_grid.AsGraph();
//     visualizer.startVisualizing("/game_engine/environment");
//     start_pos = current_pos;
//     seg_part = {0.25, 0.5, 0.8};
//     seg = 0;
//     check1 = 0;
//     check2 = 0;
//     check22 = 0;
//     check3 = 0;
//     check33 = 0;
//   }

//   // Obtain current balloon positions and popped states
//   const Eigen::Vector3d red_balloon_pos = *red_balloon_position_;
//   const Eigen::Vector3d blue_balloon_pos = *blue_balloon_position_;
//   const bool red_balloon_popped = red_balloon_status_->popped;
//   const bool blue_balloon_popped = blue_balloon_status_->popped;

//   // Condition actions or parameters on wind intensity
//   switch (wind_intensity_) {
//     case WindIntensity::Zero:
//       // Do something zero-ish
//       break;
//     case WindIntensity::Mild:
//       // Do something mild
//       break;
//     case WindIntensity::Stiff:
//       // Do something stiff
//       break;
//     case WindIntensity::Intense:
//       // Do something intense
//       break;
//     case WindIntensity::Ludicrous:
//       // Do something ludicrous
//       break;
//     default:
//       std::cerr << "Unrecognized WindIntensity value." << std::endl;
//       std::exit(EXIT_FAILURE);
//   }

//   switch (trajectoryCodeMap_[quad_name].code) {
//     case MediationLayerCode::Success:
//       // You probably won't need to do anything in response to Success.
//       break;
//     case MediationLayerCode::NotEnoughTrajectoryPoints: {
//       break;
//     }
//     case MediationLayerCode::StartPointFarFromCurrentPosition: {
//       break;
//     }
//     case MediationLayerCode::PointExceedsMapBounds: {
//       break;
//     }
//     case MediationLayerCode::PointWithinObstacle: {
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxVelocity: {
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxVelocity: {
//       // seg_part[seg] = seg_part[seg] - 0.05;
//       // std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxAcceleration: {
//       // seg_part[seg] = seg_part[seg] - 0.05;
//       // std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxAcceleration: {
//       // seg_part[seg] = seg_part[seg] - 0.05;
//       // std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       break;
//     }
//     case MediationLayerCode::TimestampsNotIncreasing: {
//       break;
//     }
//     case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
//       break;
//     }
//     default:
//       // If you want to see a numerical MediationLayerCode value, you can cast
//       // and print the code as shown below.
//       std::cout << "MediationLayerCode: "
//                 << static_cast<int>(trajectoryCodeMap_[quad_name].code)
//                 << std::endl;
//       std::cout << "Value: " << trajectoryCodeMap_[quad_name].value
//                 << std::endl;
//       std::cout << "Index: " << trajectoryCodeMap_[quad_name].index
//                 << std::endl;
//   }

//   const std::chrono::time_point<std::chrono::system_clock> current_chrono_time =
//       std::chrono::system_clock::now();
//   const std::chrono::duration<double> normalized_current_chrono_time =
//       current_chrono_time - start_chrono_time;

//   // declare the start_ptr and end_ptr for path planning
//   std::shared_ptr<game_engine::Node3D> current_ptr =
//       std::make_shared<Node3D>(current_pos);
//   std::shared_ptr<game_engine::Node3D> end_ptr;
//   // decide the order of balloon (later)

//   // ongoing segement
//   if (!red_balloon_popped && !blue_balloon_popped) {
//     end_ptr = std::make_shared<Node3D>(blue_balloon_pos);
//     seg = 0;

//   } else if (!red_balloon_popped && blue_balloon_popped) {
//     end_ptr = std::make_shared<Node3D>(red_balloon_pos);
//     seg = 1;

//   } else {
//     end_ptr = std::make_shared<Node3D>(start_pos);
//     seg = 2;
//   }

//   // Create an empty quad-to-trajectory map.  This map object associates a quad
//   // name (expressed as a std::string) with the corresponding Trajectory object.
//   std::unordered_map<std::string, Trajectory> quad_to_trajectory_map;
//   // TrajectoryVector3D is an std::vector object defined in trajectory.h
//   TrajectoryVector3D trajectory_vector;
//   // Halt at goal position when close enough
//   constexpr double goal_arrival_threshold_meters = 0.3;
//   const Eigen::Vector3d dv = current_pos - goal_position_;

//   if ((halt || dv.norm() < goal_arrival_threshold_meters) && (seg == 2)) {
//     halt = true;
//     for (size_t idx = 0; idx < 20; ++idx) {
//       const std::chrono::duration<double> flight_chrono_time =
//           current_chrono_time.time_since_epoch() + idx * dt_chrono;
//       const double flight_time = flight_chrono_time.count();
//       trajectory_vector.push_back(
//           (Eigen::Matrix<double, 11, 1>() << goal_position_.x(),
//            goal_position_.y(), goal_position_.z(), 0, 0, 0, 0, 0, 0, 0,
//            flight_time)
//               .finished());
//     }
//     Trajectory trajectory(trajectory_vector);
//     visualizer.drawTrajectory(trajectory);
//     quad_to_trajectory_map[quad_name] = trajectory;
//     return quad_to_trajectory_map;
//   }

//   // compute the remaining time for current goal
//   // convert back to seconds(double) from chrono::duration<double>
//   // T_chrono in ms, remaining_chrono_time in s
//   double remaining_seg0_time = (seg_part[0] * T_chrono.count() / 1000 -
//                                 normalized_current_chrono_time.count());
//   double remaining_seg1_time = (seg_part[1] * T_chrono.count() / 1000 -
//                                 normalized_current_chrono_time.count());
//   double remaining_seg2_time = (seg_part[2] * T_chrono.count() / 1000 -
//                                 normalized_current_chrono_time.count());

//   // in case time is not enough due to whatever wind
//   while (remaining_seg0_time < 0 && seg == 0) {
//     seg_part[0] = seg_part[0] + 0.05;
//     seg_part[1] = seg_part[1] + 0.01;
//     seg_part[2] = seg_part[2] + 0.01;
//     remaining_seg0_time = (seg_part[0] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//     remaining_seg1_time = (seg_part[1] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//     remaining_seg2_time = (seg_part[2] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//   }
//   while (remaining_seg1_time < 0 && (seg == 0 || seg == 1)) {
//     seg_part[1] = seg_part[1] + 0.05;
//     seg_part[2] = seg_part[2] + 0.01;
//     remaining_seg1_time = (seg_part[1] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//     remaining_seg2_time = (seg_part[2] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//   }
//   while (remaining_seg2_time < 0 && (seg == 2 || seg == 1)) {
//     seg_part[2] = seg_part[2] + 0.05;
//     remaining_seg2_time = (seg_part[2] * T_chrono.count() / 1000 -
//                            normalized_current_chrono_time.count());
//   }

//   // use P4 to construct a polynomial and later fit into the
//   // quad_to_trajectory_map if we can put it into a function (later)
//   std::vector<double> times = {0};  // improve this more later
//   // The parameter order for p4::NodeEqualityBound is:
//   // (dimension_index, node_idx, derivative_idx, value)
//   std::vector<p4::NodeEqualityBound> node_equality_bounds = {
//       p4::NodeEqualityBound(0, 0, 0, current_pos(0)),
//       p4::NodeEqualityBound(1, 0, 0, current_pos(1)),
//       p4::NodeEqualityBound(2, 0, 0, current_pos(2)),
//       p4::NodeEqualityBound(0, 0, 1, current_vel(0)),
//       p4::NodeEqualityBound(1, 0, 1, current_vel(1)),
//       p4::NodeEqualityBound(2, 0, 1, current_vel(2)),
//   };
//   std::vector<p4::NodeInequalityBound> node_Inequality_bounds = {};

//   std::vector<p4::SegmentInequalityBound> segment_Inequality_bounds = {

//   };

//   Eigen::Vector3d point1 = {-15, 15.5, -3.8};
//   Eigen::Vector3d point2 = {-19, 17.4, -3.2};
//   Eigen::Vector3d point22 = {-23, 17.4, -3};
//   Eigen::Vector3d point3 = {-23, 17.5, -2.8};
//   Eigen::Vector3d point33 = {-16, 15.7, -3.7};
//   Eigen::Vector3d dv1 = current_pos - point1;
//   Eigen::Vector3d dv2 = current_pos - point2;
//   Eigen::Vector3d dv22 = current_pos - point22;
//   Eigen::Vector3d dv3 = current_pos - point3;
//   Eigen::Vector3d dv33 = current_pos - point33;
//   if (dv1.norm() < 1 && seg == 0) {
//     check1 = 1;
//   } else if (dv2.norm() < 1 && seg == 1) {
//     check2 = 1;
//   } else if (dv22.norm() < 1 && seg == 1) {
//     check22 = 1;
//   } else if (dv3.norm() < 1.5 && seg == 2) {
//     check3 = 1;
//   } else if (dv33.norm() < 2.5 && seg == 2) {
//     check33 = 1;
//   }
  
//   if (seg == 0) {
//     // The second node constrains position
//     if (check1 == 0) {
//       node_equality_bounds.push_back(p4::NodeEqualityBound(0, 1, 0, point1(0)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(1, 1, 0, point1(1)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(2, 1, 0, point1(2)));
//       times.push_back(remaining_seg0_time / 2.0);
//     }

//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(0, 2 - check1, 0, blue_balloon_pos(0)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(1, 2 - check1, 0, blue_balloon_pos(1)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(2, 2 - check1, 0, blue_balloon_pos(2)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(2, 2 - check1, 1, -0.1));

//     times.push_back(remaining_seg0_time);
//   };

//   if (seg == 1) {
//     // The third node constrains position
//     if (check2 == 0) {
//       node_equality_bounds.push_back(p4::NodeEqualityBound(0, 1, 0, point2(0)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(1, 1, 0, point2(1)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(2, 1, 0, point2(2)));
//       times.push_back(remaining_seg1_time / 3.0);
//     }
//     if (check22 == 0) {
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(0, 2 - check2, 0, point22(0)));
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(1, 2 - check2, 0, point22(1)));
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(2, 2 - check2, 0, point22(2)));
//       times.push_back((2 - check2) * remaining_seg1_time / (3 - check2));
//     }
//     node_equality_bounds.push_back(p4::NodeEqualityBound(
//         0, 4 - seg - check2 - check22, 0, red_balloon_pos(0)));
//     node_equality_bounds.push_back(p4::NodeEqualityBound(
//         1, 4 - seg - check2 - check22, 0, red_balloon_pos(1)));
//     node_equality_bounds.push_back(p4::NodeEqualityBound(
//         2, 4 - seg - check2 - check22, 0, red_balloon_pos(2)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(0, 4 - seg - check2 - check22, 1, 0));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(1, 4 - seg - check2 - check22, 1, 0));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(2, 4 - seg - check2 - check22, 1, 0));
//     times.push_back(remaining_seg1_time);
//   }
//   if (seg == 2) {
//     // The fourth node constrains position
//     if (check3 == 0) {
//       node_equality_bounds.push_back(p4::NodeEqualityBound(0, 1, 0, point3(0)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(1, 1, 0, point3(1)));
//       node_equality_bounds.push_back(p4::NodeEqualityBound(2, 1, 0, point3(2)));
//       times.push_back(remaining_seg2_time / 6);
//     }
//     if (check33 == 0) {
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(0, 2 - check3, 0, point33(0)));
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(1, 2 - check3, 0, point33(1)));
//       node_equality_bounds.push_back(
//           p4::NodeEqualityBound(2, 2 - check3, 0, point33(2)));
//       times.push_back(remaining_seg2_time / 2);
//     }
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(0, 3 - check3 - check33, 0, goal_position_(0)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(1, 3 - check3 - check33, 0, goal_position_(1)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(2, 3 - check3 - check33, 0, goal_position_(2)));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(0, 3 - check3 - check33, 1, 0));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(1, 3 - check3 - check33, 1, 0));
//     node_equality_bounds.push_back(
//         p4::NodeEqualityBound(2, 3 - check3 - check33, 1, 0));

//     times.push_back(remaining_seg2_time);
//   }

//   // Options to configure the polynomial solver with
//   p4::PolynomialSolver::Options solver_options;
//   solver_options.num_dimensions = 3;    // 3D
//   solver_options.polynomial_order = 8;  // Fit an 8th-order polynomial
//   solver_options.continuity_order = 4;  // Require continuity to the 4th order
//   solver_options.derivative_order = 3;  // Minimize something (2 for accel)

//   osqp_set_default_settings(&solver_options.osqp_settings);
//   solver_options.osqp_settings.polish =
//       true;  // Polish the solution, getting the best answer possible
//   solver_options.osqp_settings.verbose = false;  // Suppress the printout

//   // Use p4::PolynomialSolver object to solve for polynomial trajectories
//   p4::PolynomialSolver solver(solver_options);
//   const p4::PolynomialSolver::Solution path =
//       solver.Run(times, node_equality_bounds, node_Inequality_bounds,
//                  segment_Inequality_bounds);

//   p4::PolynomialSampler::Options sampler1_options;
//   sampler1_options.frequency = 200;       // Number of samples per second
//   sampler1_options.derivative_order = 0;  // Derivative to sample (0 = pos)
//   // Use this object to sample a trajectory
//   p4::PolynomialSampler sampler1(sampler1_options);
//   Eigen::MatrixXd ppp = sampler1.Run(times, path);

//   p4::PolynomialSampler::Options sampler2_options;
//   sampler2_options.frequency = 200;
//   sampler2_options.derivative_order = 1;
//   p4::PolynomialSampler sampler2(sampler2_options);
//   Eigen::MatrixXd vvv = sampler2.Run(times, path);

//   p4::PolynomialSampler::Options sampler3_options;
//   sampler3_options.frequency = 200;
//   sampler3_options.derivative_order = 2;
//   p4::PolynomialSampler sampler3(sampler3_options);
//   Eigen::MatrixXd aaa = sampler3.Run(times, path);

//   // if (kkk < 1) {
//   //   kkk++;
//   //   writeToCSVfile("pos.csv", ppp);
//   //   writeToCSVfile("vel.csv", vvv);
//   //   writeToCSVfile("acc.csv", aaa);
//   // }

//   for (int t_idx = 0; t_idx < ppp.cols(); t_idx++) {
//     const std::chrono::duration<double> flight_chrono_time =
//         current_chrono_time.time_since_epoch() +
//         std::chrono::duration<double>(ppp(0, t_idx));
//     const double flight_time = flight_chrono_time.count();
//     const double x = ppp(1, t_idx);
//     const double y = ppp(2, t_idx);
//     const double z = ppp(3, t_idx);
//     const double vx = vvv(1, t_idx);
//     const double vy = vvv(2, t_idx);
//     const double vz = vvv(3, t_idx);
//     const double ax = aaa(1, t_idx);
//     const double ay = aaa(2, t_idx);
//     const double az = aaa(3, t_idx);
//     const double yaw = 0;
//     // std::cout << flight_chrono_time.count() << " time" << std::endl;
//     trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() << x, y, z, vx,
//                                  vy, vz, ax, ay, az, yaw, flight_time)
//                                     .finished());
//   }

//   Trajectory trajectory(trajectory_vector);
//   std::unordered_map<std::string, Trajectory> empty_quad_to_trajectory_map;

//   // Before submitting the trajectory, you can use this prevetting interface to
//   // determine if the trajectory violates any mediation layer constraints. This
//   // interface can be found in presubmission_trajectory_vetter.h. The PreVet
//   // function returns type TrajectoryCode which contains three values: (1) The
//   // mediation layer code that specifies success or the failure; (2) The failure
//   // value (e.g., if the velocity limit is 4.0 m/s and you submit 5.0 m/s, the
//   // value is returned as 5.0); (3) The failure index. This is the trajectory
//   // sample index that caused the mediation layer to kick back an error code.
//   // The index is the sampled index (specified from the P4 sampling process).
//   // You can figure out the waypoint index by a simple math transformation.

//   // if any error is found, send an empty map or fix it
//   TrajectoryCode prevetter_response =
//       prevetter_->PreVet(quad_name, trajectory, map3d_);
//   switch (prevetter_response.code) {
//     case MediationLayerCode::Success:
//       // Invoke the visualizer to see the proposed trajectory, which will be
//       // displayed in violet. See student_game_engine_visualizer.h for other
//       // visualization options: you can visualize a short path, a single point,
//       // etc. It will be helpful to get such visual feedback on candidate
//       // trajectories. Note that there is a built-in visualizer called
//       // "ViewManager" implemented elsewhere in the game-engine code, but you
//       // don't have full control over what it displays like you do with the
//       // Student_game_engine_visualizer invoked below.
//       visualizer.drawTrajectory(trajectory);
//       quad_to_trajectory_map[quad_name] = trajectory;
//       seg_part[seg] = seg_part[seg] - 0.1;
//       std::cout << "speed up! :" << seg_part[seg] << std::endl;
//       return quad_to_trajectory_map;
//       break;
//     case MediationLayerCode::NotEnoughTrajectoryPoints: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::StartPointFarFromCurrentPosition: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       break;
//     }
//     case MediationLayerCode::PointExceedsMapBounds: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::PointWithinObstacle: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxVelocity: {
//       std::cout << "too fast! :" << seg_part[seg] << std::endl;
//       seg_part[seg] = seg_part[seg] + 0.05;
//       // std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxVelocity: {
//       std::cout << "too fast! :" << seg_part[seg] << std::endl;
//       seg_part[seg] = seg_part[seg] + 0.05;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxAcceleration: {
//       std::cout << "too fast! :" << seg_part[seg] << std::endl;
//       seg_part[seg] = seg_part[seg] + 0.05;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxAcceleration: {
//       std::cout << "too fast! :" << seg_part[seg] << std::endl;
//       seg_part[seg] = seg_part[seg] + 0.05;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::TimestampsNotIncreasing: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
//       // Suppose your AP intends to submit a trajectory with a time that exceeds
//       // the maximum allowed time between points. The prevetter would catch this
//       // before you submit to the mediation layer.
//       std::cout << "Prevet: Shorten time between trajectory points."
//                 << std::endl;
//       std::cout << "Prevet: Time violation: " << prevetter_response.value
//                 << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//   }
// }
// }  // namespace game_engine
