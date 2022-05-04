#include "student_autonomy_protocol.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

// #include "a_star3d.h"
#include "graph.h"
#include "occupancy_grid3d.h"
#include "path_info.h"
#include "polynomial_sampler.h"
#include "polynomial_solver.h"
#include "student_game_engine_visualizer.h"

#include <queue>
#include <algorithm>
#include <stdio.h>

#include <memory>
// #include <vector>
// #include <chrono>
// #include <iostream>

// #include "graph.h"
#include "timer.h"
// #include "path_info.h"

// The length of one side of a cube in meters in the occupancy grid
constexpr double DISCRETE_LENGTH = 0.2;
// How big the "inflation" bubble around obstacles will be, in meters
constexpr double SAFETY_BOUNDS = 0.34;

namespace game_engine {
  struct AStar3D {
    // Students will implement this function
    PathInfo Run(const Graph3D& graph, 
                 const std::shared_ptr<Node3D> start_ptr, 
                 const std::shared_ptr<Node3D> end_ptr);
  };
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node3D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    ///////////////////////////////////////////////////////////////////
    // EXAMPLE HEURISTIC FUNCTION
    // YOU WILL NEED TO MODIFY THIS OR WRITE YOUR OWN FUNCTION
    ///////////////////////////////////////////////////////////////////
    double Heuristic(
        const std::shared_ptr<Node3D>& current_ptr,
        const std::shared_ptr<Node3D>& end_ptr) {
          return std::sqrt(std::pow(current_ptr->Data().x()-end_ptr->Data().x(),2) +
          std::pow(current_ptr->Data().y()-end_ptr->Data().y(),2) + 
          std::pow(current_ptr->Data().z()-end_ptr->Data().z(),2));
          
      // return 0; //zero heuristic
      // return (current_ptr->Data().x() - end_ptr->Data().x()) + (current_ptr->Data().y() - end_ptr->Data().y()); //Taking the Manhattan distane (Overestimating the cost)  
      // return sqrt((current_ptr->Data() - end_ptr->Data()).transpose() * ( current_ptr->Data() - end_ptr->Data())); //Taking the l2 norm error (Underestimating the cost)
    }

  }

  using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
  bool subset(const std::vector<NodeWrapperPtr> &explored_nodes, const NodeWrapperPtr current_node)
  {
    for(const std::shared_ptr<NodeWrapper> explored_node: explored_nodes) 
    {
      if ( *(explored_node) == *(current_node) )
      {
        return true;
      }
    }
    return false;
  }

  PathInfo AStar3D::Run(
      const Graph3D& graph, 
      const std::shared_ptr<Node3D> start_ptr, 
      const std::shared_ptr<Node3D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::priority_queue<
      NodeWrapperPtr,
      std::vector<NodeWrapperPtr>,
      std::function<bool(
          const NodeWrapperPtr&, 
          const NodeWrapperPtr& )>> 
        to_explore(NodeWrapperPtrCompare);

    std::vector<NodeWrapperPtr> explored;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    // SOME EXAMPLE CODE INCLUDED BELOW
    ///////////////////////////////////////////////////////////////////

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    nw_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(nw_ptr);

    NodeWrapperPtr node_to_explore;
    while (!to_explore.empty())
    {
      node_to_explore = to_explore.top();
      to_explore.pop();

      if (subset(explored,node_to_explore))
      {
        continue;
      }
      if (*(node_to_explore->node_ptr) == *end_ptr)
      {
        break;
      }
      else
      {
        explored.push_back(node_to_explore);

        // Iterate through the list of edges and add the neighbors
        for(const DirectedEdge3D& edge: graph.Edges(node_to_explore->node_ptr)) 
        {
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->cost = node_to_explore->cost + edge.Cost();
          nw_ptr->heuristic = Heuristic(edge.Sink(), end_ptr);
          nw_ptr->parent = node_to_explore;
          nw_ptr->node_ptr = edge.Sink();          
          to_explore.push(nw_ptr);
        }
      }
    }

    NodeWrapperPtr node = node_to_explore;

    // Create a PathInfo
    PathInfo path_info;
    path_info.details.num_nodes_explored = explored.size()+1;
    path_info.details.path_cost = node->cost;

    path_info.details.path_length = 1;
    while( node->parent != nullptr )
    {
      path_info.path.push_back( node->node_ptr );
      node = node->parent; 
      path_info.details.path_length++;
    }
    path_info.path.push_back( node->node_ptr );
    std::reverse( path_info.path.begin(), path_info.path.end() );

    path_info.details.run_time = timer.Stop();
    
    return path_info;
  }


std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(1000);

std::unordered_map<std::string, Trajectory>
StudentAutonomyProtocol::UpdateTrajectories() {
  // STUDENTS: FILL OUT THIS FUNCTION
  // Set the duration of the example trajectory
  constexpr int duration_sec = 300;
  const std::chrono::milliseconds T_chrono = std::chrono::seconds(duration_sec);

  // 'static' variables act like Matlab persistent variables, maintaining their
  // value between function calls. Their initializer is only called once, on the
  // first pass through the code.  If you prefer not to include static
  // variables, you could instead make these data members of your
  // StudentAutonomyProtocol class, which is a more standard C++ code design
  // pattern.
  static OccupancyGrid3D occupancy_grid;
  static Graph3D graph_of_arena;
  // Student_game_engine_visualizer is a class that supports visualizing paths,
  // curves, points, and whole trajectories in the RVIZ display of the arena to
  // aid in your algorithm development.
  static Student_game_engine_visualizer visualizer;
  static bool first_time = true;
  static bool halt = false;
  static Eigen::Vector3d start_pos;
  static const std::chrono::time_point<std::chrono::system_clock>
      start_chrono_time = std::chrono::system_clock::now();
  static const std::chrono::time_point<std::chrono::system_clock>
      end_chrono_time = start_chrono_time + T_chrono;

  // Load the current quad position
  const std::string &quad_name = friendly_names_[0];
  Eigen::Vector3d current_pos;
  snapshot_->Position(quad_name, current_pos);

  // Set some static variables the first time this function is called
  if (first_time) {
    first_time = false;
    occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
    // You can run A* on graph_of_arena once you created a 3D version of A*
    graph_of_arena = occupancy_grid.AsGraph();
    visualizer.startVisualizing("/game_engine/environment");
    start_pos = current_pos;
  }

  // Obtain current balloon positions and popped states
  const Eigen::Vector3d red_balloon_pos = *red_balloon_position_;
  const Eigen::Vector3d blue_balloon_pos = *blue_balloon_position_;
  const bool red_balloon_popped = red_balloon_status_->popped;
  const bool blue_balloon_popped = blue_balloon_status_->popped;

  // Condition actions or parameters on wind intensity
  switch (wind_intensity_) {
    case WindIntensity::Zero:
      // Do something zero-ish
      break;
    case WindIntensity::Mild:
      // Do something mild
      break;
    case WindIntensity::Stiff:
      // Do something stiff
      break;
    case WindIntensity::Intense:
      // Do something intense
      break;
    case WindIntensity::Ludicrous:
      // Do something ludicrous
      break;
    default:
      std::cerr << "Unrecognized WindIntensity value." << std::endl;
      std::exit(EXIT_FAILURE);
  }

  switch (trajectoryCodeMap_[quad_name].code) {
    case MediationLayerCode::Success:
      // You probably won't need to do anything in response to Success.
      break;
    case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP initially submits a trajectory with a time that exceeds
      // the maximum allowed time between points. You could fix the problem as
      // shown below.
      std::cout << "Replanning trajectory: "
                   "Shortening time between trajectory points."
                << std::endl;
      std::cout << "Value: " << trajectoryCodeMap_[quad_name].value
                << std::endl;
      dt_chrono = dt_chrono - std::chrono::milliseconds(50);
      break;
    }
    default:
      // If you want to see a numerical MediationLayerCode value, you can cast
      // and print the code as shown below.
      std::cout << "MediationLayerCode: "
                << static_cast<int>(trajectoryCodeMap_[quad_name].code)
                << std::endl;
      std::cout << "Value: " << trajectoryCodeMap_[quad_name].value
                << std::endl;
      std::cout << "Index: " << trajectoryCodeMap_[quad_name].index
                << std::endl;
  }

  // Always use the chrono::system_clock for time. Trajectories require time
  // points measured in floating point seconds from the Unix epoch.
  const double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(dt_chrono)
          .count();
  const std::chrono::time_point<std::chrono::system_clock> current_chrono_time =
      std::chrono::system_clock::now();
  const std::chrono::duration<double> remaining_chrono_time =
      end_chrono_time - current_chrono_time;

  // The following code generates and returns a new trajectory each time it
  // runs.  The new trajectory starts at the location on the original circle
  // that is closest to the current location of the quad and it creates a
  // trajectory of N position, velocity, and acceleration (PVA) points spaced by
  // dt seconds.  Thus, the code below responds to the actual position of the
  // quad and adjusts the newly-generated trajectory accordingly.  But note that
  // its strategy is not optimal for covering the greatest distance in the
  // allotted time in the presence of disturbance accelerations.

  // Number of samples in new trajectory
  const size_t N = remaining_chrono_time / dt_chrono;

  // Create an empty quad-to-trajectory map.  This map object associates a quad
  // name (expressed as a std::string) with the corresponding Trajectory object.
  std::unordered_map<std::string, Trajectory> quad_to_trajectory_map;

  // If the end time has passed, or if there are too few samples for a valid
  // trajectory, return an empty quad_to_trajectory_map
  constexpr size_t min_required_samples_in_trajectory = 2;
  if (current_chrono_time >= end_chrono_time ||
      N < min_required_samples_in_trajectory) {
    return quad_to_trajectory_map;
  }

  // Halt at goal position when close enough
  constexpr double goal_arrival_threshold_meters = 0.3;
  const Eigen::Vector3d dv = current_pos - goal_position_;
  // TrajectoryVector3D is an std::vector object defined in trajectory.h
  TrajectoryVector3D trajectory_vector;

  if (halt ||
      (remaining_chrono_time < std::chrono::seconds(duration_sec - 10) &&
       dv.norm() < goal_arrival_threshold_meters)) {
    halt = true;
    for (size_t idx = 0; idx < 20; ++idx) {
      const std::chrono::duration<double> flight_chrono_time =
          current_chrono_time.time_since_epoch() + idx * dt_chrono;
      const double flight_time = flight_chrono_time.count();
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() << goal_position_.x(),
           goal_position_.y(), goal_position_.z(), 0, 0, 0, 0, 0, 0, 0,
           flight_time)
              .finished());
    }
    Trajectory trajectory(trajectory_vector);
    visualizer.drawTrajectory(trajectory);
    quad_to_trajectory_map[quad_name] = trajectory;
    return quad_to_trajectory_map;
  }

  std::shared_ptr<game_engine::Node3D> current_ptr = std::make_shared<game_engine::Node3D>(current_pos);
  // decide the order of balloon
  std::shared_ptr<game_engine::Node3D> end_ptr;

  // check if quad is now close to the balloon
  const Eigen::Vector3d dv_red_balloon = current_pos - red_balloon_pos;
  const Eigen::Vector3d dv_blue_balloon = current_pos - blue_balloon_pos;
  if (dv_red_balloon.norm() < goal_arrival_threshold_meters) {
    red_balloon_status_->popped = true;
  } else if (dv_blue_balloon.norm() < goal_arrival_threshold_meters) {
    blue_balloon_status_->popped = true;
  }
  // ongoing segement
  int seg;
  if (!red_balloon_popped && !blue_balloon_popped) {
    end_ptr = std::make_shared<Node3D>(blue_balloon_pos);
    seg = 0;

  } else if (!red_balloon_popped && blue_balloon_popped) {
    end_ptr = std::make_shared<Node3D>(red_balloon_pos);
    seg = 1;

  } else {
    end_ptr = std::make_shared<Node3D>(start_pos);
    seg = 2;
  }


  
  // algorithm provide a trajectory
  game_engine::AStar3D a_star3d;
  game_engine::PathInfo path_info = a_star3d.Run(graph_of_arena, current_ptr, end_ptr);

  std::vector<double> times = {};  // improve this more later
  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
      p4::NodeEqualityBound(0, 0, 1, 0), p4::NodeEqualityBound(1, 0, 1, 0),
      p4::NodeEqualityBound(2, 0, 1, 0), p4::NodeEqualityBound(0, 0, 2, 0),
      p4::NodeEqualityBound(1, 0, 2, 0), p4::NodeEqualityBound(2, 0, 2, 0),
  };

  for (int i = 0; i < path_info.path.size(); i++) {
    times.push_back(i * dt);
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
    node_equality_bounds.push_back(
        p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
  }

  // idk why this outside decleration of vector cause error; *******
  // switch (seg) {
  //   case 0:
  //     node_equality_bounds = {
  //         p4::NodeEqualityBound(0, 0, 1, 0), p4::NodeEqualityBound(1, 0, 1,
  //         0), p4::NodeEqualityBound(2, 0, 1, 0), p4::NodeEqualityBound(0, 0,
  //         2, 0), p4::NodeEqualityBound(1, 0, 2, 0), p4::NodeEqualityBound(2,
  //         0, 2, 0),
  //     };

  //     for (int i = 0; i < path_info.path.size(); i++) {
  //       times.push_back(i * dt);
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
  //     }
  //     break;
  //   case 1:
  //     for (int i = 0; i < path_info.path.size(); i++) {
  //       times.push_back(i * dt);
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
  //     }
  //     break;

  //   case 2:
  //     node_equality_bounds = {
  //         p4::NodeEqualityBound(0, path_info.path.size() - 1, 1, 0),
  //         p4::NodeEqualityBound(1, path_info.path.size() - 1, 1, 0),
  //         p4::NodeEqualityBound(2, path_info.path.size() - 1, 1, 0),
  //         p4::NodeEqualityBound(0, path_info.path.size() - 1, 2, 0),
  //         p4::NodeEqualityBound(1, path_info.path.size() - 1, 2, 0),
  //         p4::NodeEqualityBound(2, path_info.path.size() - 1, 2, 0),
  //     };
  //     for (int i = 0; i < path_info.path.size(); i++) {
  //       times.push_back(i * dt);
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(0, i, 0, path_info.path[i]->Data()[0]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(1, i, 0, path_info.path[i]->Data()[1]));
  //       node_equality_bounds.push_back(
  //           p4::NodeEqualityBound(2, i, 0, path_info.path[i]->Data()[2]));
  //     }
  //     break;
  // }

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 3;    // 3D
  solver_options.polynomial_order = 8;  // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;  // Require continuity to the 4th order
  solver_options.derivative_order = 2;  // Minimize something (2 for accel)

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish =
      true;  // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;  // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path =
      solver.Run(times, node_equality_bounds, {}, {});

  p4::PolynomialSampler::Options sampler1_options;
  sampler1_options.frequency = 200;       // Number of samples per second
  sampler1_options.derivative_order = 0;  // Derivative to sample (0 = pos)
  // Use this object to sample a trajectory
  p4::PolynomialSampler sampler1(sampler1_options);
  Eigen::MatrixXd ppp = sampler1.Run(times, path);

  p4::PolynomialSampler::Options sampler2_options;
  sampler2_options.frequency = 200;
  sampler2_options.derivative_order = 1;
  p4::PolynomialSampler sampler2(sampler2_options);
  Eigen::MatrixXd vvv = sampler2.Run(times, path);

  p4::PolynomialSampler::Options sampler3_options;
  sampler3_options.frequency = 200;
  sampler3_options.derivative_order = 2;
  p4::PolynomialSampler sampler3(sampler3_options);
  Eigen::MatrixXd aaa = sampler3.Run(times, path);

  for (int t_idx = 0; t_idx < ppp.cols() + 1; t_idx++) {
    const std::chrono::duration<double> flight_chrono_time =
        current_chrono_time.time_since_epoch() +
        std::chrono::duration<double>(ppp(0,t_idx));
    const double flight_time = flight_chrono_time.count();
    const double x = ppp(1,t_idx);
    const double y = ppp(2,t_idx);
    const double z = ppp(3,t_idx);
    const double vx = vvv(1,t_idx);
    const double vy = vvv(2,t_idx);
    const double vz = vvv(3,t_idx);
    const double ax = aaa(1,t_idx);
    const double ay = aaa(2,t_idx);
    const double az = aaa(3,t_idx);
    const double yaw = 0;

    trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() << x, y, z,
    vx, vy, vz, ax, ay, az, yaw, flight_time).finished());
  }

  Trajectory trajectory(trajectory_vector);

  // Before submitting the trajectory, you can use this prevetting interface to
  // determine if the trajectory violates any mediation layer constraints. This
  // interface can be found in presubmission_trajectory_vetter.h. The PreVet
  // function returns type TrajectoryCode which contains three values: (1) The
  // mediation layer code that specifies success or the failure; (2) The failure
  // value (e.g., if the velocity limit is 4.0 m/s and you submit 5.0 m/s, the
  // value is returned as 5.0); (3) The failure index. This is the trajectory
  // sample index that caused the mediation layer to kick back an error code.
  // The index is the sampled index (specified from the P4 sampling process).
  // You can figure out the waypoint index by a simple math transformation.
  TrajectoryCode prevetter_response =
      prevetter_->PreVet(quad_name, trajectory, map3d_);
  switch (prevetter_response.code) {
    case MediationLayerCode::Success:
      // You probably won't need to do anything in response to Success.
      break;
    case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP intends to submit a trajectory with a time that exceeds
      // the maximum allowed time between points. The prevetter would catch this
      // before you submit to the mediation layer.
      std::cout << "Prevet: Shorten time between trajectory points."
                << std::endl;
      std::cout << "Prevet: Time violation: " << prevetter_response.value
                << std::endl;
      break;
    }
    default:
      std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
                << std::endl;
      std::cout << "Prevet value: " << prevetter_response.value << std::endl;
      std::cout << "Prevet index: " << prevetter_response.index << std::endl;
  }

  // Invoke the visualizer to see the proposed trajectory, which will be
  // displayed in violet. See student_game_engine_visualizer.h for other
  // visualization options: you can visualize a short path, a single point, etc.
  // It will be helpful to get such visual feedback on candidate trajectories.
  // Note that there is a built-in visualizer called "ViewManager" implemented
  // elsewhere in the game-engine code, but you don't have full control over
  // what it displays like you do with the Student_game_engine_visualizer
  // invoked below.
  visualizer.drawTrajectory(trajectory);
  quad_to_trajectory_map[quad_name] = trajectory;
  return quad_to_trajectory_map;
  }
}  // namespace game_engine
