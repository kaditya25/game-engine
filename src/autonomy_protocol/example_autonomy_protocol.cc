#include <chrono>

#include "osqp.h"
#include "polynomial_sampler.h"
#include "polynomial_solver.h"
#include "../dependencies/P4/examples/gnuplot-iostream.h"

#include "example_autonomy_protocol.h"
#include "occupancy_grid3d.h"
#include "graph.h"
#include "student_game_engine_visualizer.h"

// The length of one side of a cube in meters in the occupancy grid
#define DISCRETE_LENGTH .2
// How big the "inflation" bubble around obstacles will be, in meters
#define SAFETY_BOUNDS .34  

namespace game_engine {
  std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(40);

  // UpdateTrajectories creates and returns a proposed trajectory.  The
  // proposed trajectory gets submitted to the mediation_layer (ML), which
  // responds by setting the data member trajectoryCode_.  See the header file
  // game-engine/src/util/trajectory_codes.h for a list of possible codes.
  //
  // Any code other than TrajectoryCode::Success indicates that the ML has
  // rejected the submitted trajectory.
  //
  // trajectoryCode_ is initialized with TrajectoryCode::Success, so this will
  // be its value the first time this function is called (before any
  // trajectories have been submitted).  Thereafter, trajectoryCode_ will
  // indicate the TrajectoryCode for the most recently submitted trajectory.
  std::unordered_map<std::string, Trajectory>
  ExampleAutonomyProtocol::UpdateTrajectories() {

    // Set the duration of the example trajectory
    constexpr int duration_sec = 30;
    const std::chrono::milliseconds T_chrono =
      std::chrono::seconds(duration_sec);

    // 'static' variables act like Matlab persistent variables, maintaining
    // their value between function calls. The initializer is only called
    // once.  If you prefer, you could instead include these variables as data
    // members in your StudentAutonomyProtocol class, which is a more standard
    // C++ code design pattern.
    static OccupancyGrid3D occupancy_grid;
    static Graph3D graphOfArena;
    // Student_game_engine_visualizer is a class that supports visualizing
    // paths, curves, points, and whole trajectories in the RVIZ display of
    // the arena to aid in your algorithm development.
    static Student_game_engine_visualizer visualizer;
    static bool firstTime = true;
    static bool halt = false;
    static Eigen::Vector3d start_pos;
    static const std::chrono::time_point<std::chrono::system_clock> start_chrono_time
      = std::chrono::system_clock::now();
    static const std::chrono::time_point<std::chrono::system_clock> end_chrono_time
      = start_chrono_time + T_chrono;

    // Load the current quad position
    const std::string& quad_name = friendly_names_[0];
    Eigen::Vector3d current_pos;
    snapshot_->Position(quad_name, current_pos);

    // Set some static variables the first time this function is called
    if (firstTime) {
      firstTime = false;
      occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
      // You can run A* on graphOfArena once you created a 3D version of A*
      graphOfArena = occupancy_grid.AsGraph();  
      visualizer.startVisualizing("/game_engine/environment");
      start_pos = current_pos;
    }

    // Obtain current balloon positions and popped states
    const Eigen::Vector3d red_balloon_pos = red_balloon_status_->position;
    const Eigen::Vector3d blue_balloon_pos = blue_balloon_status_->position;
    const bool red_balloon_popped = red_balloon_status_->popped;
    const bool blue_balloon_popped = blue_balloon_status_->popped;

    // Condition some decisions on wind intensity
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

    // You can fill out this switch statement with case statements tailored to
    // each TrajectoryCode.
    switch (trajectoryCode_) {
    case TrajectoryCode::Success:
      // You probably won't need to do anything in response to Success.
      break;
    case TrajectoryCode::TimeBetweenPointsExceedsMaxTime: {
      // Suppose your AP initially submits a trajectory with a time that
      // exceeds the maximum allowed time between points. You could fix the
      // problem as shown below.
      std::cout << "Replanning trajectory: "
        "Shortening time between trajectory points." << std::endl;
      dt_chrono = dt_chrono - std::chrono::milliseconds(15);
      break;
    }
    default:
      // If you want to see a numerical TrajectoryCode value, you can cast and
      // print the code as shown below.
      std::cout << "TrajectoryCode: " <<
        static_cast<int>(trajectoryCode_) << std::endl;
    }
    
    // Always use the chrono::system_clock for time. Trajectories require time
    // points measured in floating point seconds from the Unix epoch.
    const double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(dt_chrono).count();
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time
      = std::chrono::system_clock::now();
    const std::chrono::duration<double> remaining_chrono_time =
      end_chrono_time - current_chrono_time;

    // The following code generates and returns a new trajectory each time it
    // runs.  The new trajectory starts at the location on the original circle
    // that is closest to the current location of the quad and it creates a
    // trajectory of N position, velocity, and acceleration (PVA) points
    // spaced by dt seconds.  Thus, the code below responds to the actual
    // position of the quad and adjusts the newly-generated trajectory
    // accordingly.  But note that its strategy is not optimal for covering
    // the greatest distance in the allotted time in the presence of
    // disturbance accelerations.

    // Number of samples
    const size_t N = remaining_chrono_time/dt_chrono;

    // Create an empty map
    std::unordered_map<std::string, Trajectory> trajectory_map;
    
    // If end time has been exceeded, or if there are too few samples for a
    // valid trajectory, return an empty map
    constexpr size_t min_required_samples_in_trajectory = 2;
    if (current_chrono_time >= end_chrono_time ||
       N < min_required_samples_in_trajectory) {
      return trajectory_map;
    }

    // Halt at goal position when close
    constexpr double goal_arrival_threshold_meters = 0.3;
    const Eigen::Vector3d dv = current_pos - goal_position_;
    // TrajectoryVector3D is an std::vector object defined in trajectory.h
    TrajectoryVector3D trajectory_vector;

    // Radius of circular example trajectory in meters
    const double radius = 0.3;

    // Angular speed of circular example trajectory in radians/s 
    constexpr double omega = 2*M_PI/duration_sec;

    // Place the center of the circle 1 radius in the y-direction from the
    // goal position
    const Eigen::Vector3d circle_center = goal_position_;

    // Transform the current position into an angle
    const Eigen::Vector3d r = current_pos - circle_center;
    const double theta_start = std::atan2(r.y(), r.x());

    // Generate remaining circular trajectory
    for (size_t idx = 0; idx < N; ++idx) {
      // chrono::duration<double> maintains high-precision floating point time
      // in seconds use the count function to cast into floating point
      const std::chrono::duration<double> flight_chrono_time
        = current_chrono_time.time_since_epoch() + idx * dt_chrono;

      // Angle in radians
      const double theta = theta_start + omega * idx * dt;

      // Circle
      const double x = circle_center.x() + radius * std::cos(theta);
      const double y = circle_center.y() + radius * std::sin(theta);
      const double z = circle_center.z();

      // Chain rule
      const double vx = -radius * std::sin(theta) * omega;
      const double vy =  radius * std::cos(theta) * omega;
      const double vz = 0.0;

      // Chain rule
      const double ax = -radius * std::cos(theta) * omega * omega;
      const double ay = -radius * std::sin(theta) * omega * omega;
      const double az = 0.0;

      const double yaw = 0.0;

      // The trajectory requires the time to be specified as a floating point
      // number that measures the number of seconds since the unix epoch.
      const double flight_time = flight_chrono_time.count();

      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() <<
            x,   y,   z,
            vx,  vy,  vz,
            ax,  ay,  az,
            yaw,
            flight_time).finished());
    }
    // Construct a trajectory from the trajectory vector
    Trajectory trajectory(trajectory_vector);
    // Invoke the visualizer to see the proposed trajectory, which will be
    // displayed in violet. See student_game_engine_visualizer.h for other
    // visualization options: you can visualize a short path, a single point,
    // etc.  It will be helpful to get such visual feedback on candidate
    // trajectories.  Note that there is a built-in visualizer called
    // "ViewManager" implemented elsewhere in the game-engine code, but you
    // don't have full control over what it displays like you do with the
    // Student_game_engine_visualizer invoked below.
    visualizer.drawTrajectory(trajectory);
    trajectory_map[quad_name] = trajectory;

    return trajectory_map;
  }
}
