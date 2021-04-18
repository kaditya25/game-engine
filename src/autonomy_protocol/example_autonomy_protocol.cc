// Author: Tucker Haydon

#include <chrono>

#include "../dependencies/P4/dependencies/osqp/include/osqp.h"
#include "../dependencies/P4/src/polynomial_sampler.h"
#include "../dependencies/P4/src/polynomial_solver.h"
#include "../dependencies/P4/examples/gnuplot-iostream.h"

#include "example_autonomy_protocol.h"
#include "occupancy_grid3d.h"
#include "graph.h"
#include "student_game_engine_visualizer.h"

#define DISCRETE_LENGTH .2 // The length of one side of a cube in meters in the occupancy grid
#define SAFETY_BOUNDS .34  // How big the bubble around obstacles will be

namespace game_engine {
  std::chrono::milliseconds dt_chrono = std::chrono::milliseconds(40);
  std::unordered_map<std::string, Trajectory>
  ExampleAutonomyProtocol::UpdateTrajectories() {

    // This function creates and returns a proposed trajectory.  The
    // trajectory gets submitted to the mediation_layer (ML), which responds
    // by setting the data member trajectoryCode_.  See the header file
    // game-engine/src/util/trajectory_codes.h for a list of possible codes.
    //
    // Any code other than TrajectoryCode::Success indicates that the ML has
    // rejected the submitted trajectory.
    //
    // trajectoryCode_ is initialized with TrajectoryCode::Success, so this
    // will be its value the first time this function is called (before any
    // trajectories have been submitted).  Thereafter, trajectoryCode_ will
    // indicate the response code for the most recently submitted trajectory.

    static OccupancyGrid3D occupancy_grid;
    static Graph3D graphOfArena;   // Used by Astar, need to convert Astar 2d to 3D
    static Student_game_engine_visualizer visualizer;
    static bool firstTime = true;
    static std::string& quad_name = friendly_names_[0];
    static Eigen::Vector3d red_balloon_pos;
    static Eigen::Vector3d blue_balloon_pos;

    if(firstTime)
    {
      firstTime = false;
      occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
      graphOfArena = occupancy_grid.AsGraph();  // You can run Astar on this graph
      visualizer.startVisualizing("/game_engine/environment");
      red_balloon_pos = red_balloon_status_->position;
      blue_balloon_pos = blue_balloon_status_->position;
      // Note the balloon popped status of the balloon can be read via the following commented out line of code:
      // red_balloon_status_->popped
    }

    // If you want to see a numerical value for the code, you can cast and
    // print the code as shown below.
    if (trajectoryCode_ != TrajectoryCode::Success) {
      std::cout << "Response code: " <<
        static_cast<unsigned int>(trajectoryCode_) << std::endl;
    }

    // Suppose your AP initially submits a trajectory with a time that exceeds
    // the maximum allowed time between points. You could apply a conditional
    // statement that checks the TrajectoryCode associated with the submitted
    // trajectory and fix the problem, as shown below.
    if (trajectoryCode_ == TrajectoryCode::TimeBetweenPointsExceedsMaxTime) {
      std::cout << "Replanning trajectory: "
        "Shortening time between trajectory points." << std::endl;
      dt_chrono = dt_chrono - std::chrono::milliseconds(15);
    }


    // Always use the chrono::system_clock for time. Trajectories require time
    // points measured in floating point seconds from the unix epoch.
    const std::chrono::milliseconds T_chrono = std::chrono::seconds(30);
    const double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(dt_chrono).count();

    // Use a static function variable to log the first time this function was
    // called. The static function variable acts like a matlab persistent
    // variable, maintaining its value between function calls. The initializer
    // is only called once.
    static const std::chrono::time_point<std::chrono::system_clock> start_chrono_time
      = std::chrono::system_clock::now();
    static const std::chrono::time_point<std::chrono::system_clock> end_chrono_time
      = start_chrono_time + T_chrono;
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time
      = std::chrono::system_clock::now();

    // If beyond the end time, return an empty map
    std::unordered_map<std::string, Trajectory> trajectory_map;
    if(current_chrono_time > end_chrono_time) {return trajectory_map;}

    // The following code generates and returns a new trajectory each time it
    // runs.  The new trajectory starts at the location on the original circle
    // that is closest to the current location of the quad and it creates a
    // trajectory of N position, velocity, and acceleration (PVA) points
    // spaced by dt seconds.  Thus, the code below responds to the actual
    // position of the quad and adjusts the newly generated trajectory
    // accordingly.  But note that its strategy is not optimal for covering
    // the greatest distance in the allotted time in the presence of
    // disturbance accelerations.

    const std::chrono::duration<double> remaining_chrono_time =
      end_chrono_time - current_chrono_time;

    // Number of samples
    const size_t N = remaining_chrono_time/dt_chrono;
    // const size_t N = 1;
    // Radius
    const double radius = 0.5;

    // Angular speed in radians/s
    constexpr double omega = 2*M_PI/30;

    // Load the current position
    Eigen::Vector3d current_pos;
    this->snapshot_->Position(quad_name, current_pos);

    // Place the center of the circle 1 radius in the y-direction from the
    // starting position
    static Eigen::Vector3d circle_center =
      current_pos + Eigen::Vector3d(0, radius, 0);

    // Transform the current position into an angle
    const Eigen::Vector3d r = current_pos - circle_center;
    const double theta_start = std::atan2(r.y(), r.x());

    // TrajectoryVector3D is an std::vector object defined in the trajectory.h
    // file. It's aliased for convenience.
    TrajectoryVector3D trajectory_vector;
    for(size_t idx = 0; idx < N; ++idx) 
    {
      // chrono::duration<double> maintains high-precision floating point time
      // in seconds use the count function to cast into floating point
      const std::chrono::duration<double> flight_chrono_time
        = current_chrono_time.time_since_epoch() + idx * dt_chrono;
      const double flight_time = flight_chrono_time.count();

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
      const double time = flight_chrono_time.count();

      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() <<
            x,   y,   z,
            vx,  vy,  vz,
            ax,  ay,  az,
            yaw,
            time
            ).finished());
    }
    // Construct a trajectory from the trajectory vector
    Trajectory trajectory(trajectory_vector);
    visualizer.drawTrajectory(trajectory);
    trajectory_map[quad_name] = trajectory;

    return trajectory_map;
  }

}
