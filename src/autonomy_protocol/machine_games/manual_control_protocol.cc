#include <chrono>
#include <queue>
#include <bits/stdc++.h>

#include "manual_control_protocol.h"
#include "student_game_engine_visualizer.h"

#define DISCRETE_LENGTH .4 // The length of one side of a cube in meters in the occupancy grid
#define SAFETY_BOUNDS .8  // How big the bubble around obstacles will be

namespace game_engine {

  void ManualControlProtocol::loadJoyParams()
  {
    //Copied from Px4
    double RollMaxDeg, PitchMaxDeg, YawRateMaxDeg;
    double xRate, yRate, zRate;
    
    RollMaxDeg = 40.0;
    PitchMaxDeg = 40.0;
    YawRateMaxDeg = 60.0;
    
    joy_params_.x_rate = 3.0;
    joy_params_.y_rate = 3.0;
    joy_params_.z_rate = 2.0;
    
    joy_params_.roll_max     = RollMaxDeg*M_PI/180.0; //convert to radians
    joy_params_.pitch_max    = PitchMaxDeg*M_PI/180.0;
    joy_params_.yaw_rate_max = YawRateMaxDeg*M_PI/180.0;
  }


  TrajectoryVector3D ManualControlProtocol::joyToTrajectory(Eigen::Vector3d current_position, Eigen::Vector3d current_velocity, 
    std::vector<Eigen::Vector3d> path_points, double yaw, std::vector<double> times)
  {
    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time
    = std::chrono::system_clock::now();

    double time;
    
    const int M = (int)((times[1] - times[0])/.01)+1;
    const Eigen::Vector3d vel = (path_points[1] - path_points[0])/(times[1] - times[0]);
    
    TrajectoryVector3D trajectory_vector;
    for(int idx = 0; idx < M; ++idx) 
    {
      // chrono::duration<double> maintains high-precision floating point time
      // in seconds use the count function to cast into floating point
      std::chrono::duration<double> flight_chrono_time
        = current_chrono_time.time_since_epoch() + std::chrono::duration<double>(idx*.01);
      // The trajectory requires the time to be specified as a floating point
      // number that measures the number of seconds since the unix epoch.

      time = flight_chrono_time.count();

      // Push an Eigen instance onto the trajectory vector
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() <<
            path_points[0](0) + vel(0)*.01*idx, path_points[0](1) + vel(1)*.01*idx, path_points[0](2) + vel(2)*.01*idx,
            vel(0), vel(1), vel(2),
            0, 0, 0,
            yaw,
            time
            ).finished());
    }
  return trajectory_vector;
  }



  std::unordered_map<std::string, Trajectory> ManualControlProtocol::UpdateTrajectories() 
  {
    const int M = friendly_names_.size();

    static std::vector<Student_game_engine_visualizer> visualizer(M);
    static std::vector<Eigen::Vector3d> current_pos(M);
    static std::vector<Eigen::Vector3d> current_vel(M);

    loadJoyParams();

    TrajectoryVector3D trajectory_vector;
    std::unordered_map<std::string, Trajectory> trajectory_map;

    for(size_t ii=0; ii<friendly_names_.size(); ii++) 
    { 
      std::string& quad_name = friendly_names_[ii];
      this->snapshot_->Position(quad_name, current_pos[ii]);
      this->snapshot_->Velocity(quad_name, current_vel[ii]);
      std::vector<Eigen::Vector3d> path_points;
      std::vector<double> times;
      double dt = 1;
      
      joyModeMap(joy_params_, *joy_input_, dt, vel_output_, rpy_output_);

      //scale_back sets the velocity of the quad. Increase it to make the quad faster,
      //but also increase the safety bounds in quad_state_watchdog.h
      double scale_back = .7;
      path_points.push_back(current_pos[ii]);
      path_points.push_back(current_pos[ii] + dt*vel_output_*scale_back);

      double yaw = .1*rpy_output_(2);

      times.push_back(0);
      times.push_back(dt);

      trajectory_vector = joyToTrajectory(current_pos[ii], current_vel[ii], path_points, yaw, times);

      Trajectory trajectory(trajectory_vector);
      visualizer[ii].drawTrajectory(trajectory);
      trajectory_map[quad_name] = trajectory;
    }//End for loop

    return trajectory_map;
  }//End UpdateTrajectories() function


}

