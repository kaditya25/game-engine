#include <chrono>
#include <queue>
#include <bits/stdc++.h>

#include "blue_team_autonomy_protocol.h"
#include "student_game_engine_visualizer.h"

#define DISCRETE_LENGTH .4 // The length of one side of a cube in meters in the occupancy grid
#define SAFETY_BOUNDS .8  // How big the bubble around obstacles will be

namespace game_engine {
  // The blue team autonomy protocol attempts to pop a red balloon while defending 
  // its own blue balloon. The protocol switches between offense and defense
  // depending on its proximity to its own balloon, the opposing balloon, and the
  // position of the enemy quad. Within this protocol "quad" refers to the blue team,
  // and "enemy quad" refers to the opposing team. If two quads are frozen at any point;
  // the offense and defense strategies both prioritize unfreezing the quads. The 
  // defense strategy consists of staying between the enemy and our balloon. The offense
  // strategy consits of moving towards the enemy balloon, and rapidly moving in the y
  // or z directions if the enemy quad is in the way of the balloon.
  
  TrajectoryVector3D BlueTeamAutonomyProtocol::targetToTrajectory(Eigen::Vector3d current_position, Eigen::Vector3d current_velocity, 
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

  std::unordered_map<std::string, Trajectory> BlueTeamAutonomyProtocol::UpdateTrajectories() 
  {
    const int M = friendly_names_.size();

    static std::vector<Student_game_engine_visualizer> visualizer(M);
    static std::vector<Eigen::Vector3d> current_pos(M);
    static std::vector<Eigen::Vector3d> current_vel(M);
    static std::vector<Eigen::Vector3d> enemy_current_pos(M);
    static std::vector<Eigen::Vector3d> enemy_current_vel(M);
    static std::vector<Eigen::Vector3d> target_pos(M);

    static Eigen::Vector3d red_balloon_pos;
    static Eigen::Vector3d blue_balloon_pos;

    TrajectoryVector3D trajectory_vector;
    std::unordered_map<std::string, Trajectory> trajectory_map;

    for(size_t ii=0; ii<friendly_names_.size(); ii++) 
    { 
      std::string& quad_name = friendly_names_[ii];
      std::string& enemy_quad_name = enemy_names_[ii];
      this->snapshot_->Position(quad_name, current_pos[ii]);
      this->snapshot_->Velocity(quad_name, current_vel[ii]);
      this->snapshot_->Position(enemy_quad_name, enemy_current_pos[ii]);
      this->snapshot_->Velocity(enemy_quad_name, enemy_current_vel[ii]);

      red_balloon_pos = *red_balloon_position_;
      blue_balloon_pos = *blue_balloon_position_;

      std::vector<Eigen::Vector3d> path_points;
      std::vector<double> times;
      double dt = 2; //start large but make this smaller later

      /////////////////////////////////////////////////////////////
      //strategies to find target_pos
      bool offense;
      
      if((red_balloon_pos - current_pos[ii]).norm() < (blue_balloon_pos - current_pos[ii]).norm())
      {
        bool offense = true;
      }
      else
      {
        bool offense = false;
      }

      if(offense==true) //play offense
      {
        Eigen::Vector3d friendly_to_enemy_vec = (enemy_current_pos[ii] - current_pos[ii])
          /(enemy_current_pos[ii] - current_pos[ii]).norm();

        Eigen::Vector3d vec_to_balloon = (red_balloon_pos - current_pos[ii])
          /(red_balloon_pos - current_pos[ii]).norm();

        if(trajectoryCodeMap_[quad_name].code == MediationLayerCode::QuadTooCloseToAnotherQuad)
        {
          // Move away from other quad
          target_pos[ii] = -1*friendly_to_enemy_vec + current_pos[ii]; // -1 reverses direction
        }
        else
        {
          if(abs((vec_to_balloon.dot(friendly_to_enemy_vec))
            /(vec_to_balloon.norm()*friendly_to_enemy_vec.norm()) - 1) < .004 
            && (current_pos[ii] - enemy_current_pos[ii]).norm() < 1)
          //Checks to see if the enemy is < 1m away, and that sin(theta)<.004,
          //where theta is the angle between the vector from the current quad
          //position to the balloon and the vector from the current quad position
          //to the enemy quad position (checks to see if enemy is in the way) 
          {
            Eigen::Vector3d juke_vec;

            int decide = rand() %4+1; //Selects random # between 1 and 4

            if(decide==1) // +y
            {
              juke_vec << 0,1,0;
              target_pos[ii] = current_pos[ii] + juke_vec + .1*vec_to_balloon;
            }
            if(decide==2) // -y
            {
              juke_vec << 0,-1,0;
              target_pos[ii] = current_pos[ii] + juke_vec + .1*vec_to_balloon;
            }
            if(decide==3) // +z
            {
              juke_vec << 0,0,1;
              target_pos[ii] = current_pos[ii] + juke_vec + .1*vec_to_balloon;
            }
            if(decide==4) // -z
            {
              juke_vec << 0,0,-1;
              target_pos[ii] = current_pos[ii] + juke_vec + .1*vec_to_balloon;
            }
          }
          else 
          {
            //Target the red balloon
            target_pos[ii] = (red_balloon_pos - current_pos[ii])/(red_balloon_pos - current_pos[ii]).norm()
              + current_pos[ii];
          }
        }
      }

      else //play defense
      {
        if(trajectoryCodeMap_[quad_name].code == MediationLayerCode::QuadTooCloseToAnotherQuad)
        {
          // Move away from other quad
          Eigen::Vector3d friendly_to_enemy_vec = (enemy_current_pos[ii] - current_pos[ii])
            /(enemy_current_pos[ii] - current_pos[ii]).norm();
          target_pos[ii] = -1*friendly_to_enemy_vec + current_pos[ii]; // -1 reverses direction
        }
        else
        {
          // Stay 1/4 the distance between enemy quad and our balloon
          Eigen::Vector3d enemy_vec_to_balloon = blue_balloon_pos - enemy_current_pos[ii];
          target_pos[ii] = ((.25*enemy_vec_to_balloon + enemy_current_pos[ii]) - current_pos[ii])
            /((.25*enemy_vec_to_balloon + enemy_current_pos[ii]) - current_pos[ii]).norm() + current_pos[ii];
        }
      }
      ////////////////////////////////////////////////////////////

      path_points.push_back(current_pos[ii]);
      path_points.push_back(target_pos[ii]);

      times.push_back(0);
      times.push_back(dt);

      double yaw = 0;

      trajectory_vector = targetToTrajectory(current_pos[ii], current_vel[ii], path_points, yaw, times);

      Trajectory trajectory(trajectory_vector);
      visualizer[ii].drawTrajectory(trajectory);
      trajectory_map[quad_name] = trajectory;
    }//End for loop

    return trajectory_map;
  }//End UpdateTrajectories() function


}

