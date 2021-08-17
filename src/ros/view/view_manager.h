#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include "map3d.h"

#include "marker_publisher_node.h"
#include "polyhedron_view.h"
#include "plane3d_view.h"

#include "quad_view.h"
#include "balloon_view.h"
#include "goal_view.h"

#include <ros/ros.h>
#include "balloon_status.h"
#include "balloon_status_subscriber_node.h"
#include "balloon_position_subscriber_node.h"

#include "goal_status.h"
#include "goal_status_subscriber_node.h"

#include "trajectory_visualizer_node.h"

#include "warden.h"

namespace game_engine {
  // The ViewManager is a convenience object that encapsulates all of the code
  // used to publish views to RViz. 
  //
  // The ViewManager should run as its own thread.
  class ViewManager {
    public:
      struct BalloonViewOptions {
        std::string balloon_mesh_file_path;
        std::vector<std::pair<std::string, Eigen::Vector3d>> balloons;

        BalloonViewOptions() {}
      }; 

      struct GoalViewOptions {
        std::string goal_mesh_file_path;
        std::vector<std::pair<std::string, Eigen::Vector3d>> goals;

        GoalViewOptions() {}
      }; 

      struct QuadViewOptions {
        std::string quad_mesh_file_path;
        std::vector<std::pair<std::pair<std::string, std::string>, std::shared_ptr<QuadStateWarden>>> quads;

        QuadViewOptions() {}
      };

      struct EnvironmentViewOptions { 
        Plane3DView::Options ground_view_options{
          "world", 0.0f, 1.0f, 0.0f, 1.0f};
        Plane3DView::Options wall_view_options{
            "world", 0.0f, 0.0f, 0.0f, 0.1f};
        PolyhedronView::Options obstacle_view_options{
            "world", 1.0f, 1.0f, 1.0f, 1.0f};
        Map3D map;

        EnvironmentViewOptions() {}
      };

    struct TrajectoryViewOptions {
      std::vector<std::pair<std::string,
                            std::shared_ptr<TrajectoryVisualizerNode>>> trajectories;
      
      TrajectoryViewOptions() {}
    }; 

      ViewManager() {};

      void Run(const QuadViewOptions quad_view_options,
               const BalloonViewOptions balloon_view_options,
               const GoalViewOptions goal_view_options,
               const EnvironmentViewOptions environment_view_options,
               const TrajectoryViewOptions trajectory_view_options);

      void Stop();

    private:
      void RunQuadPublisher(
          const QuadViewOptions quad_view_options);
      void RunBalloonPublisher(
          const BalloonViewOptions balloon_view_options);
      void RunGoalPublisher(
          const GoalViewOptions goal_view_options);
      void RunEnvironmentPublisher(
          const EnvironmentViewOptions environment_view_options);
      void RunTrajectoryPublisher(
          const TrajectoryViewOptions trajectory_view_options);
    
      volatile std::atomic<bool> ok_{true};
  };
}
