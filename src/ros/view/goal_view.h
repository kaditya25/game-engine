// Author: Tucker Haydon

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>
#include <Eigen/Core>

#include "marker_view.h"

namespace game_engine {
  class GoalView : public MarkerView {
    public:
      struct Options {
        // ROS frame
        std::string frame_id;
        // Mesh resource file
        std::string mesh_resource;
        // RGB red value [0,1]
        float r;
        // RGB green value [0,1]
        float g;
        // RGB blue value [0,1]
        float b;
        // RGB alpha value [0,1]
        float a;

        Options(
            const std::string& frame_id_ = "world",
            const std::string& mesh_resource_ = "",
            const float r_ = 1.0f,
            const float g_ = 1.0f,
            const float b_ = 1.0f,
            const float a_ = 1.0f
            ) 
        : frame_id(frame_id_),
          r(r_),
          g(g_),
          b(b_),
          a(a_)
        {}
      };

      GoalView(
          Eigen::Vector3d& goal_position,
          const Options& options = Options())
        : goal_position_(goal_position),
          options_(options),
          unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

      Eigen::Vector3d goal_position_;
      
      Options options_;

    private: 
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };
}
