#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <mutex>

#include "marker_view.h"
#include "warden.h"

namespace game_engine {
  class QuadView : public MarkerView {
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
          mesh_resource(mesh_resource_),
          r(r_),
          g(g_),
          b(b_),
          a(a_)
        {}
      };

      QuadView(
          std::string quad_name = "",
          std::shared_ptr<QuadStateWarden> quad_state_warden = nullptr,
          const Options& options = Options())
          : quad_name_(quad_name),
            quad_state_warden_(quad_state_warden),
            options_(options),
            unique_id_(GenerateUniqueId()) {}

      std::vector<visualization_msgs::Marker> Markers() const override;

    private: 
      Options options_;
      std::string quad_name_;
      std::shared_ptr<QuadStateWarden> quad_state_warden_;
      uint32_t unique_id_;

      static uint32_t GenerateUniqueId();
  };
}
