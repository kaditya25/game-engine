

#pragma once

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <mutex>
#include <vector>

#include "marker_view.h"

namespace game_engine {
class BalloonView : public MarkerView {
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

    Options(const std::string& frame_id_ = "world",
            const std::string& mesh_resource_ = "", const float r_ = 1.0f,
            const float g_ = 1.0f, const float b_ = 1.0f, const float a_ = 1.0f)
        : frame_id(frame_id_),
          mesh_resource(mesh_resource_),
          r(r_),
          g(g_),
          b(b_),
          a(a_) {}
  };

  BalloonView(Eigen::Vector3d& balloon_position,
              const Options& options = Options())
      : balloon_position_(balloon_position),
        options_(options),
        unique_id_(GenerateUniqueId()) {}

  std::vector<visualization_msgs::Marker> Markers() const override;

  Eigen::Vector3d balloon_position_;

  Options options_;

 private:
  uint32_t unique_id_;

  static uint32_t GenerateUniqueId();
};
}  // namespace game_engine
