// Author: Dan LaChapelle

#include "goal_view.h"

namespace game_engine {
std::vector<visualization_msgs::Marker> GoalView::Markers() const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = this->options_.frame_id;
  marker.id = this->unique_id_;
  marker.ns = "Goal";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05f;
  marker.scale.y = 0.05f;
  marker.scale.z = 0.05f;
  marker.pose.position.x = this->goal_position_.x();
  marker.pose.position.y = this->goal_position_.y();
  marker.pose.position.z = this->goal_position_.z();  // Heuristic offset
  marker.color.r = this->options_.r;
  marker.color.g = this->options_.g;
  marker.color.b = this->options_.b;
  marker.color.a = this->options_.a;
  marker.mesh_resource = this->options_.mesh_resource;
  marker.mesh_use_embedded_materials = true;

  return {marker};
}

uint32_t GoalView::GenerateUniqueId() {
  static std::mutex mtx;
  static uint32_t id = 0;

  std::lock_guard<std::mutex> lock(mtx);
  id++;
  return id;
}
}  // namespace game_engine
