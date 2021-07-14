#ifndef GAMEENGINE_SUPER_DUPER_VISUALIZIFIER_H
#define GAMEENGINE_SUPER_DUPER_VISUALIZIFIER_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "marker_publisher_node.h"

#include "occupancy_grid3d.h"
#include "trajectory.h"

class Student_game_engine_visualizer {
private:
  ros::Publisher publisher_;
public:
  Student_game_engine_visualizer() {}
  void startVisualizing(std::string msg_name);
  void spin();
  void drawPath(std::vector<Eigen::Vector3d> path, int id);
  void drawDot(Eigen::Vector3d pt, int id, bool good);
  void drawTrajectory(game_engine::Trajectory traj, int id_offset=0);
  void drawCurve(std::vector<Eigen::Vector3d> pts, int id, Eigen::Vector3d rgb);
};


#endif //GAMEENGINE_SUPER_DUPER_VISUALIZIFIER_H
