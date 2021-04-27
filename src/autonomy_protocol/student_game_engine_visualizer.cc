//
// Created by james on 4/29/20.
//

#include "student_game_engine_visualizer.h"

#include <thread>
#include <cmath>
void Student_game_engine_visualizer::startVisualizing(std::string msg_name) {
  auto nodeHandle = ros::NodeHandle("/occupancy_visualizer/");
  publisher_ = nodeHandle.advertise<visualization_msgs::Marker>(msg_name, 100);
}

void Student_game_engine_visualizer::spin() {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 12345;
  msg.ns = "occupancy_visualizer";
  msg.type = visualization_msgs::Marker::CUBE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 1;
  msg.scale.y = 1;
  msg.scale.z = 1;
  msg.color.r = 1;
  msg.color.a = 1;
  msg.pose.orientation.w = 1;

  publisher_.publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Student_game_engine_visualizer::drawOccupancyGrid(game_engine::OccupancyGrid3D *grid) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 12346;
  msg.ns = "occupancy_grid";
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.action = visualization_msgs::Marker::ADD;

  msg.scale.x = grid->GridSize();
  msg.scale.y = grid->GridSize();
  msg.scale.z = grid->GridSize();
  msg.color.r = 1;
  msg.color.a = 0.5;
  msg.pose.orientation.w = 1;

  // Iterate through occupancy grid points, adding a cube at that spot if it's occupied
  for(int x = 0; x < grid->SizeX(); x++) {
    for(int y = 0; y < grid->SizeY(); y++) {
      for(int z = 0; z < grid->SizeZ(); z++)
      {
        if(grid->IsOccupied(z,y,x)) 
        {
          Eigen::Vector3d pt = grid->boxCenter(x,y,z);
          geometry_msgs::Point point;
          point.x = pt[0];
          point.y = pt[1];
          point.z = pt[2];
          msg.points.push_back(point);
        }
      }
    }
  }

  publisher_.publish(msg);

}

void Student_game_engine_visualizer::drawPath(std::vector<Eigen::Vector3d> path, int id) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 12347 + id;
  msg.ns = "path";
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 0.05;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;
  msg.color.b = 1;
  msg.color.a = 1;
  msg.pose.orientation.w = 1;

  for (auto pt : path) {
    geometry_msgs::Point point;
    point.x = pt[0];
    point.y = pt[1];
    point.z = pt[2];
    msg.points.push_back(point);
  }

  publisher_.publish(msg);

  auto msg2 = msg;

  // hazy outer cell
  msg2.ns = "outer_cell";
  msg2.scale.x = 0.2;
  msg2.scale.y = 0.2;
  msg2.scale.z = 0.2;
  msg2.color.a = 0.35;

  publisher_.publish(msg2);
}

void Student_game_engine_visualizer::drawTrajectory(game_engine::Trajectory trajectory) {

  // Draw position points as a LineStrip
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 12348;
  msg.ns = "trajectory";
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 0.03;
  msg.color.b = 1;
  msg.color.r = 1;
  msg.color.a = 1;
  msg.pose.orientation.w = 1;

  for (int i = 0; (i < trajectory.Size() && i < 15000); i++) {
    auto pva = trajectory.PVA(i);
    geometry_msgs::Point point;

    double x = pva[0];
    if (std::isnan(x)) x = 100;
    if (x > 1000) x = 1000;
    if (x < -1000) x = -1000;

    double y = pva[1];
    if (std::isnan(y)) y = 100;
    if (y > 1000) y = 1000;
    if (y < -1000) y = -1000;

    double z = pva[2];
    if (std::isnan(z)) z = 100;
    if (z > 1000) z = 1000;
    if (z < -1000) z = -1000;

    point.x = x;
    point.y = y;
    point.z = z;
    msg.points.push_back(point);
  }

  publisher_.publish(msg);
}


void Student_game_engine_visualizer::drawDot(Eigen::Vector3d pt, int id, bool good) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = id;
  msg.ns = "dot";
  msg.type = visualization_msgs::Marker::CUBE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  msg.color.g = good?1:0;
  msg.color.r = good?0:1;
  msg.color.a = 1;
  msg.pose.orientation.w = 1;
  msg.pose.position.x = pt[0];
  msg.pose.position.y = pt[1];
  msg.pose.position.z = pt[2];

  publisher_.publish(msg);
}
void Student_game_engine_visualizer::drawCurve(std::vector<Eigen::Vector3d> pts, int id, Eigen::Vector3d rgb) {
  // Draw position points as a LineStrip
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = id;
  msg.ns = "curve";
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 0.03;
  msg.color.r = rgb[0];
  msg.color.g = rgb[1];
  msg.color.b = rgb[2];
  msg.color.a = 1;
  msg.pose.orientation.w = 1;

  for (Eigen::Vector3d pt : pts) {
    geometry_msgs::Point point;

    double x = pt[0];
    if (std::isnan(x)) x = 100;
    if (x > 1000) x = 1000;
    if (x < -1000) x = -1000;

    double y = pt[1];
    if (std::isnan(y)) y = 100;
    if (y > 1000) y = 1000;
    if (y < -1000) y = -1000;

    double z = pt[2];
    if (std::isnan(z)) z = 100;
    if (z > 1000) z = 1000;
    if (z < -1000) z = -1000;

    point.x = x;
    point.y = y;
    point.z = z;
    msg.points.push_back(point);
  }

  publisher_.publish(msg);
}