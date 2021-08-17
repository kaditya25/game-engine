#include "potential_field.h"

namespace game_engine {

    constexpr bool AlmostEqual(double d1, double d2, double epsilon = 1.0e-12) {
      if (fabs(d1 - d2) < epsilon) {
        return true;
      } else {
        return false;
      }
    }

    Eigen::Vector3d PotentialField::AttractiveGradient(const Eigen::Vector3d& current_position, const Eigen::Vector3d& goal_position) {
      Eigen::Vector3d dUA;

      double d_goal = (current_position - goal_position).norm();

      if (d_goal <= options_.d_thresh or AlmostEqual(d_goal, options_.d_thresh)) {
        // Quadratic
        dUA = {options_.zeta * (current_position.x() - goal_position.x()),
               options_.zeta * (current_position.y() - goal_position.y()),
               options_.zeta * (current_position.z() - goal_position.z())};
      } else {
        // Conic
        dUA = {options_.d_thresh * options_.zeta * (current_position.x() - goal_position.x()) / d_goal,
               options_.d_thresh * options_.zeta * (current_position.y() - goal_position.y()) / d_goal,
               options_.d_thresh * options_.zeta * (current_position.z() - goal_position.z()) / d_goal};
      }

      return dUA;
    }

    Eigen::Vector3d PotentialField::RepulsiveGradient(const Eigen::Vector3d& current_position, const Map3D& map) {

      Eigen::Vector3d dUR(0.0, 0.0, 0.0);

      const std::vector<Polyhedron>& obstacles = map.Obstacles();
      for (auto obs_iter = obstacles.begin(); obs_iter < obstacles.end(); ++obs_iter) {
        Eigen::Vector3d closest_point = map.ClosestPoint(current_position);

        double q_obs = (current_position - closest_point).norm();

        if (q_obs <= options_.q_thresh or AlmostEqual(q_obs, options_.q_thresh)) {
          // Closest point within range of influence
          double delta_dx = (current_position.x() - closest_point.x()) / q_obs;
          double delta_dy = (current_position.y() - closest_point.y()) / q_obs;
          double delta_dz = (current_position.z() - closest_point.z()) / q_obs;

          dUR.x() += options_.ada * ((1.0 / options_.q_thresh) - (1.0 / q_obs)) * (1.0 / (std::pow(q_obs, 2))) * delta_dx;
          dUR.y() += options_.ada * ((1.0 / options_.q_thresh) - (1.0 / q_obs)) * (1.0 / (std::pow(q_obs, 2))) * delta_dy;
          dUR.z() += options_.ada * ((1.0 / options_.q_thresh) - (1.0 / q_obs)) * (1.0 / (std::pow(q_obs, 2))) * delta_dz;
        }
      }

      return dUR;
    }

    double PotentialField::MultiDimNorm(const std::vector<double> partials) {
      double norm_squared = 0.0;

      for (auto iter = partials.begin(); iter < partials.end(); ++iter) {
        norm_squared += std::pow(*iter, 2);
      }

      // Avoid division by zero
      norm_squared += 1e-3;

      return std::sqrt(norm_squared);
    }

    Eigen::Vector3d PotentialField::OneStepGradientDescent(const Eigen::Vector3d& current_position, const Eigen::Vector3d& goal_position, const Map3D& map) {
      Eigen::Vector3d dUA = AttractiveGradient(current_position, goal_position);
      Eigen::Vector3d dUR = RepulsiveGradient(current_position, map);

      Eigen::Vector3d dU(dUA.x() + dUR.x(), dUA.y() + dUR.y(), dUA.z() + dUR.z());

      std::vector<double> partials{dU.x(), dU.y(), dU.z()};

      double norm = MultiDimNorm(partials);

      double descent_x = dU.x() / norm;
      double descent_y = dU.y() / norm;
      double descent_z = dU.z() / norm;

      Eigen::Vector3d new_position(current_position.x() - descent_x * options_.eta,
                                   current_position.y() - descent_y * options_.eta,
                                   current_position.z() - descent_z * options_.eta);

      double dist2goal = (new_position - goal_position).norm();

      if (dist2goal <= options_.eta or AlmostEqual(dist2goal, options_.eta)) {
        options_.terminate = true;
      }

      return new_position;
    }

    bool PotentialField::ReturnTerminate() {
      return options_.terminate;
    }
}