#include "trajectory_vetter.h"

#include <Eigen/Core>
#include <iostream>

namespace game_engine {

  TrajectoryCode TrajectoryVetter::
  Vet(const Trajectory& trajectory,
      const Map3D& map,
      const std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::string& quad_name
      ) const {

    // Define return variable
    TrajectoryCode trajectory_code_;

    const size_t trajectory_size = trajectory.Size();
    if(trajectory_size < 2) {
      std::cerr << "Specified trajectory for "
                << quad_name
                << " has a size of "
                << trajectory_size
                << ". Trajectories must have a size of 2 or greater. Rejecting."
                << std::endl;
      trajectory_code_.code = MediationLayerCode::NotEnoughTrajectoryPoints;
      return trajectory_code_;
    }

    // Initial position constraints
    QuadState current_quad_state;
    quad_state_warden->Read(quad_name, current_quad_state);

    const Eigen::Vector3d initial_position = trajectory.Position(0);
    const Eigen::Vector3d current_position = current_quad_state.Position();
    if(this->options_.max_distance_from_current_position
        < (initial_position - current_position).norm()) {
      std::cerr
        << "Specified trajectory start point ["
        << initial_position.transpose()
        << "] deviates from the current quad position ["
        << current_position.transpose()
        << "] by more than the maximum distance of "
        << this->options_.max_distance_from_current_position
        << " meters"
        << std::endl;
      trajectory_code_.code = MediationLayerCode::StartPointFarFromCurrentPosition;
      trajectory_code_.value = (initial_position - current_position).norm();
      return trajectory_code_;
    }

    // Position constraints
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d point = trajectory.Position(idx);
      const Map3D inflated_map = map.Inflate(1);
      if(false == inflated_map.Contains(point)) {
        std::cerr
          << "Specified trajectory point ["
          << point.transpose()
          << "] exceeded map bounds"
          << std::endl;

        trajectory_code_.code = MediationLayerCode::PointExceedsMapBounds;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
      if(false == map.IsFreeSpace(point)) {
        std::cerr
          << "Specified trajectory point ["
          << point.transpose()
          << "] is contained within an obstacle"
          << std::endl;

        trajectory_code_.code = MediationLayerCode::PointWithinObstacle;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

    // Velocity constraints
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d vel = trajectory.Velocity(idx);
      if(this->options_.max_velocity_magnitude < vel.norm()) {
        std::cerr
          << "Specified trajectory velocity"
          << " exceeds maximum velocity constraint of "
          << this->options_.max_velocity_magnitude
          << " m/s"
          << std::endl;
        trajectory_code_.code = MediationLayerCode::ExceedsMaxVelocity;
        trajectory_code_.value = vel.norm();
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

    // Mean value theorem for velocity constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const Eigen::Vector3d current_pos = trajectory.Position(idx);
      const Eigen::Vector3d next_pos = trajectory.Position(idx+1);
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double mean_value_velocity
        = ((next_pos - current_pos) / (next_time - current_time)).norm();
      if(this->options_.max_velocity_magnitude < mean_value_velocity) {
        std::cerr
          << "Specified mean-value trajectory velocity "
          << " exceeds maximum velocity constraint of "
          << this->options_.max_velocity_magnitude
          << " m/s"
          << std::endl;

        trajectory_code_.code = MediationLayerCode::MeanValueExceedsMaxVelocity;
        trajectory_code_.value = mean_value_velocity;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

// Acceleration constraints
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d acc = trajectory.Acceleration(idx);
      if(this->options_.max_acceleration_magnitude < acc.norm()) {
        std::cerr
          << "Specified trajectory acceleration "
          << " exceeds maximum acceleration constraint of "
          << this->options_.max_acceleration_magnitude
          << " m/s^2"
          << std::endl;
        trajectory_code_.code = MediationLayerCode::ExceedsMaxAcceleration;
        trajectory_code_.value = acc.norm();
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

    // Mean value theorem for acceleration constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const Eigen::Vector3d current_vel = trajectory.Velocity(idx);
      const Eigen::Vector3d next_vel = trajectory.Velocity(idx+1);
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double mean_value_acceleration
        = ((next_vel - current_vel) / (next_time - current_time)).norm();
      if(this->options_.max_acceleration_magnitude < mean_value_acceleration) {
        std::cerr
          << "Specified mean-value trajectory acceleration"
          << " exceeds maximum acceleration constraint of "
          << this->options_.max_acceleration_magnitude
          << " m/s"
          << std::endl;
        trajectory_code_.code = MediationLayerCode::MeanValueExceedsMaxAcceleration;
        trajectory_code_.value = mean_value_acceleration;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

    // Time constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double delta_time = next_time - current_time;

      if( true == (delta_time < 0.0) ) {
        std::cerr
          << "Trajectory timestamps must be monotonically increasing"
          << std::endl;

        trajectory_code_.code = MediationLayerCode::TimestampsNotIncreasing;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }

      if(this->options_.max_delta_t < delta_time) {
        std::cerr
          << "Time between adjacent trajectory samples"
          << " exceeds maximum time of "
          << this->options_.max_delta_t
          << " seconds."
          << std::endl;
        trajectory_code_.code = MediationLayerCode::TimeBetweenPointsExceedsMaxTime;
        trajectory_code_.value = delta_time;
        trajectory_code_.index = idx;
        return trajectory_code_;
      }
    }

    trajectory_code_.code = MediationLayerCode::Success;
    return trajectory_code_;
  }
}
