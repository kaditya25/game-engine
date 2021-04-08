#pragma once

namespace game_engine {
  enum class TrajectoryCode {
    Success = 0,
    
    // Vetter Codes
    NotEnoughTrajectoryPoints = 1,
    StartPointFarFromCurrentPosition = 2,
    PointExceedsMapBounds = 3,
    PointWithinObstacle = 4,
    ExceedsMaxVelocity = 5,
    MeanValueExceedsMaxVelocity = 6,
    ExceedsMaxAcceleration = 7,
    MeanValueExceedsMaxAcceleration = 8,
    TimestampsNotIncreasing = 9,
    TimeBetweenPointsExceedsMaxTime = 10,

    // Trajectory Warden Codes
    KeyAlreadyExists = 11,
    KeyDoesNotExist = 12,
    ThreadStopped = 13,

    // Client Codes
    FailedToCallService = 14,
  };

  // std::ostream& operator << (std::ostream& os, const TrajectoryCode& tc) {
  //   os << static_cast<unsigned int>(tc);
  //   return os;
  // }  
}
