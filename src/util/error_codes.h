#pragma once

namespace game_engine {

  typedef enum {
    Success = 1,

    // Vetter Codes
    NotEnoughTrajectoryPoints,
    StartPointFarFromCurrentPosition,
    PointExceedsMapBounds,
    PointWithinObstacle,
    ExceedsMaxVelocity,
    MeanValueExceedsMaxVelocity,
    ExceedsMaxAcceleration,
    MeanValueExceedsMaxAcceleration,
    TimestampsNotIncreasing,
    TimeBetweenPointsExceedsMaxTime,

    // Trajectory Warden Codes
    KeyAlreadyExists,
    KeyDoesNotExist,
    OkVariableChanged,

    // Client Codes
    FailedToCallService,
    
  } StatusCode;
}
