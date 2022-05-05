#ifndef TRAJECTORY_CODE_H
#define TRAJECTORY_CODE_H

namespace game_engine {
enum class MediationLayerCode {
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

  // Quad Scenario Codes
  QuadViolatesMapBoundaries = 15,
  QuadTooCloseToAnotherQuad = 16,
  QuadTrajectoryCollidesWithAnotherQuad = 17,

  // Watchdog Codes
  QuadNotRegistered = 18,
  RegisterQuadWithWatchdog = 19,
};

// TrajectoryCode is used for returning the code, value, and index for
// violations. Initialized with Success and 0's.
struct TrajectoryCode {
  MediationLayerCode code = MediationLayerCode::Success;
  double value = 0.0;
  int index = 0;
};
}  // namespace game_engine

#endif
