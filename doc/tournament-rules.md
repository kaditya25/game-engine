# Tournament Rules

## Obstacle Course Scoring
Pre-tournament and tournament obstacle course scoring is as follows:
1) Scores are first ordered by the number of balloons popped. The greater
the number of balloons popped, the greater the score.
2) Scores are next ordered by the time it took for both balloons to be
popped and for the quadcopter to return to the start position. The shorter the
run time, the greater the score.

For example, a team that pops both balloons within the time limit will be
scored higher than a team that only pops one balloon even if the second team's
round-trip time was shorter.

The scoring reflects a law of software design: 
1) get it working
2) get it working right
3) get it working right and fast

## Number of Attempts
Each team will have a single attempt to pop both the red and blue balloons on
tournament day. If the instructors determine that a team's attempt failed due a
failure of course materials (e.g., balloon gets sucked into quadcopter's motor),
the attempt is refunded and the team is permitted to try again.

## Path Constraints for the Obstacle Course
The following subsections introduce path constraints that teams must respect.

### No-Fly Zones
Quadcopters must not fly into obstacles or exceed the bounds of the map. This
constraint manifests in two ways:

1) If the Mediation Layer determines that a proposed trajectory intersects an
obstacle or exceeds the bounds of the map, the proposed trajectory is rejected
and the quadcopter will continue to follow the most recent, valid trajectory.

2) If the Mediation Layer determines that a quadcopter has intersected or
touched an obstacle during the flight as a result of disturbances, the
quadcopter is frozen, forced to land, and the balloon-popping attempt is
forfeit. This constraint has a strict punishment because violations imperil
the quadcopters. Teams should take care not to fly too close to obstacles in
case the wind or another disturbance forces the quadcopter into the
obstacle. A quadcopter is considered to be touching a boundary or obstacle if
the center-of-mass position of the quadcopter has an L-infinity distance of
**40 cm** or less to any point on any boundary of an obstacle.

### Maximum Velocity
Proposed trajectories must not specify flight in any direction with a velocity
magnitude greater than or equal to **2.0 m/s**. If the Mediation Layer
determines that a proposed trajectory will exceed this bound, the trajectory
is rejected and the quadcopter will continue to follow the most recent valid
trajectory.

### Maximum Acceleration
Proposed trajectories must not specify flight in any direction with an
acceleration magnitude greater than **0.4 m/s^2**. If the Mediation Layer
determines that a proposed trajectory will exceed this bound, the trajectory
is rejected and the quadcopter will continue to follow the most recent, valid
trajectory.

### Maximum Time
Teams will have up to 5 minutes (300 seconds) to complete their
balloon-popping mission. The timer starts as soon as a team's autonomy
protocol is engaged. At the end of the time period, the quadcopter is frozen
and forced to land. The attempt will be scored as if the quadcopter returned
to the starting position by the maximum time.

