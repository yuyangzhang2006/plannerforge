# Planner implementation and parameters

The existing `/goal_pose`, `/grid_map`, `/Odometry_to_base_link`, and
`/minco_trajectory` interfaces are unchanged. The demo map node additionally
publishes `/dynamic_obstacles` as `plan_interfaces/msg/DynamicObstacleArray` so
the planner can perform constant-velocity prediction instead of inferring
velocity from occupancy snapshots.

## Search and map

- `RobotRadiusNormal`: ordinary-navigation footprint radius in metres.
- `RobotRadiusCompact`: compact footprint radius used inside configured cross-hole regions.
- `StaticSafetyMargin`: margin added to both footprint radii before hard inflation.
- `TurnCostWeight`: equivalent-distance weight per radian of direction change.
- `SafetyCostWeight`: soft-clearance penalty weight; zero disables the penalty.
- `SafetyCostDistance`: clearance distance in metres at which the soft penalty becomes zero.
- `SpecialRegionCostWeight`: additional equivalent-distance multiplier inside cross-hole cells.
- `CrossHoleRegions`: flat `xmin,xmax,ymin,ymax` tuples in the map frame. Omit the parameter when no region is configured because ROS 2 does not accept an untyped empty YAML array.

The planner derives normal/compact hard occupancy and an 8-neighbour distance
field from each `OccupancyGrid`. Unknown and non-zero cells are treated as
occupied. Cross-hole regions use compact inflation but remain semantically
labelled for search cost and interval extraction.

## Corridor

- `CorridorObstacleSearchRadius`: maximum local inflation extent in metres.
- `CorridorFiriIterations`: bounded inflation passes; default `4`.
- `CorridorMinOverlap`: required overlap seed diameter at route joints.
- `CorridorMergeTolerance`: numerical half-space containment tolerance.

Corridors are ordered oriented convex boxes represented as `A*x <= b`. Each is
grown from its complete path segment and sampled conservatively against the
hard map. If sparse-seed construction fails, the existing A* points are restored
and corridor construction is retried without rerunning the global search.

## MINCO trajectory

- `NominalVel`: nominal speed used for initial segment durations.
- `MaxVelMag`: deprecated compatibility alias for `NominalVel`.
- `MaxPlanVel`, `MaxPlanAcc`: planning-envelope speed and acceleration limits.
- `MinSegmentDuration`: lower duration bound for numerical stability.
- `WeightTime`, `WeightVelPenalty`, `WeightAccPenalty`: backend objective weights.
- `OptimizationMaxIterations`, `OptimizationTimeBudgetMs`: projected-search bounds.

The backend keeps `MINCO_S3NU`, adjusts intermediate points only inside the
ordered adjacent-corridor overlap, and applies positive time scaling to meet the
planning envelope. A result is rejected if sampled pieces leave their assigned
corridor, collide, contain invalid data, or exceed the envelope.

## Dynamic obstacles

- `PredictionHorizonSec`: future trajectory/prediction window.
- `DynamicSafetyMargin`: extra separation added to obstacle and robot radii.
- `DynamicObstacleTimeoutSec`: maximum accepted observation age.
- `ReplanCooldownMs`: minimum interval between accepted replans.

Each dynamic observation contains position, velocity, box size or circle radius,
timestamp, shape, and validity duration. Only predicted conflicts with the active
future trajectory trigger replanning. A failed replan retains the old trajectory
only if the conflict check says it is still safe; otherwise the planner clears its
active-plan state and emits an error for the controller/supervisor.

## Cross-hole timing

- `CrossHoleShapeTime`: preparation time before actual entry.
- `CrossHoleRecoveryTime`: recovery time after actual exit.
- `CrossHoleTimeMargin`: additional time margin on both sides.

Intervals are extracted from actual continuous-trajectory samples inside the
semantic region. Near passes do not create intervals, and stale intervals are
cleared whenever a new trajectory is built.

## Diagnostics and current limits

Every successful plan logs A*, corridor, MINCO, and total runtime, discrete path
length, trajectory duration, minimum clearance, maximum velocity, and maximum
acceleration. The current corridor inflation is a bounded 2D FIRI-style oriented
box implementation rather than a general maximum-volume polygon solver. Dynamic
box collision prediction uses a conservative circumscribed radius. The existing
external message set has no explicit stop command, so unsafe replan failure is
exposed through the active-plan state and ROS error log; integration should map
that condition to the robot controller's stop interface.
