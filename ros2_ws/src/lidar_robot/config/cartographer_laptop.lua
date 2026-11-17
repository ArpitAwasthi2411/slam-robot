-- Cartographer on the laptop riding on the robot (scripts/laptop_slam.py, docs/laptop_slam.md).
-- Same as the Pi's cartographer_odom.lua (corridor tuning, strict loop closures) but the laptop has the CPU
-- for finer steps and more loop-closure tries. Everything not changed here comes from cartographer_odom.lua.

include "cartographer_odom.lua"

MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- a node every 10 cm / 2 degrees (the Pi needs 20 cm / 3 degrees to keep up)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.0)

POSE_GRAPH.optimize_every_n_nodes = 90                     -- ~9 m of driving between whole-map optimisations
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3         -- Cartographer's default (the Pi uses 0.15)
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

return options
