-- Cartographer 2D: HIGH-QUALITY OFFLINE MAPPING from a recorded run (scripts/map_offline.sh build <name>).
-- Same sensor setup as the live mapping (cartographer_odom.lua), but with no real-time limit: all cores,
-- many more loop-closure checks, more optimisation, and a long final optimisation over the whole floor.
-- Runs on the Pi with the robot service stopped (a 15 min recording takes roughly 10-30 min), or on a laptop.

include "cartographer_odom.lua"

MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimize_every_n_nodes = 20               -- optimise twice as often while building
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5   -- look for loop closures at more nodes (default 0.3)
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.max_num_final_iterations = 400            -- the final whole-map optimisation (default 200)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

return options
