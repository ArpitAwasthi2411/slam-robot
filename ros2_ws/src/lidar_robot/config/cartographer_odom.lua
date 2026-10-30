-- Cartographer 2D: LiDAR + ESP32 wheel odometry (preferred mode), live mapping on the Pi.
-- For the most accurate map, also record the run and rebuild it with cartographer_mapping_hq.lua (scripts/map_offline.sh).
-- TF:  map --(cartographer)--> odom --(esp32_bridge)--> base_link --(static)--> laser

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",          -- the bridge publishes odom->base_link
  odom_frame = "odom",
  provide_odom_frame = false,        -- ...so Cartographer must NOT also publish it
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,      -- 20 Hz: the navigator takes the fast part from the wheels (50 Hz) and only the slow map->odom correction from here; 200 Hz overloaded the Pi
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2            -- 2 of the Pi 4's 4 cores: the rest stay free for the bridge + navigator

TRAJECTORY_BUILDER_2D.use_imu_data = false           -- the gyro is already fused into /odom by the bridge
TRAJECTORY_BUILDER_2D.min_range = 0.15            -- ignore hits on the robot body
TRAJECTORY_BUILDER_2D.max_range = 10.0            -- A1 reaches ~12 m: the far end of a corridor is often the only
                                                  -- thing that pins down where along the corridor the robot is
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0   -- big open halls: don't carve a 10 m white disc
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90     -- bigger submaps = more wall per submap = steadier in corridors

-- LONG CORRIDORS (docs/mapping.md). Two parallel walls tell the scan matcher the robot's sideways position and
-- heading very well, but NOTHING about how far along the corridor it is. Only the wheels know that. So:
--  * no online correlative search: it picks the best-looking match anywhere in its window and in a corridor
--    "best" can be 10 cm further along -> corridors come out short / long / kinked
--  * a strong pull towards the odometry prediction for translation, a medium one for rotation (gyro)
-- This needs good odometry: encoders counting, ticks_per_rev measured over 5 m, gyro working + gyro_scale.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0   -- standing still adds no new nodes (0.5 grew the graph + CPU load every minute)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- the whole-map optimisation: trust wheel distance AND gyro heading between nodes (defaults; the old 1e3
-- for rotation was for slipping wheels without a gyro and let corridors bend)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2   -- leave cores for the Python nodes (all 4 busy = stale odometry = "position lost")
-- loop closures: corridors with identical doors look alike. A wrong closure folds the map, a missed one only
-- leaves a little drift, so be strict.
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

return options
