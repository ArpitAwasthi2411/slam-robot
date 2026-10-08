-- Cartographer 2D: LiDAR + ESP32 wheel odometry (preferred mode)
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
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4            -- Pi 4 has 4 cores

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.15            -- ignore hits on the robot body
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0   -- big open halls: don't carve an 8 m white disc
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- odometry is decent, so a small correlative search is enough to absorb wheel slip
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- trust the LiDAR more than the wheels for rotation (wheels slip when turning)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4   -- silences the "7 threads" warning
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
