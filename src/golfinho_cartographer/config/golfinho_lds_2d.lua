
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.00025

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30. -- Decrease
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 20.0 -- Increase
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150 -- Decrease

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 30. -- Decrease
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 20.0 -- Increase
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 150 -- Decrease

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)


POSE_GRAPH.optimization_problem.huber_scale = 1e2


return options













-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50. -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 10.8 -- Increase
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1 -- Increase

------------Global SLAM------------
-- POSE_GRAPH.optimize_every_n_nodes = 1 -- Decrease
-- MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
-- POSE_GRAPH.global_sampling_ratio = 0.001 -- Decrease
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.01 -- Decrease
-- POSE_GRAPH.constraint_builder.min_score = 0.75 -- Increase
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 20 -- Increase
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50 -- Decrease


-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1 -- Decrease

-- POSE_GRAPH.constraint_builder.min_score = 0.065
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.07

-- POSE_GRAPH.optimize_every_n_nodes = 0
