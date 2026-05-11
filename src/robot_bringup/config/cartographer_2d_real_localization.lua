include "cartographer_2d_real.lua"

options.published_frame = "odom"
options.provide_odom_frame = false
options.use_odometry = true

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 20

return options
