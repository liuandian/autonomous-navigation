include "2d_online.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
MAP_BUILDER.num_background_threads = 8
POSE_GRAPH.optimize_every_n_nodes = 80

return options
