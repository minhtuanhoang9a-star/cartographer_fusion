include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  visual_constraint_mode = "GUIDED_LIDAR_5B",
  visual_sequential_weight = 1.0,
  visual_loop_weight = 1.0,
  guided_search_window = 100,
}
return options
