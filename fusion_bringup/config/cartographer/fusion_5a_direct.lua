include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  visual_constraint_mode = "DIRECT_4WAY_5A",
  visual_sequential_weight = 1.0,
  visual_loop_weight = 0.5,
}
return options
