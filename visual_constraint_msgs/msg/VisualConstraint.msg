# header.stamp: set equal to stamp_j (emission timestamp)
# header.frame_id: camera optical frame in which relative_pose is expressed
std_msgs/Header header

builtin_interfaces/Time stamp_i
builtin_interfaces/Time stamp_j

uint64 keyframe_id_i
uint64 keyframe_id_j
uint64 map_id

# Relative pose of KeyFrame j expressed in KeyFrame i frame:
# T_{Ki, Kj} = T_{Ki, W} * inv(T_{Kj, W})
geometry_msgs/Pose relative_pose

uint8 SEQUENTIAL=0
uint8 LOOP_CLOSURE=1
uint8 type

uint32 covisible_points
uint32 sim3_inliers