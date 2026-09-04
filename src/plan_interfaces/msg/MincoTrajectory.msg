# 轨迹发布信息
std_msgs/Header header

# 起点和终点的 position、velocity、acceleration，顺序为 x、y、z
float64[3] head_position
float64[3] head_velocity
float64[3] head_acceleration
float64[3] tail_position
float64[3] tail_velocity
float64[3] tail_acceleration

# 分段数、内部连接点和每段持续时间
int32 piece_n
geometry_msgs/Point[] points
int32[] is_point_spacious
float64[] times

# 25 cm 狗洞相交区间，时间相对于本条轨迹起点
float64[] cross_hole_enter_times
float64[] cross_hole_exit_times
float64[] cross_hole_active_start_times
float64[] cross_hole_active_end_times
int32[] cross_hole_black_pixel_counts
