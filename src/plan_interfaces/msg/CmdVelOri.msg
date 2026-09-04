std_msgs/Header header

# base_link 坐标系下的速度与朝向指令
float64 v_x
float64 v_y
float64 v_yaw
float64 ori_yaw

# MPC 前馈力和力矩
float64 f_x
float64 f_y
float64 tau_z

int32 follow_chassis
int32 cross_hole_deform
