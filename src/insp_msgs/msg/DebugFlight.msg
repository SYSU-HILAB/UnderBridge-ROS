std_msgs/Header header

float64 traj_begin_time  # out
float64 traj_cur_time    # in
bool if_traj_end         # in
bool if_gimbal_beg       # out
bool if_gimbal_end       # out
float64 err              # in
float64 vel              # in
float64 ttt              # in
float64 traj_cur_time_minus_t # in
bool gimbal_keep_yaw     # out

geometry_msgs/Point traj_pt0

bool have_obs
uint8 check_obs_cnt

geometry_msgs/Point foot_pt
geometry_msgs/Point crt_pt

float64 tip_wait_dist

uint8 zoom_wait_cnt
uint8 focus_wait_cnt
uint8 tip_focus_wait_cnt
uint8 tip_focus_cnt