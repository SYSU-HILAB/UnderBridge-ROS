std_msgs/Header header

# point(x, y, z) in tower line, random
float64 tower_x
float64 tower_y
float64 tower_z
# vector(vx, vy, vz) of tower line, down -> up
float64 tower_vx
float64 tower_vy
float64 tower_vz
# dist between local_position and foot_point in line
float64 tower_dist
# vector(nx, ny, nz) of tower, local_position -> tower
float64 tower_nx
float64 tower_ny
float64 tower_nz


# point(x, y, z) of hub center
float64 hub_x
float64 hub_y
float64 hub_z
# vector(vx, vy, vz) of hub plane, local_position -> hub
float64 hub_vx
float64 hub_vy
float64 hub_vz


# point(x, y, z) in blade line
float64 blade_x
float64 blade_y
float64 blade_z
# vector(vx, vy, vz) of blade line, track direction
float64 blade_vx
float64 blade_vy
float64 blade_vz
# dist between local_position and foot_point in line
float64 blade_dist
# vector(nx, ny, nz) of blade, local_position -> blade
float64 blade_nx
float64 blade_ny
float64 blade_nz
# point(x, y, z), foot point of blade
float64 blade_fx
float64 blade_fy
float64 blade_fz
# vector(vx, vy, vz), original blade dir vec
float64 blade_ox
float64 blade_oy
float64 blade_oz
# vector(cx, cy, cz), correct foot point of blade by ptcloud radius search
float64 blade_cx
float64 blade_cy
float64 blade_cz

float64 tip_x
float64 tip_y
float64 tip_z

bool tip_arrive


bool tower_legal
bool hub_legal
bool blade_legal

bool ris_to_adj
bool adj_to_loc
bool loc_dtc_fns
