#include "wind_turbine_insp/flight_node.h"


using namespace wind_turbine_insp;


/*################*/
/*  Constructor   */
/*################*/


FlightNode::FlightNode(ros::NodeHandle nh): nh_(nh) {

  initVariable();

  initServiceClient();
  initSubscriber();

  ros::Duration(1.0).sleep();

  initServiceServer();
  initPublisher();



  initTimer();
}
 

/*##############*/
/*  Destructor  */
/*##############*/

FlightNode::~FlightNode(){

}

bool FlightNode::initVariable(){

  nh_.getParam("flight_node/track_mode",   track_mode_);
  nh_.getParam("flight_node/single_side",   single_side_);
  nh_.getParam("flight_node/enable_planning",   enable_planning_);

  nh_.getParam("flight_node/fan_tower_height",   fan_tower_height_);
  nh_.getParam("flight_node/fan_blade_length",   fan_blade_length_);
  nh_.getParam("flight_node/fan_blade_width",   fan_blade_width_);

  nh_.getParam("flight_node/rate_stage_mode_update", rate_sm_update_);
  nh_.getParam("flight_node/rate_flight_control",    rate_flight_ctrl_);
  nh_.getParam("flight_node/rate_waypoint_sample",   rate_wps_sample_);
  nh_.getParam("flight_node/rate_set_gimbal_position",   rate_set_gim_posi_);
  nh_.getParam("flight_node/rate_set_camera_zoom",   rate_set_camera_zoom_);
  nh_.getParam("flight_node/rate_msg_to_vision",   rate_msg_to_vision_);


  flag_if_takeoff_ = false;
  flag_if_land_ = false;

  last_info_time_ = ros::Time::now();
  loc_beg_time_ = ros::Time::now();
  traj_begin_time_ = ros::Time::now();

  nh_.getParam("flight_node/kp_yaw",   kp_yaw_);
  nh_.getParam("flight_node/ki_yaw",   ki_yaw_);
  nh_.getParam("flight_node/kd_yaw",   kd_yaw_);
  nh_.getParam("flight_node/max_vel_yaw",   mv_yaw_);
  nh_.getParam("flight_node/max_acc_yaw",   ma_yaw_);

  nh_.getParam("flight_node/yaw_ctrl_in_deg",   yaw_ctrl_in_deg_);

  
  ctrl_yaw_ = px4_ctrl::PIDctrl(rate_flight_ctrl_);

  if(yaw_ctrl_in_deg_)
    ctrl_yaw_.setParams(kp_yaw_, ki_yaw_, kd_yaw_, mv_yaw_, ma_yaw_);
  else
    ctrl_yaw_.setParams(DEG2RAD(kp_yaw_), DEG2RAD(ki_yaw_), DEG2RAD(kd_yaw_), DEG2RAD(mv_yaw_), DEG2RAD(ma_yaw_));


  set_stage_mode_.request.source = insp_msgs::SetStageMode::Request::SOURCE_CODE;

  err_yaw_ = 0;
  tgt_yaw_ = 0;

  desire_gim_vec_ = Eigen::Vector3d(0, 0, 0);
  gim_cpst_vec_ = Eigen::Vector3d(0, 0, 0);

  gimbal_set_vec_msg_.x = desire_gim_vec_(0);
  gimbal_set_vec_msg_.y = desire_gim_vec_(1);
  gimbal_set_vec_msg_.z = desire_gim_vec_(2);


  nh_.getParam("flight_node/detect_dist",   detect_dist_);
  nh_.getParam("flight_node/track_dist",   track_dist_);

  nh_.getParam("flight_node/desire_dist_rise",   desire_dist_rise_);
  nh_.getParam("flight_node/desire_dist_track",   desire_dist_track_);
  nh_.getParam("flight_node/loc_rotate_radius",   loc_rotate_radius_);
  nh_.getParam("flight_node/loc_rotate_period",   loc_rotate_period_);

  desire_posi_ = Eigen::Vector3d(0, 0, 5);
  desire_yaw_vec_ = Eigen::Vector2d(1, 0);

  last_legal_yaw_vec_ = Eigen::Vector3d(desire_yaw_vec_(0), desire_yaw_vec_(1), 0);
  last_legal_gim_vec_ = desire_gim_vec_;
  last_dir_vec_track_ = Eigen::Vector3d::Zero();

  visPtr_ = std::make_shared<visualization::Visualization>(nh_);
  
  nh_.getParam("flight_node/arrow_width",   arrow_width_);
  nh_.getParam("flight_node/arrow_history",      arrow_history_);

  velo_history_ = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>();
  gimb_history_ = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>();

  history_time_ = ros::Time::now();

  str_topic_hst_velo_ = "/wind_turbine_insp/marker_hst_vel";
  str_topic_hst_gimb_ = "/wind_turbine_insp/marker_hst_gim";

  str_topic_wps_ = "/wind_turbine_insp/marker_wps";
  str_topic_vel_ = "/wind_turbine_insp/marker_vel";
  str_topic_yaw_ = "/wind_turbine_insp/marker_yaw";
  str_topic_gim_ = "/wind_turbine_insp/marker_gim";
  str_topic_pts_ = "/wind_turbine_insp/marker_pts";
  str_topic_ftr_ = "/wind_turbine_insp/marker_ftr";
  str_topic_cps_ = "/wind_turbine_insp/marker_cps";
  str_topic_crt_ = "/wind_turbine_insp/marker_crt";

  nh_.getParam("flight_node/traj_pt_size",   traj_pt_size_);

  next_track_blade_ = 1;

  dist_to_hub_front_ = 0;

  if_fan_tip_arrive_ = false;

  if_save_tip_front_posi_ = false;

  if_emergency_save_tip_ = false;

  log_state_ = 0;

  for(int i = 0; i < 3; i++)
  {
    have_sample_first_wp_[i] = false; 
  }

  nh_.getParam("flight_node/thold_arrive_target",      thold_arrive_target_);
  nh_.getParam("flight_node/thold_keep_yaw_dist",      thold_keep_yaw_dist_);
  
  nh_.getParam("flight_node/thold_update_yaw",   thold_update_yaw_);

  nh_.getParam("flight_node/thold_enough_vel_slow",   thold_enough_vel_slow_);
  nh_.getParam("flight_node/thold_enough_yaw_accu",   thold_enough_yaw_accu_);

  nh_.getParam("flight_node/dist_enable_sample_track_wps",   dist_enable_sample_track_wps_);
  nh_.getParam("flight_node/deep_root_offset",   deep_root_offset_);

  nh_.getParam("flight_node/traj_vel_track",   traj_vel_track_);
  nh_.getParam("flight_node/traj_vel_turn",   traj_vel_turn_);
  nh_.getParam("flight_node/traj_vel_gimbal",   traj_vel_gimbal_);
  nh_.getParam("flight_node/traj_delay_time",   traj_delay_time_);
  nh_.getParam("flight_node/traj_down_angle",   traj_down_angle_);

  traj_delay_duration_ = ros::Duration(traj_delay_time_);

  nh_.getParam("flight_node/video_record_mode",   video_record_mode_);
  nh_.getParam("flight_node/if_include_tower",   if_include_tower_);
  nh_.getParam("flight_node/zoom_factor_init",   zoom_factor_init_);
  nh_.getParam("flight_node/camera_zoom_k",   camera_zoom_k_);

  camera_zoom_factor_ = zoom_factor_init_;

  nh_.getParam("flight_node/shoot_photo_coverage",   shoot_photo_coverage_);

  nh_.getParam("flight_node/enable_emergency_terminal",   enable_emergency_terminal_);
  nh_.getParam("flight_node/future_occupied_dist",   future_occupied_dist_);
  nh_.getParam("flight_node/consequent_occupied_thold",   consequent_occupied_thold_);

  nh_.getParam("flight_node/tip_focus_thold",   tip_focus_thold_);
  nh_.getParam("flight_node/tip_focus_gap",   tip_focus_gap_);
  nh_.getParam("flight_node/gap_focus",   gap_focus_);
  nh_.getParam("flight_node/gap_zoom",   gap_zoom_);
  nh_.getParam("flight_node/focus_time_root",   focus_time_root_);
  nh_.getParam("flight_node/focus_time_tip",   focus_time_tip_);
  nh_.getParam("flight_node/zoom_time_root",   zoom_time_root_);
  nh_.getParam("flight_node/zoom_time_tip",   zoom_time_tip_);
  nh_.getParam("flight_node/enable_ptcloud_correct",   enable_ptcloud_correct_);




  if_gimbal_end_ = false;
  if_gimbal_beg_ = false;

  gimbal_reset_ = true;
  gimbal_keep_yaw_ = false;     

  hub_front_posi_ = Eigen::Vector3d::Zero();
  hub_back_posi_  = Eigen::Vector3d::Zero();

  for(int i = 0; i < 3; i++)
  {
    tip_side1_posi_[i]  = Eigen::Vector3d::Zero();
    tip_side3_posi_[i]  = Eigen::Vector3d::Zero();
    tip_side4_posi_[i]  = Eigen::Vector3d::Zero();
    tip_side2_posi_[i]  = Eigen::Vector3d::Zero();
    tip_up_posi_[i]     = Eigen::Vector3d::Zero();
    tip_down_posi_[i]   = Eigen::Vector3d::Zero();
    root_side1_posi_[i] = Eigen::Vector3d::Zero();
    root_side3_posi_[i] = Eigen::Vector3d::Zero();
    root_side4_posi_[i] = Eigen::Vector3d::Zero();
    root_side2_posi_[i] = Eigen::Vector3d::Zero();
    root_up_posi_[i]    = Eigen::Vector3d::Zero();
    root_down_posi_[i]  = Eigen::Vector3d::Zero();
    tip_blade_posi_[i]  = Eigen::Vector3d::Zero();
    root_blade_posi_[i] = Eigen::Vector3d::Zero();
  }

  key_positions_.reserve(36);
  for(int i = 0; i < 36; i++)
    key_positions_.push_back(Eigen::Vector3d::Zero());

  for(int i = 0; i < 3; i++)
  {
    LineTrajPtr_0_[i] = std::make_shared<line_traj::LineTraj>(nh_);
    LineTrajPtr_0_[i] -> reset();

    LineTrajPtr_1_[i] = std::make_shared<line_traj::LineTraj>(nh_);
    LineTrajPtr_1_[i] -> reset();

    LineTrajPtr_2_[i] = std::make_shared<line_traj::LineTraj>(nh_);
    LineTrajPtr_2_[i] -> reset();

    LineTrajPtr_3_[i] = std::make_shared<line_traj::LineTraj>(nh_);
    LineTrajPtr_3_[i] -> reset();

    LineTrajPtr_4_[i] = std::make_shared<line_traj::LineTraj>(nh_);
    LineTrajPtr_4_[i] -> reset();

    CircleTrajPtr_1_[i] = std::make_shared<circle_traj::CircleTraj>(nh_);
    CircleTrajPtr_2_[i] = std::make_shared<circle_traj::CircleTraj>(nh_);
    CircleTrajPtr_3_[i] = std::make_shared<circle_traj::CircleTraj>(nh_);
    CircleTrajPtr_4_[i] = std::make_shared<circle_traj::CircleTraj>(nh_);

    pts_traj_0_[i] = std::vector<Eigen::Vector3d>();
    pts_traj_1_[i] = std::vector<Eigen::Vector3d>();
    pts_traj_2_[i] = std::vector<Eigen::Vector3d>();
    pts_traj_3_[i] = std::vector<Eigen::Vector3d>();
    pts_traj_4_[i] = std::vector<Eigen::Vector3d>();
    
  }

  last_shoot_time_ = ros::Time::now();

  track_index_msg_.blade_index = 0;
  track_index_msg_.track_plane = 1;

  consequent_occupied_count_ = 0;
  if_pre_occupied_ = false;
  last_terminate_req_time_ = ros::Time::now();

  blade_width_ = fan_blade_width_;

  zoom_wait_cnt_ = 0;
  focus_wait_cnt_ = 0;
  tip_focus_wait_cnt_ = 0;
  tip_focus_cnt_ = 0;

  correct_posi_ = Eigen::Vector3d(0, 0, 0);
  for(int i = 0; i < 3; i++)
  {
    tip_posi_true_[i] = Eigen::Vector3d(0, 0, 0);
    tip_wait_dist_[i] = 0.0;
  }

  front_zoom_cnt_ = 0;

  printf("--------------------\n");
  printf("Loading Params Start\n");

  printf("track_mode: %d\n", track_mode_);
  printf("single_side: %d\n", single_side_);
  printf("enable_planning: %d\n", enable_planning_);
  printf("fan_tower_height: %.2f\n", fan_tower_height_);
  printf("fan_blade_length: %.2f\n", fan_blade_length_);
  printf("fan_blade_width: %.2f\n", fan_blade_width_);
  printf("rate_stage_mode_update: %.2f\n", rate_sm_update_);
  printf("rate_flight_control: %.2f\n", rate_flight_ctrl_);
  printf("rate_waypoint_sample: %.2f\n", rate_wps_sample_);
  printf("rate_set_gimbal_position: %.2f\n", rate_set_gim_posi_);
  printf("rate_set_camera_zoom: %.2f\n", rate_set_camera_zoom_);
  printf("rate_msg_to_vision: %.2f\n", rate_msg_to_vision_);

  printf("kp_yaw: %.2f\n", kp_yaw_);
  printf("ki_yaw: %.2f\n", ki_yaw_);
  printf("kd_yaw: %.2f\n", kd_yaw_);
  printf("max_vel_yaw: %.2f\n", mv_yaw_);
  printf("max_acc_yaw: %.2f\n", ma_yaw_);
  printf("detect_dist: %.2f\n", detect_dist_);
  printf("track_dist: %.2f\n", track_dist_);
  printf("desire_dist_rise: %.2f\n", desire_dist_rise_);
  printf("desire_dist_track: %.2f\n", desire_dist_track_);  
  printf("arrow_width: %.2f\n", arrow_width_);
  printf("arrow_history: %d\n", arrow_history_);
  printf("traj_pt_size: %.2f\n", traj_pt_size_);
  printf("thold_arrive_target: %.2f\n", thold_arrive_target_);
  printf("thold_update_yaw: %.2f\n", thold_update_yaw_);
  printf("yaw_ctrl_in_deg: %d\n", yaw_ctrl_in_deg_);
  printf("thold_enough_vel_slow: %.2f\n", thold_enough_vel_slow_);
  printf("thold_enough_yaw_accu: %.2f\n", thold_enough_yaw_accu_);
  printf("dist_enable_sample_track_wps: %.2f\n", dist_enable_sample_track_wps_);
  printf("deep_root_offset: %.2f\n", deep_root_offset_);
  printf("traj_vel_track: %.2f\n", traj_vel_track_);
  printf("traj_vel_turn: %.2f\n", traj_vel_turn_);
  printf("traj_vel_gimbal: %.2f\n", traj_vel_gimbal_);
  printf("traj_delay_time: %.2f\n", traj_delay_time_);
  printf("traj_down_angle: %.2f\n", traj_down_angle_);
  printf("video_record_mode: %d\n", video_record_mode_);
  printf("if_include_tower: %d\n", if_include_tower_);
  printf("zoom_factor_init: %.2f\n", zoom_factor_init_);
  printf("camera_zoom_k: %.2f\n", camera_zoom_k_);
  printf("shoot_photo_coverage: %.2f\n", shoot_photo_coverage_);
  printf("enable_emergency_terminal: %d\n", enable_emergency_terminal_);
  printf("future_occupied_dist: %.2f\n", future_occupied_dist_);
  printf("consequent_occupied_thold: %d\n", consequent_occupied_thold_);
  printf("tip_focus_thold: %d\n", tip_focus_thold_);
  printf("tip_focus_gap: %.2f\n", tip_focus_gap_);
  printf("gap_focus: %.2f\n", gap_focus_);
  printf("gap_zoom: %.2f\n", gap_zoom_);
  printf("focus_time_root: %.2f\n", focus_time_root_);
  printf("focus_time_tip: %.2f\n", focus_time_tip_);
  printf("zoom_time_root: %.2f\n", zoom_time_root_);
  printf("zoom_time_tip: %.2f\n", zoom_time_tip_);
  printf("enable_ptcloud_correct: %d\n", enable_ptcloud_correct_);




  printf("Loading Params Finish\n");
  printf("---------------------\n");

  

  ROS_INFO_STREAM("Flight_node: init Variable");


  return true;
}

bool FlightNode::initSubscriber(){

  stage_mode_subscriber_         = nh_.subscribe<insp_msgs::StageMode>("/wind_turbine_insp/stage_mode", 2, &FlightNode::getStageModeCallback, this);
  
  local_odom_subscriber_         = nh_.subscribe<nav_msgs::Odometry>("/wind_turbine_insp/local_odom", 2, &FlightNode::getLocalOdomCallback, this);
  
  ptcloud_key_subscriber_        = nh_.subscribe<insp_msgs::PtCloudKey>("/wind_turbine_insp/ptcloud_key", 2, &FlightNode::getPtcloudKeyCallback, this);
  
  traj_velo_subscriber_          = nh_.subscribe<geometry_msgs::Twist>("/wind_turbine_insp/traj_velo_ctrl", 2, &FlightNode::trajVeloCtrlCallback, this);

  gimbal_compensate_subscriber_  = nh_.subscribe<insp_msgs::GimbalCompensate>("/wind_turbine_insp/gimbal_compensate", 2, &FlightNode::gimbalCompensateCallback, this);
  
  ROS_INFO_STREAM("Flight_node: init Subscriber");

  return true;
}

/*###################################*/
/* ServiceServer initialize function */
/*###################################*/

bool FlightNode::initServiceClient(){

  flight_task_control_client_ = nh_.serviceClient<insp_msgs::FlightTaskControl>("/wind_turbine_insp/flight_task_control");
  
  set_stage_mode_client_      = nh_.serviceClient<insp_msgs::SetStageMode>("/wind_turbine_insp/set_stage_mode");

  occupied_check_client_      = nh_.serviceClient<insp_msgs::OccupyCheck>("/detection/Occupied_check");

  emergency_terminate_client_ = nh_.serviceClient<insp_msgs::EmergencyTerminate>("/wind_turbine_insp/emergency_terminate");





  correct_foot_point_client_    = nh_.serviceClient<insp_msgs::FootPointCorrect>("/detection/cross_section_center");

  true_tip_point_client_        = nh_.serviceClient<insp_msgs::BladeTipQuery>("/wind_turbine_insp/blade_tip_query");

  ROS_INFO_STREAM("Flight_node: init ServiceClient");

  return true;
}

bool FlightNode::initPublisher(){

  set_velo_publisher_                  = nh_.advertise<insp_msgs::VeloCmd>("/wind_turbine_insp/set_velo", 2);
  
  wps_servo_publisher_                 = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_common", 2);
  wps_traj_publisher_                  = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_minco", 1);
  
  send_force_hover_signal_publisher_   = nh_.advertise<std_msgs::Bool>("/wind_turbine_insp/force_hover", 2);

  set_gimbal_publisher_                = nh_.advertise<insp_msgs::GimbalSet>("/wind_turbine_insp/set_gimbal_vec", 2);

  camera_set_zoom_para_publisher_      = nh_.advertise<std_msgs::Float32>("/wind_turbine_insp/set_camera_zoom", 2);
  camera_set_focus_para_publisher_     = nh_.advertise<insp_msgs::CameraFocusPoint>("/wind_turbine_insp/set_camera_focus", 2);
  camera_record_video_publisher_       = nh_.advertise<std_msgs::Bool>("/wind_turbine_insp/camera_record_video", 2);

  camera_shoot_photo_publisher_        = nh_.advertise<std_msgs::Bool>("/wind_turbine_insp/camera_shoot_photo", 2);



  blade_distance_publisher_            = nh_.advertise<std_msgs::Float32>("/wind_turbine_insp/blade_distance", 2);

  blade_target_point_publisher_        = nh_.advertise<geometry_msgs::Point>("/wind_turbine_insp/blade_target_point", 2);

  blade_width_publisher_               = nh_.advertise<std_msgs::Float32>("/wind_turbine_insp/blade_width", 2);

  blade_side_state_publisher_          = nh_.advertise<insp_msgs::BladeSideState>("/wind_turbine_insp/blade_side_state", 2);

  node_status_publisher_               = nh_.advertise<std_msgs::Int32>("/wind_turbine_insp/node_status", 2);

  log_state_publisher_                 = nh_.advertise<std_msgs::Int32>("/wind_turbine_insp/log_state", 2);

  track_index_publisher_               = nh_.advertise<insp_msgs::TrackIndex>("/wind_turbine_insp/track_index", 2);

  debug_flight_publisher_          = nh_.advertise<insp_msgs::DebugFlight>("/wind_turbine_insp/debug_flight", 2);

  ROS_INFO_STREAM("Flight_node: init Publisher");

  return true;
}


/*###############################*/
/*  subscriber Callback function */
/*################################*/

void FlightNode::getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg){
  stage_mode_= *msg;
  if(msg -> stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
  {
    ros::Duration(1.0).sleep();
    ros::shutdown();
  }
}
void FlightNode::getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  PointMsgToXYZVec((msg -> pose).pose.position, local_posi_);
  QuatnMsgToRPYVec((msg -> pose).pose.orientation, local_euler_);
  VectorMsgToEigen((*msg).twist.twist.linear, local_velo_);
}
void FlightNode::trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg){
  set_velo_.x = msg -> linear.x;
  set_velo_.y = msg -> linear.y;
  set_velo_.z = msg -> linear.z;
}
void FlightNode::getPtcloudKeyCallback(const insp_msgs::PtCloudKey::ConstPtr& msg){
  ptcloud_key_ = *msg;

  tower_pt_posi_  = Eigen::Vector3d(ptcloud_key_.tower_x,  ptcloud_key_.tower_y,  ptcloud_key_.tower_z);
  tower_dir_vec_  = Eigen::Vector3d(ptcloud_key_.tower_vx, ptcloud_key_.tower_vy, ptcloud_key_.tower_vz).normalized();
  tower_vrt_dst_  = ptcloud_key_.tower_dist;
  tower_nor_vec_  = Eigen::Vector3d(ptcloud_key_.tower_nx, ptcloud_key_.tower_ny, ptcloud_key_.tower_nz).normalized();
  hub_pt_posi_    = Eigen::Vector3d(ptcloud_key_.hub_x,    ptcloud_key_.hub_y,    ptcloud_key_.hub_z);
  hub_nor_vec_    = Eigen::Vector3d(ptcloud_key_.hub_vx,   ptcloud_key_.hub_vy,   ptcloud_key_.hub_vz).normalized();
  blade_pt_posi_  = Eigen::Vector3d(ptcloud_key_.blade_x,  ptcloud_key_.blade_y,  ptcloud_key_.blade_z);
  blade_dir_vec_  = Eigen::Vector3d(ptcloud_key_.blade_vx, ptcloud_key_.blade_vy, ptcloud_key_.blade_vz).normalized();
  blade_vrt_dst_  = ptcloud_key_.blade_dist;
  blade_nor_vec_  = Eigen::Vector3d(ptcloud_key_.blade_nx, ptcloud_key_.blade_ny, ptcloud_key_.blade_nz).normalized();
  blade_fpt_posi_ = Eigen::Vector3d(ptcloud_key_.blade_fx, ptcloud_key_.blade_fy, ptcloud_key_.blade_fz);
  blade_ori_vec_  = Eigen::Vector3d(ptcloud_key_.blade_ox, ptcloud_key_.blade_oy, ptcloud_key_.blade_oz).normalized();
  blade_crt_posi_ = Eigen::Vector3d(ptcloud_key_.blade_cx, ptcloud_key_.blade_cy, ptcloud_key_.blade_cz);

  hub_pt_posi_ = hub_pt_posi_ + hub_nor_vec_ * deep_root_offset_;

  if(ptcloud_key_.tip_arrive)
  {
    if_fan_tip_arrive_ = true;
    if(local_posi_.z() < fan_tower_height_)
    {
      tip_blade_posi_[next_track_blade_ - 2] = Eigen::Vector3d(ptcloud_key_.tip_x, ptcloud_key_.tip_y, ptcloud_key_.tip_z);
    }
    else
    {
      tip_blade_posi_[next_track_blade_ - 2] = Eigen::Vector3d(ptcloud_key_.tip_x, ptcloud_key_.tip_y, ptcloud_key_.tip_z) + blade_dir_vec_ * 2.0;
    }
    tip_side1_posi_[next_track_blade_ - 2] = tip_blade_posi_[next_track_blade_ - 2] - hub_nor_vec_ * track_dist_;
    
    if(track_mode_ == 1 || (track_mode_ == 2 && single_side_ == 1) || track_mode_ == 4)
      if(video_record_mode_ == 0)
        cameraRecordVideo(false); // 1-track, ~blade n-1
      else if(video_record_mode_ == 2)
        cameraShootPhoto(false);

    ROS_INFO("check tip arrive");
  }
}

void FlightNode::gimbalCompensateCallback(const insp_msgs::GimbalCompensate::ConstPtr& msg)
{
  Eigen::Vector3d cpst_vec = Eigen::Vector3d(msg -> compensate_vec.x, msg -> compensate_vec.y, msg -> compensate_vec.z);
  gim_cpst_vec_ += cpst_vec;
}

void FlightNode::ForceHoverFromUp(bool hover)
{
  std_msgs::Bool tmp_msg;
  tmp_msg.data = hover;
  send_force_hover_signal_publisher_.publish(tmp_msg);
}

double FlightNode::getYawErr(double cur, double& tgt, double k){
  
  if(tgt > 0 && cur < 0 && tgt - cur > PI * 2 * k)
    tgt -= PI * 2;
  if(tgt < 0 && cur > 0 && cur - tgt > PI * 2 * k)
    tgt += PI * 2;

  return tgt - cur;
}

void FlightNode::updateTargetYaw(Eigen::Vector3d vec_to_tgt)
{
  if(Eigen::Vector2d(vec_to_tgt(0), vec_to_tgt(1)).norm() > thold_update_yaw_)
  {
    desire_yaw_vec_ = Eigen::Vector2d(vec_to_tgt(0), vec_to_tgt(1));
    last_legal_yaw_vec_ = vec_to_tgt;
  }
  else
  {
    desire_yaw_vec_ = Eigen::Vector2d(last_legal_yaw_vec_(0), last_legal_yaw_vec_(1));
  }

  tgt_yaw_ = atan2(desire_yaw_vec_(1), desire_yaw_vec_(0));
  tgt_yaw_ = std::isnan(tgt_yaw_)? last_legal_yaw_ : tgt_yaw_;
  last_legal_yaw_ = tgt_yaw_;
}


Eigen::Vector3d FlightNode::getCirclePoint3D(Eigen::Vector3d center, double radius, Eigen::Vector3d nor_vec, double t_cur, double omega)
{
  double theta = omega * t_cur;

  Eigen::Vector3d horizon_vector = Eigen::Vector3d(0, 0, 1).cross(nor_vec); // 水平方向
  horizon_vector.normalize();
  Eigen::Vector3d from_center_to_circle_vector_origin = horizon_vector;

  Eigen::AngleAxisd temp_rotation(theta, nor_vec);
  Eigen::Matrix3d temp_rotatation_matrix(temp_rotation);
  Eigen::Vector3d  from_center_to_circle_vector = temp_rotatation_matrix *  from_center_to_circle_vector_origin; // 这里可以优化

  from_center_to_circle_vector = from_center_to_circle_vector * radius;
  return (center + from_center_to_circle_vector);
}


bool FlightNode::curBladeIsToRight()
{
  Eigen::Vector3d to_right_vec = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1));

  double angle_with_right = acos(blade_ori_vec_.dot(to_right_vec) / (blade_ori_vec_.norm() * to_right_vec.norm()));

  bool ret = abs(angle_with_right) < M_PI / 2;
  
  return ret;
}


bool FlightNode::curBladeIsNormal(int idx_blade)
{
  return((blade_angle_[idx_blade] > PI / 3 && blade_angle_[idx_blade] < PI * 2 / 3) || (blade_angle_[idx_blade] > -PI * 2 / 3 && blade_angle_[idx_blade] < -PI / 3));
}

bool FlightNode::curBladeIsToDown(int idx_blade)
{
  return(blade_angle_[idx_blade] > -PI * 2 / 3 && blade_angle_[idx_blade] < -PI / 3);
}


bool FlightNode::enoughSlowDown(double velo_thold)
{
  return(abs(set_velo_.x) < velo_thold && abs(set_velo_.y) < velo_thold && abs(set_velo_.z) < velo_thold);
}

bool FlightNode::enoughYawAlign(double yaw_thold)
{
  return(abs(err_yaw_) < DEG2RAD(yaw_thold));
}


Eigen::Vector3d FlightNode::getHrzRotAxisBladeTip()
{
  Eigen::Vector3d to_right_vec = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1));
  
  Eigen::Vector3d axis_vec = curBladeIsToRight()? to_right_vec.cross(hub_nor_vec_) : hub_nor_vec_.cross(to_right_vec);

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getVtcRotAxisBladeTip(int idx_blade)
{
  Eigen::Vector3d axis_vec;

  if(curBladeIsNormal(idx_blade))
  {
    if(curBladeIsToDown(idx_blade))
      axis_vec = curBladeIsToRight()? (blade_ori_vec_) : (-blade_ori_vec_);
    else
      axis_vec = curBladeIsToRight()? (hub_nor_vec_) : (-hub_nor_vec_);
  }
  else
  {
    Eigen::Vector3d beg_to_end = tip_down_posi_[idx_blade] - tip_up_posi_[idx_blade];
    axis_vec = blade_ori_vec_.cross(beg_to_end);
  }

  return axis_vec.normalized();
}


Eigen::Vector3d FlightNode::getHfRotAxisBladeHub()
{
  Eigen::Vector3d aa_axis = curBladeIsToRight()? (hub_nor_vec_) : (-hub_nor_vec_);
  
  Eigen::AngleAxisd aa(M_PI / 6, aa_axis);
  Eigen::Matrix3d rot_mat(aa);
  Eigen::Vector3d axis_vec = rot_mat * blade_ori_vec_;

  axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getUpRotAxisBladeHub()
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getDnRotAxisBladeHub(int idx_blade)
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  if(curBladeIsNormal(idx_blade))
  {
    axis_vec = curBladeIsToRight()? (-axis_vec) : (axis_vec);
  }
  else
  {
    axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);
  }

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getUpQuatRotAxisBladeTip()
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  axis_vec = curBladeIsToRight()? (-axis_vec) : (axis_vec);

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getUpQuatRotAxisBladeHub()
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getDnQuatRotAxisBladeTip(int idx_blade)
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  if(curBladeIsNormal(idx_blade))
  {
    axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);
  }
  else
  {
    axis_vec = curBladeIsToRight()? (-axis_vec) : (axis_vec);
  }

  return axis_vec.normalized();
}

Eigen::Vector3d FlightNode::getDnQuatRotAxisBladeHub(int idx_blade)
{
  Eigen::Vector3d axis_vec = hub_nor_vec_.cross(blade_ori_vec_).cross(hub_nor_vec_);

  if(curBladeIsNormal(idx_blade))
  {
    axis_vec = curBladeIsToRight()? (-axis_vec) : (axis_vec);
  }
  else
  {
    axis_vec = curBladeIsToRight()? (axis_vec) : (-axis_vec);
  }

  return axis_vec.normalized();
}

void FlightNode::cameraRecordVideo(bool start)
{
  std_msgs::Bool msg;
  msg.data = start;
  camera_record_video_publisher_.publish(msg);
  if(msg.data) ROS_INFO("pub record start msg");
  else ROS_INFO("pub record end msg");
}

void FlightNode::cameraShootPhoto(bool start)
{
  std_msgs::Bool msg;
  msg.data = start;
  camera_shoot_photo_publisher_.publish(msg);
  if(msg.data) ROS_INFO("pub shoot start msg");
  else ROS_INFO("pub shoot end msg");
}

double FlightNode::getShootInterval()
{
  double dist_to_blade = desire_gim_vec_.norm();
  double target_view = GET_TARGET_VIEW(dist_to_blade, camera_zoom_factor_);
  double cover_view = target_view * shoot_photo_coverage_;
  double interval_distance = target_view - cover_view;
  double cur_vel = Eigen::Vector3d(set_velo_.x, set_velo_.y, set_velo_.z).norm();
  double interval_time = interval_distance / cur_vel;
  return interval_time;
}

double FlightNode::getBladeWidthHorizontal(double dist_to_root)
{
  double x = std::max(std::min(dist_to_root, fan_blade_length_), 0.0);
  double ret;
  if(x < fan_blade_length_ / 5.0)
    ret = -(5.0 / 2.0 * fan_blade_width_ / fan_blade_length_) * x + (fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 2)
    ret = -(5.0 / 6.0 * fan_blade_width_ / fan_blade_length_) * x + (2.0 / 3.0 * fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 3)
    ret = -(5.0 / 6.0 * fan_blade_width_ / fan_blade_length_) * x + (2.0 / 3.0 * fan_blade_width_);
  // else if(x < fan_blade_length_ / 5.0 * 4)
  //   ret = -(5.0 / 12.0 * fan_blade_width_ / fan_blade_length_) * x + (5.0 / 12.0 * fan_blade_width_);
  // else
  //   ret = +(1.0 / 12.0 * fan_blade_width_);
  else
    ret = +(1.0 / 6.0 * fan_blade_width_);
  return ret;
}

double FlightNode::getBladeWidthVertical(double dist_to_root)
{
  double x = std::max(std::min(dist_to_root, fan_blade_length_), 0.0);
  double ret;
  // if(x < fan_blade_length_ / 5.0)
  //   ret = +(5.0 * fan_blade_width_ / fan_blade_length_) * x + (fan_blade_width_);
  if(x < fan_blade_length_ / 20.0)
    ret = +(20.0 * fan_blade_width_ / fan_blade_length_) * x + (fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 2.0)
    ret = +(2.0 * fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 3.0)
    ret = -(35.0 / 6.0 * fan_blade_width_ / fan_blade_length_) * x + (13.0 / 3.0 * fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 4.0)
    ret = -(5.0 / 3.0 * fan_blade_width_ / fan_blade_length_) * x + (11.0 / 6.0 * fan_blade_width_);
  else
    ret = -(5.0 / 3.0 * fan_blade_width_ / fan_blade_length_) * x + (11.0 / 6.0 * fan_blade_width_);
  return ret;
}

double FlightNode::getBladeWidthInclined(double dist_to_root)
{
  double x = std::max(std::min(dist_to_root, fan_blade_length_), 0.0);
  double ret;
  if(x < fan_blade_length_ / 5.0)
    ret = +(fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 2.0)
    ret = +(fan_blade_width_);
  else if(x < fan_blade_length_ / 5.0 * 3.0)
    ret = -(5.0 / 2.0 * fan_blade_width_ / fan_blade_length_) * x + (2.0 * fan_blade_width_);
  // else if(x < fan_blade_length_ / 5.0 * 4.0)
  //   ret = -(5.0 / 6.0 * fan_blade_width_ / fan_blade_length_) * x + (1.0 * fan_blade_width_);
  // else
  //   ret = +(1.0 / 3.0 * fan_blade_width_);
  else
    ret = +(1.0 / 2.0 * fan_blade_width_); 
  return ret;
}


bool FlightNode::vector3dIsZero(Eigen::Vector3d vec)
{
  return ((vec - Eigen::Vector3d::Zero()).norm() < 0.1);
}

bool FlightNode::vector3dIsTheSame(Eigen::Vector3d vec1, Eigen::Vector3d vec2)
{
  return ((std::abs(vec1(0) - vec2(0)) < 1e-5) && (std::abs(vec1(1) - vec2(1)) < 1e-5) && (std::abs(vec1(2) - vec2(2)) < 1e-5));
}


/*################################*/
/*    Timer initialize function   */
/*################################*/

bool FlightNode::initTimer(){

  update_stage_mode_timer_ = nh_.createTimer(ros::Rate(rate_sm_update_), &FlightNode::updateStageModeCallback, this);
  servo_control_timer_  = nh_.createTimer(ros::Rate(rate_flight_ctrl_), &FlightNode::servoControlCallback, this);
  sample_traj_point_timer_ = nh_.createTimer(ros::Rate(rate_wps_sample_), &FlightNode::sampleTrajPointCallback, this);
  set_gimbal_position_timer_ = nh_.createTimer(ros::Rate(rate_set_gim_posi_), &FlightNode::setGimbalPositionCallback, this);
  set_camera_zoom_timer_ = nh_.createTimer(ros::Rate(rate_set_camera_zoom_), &FlightNode::setCameraZoomCallback, this);
  send_msg_to_vision_timer_ = nh_.createTimer(ros::Rate(rate_msg_to_vision_), &FlightNode::sendMsgToVisionCallback, this);




  node_status_timer_ = nh_.createTimer(ros::Rate(1.0), &FlightNode::nodeStatusPublisherCallback, this);

  ROS_INFO_STREAM("Flight_node: init timer");

  return true;
}

bool FlightNode::initServiceServer(){
  
  ROS_INFO_STREAM("Flight_node: init ServiceServer");
}























/*##########################*/
/*  Timer Callback function */
/*##########################*/

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------------- update control stage mode ------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
void FlightNode::nodeStatusPublisherCallback(const ros::TimerEvent& event)
{
  std_msgs::Int32 msg;
  msg.data = 10;
  node_status_publisher_.publish(msg);
}

void FlightNode::updateStageModeCallback(const ros::TimerEvent& event){

  int idx_blade = next_track_blade_ - 2;

  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(local_posi_(2) <= (fan_tower_height_ - fan_blade_length_ - 10))
    {
      if(vector3dIsZero(tip_blade_posi_[idx_blade]) && !if_emergency_save_tip_)
      {
        if_fan_tip_arrive_ = true;
        tip_blade_posi_[idx_blade] = local_posi_ + hub_nor_vec_ * ptcloud_key_.blade_dist;
        if_emergency_save_tip_ = true;
        ROS_INFO("height too low, trigger emergency return tip!!!");
      }
    }
  }

  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_INIT_MODE_INIT)
  {
    if(!flag_if_takeoff_)
    {
      insp_msgs::FlightTaskControl flight_task_control;
      flight_task_control.request.task = insp_msgs::FlightTaskControl::Request::TASK_TAKEOFF;
      
      if(flight_task_control_client_.call(flight_task_control) && flight_task_control.response.result == true){
        ROS_INFO_STREAM("flight_node: take off ok");
        flag_if_takeoff_ = true;
      }
    }










    else
    { 
      set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_RISE;
      if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
      {
        stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
        ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_RISE");
        ForceHoverFromUp(true);
        last_dir_vec_vert_ = Eigen::Vector3d(0, 0, 1) * desire_dist_rise_;
        gimbal_reset_ = false;

        if(if_include_tower_)
          if(video_record_mode_ != 2)
            cameraRecordVideo(true);
          else
            cameraShootPhoto(true);
        
        zoom_wait_cnt_ = 0;
        focus_wait_cnt_ = 0;
      }
    }
  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
  {
    if(local_posi_(2) > fan_tower_height_){

      set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_ADJ;
      if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
      {
        stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
        ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_ADJ");
        ForceHoverFromUp(true);
        hub_front_posi_ = local_posi_;

        if(if_include_tower_)
          if(video_record_mode_ != 2)
            cameraRecordVideo(false);
          else
            cameraShootPhoto(false);
        
        zoom_wait_cnt_ = 0;
        focus_wait_cnt_ = 0;
      }
    }
  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_ADJ)
  {
    Eigen::Vector3d goal_vec = Eigen::Vector3d(wps_msg_.pose.position.x, wps_msg_.pose.position.y, wps_msg_.pose.position.z) - local_posi_;

    if(goal_vec.norm() < thold_arrive_target_ && ptcloud_key_.adj_to_loc && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
    {
      if_fan_tip_arrive_ = false;

      if_save_tip_front_posi_ = false;

      zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_root_) * 10);
      focus_wait_cnt_ = int((gap_focus_ - focus_time_root_) * 10);

      front_zoom_cnt_ = 0;
      
      if(track_mode_ == 1)
      {
        if(next_track_blade_ == 1){
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_LOC;

          hub_front_posi_ = hub_pt_posi_ - hub_nor_vec_ * track_dist_;
          hub_back_posi_  = hub_pt_posi_ + hub_nor_vec_ * track_dist_;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 2;
            ForceHoverFromUp(true);
            loc_beg_time_ = ros::Time::now();

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_LOC");
          }
        }
        else if(next_track_blade_ == 2)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_2_GO;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 3;
            ForceHoverFromUp(true);
            
            if(video_record_mode_ != 2)
              cameraRecordVideo(true); // single track side 1, record mode 0 or 1, blade 2
            else
              cameraShootPhoto(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_1_MODE_BLD_2_GO");
          }
        }
        else if(next_track_blade_ == 3)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_3_GO;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 4;
            ForceHoverFromUp(true);
            
            if(video_record_mode_ != 2)
              cameraRecordVideo(true); // single track side 1, record mode 0 or 1, blade 2
            else
              cameraShootPhoto(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_1_MODE_BLD_3_GO");
          }
        }
        else
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_FNS_MODE_DES;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            last_dir_vec_vert_ = Eigen::Vector3d(0, 0, -1) * desire_dist_rise_;
            ForceHoverFromUp(true);
            
            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_FNS_MODE_DES");
          }
        }
      }
      else if(track_mode_ == 2)
      {
        if(next_track_blade_ == 1)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_LOC;

          hub_front_posi_ = hub_pt_posi_ - hub_nor_vec_ * track_dist_;
          hub_back_posi_  = hub_pt_posi_ + hub_nor_vec_ * track_dist_;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 2;
            ForceHoverFromUp(true);
            loc_beg_time_ = ros::Time::now();
            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_LOC");
          }
        }
        else if(next_track_blade_ == 2)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_GO;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 3;
            ForceHoverFromUp(true);

            if(video_record_mode_ == 0)
              if(single_side_ == 1)
                cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
            else if(video_record_mode_ == 1)
              cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
            else if(video_record_mode_ == 2)
              if(single_side_ == 1)
                cameraShootPhoto(true); // double track side 1 and 3, record mode 0 or 1, blade 2

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_2_GO");
          }
        }
        else if(next_track_blade_ == 3)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_GO;
          
          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 4;
            ForceHoverFromUp(true);

            if(video_record_mode_ == 0)
              if(single_side_ == 1)
                cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
            else if(video_record_mode_ == 1)
              cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
            else if(video_record_mode_ == 2)
              if(single_side_ == 1)
                cameraShootPhoto(true); // double track side 1 and 3, record mode 0 or 1, blade 2

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_3_GO");
          }
        }
        else
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_FNS_MODE_DES;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            last_dir_vec_vert_ = Eigen::Vector3d(0, 0, -1) * desire_dist_rise_;
            ForceHoverFromUp(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_FNS_MODE_DES");
          }
        }
      }
      else if(track_mode_ == 4)
      {
        if(next_track_blade_ == 1)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_LOC;

          hub_front_posi_ = hub_pt_posi_ - hub_nor_vec_ * track_dist_;
          hub_back_posi_  = hub_pt_posi_ + hub_nor_vec_ * track_dist_;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 2;
            ForceHoverFromUp(true);
            loc_beg_time_ = ros::Time::now();
            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_LOC");
          }
        }
        else if(next_track_blade_ == 2)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S1;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 3;
            ForceHoverFromUp(true);

            if(video_record_mode_ != 2)
              cameraRecordVideo(true); // 4-track, blade 2-1
            else
              cameraShootPhoto(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_2_S1");
          }
        }
        else if(next_track_blade_ == 3)
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S1;
          
          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            next_track_blade_ = 4;
            ForceHoverFromUp(true);

            if(video_record_mode_ != 2)
              cameraRecordVideo(true); // 4-track, blade 2-1
            else
              cameraShootPhoto(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_3_S1");
          }
        }
        else
        {
          set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_FNS_MODE_DES;

          if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
          {
            stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
            last_dir_vec_vert_ = Eigen::Vector3d(0, 0, -1) * desire_dist_rise_;
            ForceHoverFromUp(true);

            ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_FNS_MODE_DES");
          }
        }
      }
    }
  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_LOC)
  {
    double dist_to_hub_front = (hub_front_posi_ - local_posi_).norm();

    if(ptcloud_key_.loc_dtc_fns && dist_to_hub_front < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
    {
      if(track_mode_ == 1)
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_1_GO;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ForceHoverFromUp(true);

          if(video_record_mode_ != 2)
            cameraRecordVideo(true); // 1-track, blade 1-1
          else
            cameraShootPhoto(true);

          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_1_MODE_BLD_1_GO");
        }
      }
      else if(track_mode_ == 2)
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_GO;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ForceHoverFromUp(true);

          if(video_record_mode_ == 0)
            if(single_side_ == 1)
              cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
          else if(video_record_mode_ == 1)
            cameraRecordVideo(true); // double track side 1 and 3, record mode 0 or 1, blade 2
          else if(video_record_mode_ == 2)
            if(single_side_ == 1)
              cameraShootPhoto(true); // double track side 1 and 3, record mode 0 or 1, blade 2

          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_1_GO");
        }
      }
      else if(track_mode_ == 4)
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ForceHoverFromUp(true);

          if(video_record_mode_ != 2)
            cameraRecordVideo(true); // 4-track, blade 1-1
          else
            cameraShootPhoto(true);

          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_1_S1");
        }
      }
      zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_root_) * 10);
      focus_wait_cnt_ = int((gap_focus_ - focus_time_root_) * 10);
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1)
    {
      double dist_to_tip_front = (local_posi_ - tip_side1_posi_[idx_blade]).norm();
      if(if_fan_tip_arrive_ && dist_to_tip_front < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_)){

        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_1_MODE_BLD_N_BK");
          if_fan_tip_arrive_ = false;

          Eigen::Vector3d x_base_vector = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1)).normalized();
          Eigen::Vector3d y_base_vector = x_base_vector.cross(hub_nor_vec_).normalized();
          blade_angle_[idx_blade] = atan2(blade_ori_vec_.dot(y_base_vector), blade_ori_vec_.dot(x_base_vector));

          std::reverse(pts_traj_0_[idx_blade].begin(), pts_traj_0_[idx_blade].end());
          LineTrajPtr_0_[idx_blade] -> reset();
          LineTrajPtr_0_[idx_blade] -> initTraj(pts_traj_0_[idx_blade]);
          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          std::reverse(pts_traj_1_[idx_blade].begin(), pts_traj_1_[idx_blade].end());
          LineTrajPtr_1_[idx_blade] -> initTraj(pts_traj_1_[idx_blade]);
          LineTrajPtr_1_[idx_blade] -> smooth_v3(8);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

        }
      }

    }
    else if(stage_mode_.stage_mode % 2 == 0)
    {
      double dist_to_hub_front = (hub_front_posi_ - local_posi_).norm();
      if(dist_to_hub_front < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_)){

        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_ADJ;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_ADJ");
          
          if_save_tip_front_posi_ = false;
          
          ForceHoverFromUp(true);

          gimbal_reset_ = true;

          if(video_record_mode_ == 1)
            cameraRecordVideo(false); // 1-track, ~blade n
        }
      }
    }
  }

  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1)
    {
      double dist_to_tip_front = (local_posi_ - tip_side1_posi_[idx_blade]).norm();
      if(if_fan_tip_arrive_ && dist_to_tip_front < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;

          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_N_TP");
          if_fan_tip_arrive_ = false;

          Eigen::Vector3d x_base_vector = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1)).normalized();
          Eigen::Vector3d y_base_vector = x_base_vector.cross(hub_nor_vec_).normalized();
          blade_angle_[idx_blade] = atan2(blade_ori_vec_.dot(y_base_vector), blade_ori_vec_.dot(x_base_vector));

          Eigen::Vector3d vtc_vec = (hub_nor_vec_.cross(blade_ori_vec_)).normalized();

          if(curBladeIsNormal(idx_blade))
          {
            for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
            {
              Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];
              Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_;
              pts_traj_4_[idx_blade].push_back(pt4);

              Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_;
              pts_traj_2_[idx_blade].push_back(pt2);
            }
          }
          else
          {
            if(curBladeIsToRight())
            {
              for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
              {
                Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];

                Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_;
                pts_traj_4_[idx_blade].push_back(pt4);
                
                Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
                pts_traj_2_[idx_blade].push_back(pt2);
              }
            }
            else
            {
              for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
              {
                Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];

                Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
                pts_traj_4_[idx_blade].push_back(pt4);
                
                Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_;
                pts_traj_2_[idx_blade].push_back(pt2);
              }
            }
          }


          Eigen::Vector3d pt0_tip = blade_fpt_posi_;
          pts_traj_0_[idx_blade].push_back(pt0_tip);
          Eigen::Vector3d pt1_tip = pt0_tip - hub_nor_vec_ * track_dist_;
          pts_traj_1_[idx_blade].push_back(pt1_tip);
          Eigen::Vector3d pt3_tip = pt0_tip + hub_nor_vec_ * track_dist_;
          pts_traj_3_[idx_blade].push_back(pt3_tip);

          if(curBladeIsNormal(idx_blade))
          {
            Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_;
            pts_traj_4_[idx_blade].push_back(pt4_tip);

            Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_;
            pts_traj_2_[idx_blade].push_back(pt2_tip);
          }
          else
          {
            if(curBladeIsToRight())
            {
              Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_;
              pts_traj_4_[idx_blade].push_back(pt4_tip);
              
              Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
              pts_traj_2_[idx_blade].push_back(pt2_tip);
            }
            else
            {
              Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
              pts_traj_4_[idx_blade].push_back(pt4_tip);
              
              Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_;
              pts_traj_2_[idx_blade].push_back(pt2_tip);
            }
          }


          std::reverse(pts_traj_0_[idx_blade].begin(), pts_traj_0_[idx_blade].end());
          LineTrajPtr_0_[idx_blade] -> reset();
          LineTrajPtr_0_[idx_blade] -> initTraj(pts_traj_0_[idx_blade]);
          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          std::reverse(pts_traj_1_[idx_blade].begin(), pts_traj_1_[idx_blade].end());
          LineTrajPtr_1_[idx_blade] -> initTraj(pts_traj_1_[idx_blade]);
          LineTrajPtr_1_[idx_blade] -> smooth_v3(8);
          
          if(single_side_ == 1 || single_side_ == 3)
          {
            std::reverse(pts_traj_3_[idx_blade].begin(), pts_traj_3_[idx_blade].end());
            LineTrajPtr_3_[idx_blade] -> initTraj(pts_traj_3_[idx_blade]);
            LineTrajPtr_3_[idx_blade] -> smooth_v3(8);

            root_side3_posi_[idx_blade] = LineTrajPtr_3_[idx_blade] -> getEndPos_SmoothTraj();
            tip_side3_posi_[idx_blade] = LineTrajPtr_3_[idx_blade] -> getBgnPos_SmoothTraj();
          }
          else if(single_side_ == 2)
          {
            std::reverse(pts_traj_2_[idx_blade].begin(), pts_traj_2_[idx_blade].end());
            LineTrajPtr_2_[idx_blade] -> initTraj(pts_traj_2_[idx_blade]);
            LineTrajPtr_2_[idx_blade] -> smooth_v3(8);

            root_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getEndPos_SmoothTraj();
            tip_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getBgnPos_SmoothTraj();
          }
          else if(single_side_ == 4)
          {
            std::reverse(pts_traj_4_[idx_blade].begin(), pts_traj_4_[idx_blade].end());
            LineTrajPtr_4_[idx_blade] -> initTraj(pts_traj_4_[idx_blade]);
            LineTrajPtr_4_[idx_blade] -> smooth_v3(8);

            root_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getEndPos_SmoothTraj();
            tip_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getBgnPos_SmoothTraj();
          }

          root_side1_posi_[idx_blade] = LineTrajPtr_1_[idx_blade] -> getEndPos_SmoothTraj();
          tip_side1_posi_[idx_blade] = LineTrajPtr_1_[idx_blade] -> getBgnPos_SmoothTraj();
          
          tip_blade_posi_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getBgnPos_SmoothTraj();
          root_blade_posi_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getEndPos_SmoothTraj();

          int del_num = LineTrajPtr_0_[idx_blade] -> push_back(hub_pt_posi_);
          for(int i = 0; i < del_num; i++)
            pts_traj_0_[idx_blade].pop_back();
          pts_traj_0_[idx_blade].push_back(hub_pt_posi_);

          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          if(single_side_ == 1 || single_side_ == 3)
          {
            Eigen::Vector3d rot_axis = getHrzRotAxisBladeTip();
            CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::half);
          }
          else if(single_side_ == 2)
          {
            Eigen::Vector3d rot_axis;
            if(curBladeIsToRight())
            {
              rot_axis = getDnQuatRotAxisBladeTip(idx_blade);
              if(curBladeIsNormal(idx_blade))
                CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
              else
                CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::special, (1.0 + traj_down_angle_ / 180.0));
            }
            else
            {
              rot_axis = getUpQuatRotAxisBladeTip();
              CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
            }
          }
          else if(single_side_ == 4)
          {
            Eigen::Vector3d rot_axis;
            if(curBladeIsToRight())
            {
              rot_axis = getUpQuatRotAxisBladeTip();
              CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
            }
            else
            {
              rot_axis = getDnQuatRotAxisBladeTip(idx_blade);
              if(curBladeIsNormal(idx_blade))
                CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
              else
                CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::special, (1.0 + traj_down_angle_ / 180.0));
            }
          }
          CircleTrajPtr_1_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);

          // insp_msgs::BladeTipQuery srv;
          // srv.request.fake_tip.x = tip_blade_posi_[idx_blade].x();
          // srv.request.fake_tip.y = tip_blade_posi_[idx_blade].y();
          // srv.request.fake_tip.z = tip_blade_posi_[idx_blade].z();
          
          // if(true_tip_point_client_.call(srv))
          //   tip_posi_true_[idx_blade] = Eigen::Vector3d(srv.response.true_tip.x, srv.response.true_tip.y, srv.response.true_tip.z);
          // else
          //  tip_posi_true_[idx_blade] = tip_blade_posi_[idx_blade];

          // tip_wait_dist_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getDistFromStartPoint(tip_posi_true_[idx_blade]);
          // df_msg_.tip_wait_dist = tip_wait_dist_[idx_blade];

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);
          
          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

        }
      }
    }
    else if(stage_mode_.stage_mode % 4 == 2)
    {
      double dist_to_tgt;
      if(single_side_ == 1 || single_side_ == 3)
        dist_to_tgt = (local_posi_ - tip_side3_posi_[idx_blade]).norm();
      else if(single_side_ == 2)
        dist_to_tgt = (local_posi_ - tip_side2_posi_[idx_blade]).norm();
      else if(single_side_ == 4)
        dist_to_tgt = (local_posi_ - tip_side4_posi_[idx_blade]).norm();

      if(dist_to_tgt < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {        
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_N_BK");
          
          if_save_tip_front_posi_ = false;

          ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(true); // 2-track, blade n-2
          else if(video_record_mode_ == 2)
            cameraShootPhoto(true);
          
          zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_tip_) * 10);
          focus_wait_cnt_ = int((gap_focus_ - focus_time_tip_) * 10);
          tip_focus_cnt_ = 0;
          tip_focus_wait_cnt_ = 0;
        }
      }
    }
    else if(stage_mode_.stage_mode % 4 == 3)
    {
      double dist_to_tgt;
      if(single_side_ == 1 || single_side_ == 3)
        dist_to_tgt = (local_posi_ - root_side3_posi_[idx_blade]).norm();
      else if(single_side_ == 2)
        dist_to_tgt = (local_posi_ - root_side2_posi_[idx_blade]).norm();
      else if(single_side_ == 4)
        dist_to_tgt = (local_posi_ - root_side4_posi_[idx_blade]).norm();

      if(dist_to_tgt < thold_arrive_target_ && if_gimbal_end_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {        
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_2_MODE_BLD_N_RT");

          if_gimbal_end_ = false;

          if(single_side_ == 1 || single_side_ == 3)
          {
            Eigen::Vector3d rot_axis = getUpQuatRotAxisBladeHub();
            CircleTrajPtr_2_[idx_blade] -> initTraj(root_side3_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::half);
            CircleTrajPtr_2_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);
          }
          else if(single_side_ == 2)
          {
            Eigen::Vector3d rot_axis;
            if(curBladeIsToRight())
            {
              rot_axis = getDnQuatRotAxisBladeHub(idx_blade);
              if(curBladeIsNormal(idx_blade))
                CircleTrajPtr_2_[idx_blade] -> initTraj(root_side2_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
              else
                CircleTrajPtr_2_[idx_blade] -> initTraj(root_side2_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::special, (1.0 + traj_down_angle_ / 180.0));
            }
            else
            {
              rot_axis = getUpQuatRotAxisBladeHub();
              CircleTrajPtr_2_[idx_blade] -> initTraj(root_side2_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
            }
          }
          else if(single_side_ == 4)
          {
            Eigen::Vector3d rot_axis;
            if(curBladeIsToRight())
            {
              rot_axis = getUpQuatRotAxisBladeHub();
              CircleTrajPtr_2_[idx_blade] -> initTraj(root_side4_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
            }
            else
            {
              rot_axis = getDnQuatRotAxisBladeHub(idx_blade);
              if(curBladeIsNormal(idx_blade))
                CircleTrajPtr_2_[idx_blade] -> initTraj(root_side4_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
              else
                CircleTrajPtr_2_[idx_blade] -> initTraj(root_side4_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::special, (1.0 + traj_down_angle_ / 180.0));
            }
          }
          CircleTrajPtr_2_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(false); // 2-track, ~blade n-2
          else if(video_record_mode_ == 2)
            cameraShootPhoto(false);
        }
      }
    }
    else if(stage_mode_.stage_mode % 4 == 0)
    {
      double dist_to_hub_front = (local_posi_ - hub_front_posi_).norm();
      if(dist_to_hub_front < thold_arrive_target_ && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_ADJ;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_ADJ");
          
          gimbal_reset_ = true;

          ForceHoverFromUp(true);

          if(video_record_mode_ == 1)
            cameraRecordVideo(false); // 2-track, ~blade n
        }
      }
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1)
    {
      double dist_to_tip_front = (local_posi_ - tip_side1_posi_[idx_blade]).norm();
      if(if_fan_tip_arrive_ && dist_to_tip_front < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_13");
          if_fan_tip_arrive_ = false;

          Eigen::Vector3d x_base_vector = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1)).normalized();
          Eigen::Vector3d y_base_vector = x_base_vector.cross(hub_nor_vec_).normalized();
          blade_angle_[idx_blade] = atan2(blade_ori_vec_.dot(y_base_vector), blade_ori_vec_.dot(x_base_vector));

          Eigen::Vector3d vtc_vec = (hub_nor_vec_.cross(blade_ori_vec_)).normalized();

          if(curBladeIsNormal(idx_blade))
          {
            for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
            {
              Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];
              Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_;
              pts_traj_4_[idx_blade].push_back(pt4);

              Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_;
              pts_traj_2_[idx_blade].push_back(pt2);
            }
          }
          else
          {
            if(curBladeIsToRight())
            {
              for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
              {
                Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];

                Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_;
                pts_traj_4_[idx_blade].push_back(pt4);
                
                Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
                pts_traj_2_[idx_blade].push_back(pt2);
              }
            }
            else
            {
              for(int i = 0; i < pts_traj_0_[idx_blade].size(); i++)
              {
                Eigen::Vector3d pt0 = pts_traj_0_[idx_blade][i];
                Eigen::Vector3d pt4 = pt0 - vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
                pts_traj_4_[idx_blade].push_back(pt4);
                
                Eigen::Vector3d pt2 = pt0 + vtc_vec * track_dist_;
                pts_traj_2_[idx_blade].push_back(pt2);
              }
            }
          }


          Eigen::Vector3d pt0_tip = blade_fpt_posi_;
          pts_traj_0_[idx_blade].push_back(pt0_tip);
          Eigen::Vector3d pt1_tip = pt0_tip - hub_nor_vec_ * track_dist_;
          pts_traj_1_[idx_blade].push_back(pt1_tip);
          Eigen::Vector3d pt3_tip = pt0_tip + hub_nor_vec_ * track_dist_;
          pts_traj_3_[idx_blade].push_back(pt3_tip);


          if(curBladeIsNormal(idx_blade))
          {
            Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_;
            pts_traj_4_[idx_blade].push_back(pt4_tip);

            Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_;
            pts_traj_2_[idx_blade].push_back(pt2_tip);
          }
          else
          {
            if(curBladeIsToRight())
            {
              Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_;
              pts_traj_4_[idx_blade].push_back(pt4_tip);
              
              Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
              pts_traj_2_[idx_blade].push_back(pt2_tip);
            }
            else
            {
              Eigen::Vector3d pt4_tip = pt0_tip - vtc_vec * track_dist_ * sin(DEG2RAD(traj_down_angle_)) + hub_nor_vec_ * track_dist_ * cos(DEG2RAD(traj_down_angle_));
              pts_traj_4_[idx_blade].push_back(pt4_tip);
              
              Eigen::Vector3d pt2_tip = pt0_tip + vtc_vec * track_dist_;
              pts_traj_2_[idx_blade].push_back(pt2_tip);
            }
          }


          std::reverse(pts_traj_0_[idx_blade].begin(), pts_traj_0_[idx_blade].end());
          LineTrajPtr_0_[idx_blade] -> reset();
          LineTrajPtr_0_[idx_blade] -> initTraj(pts_traj_0_[idx_blade]);
          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          std::reverse(pts_traj_1_[idx_blade].begin(), pts_traj_1_[idx_blade].end());
          LineTrajPtr_1_[idx_blade] -> initTraj(pts_traj_1_[idx_blade]);
          LineTrajPtr_1_[idx_blade] -> smooth_v3(8);

          std::reverse(pts_traj_3_[idx_blade].begin(), pts_traj_3_[idx_blade].end());
          LineTrajPtr_3_[idx_blade] -> initTraj(pts_traj_3_[idx_blade]);
          LineTrajPtr_3_[idx_blade] -> smooth_v3(8);

          root_side1_posi_[idx_blade] = LineTrajPtr_1_[idx_blade] -> getEndPos_SmoothTraj();
          tip_side1_posi_[idx_blade] = LineTrajPtr_1_[idx_blade] -> getBgnPos_SmoothTraj();;
          
          root_side3_posi_[idx_blade] = LineTrajPtr_3_[idx_blade] -> getEndPos_SmoothTraj();
          tip_side3_posi_[idx_blade] = LineTrajPtr_3_[idx_blade] -> getBgnPos_SmoothTraj();
          
          tip_blade_posi_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getBgnPos_SmoothTraj();
          root_blade_posi_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getEndPos_SmoothTraj();

          int del_num = LineTrajPtr_0_[idx_blade] -> push_back(hub_pt_posi_);
          for(int i = 0; i < del_num; i++)
            pts_traj_0_[idx_blade].pop_back();
          pts_traj_0_[idx_blade].push_back(hub_pt_posi_);

          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          hub_root_dist_[idx_blade] = (hub_pt_posi_ - root_blade_posi_[idx_blade]).norm();
          gimbal_traj_time_[idx_blade] = hub_root_dist_[idx_blade] / traj_vel_gimbal_;

          Eigen::Vector3d rot_axis = getHrzRotAxisBladeTip();
          CircleTrajPtr_1_[idx_blade] -> initTraj(tip_side1_posi_[idx_blade], tip_blade_posi_[idx_blade], rot_axis, circle_traj::half);
          CircleTrajPtr_1_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);

          // insp_msgs::BladeTipQuery srv;
          // srv.request.fake_tip.x = tip_blade_posi_[idx_blade].x();
          // srv.request.fake_tip.y = tip_blade_posi_[idx_blade].y();
          // srv.request.fake_tip.z = tip_blade_posi_[idx_blade].z();
          
          // if(true_tip_point_client_.call(srv))
          //   tip_posi_true_[idx_blade] = Eigen::Vector3d(srv.response.true_tip.x, srv.response.true_tip.y, srv.response.true_tip.z);
          // else
          //   tip_posi_true_[idx_blade] = tip_blade_posi_[idx_blade];

          // tip_wait_dist_[idx_blade] = LineTrajPtr_0_[idx_blade] -> getDistFromStartPoint(tip_posi_true_[idx_blade]);
          // df_msg_.tip_wait_dist = tip_wait_dist_[idx_blade];

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 2)
    {
      double dist_to_tip_back = (local_posi_ - tip_side3_posi_[idx_blade]).norm();
      if(dist_to_tip_back < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_S3");

          if_save_tip_front_posi_ = false;

          ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(true); // 4-track, blade n-2
          else if(video_record_mode_ == 2)
            cameraShootPhoto(true);
          
          zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_tip_) * 10);
          focus_wait_cnt_ = int((gap_focus_ - focus_time_tip_) * 10);
          tip_focus_cnt_ = 0;
          tip_focus_wait_cnt_ = 0;
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 3)
    {
      double dist_to_root_back = (local_posi_ - root_side3_posi_[idx_blade]).norm();
      if(dist_to_root_back < thold_arrive_target_ && if_gimbal_end_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_34");

          if_gimbal_end_ = false;

          if(curBladeIsToRight())
          {
            LineTrajPtr_4_[idx_blade] -> initTraj(pts_traj_4_[idx_blade]);
            LineTrajPtr_4_[idx_blade] -> smooth_v3(8);
            
            root_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getBgnPos_SmoothTraj();
            tip_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getEndPos_SmoothTraj();
          }
          else
          {
            LineTrajPtr_2_[idx_blade] -> initTraj(pts_traj_2_[idx_blade]);
            LineTrajPtr_2_[idx_blade] -> smooth_v3(8);

            root_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getBgnPos_SmoothTraj();
            tip_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getEndPos_SmoothTraj();
          }

          Eigen::Vector3d rot_axis = getUpRotAxisBladeHub();
          CircleTrajPtr_2_[idx_blade] -> initTraj(root_side3_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
          CircleTrajPtr_2_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);
          std::reverse(pts_traj_0_[idx_blade].begin(), pts_traj_0_[idx_blade].end());
          LineTrajPtr_0_[idx_blade] -> reset();
          LineTrajPtr_0_[idx_blade] -> initTraj(pts_traj_0_[idx_blade]);
          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(false); // 4-track, ~blade n-2
          else if(video_record_mode_ == 2)
            cameraShootPhoto(false);
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 4)
    {
      root_up_posi_[idx_blade] = curBladeIsToRight()? root_side4_posi_[idx_blade] : root_side2_posi_[idx_blade];
      double dist_to_root_up = (local_posi_ - root_up_posi_[idx_blade]).norm();
      if(dist_to_root_up < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_S4");

          ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(true); // 4-track, blade n-3
          else if(video_record_mode_ == 2)
            cameraShootPhoto(true);
        
          zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_root_) * 10);
          focus_wait_cnt_ = int((gap_focus_ - focus_time_root_) * 10);
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 5)
    {
      tip_up_posi_[idx_blade] = curBladeIsToRight()? tip_side4_posi_[idx_blade] : tip_side2_posi_[idx_blade];
      double dist_to_tip_up = (local_posi_ - tip_up_posi_[idx_blade]).norm();
      if(dist_to_tip_up < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_42");

          if_gimbal_beg_ = false;

          if(curBladeIsToRight())
          {
            std::reverse(pts_traj_2_[idx_blade].begin(), pts_traj_2_[idx_blade].end());

            LineTrajPtr_2_[idx_blade] -> initTraj(pts_traj_2_[idx_blade]);
            LineTrajPtr_2_[idx_blade] -> smooth_v3(8);
            
            root_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getEndPos_SmoothTraj();
            tip_side2_posi_[idx_blade] = LineTrajPtr_2_[idx_blade] -> getBgnPos_SmoothTraj();         
          }
          else
          {
            std::reverse(pts_traj_4_[idx_blade].begin(), pts_traj_4_[idx_blade].end());

            LineTrajPtr_4_[idx_blade] -> initTraj(pts_traj_4_[idx_blade]);
            LineTrajPtr_4_[idx_blade] -> smooth_v3(8);

            root_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getEndPos_SmoothTraj();
            tip_side4_posi_[idx_blade] = LineTrajPtr_4_[idx_blade] -> getBgnPos_SmoothTraj();
          }

          tip_up_posi_[idx_blade]   = curBladeIsToRight()? tip_side4_posi_[idx_blade] : tip_side2_posi_[idx_blade];
          tip_down_posi_[idx_blade] = curBladeIsToRight()? tip_side2_posi_[idx_blade] : tip_side4_posi_[idx_blade];
          Eigen::Vector3d rot_axis = getVtcRotAxisBladeTip(idx_blade);
          Eigen::Vector3d rot_center = (tip_up_posi_[idx_blade] + tip_down_posi_[idx_blade]) / 2;
          CircleTrajPtr_3_[idx_blade] -> initTraj(tip_up_posi_[idx_blade], rot_center, rot_axis, circle_traj::half);
          CircleTrajPtr_3_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);
          std::reverse(pts_traj_0_[idx_blade].begin(), pts_traj_0_[idx_blade].end());
          LineTrajPtr_0_[idx_blade] -> reset();
          LineTrajPtr_0_[idx_blade] -> initTraj(pts_traj_0_[idx_blade]);
          LineTrajPtr_0_[idx_blade] -> smooth_v3(8);

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);
        
          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(false); // 4-track, ~blade n-3
          else if(video_record_mode_ == 2)
            cameraShootPhoto(false);
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 6)
    {
      tip_down_posi_[idx_blade] = curBladeIsToRight()? tip_side2_posi_[idx_blade] : tip_side4_posi_[idx_blade];
      double dist_to_tip_down = (local_posi_ - tip_down_posi_[idx_blade]).norm();
      
      if(dist_to_tip_down < thold_arrive_target_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_S2");

          ForceHoverFromUp(true);

          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(true); // 4-track, blade n-4
          else if(video_record_mode_ == 2)
            cameraShootPhoto(true);
                  
          zoom_wait_cnt_ = int((gap_zoom_ - zoom_time_tip_) * 10);
          focus_wait_cnt_ = int((gap_focus_ - focus_time_tip_) * 10);
          tip_focus_cnt_ = 0;
          tip_focus_wait_cnt_ = 0;
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 7)
    {
      root_down_posi_[idx_blade] = curBladeIsToRight()? root_side2_posi_[idx_blade] : root_side4_posi_[idx_blade];
      double dist_to_root_down = (local_posi_ - root_down_posi_[idx_blade]).norm();

      if(dist_to_root_down < thold_arrive_target_ && if_gimbal_end_ && enoughYawAlign(thold_enough_yaw_accu_) && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = stage_mode_.stage_mode + 1;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_TRK_4_MODE_BLD_N_20");

          if_gimbal_end_ = false;

          root_down_posi_[idx_blade] = curBladeIsToRight() ? root_side2_posi_[idx_blade] : root_side4_posi_[idx_blade];
          
          if(curBladeIsNormal(idx_blade))
          {
            Eigen::Vector3d rot_axis = getDnRotAxisBladeHub(idx_blade);
            CircleTrajPtr_4_[idx_blade] -> initTraj(root_down_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::quarter);
            CircleTrajPtr_4_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);
          }
          else
          {
            Eigen::Vector3d rot_axis = getDnRotAxisBladeHub(idx_blade);
            CircleTrajPtr_4_[idx_blade] -> initTraj(root_down_posi_[idx_blade], root_blade_posi_[idx_blade], rot_axis, circle_traj::special, (1.0 + traj_down_angle_ / 180.0));
            CircleTrajPtr_4_[idx_blade] -> setV(traj_vel_turn_, 0.0, 0.0, 1.0);
          }

          if(enable_planning_)
            ForceHoverFromUp(false);
          else
            ForceHoverFromUp(true);
          
          gimbal_reset_ = true;

          traj_begin_time_ = ros::Time::now() + traj_delay_duration_;

          if(video_record_mode_ == 0)
            cameraRecordVideo(false); // 4-track, ~blade n-4
          else if(video_record_mode_ == 2)
            cameraShootPhoto(false);
        }
      }
    }
    else if(stage_mode_.stage_mode % 8 == 0)
    {
      double dist_to_hub_front = (local_posi_ - hub_front_posi_).norm();
      if(dist_to_hub_front < thold_arrive_target_ && enoughSlowDown(thold_enough_vel_slow_))
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_DTC_MODE_ADJ;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_DTC_MODE_ADJ");
          
          gimbal_reset_ = true;

          ForceHoverFromUp(true);

          if(video_record_mode_ == 1)
            cameraRecordVideo(false); // 4-track, ~blade n
        }
      }
    }
  }

  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_DES)
  {
    if(local_posi_(2) < 8.0){
      if(!flag_if_land_){

        insp_msgs::FlightTaskControl flight_task_control;
        flight_task_control.request.task = insp_msgs::FlightTaskControl::Request::TASK_LAND;
        
        if(flight_task_control_client_.call(flight_task_control) && flight_task_control.response.result == true){
          ROS_INFO_STREAM("flight_node: land ok");
          flag_if_land_ = true;
        }
      }
      else
      {
        set_stage_mode_.request.new_stage_mode = insp_msgs::StageMode::STAGE_FNS_MODE_LAND;
        if(set_stage_mode_client_.call(set_stage_mode_) && set_stage_mode_.response.result == true)
        {
          stage_mode_.stage_mode = set_stage_mode_.request.new_stage_mode;
          ROS_INFO_STREAM("flight_node: stage_mode update to STAGE_FNS_MODE_LAND");
          ros::Duration(1.0).sleep();
          ros::shutdown();
        }
      }
    }
  }

  return;
}


/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* --------------------------------------------------- main control function --------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */

void FlightNode::servoControlCallback(const ros::TimerEvent& event){

  FlightNode::vehicleXYZCtrl();

  if(enable_emergency_terminal_ && stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
    FlightNode::futureOccupiedCheck();

  set_velo_publisher_.publish(set_velo_);

  if(ros::Time::now() - last_info_time_ > ros::Duration(1.0))
  {
    FlightNode::showCtrlInfo();
    last_info_time_ = ros::Time::now();
  }

  return;
}



/*##########################*/
/*     Control function     */
/*##########################*/

void FlightNode::vehicleXYZCtrl(){

  int idx_blade = next_track_blade_ - 2;

  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_INIT_MODE_INIT)
  {
    // calculate target yaw
    // tgt_yaw_ = local_euler_(2);
    updateTargetYaw(Eigen::Vector3d(1, 0, 0));
    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

    gimbal_reset_ = true;
    gimbal_keep_yaw_ = false;
  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
  {
    if(ptcloud_key_.tower_legal)
    {
      Eigen::Vector3d dir_vec = Eigen::Vector3d(0, 0, 1);
      dir_vec = dir_vec * desire_dist_rise_;

      last_dir_vec_vert_ = dir_vec;

      Eigen::Vector3d dist_vec = tower_nor_vec_ * (detect_dist_ - tower_vrt_dst_);

      desire_posi_ = local_posi_ + dir_vec - dist_vec;

      // calculate target yaw
      updateTargetYaw(tower_nor_vec_);

      // calculate gimbal posi
      desire_gim_vec_ = tower_nor_vec_ * track_dist_;    
      last_legal_gim_vec_ = desire_gim_vec_;
    }
    else
    {
      desire_posi_ = local_posi_ + last_dir_vec_vert_ ;

      // calculate target yaw
      tgt_yaw_ = last_legal_yaw_;
      
      // calculate gimbal posi
      if(vector3dIsZero(last_legal_gim_vec_))
        gimbal_reset_ = true;
      else
        desire_gim_vec_ = last_legal_gim_vec_;

    }

    wps_msg_.pose.position.x = desire_posi_(0);
    wps_msg_.pose.position.y = desire_posi_(1);
    wps_msg_.pose.position.z = desire_posi_(2);

    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
    // wps ctrl
    wps_servo_publisher_.publish(wps_msg_);
    last_wps_msg_ = wps_msg_;

    track_index_msg_.blade_index = 0;
    track_index_msg_.track_plane = 1;
  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_ADJ)
  { 
  
    if(ptcloud_key_.hub_legal)
    {
      hub_front_posi_ = hub_pt_posi_ - hub_nor_vec_ * track_dist_;
      hub_back_posi_  = hub_pt_posi_ + hub_nor_vec_ * track_dist_;

      desire_posi_ = hub_front_posi_;
      
      // calculate target yaw 
      Eigen::Vector3d vec_to_hub = hub_pt_posi_ - local_posi_;

      updateTargetYaw(vec_to_hub);

      // calculate gimbal posi
      desire_gim_vec_ = hub_pt_posi_ - local_posi_;  
      last_legal_gim_vec_ = desire_gim_vec_;  
    }
    else
    {
      desire_posi_ = hub_front_posi_;

      // calculate target yaw 
      tgt_yaw_ = last_legal_yaw_;

      // calculate gimbal posi
      desire_gim_vec_ = last_legal_gim_vec_;    
    }

    wps_msg_.pose.position.x = desire_posi_(0);
    wps_msg_.pose.position.y = desire_posi_(1);
    wps_msg_.pose.position.z = desire_posi_(2);

    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
    // wps ctrl
    wps_servo_publisher_.publish(wps_msg_);
    last_wps_msg_ = wps_msg_;

  }
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_LOC)
  {
    hub_front_posi_ = hub_pt_posi_ - hub_nor_vec_ * track_dist_;
    hub_back_posi_  = hub_pt_posi_ + hub_nor_vec_ * track_dist_;


    if(!ptcloud_key_.loc_dtc_fns)
    {
      double t_cur = (ros::Time::now() - loc_beg_time_).toSec();
      desire_posi_ = getCirclePoint3D(hub_front_posi_, loc_rotate_radius_, hub_nor_vec_, t_cur, PI / loc_rotate_period_);
    }else
    {
      desire_posi_ = hub_front_posi_;
    }

    wps_msg_.pose.position.x = desire_posi_(0);
    wps_msg_.pose.position.y = desire_posi_(1);
    wps_msg_.pose.position.z = desire_posi_(2);

    // wps ctrl
    wps_servo_publisher_.publish(wps_msg_);
    last_wps_msg_ = wps_msg_;

    // calculate target yaw
    updateTargetYaw(hub_nor_vec_);

    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

    // calculate gimbal posi
    desire_gim_vec_ = hub_nor_vec_ * track_dist_;    
    last_legal_gim_vec_ = desire_gim_vec_;

  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1)
    {
      if(!if_fan_tip_arrive_)
      {
        if(ptcloud_key_.blade_legal)
        {
          Eigen::Vector3d dir_vec = blade_dir_vec_ * desire_dist_track_;
          
          last_dir_vec_track_ = dir_vec;
          
          Eigen::Vector3d tmp_vec = blade_fpt_posi_ - hub_nor_vec_ * track_dist_ - local_posi_;

          Eigen::Vector3d adj_vec = tmp_vec - blade_dir_vec_ * tmp_vec.dot(blade_dir_vec_);

          desire_posi_ = local_posi_ + dir_vec + adj_vec;

          // calculate gimbal
          if(enable_ptcloud_correct_ && dist_to_hub_front_ > dist_enable_sample_track_wps_ * 2)
          {
            desire_gim_vec_ = blade_crt_posi_ - local_posi_;
          }
          else
          {
            desire_gim_vec_ = blade_fpt_posi_ - local_posi_;
          }
          last_legal_gim_vec_ = desire_gim_vec_;

          // calculate target yaw
          Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
          updateTargetYaw(desire_yaw_vec);
        }
        else
        {
          desire_posi_ = local_posi_ + last_dir_vec_track_;
          
          // calculate target yaw
          tgt_yaw_ = last_legal_yaw_;

          // calculate gimbal posi
          desire_gim_vec_ = last_legal_gim_vec_;
        }
      }
      else
      {
        if(vector3dIsZero(tip_side1_posi_[idx_blade]))
          tip_side1_posi_[idx_blade] = Eigen::Vector3d(ptcloud_key_.tip_x, ptcloud_key_.tip_y, ptcloud_key_.tip_z) - hub_nor_vec_ * track_dist_;

        desire_posi_ = tip_side1_posi_[idx_blade];

        // calculate target yaw
        tgt_yaw_ = last_legal_yaw_;

        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;

      }
      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = 1;
      df_msg_.foot_pt.x = blade_fpt_posi_.x();
      df_msg_.foot_pt.y = blade_fpt_posi_.y();
      df_msg_.foot_pt.z = blade_fpt_posi_.z();
      df_msg_.crt_pt.x = blade_crt_posi_.x();
      df_msg_.crt_pt.y = blade_crt_posi_.y();
      df_msg_.crt_pt.z = blade_crt_posi_.z();
    }
    else if(stage_mode_.stage_mode % 2 == 0)
    {
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = LineTrajPtr_1_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end);

      if(!if_traj_end)
      {
        desire_posi_ = traj_cur_posi;
        
        // calculate gimbal posi
        double err = (desire_posi_ - local_posi_).norm();
        double vel = sqrt(pow(set_velo_.x, 2) + pow(set_velo_.y, 2) + pow(set_velo_.z, 2));
        double t = err / vel;
        Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - t, traj_vel_track_, if_traj_end);
        
        df_msg_.err = err;
        df_msg_.vel = vel;
        df_msg_.ttt = t;
        df_msg_.traj_cur_time_minus_t = traj_cur_time - t;
        
        desire_gim_vec_ = gimbal_tgt - local_posi_;
        last_legal_gim_vec_ = desire_gim_vec_;  

      }
      else
      {
        desire_posi_ = hub_front_posi_;
        
        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;
      }
      
      // calculate target yaw
      Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
      updateTargetYaw(desire_yaw_vec);

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = 1;
    }

    wps_msg_.pose.position.x = desire_posi_(0);
    wps_msg_.pose.position.y = desire_posi_(1);
    wps_msg_.pose.position.z = desire_posi_(2);
    
    // wps ctrl
    wps_servo_publisher_.publish(wps_msg_);
    last_wps_msg_ = wps_msg_;

    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1)
    {
      if(!if_fan_tip_arrive_)
      {
        if(ptcloud_key_.blade_legal)
        {
          Eigen::Vector3d dir_vec = blade_dir_vec_ * desire_dist_track_;
          
          last_dir_vec_track_ = dir_vec;
          
          Eigen::Vector3d tmp_vec = blade_fpt_posi_ - hub_nor_vec_ * track_dist_ - local_posi_;

          Eigen::Vector3d adj_vec = tmp_vec - blade_dir_vec_ * tmp_vec.dot(blade_dir_vec_);

          desire_posi_ = local_posi_ + dir_vec + adj_vec;

          // calculate gimbal posi
          if(enable_ptcloud_correct_ && dist_to_hub_front_ > dist_enable_sample_track_wps_ * 2)
          {
            desire_gim_vec_ = blade_crt_posi_ - local_posi_;
          }
          else
          {
            desire_gim_vec_ = blade_fpt_posi_ - local_posi_;
          }
          last_legal_gim_vec_ = desire_gim_vec_;

          // calculate target yaw
          Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
          updateTargetYaw(desire_yaw_vec);
        }
        else
        {
          desire_posi_ = local_posi_ + last_dir_vec_track_;
          
          // calculate target yaw
          tgt_yaw_ = last_legal_yaw_;

          // calculate gimbal posi
          desire_gim_vec_ = last_legal_gim_vec_;
        }
      }
      else
      {
        if(vector3dIsZero(tip_side1_posi_[idx_blade]))
          tip_side1_posi_[idx_blade] = Eigen::Vector3d(ptcloud_key_.tip_x, ptcloud_key_.tip_y, ptcloud_key_.tip_z) - hub_nor_vec_ * track_dist_;

        desire_posi_ = tip_side1_posi_[idx_blade];
        
        // calculate target yaw
        tgt_yaw_ = last_legal_yaw_;

        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;
      }
        

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      gimbal_keep_yaw_ = false;

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = 1;

      df_msg_.foot_pt.x = blade_fpt_posi_.x();
      df_msg_.foot_pt.y = blade_fpt_posi_.y();
      df_msg_.foot_pt.z = blade_fpt_posi_.z();
      df_msg_.crt_pt.x = blade_crt_posi_.x();
      df_msg_.crt_pt.y = blade_crt_posi_.y();
      df_msg_.crt_pt.z = blade_crt_posi_.z();
    }
    else if(stage_mode_.stage_mode % 4 == 2)
    {
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;

      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_1_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      if(single_side_ == 1 || single_side_ == 3)
      {
        desire_posi_ = if_traj_end ? tip_side3_posi_[idx_blade] : traj_cur_posi;

        Eigen::Vector3d vec_to_tip = tip_blade_posi_[idx_blade] - local_posi_;
        updateTargetYaw(vec_to_tip);
      }
      else if(single_side_ == 2)
      {
        desire_posi_ = if_traj_end ? tip_side2_posi_[idx_blade] : traj_cur_posi;

        Eigen::Vector3d vec_to_tgt;

        if(curBladeIsToRight())
          if(curBladeIsNormal(idx_blade))
            if(blade_angle_[idx_blade] > 0)
              vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
            else
              vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = -hub_nor_vec_;
        else
          if(blade_angle_[idx_blade] > 0)
            vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;

        updateTargetYaw(vec_to_tgt);
      }
      else if(single_side_ == 4)
      {
        desire_posi_ = if_traj_end ? tip_side4_posi_[idx_blade] : traj_cur_posi;

        Eigen::Vector3d vec_to_tgt;

        if(curBladeIsToRight())
          if(blade_angle_[idx_blade] > 0)
            vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
        else
          if(curBladeIsNormal(idx_blade))
            if(blade_angle_[idx_blade] > 0)
              vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
            else
              vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = -hub_nor_vec_;

        updateTargetYaw(vec_to_tgt);
      }

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      
      
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

      // calculate gimbal posi
      desire_gim_vec_ = tip_blade_posi_[idx_blade] - local_posi_;
      last_legal_gim_vec_ = desire_gim_vec_;

      gimbal_keep_yaw_ = false;
    }
    else if(stage_mode_.stage_mode % 4 == 3)
    {
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;

      Eigen::Vector3d traj_cur_posi;
      
      if(single_side_ == 1 || single_side_ == 3)
        traj_cur_posi = LineTrajPtr_3_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end);
      else if(single_side_ == 2)
        traj_cur_posi = LineTrajPtr_2_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end);
      else if(single_side_ == 4)
        traj_cur_posi = LineTrajPtr_4_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      Eigen::Vector3d tmp_tip;

      if(!if_traj_end)
      {
        desire_posi_ = traj_cur_posi;
      }
      else
      {
        if(single_side_ == 1 || single_side_ == 3)
        {
          desire_posi_ = root_side3_posi_[idx_blade];
          tmp_tip = tip_side3_posi_[idx_blade];
        }
        else if(single_side_ == 2)
        {
          desire_posi_ = root_side2_posi_[idx_blade];
          tmp_tip = tip_side2_posi_[idx_blade];
        }
        else if(single_side_ == 4)
        {
          desire_posi_ = root_side4_posi_[idx_blade];
          tmp_tip = tip_side4_posi_[idx_blade];
        }
      }

      if(!if_gimbal_end_)
      {
        // if((local_posi_ - tmp_tip).norm() < tip_wait_dist_[idx_blade])
        // {
        //   desire_gim_vec_ = tip_posi_true_[idx_blade] - local_posi_;
        //   last_legal_gim_vec_ = desire_gim_vec_;  
        // }
        // else
        // {
          // calculate gimbal posi
          double err = (desire_posi_ - local_posi_).norm();
          double vel = sqrt(pow(set_velo_.x, 2) + pow(set_velo_.y, 2) + pow(set_velo_.z, 2));
          double t = err / vel;
          Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - t, traj_vel_track_, if_gimbal_end_);
        
          df_msg_.err = err;
          df_msg_.vel = vel;
          df_msg_.ttt = t;
          df_msg_.traj_cur_time_minus_t = traj_cur_time - t;

          desire_gim_vec_ = gimbal_tgt - local_posi_;
          last_legal_gim_vec_ = desire_gim_vec_;
        // }
      }
      else
      {
        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;
      }

      if((single_side_ == 2 && !curBladeIsToRight()) || (single_side_ == 4 && curBladeIsToRight()))
        if((local_posi_ - desire_posi_).norm() < thold_keep_yaw_dist_ && !if_gimbal_end_)
          gimbal_keep_yaw_ = true;
        else
          gimbal_keep_yaw_ = false;
      else
        gimbal_keep_yaw_ = false;

      // calculate target yaw
      Eigen::Vector3d vec_to_tgt;

      if(single_side_ == 1 || single_side_ == 3)
      {
        vec_to_tgt = -hub_nor_vec_;
      }
      else if(single_side_ == 2)
      {
        if(curBladeIsToRight())
          if(curBladeIsNormal(idx_blade))
            if(blade_angle_[idx_blade] > 0)
              vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
            else
              vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = -hub_nor_vec_;
        else
          if(blade_angle_[idx_blade] > 0)
            vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
      }
      else if(single_side_ == 4)
      {
        if(curBladeIsToRight())
          if(blade_angle_[idx_blade] > 0)
            vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
        else
          if(curBladeIsNormal(idx_blade))
            if(blade_angle_[idx_blade] > 0)
              vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
            else
              vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
          else
            vec_to_tgt = -hub_nor_vec_;
      }

      // calculate target yaw
      if(single_side_ == 1 || single_side_ == 3)
      {
        Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
        updateTargetYaw(desire_yaw_vec);
      }
      else if(single_side_ == 2)
      {
        if(curBladeIsToRight())
        {
          Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
          updateTargetYaw(desire_yaw_vec);
        }
        else
        {
          if(curBladeIsNormal(idx_blade))
          {
            Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
            updateTargetYaw(desire_yaw_vec);
          }
          else
          {
            updateTargetYaw(vec_to_tgt);
          }
        }
      }
      else if(single_side_ == 4)
      {
        if(curBladeIsToRight())
        {
          if(curBladeIsNormal(idx_blade))
          {
            Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
            updateTargetYaw(desire_yaw_vec);
          }
          else
          {
            updateTargetYaw(vec_to_tgt);
          }
        }
        else
        {
          Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
          updateTargetYaw(desire_yaw_vec);
        }
      }

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      
      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      if(single_side_ == 1 || single_side_ == 3)
        track_index_msg_.track_plane = 2;
      else if(single_side_ == 2)
        track_index_msg_.track_plane = 3;
      else if(single_side_ == 4)
        track_index_msg_.track_plane = 4;

      track_index_msg_.blade_index = idx_blade + 1;
    }
    else if(stage_mode_.stage_mode % 4 == 0)
    {
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;

      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_2_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      desire_posi_ = if_traj_end ? hub_front_posi_ : traj_cur_posi;

      Eigen::Vector3d vec_to_hub = hub_pt_posi_ - local_posi_;
      updateTargetYaw(vec_to_hub);

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

      // calculate gimbal posi
      desire_gim_vec_ = hub_pt_posi_ - local_posi_;
      last_legal_gim_vec_ = desire_gim_vec_;

      gimbal_keep_yaw_ = false;
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1)
    {
      if(!if_fan_tip_arrive_)
      {
        if(ptcloud_key_.blade_legal)
        {
          // calculate target posi
          Eigen::Vector3d dir_vec = blade_dir_vec_ * desire_dist_track_;
          last_dir_vec_track_ = dir_vec;
          
          Eigen::Vector3d tmp_vec = blade_fpt_posi_ - hub_nor_vec_ * track_dist_ - local_posi_;
          Eigen::Vector3d adj_vec = tmp_vec - blade_dir_vec_ * tmp_vec.dot(blade_dir_vec_);

          desire_posi_ = local_posi_ + dir_vec + adj_vec;

          // calculate gimbal posi
          if(enable_ptcloud_correct_ && dist_to_hub_front_ > dist_enable_sample_track_wps_ * 2)
          {
            desire_gim_vec_ = blade_crt_posi_ - local_posi_;
          }
          else
          {
            desire_gim_vec_ = blade_fpt_posi_ - local_posi_;
          }
          last_legal_gim_vec_ = desire_gim_vec_;

          // calculate target yaw
          Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
          updateTargetYaw(desire_yaw_vec);
        }
        else
        {
          // calculate target posi
          desire_posi_ = local_posi_ + last_dir_vec_track_;
          
          // calculate target yaw
          tgt_yaw_ = last_legal_yaw_;

          // calculate gimbal posi
          desire_gim_vec_ = last_legal_gim_vec_;
        }
      }
      else
      {
        if(vector3dIsZero(tip_side1_posi_[idx_blade]))
          tip_side1_posi_[idx_blade] = Eigen::Vector3d(ptcloud_key_.tip_x, ptcloud_key_.tip_y, ptcloud_key_.tip_z) - hub_nor_vec_ * track_dist_;

        // calculate target posi
        desire_posi_ = tip_side1_posi_[idx_blade];
        
        // calculate target yaw
        tgt_yaw_ = last_legal_yaw_;

        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;
      }
        

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      gimbal_keep_yaw_ = false;

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = 1;

      df_msg_.foot_pt.x = blade_fpt_posi_.x();
      df_msg_.foot_pt.y = blade_fpt_posi_.y();
      df_msg_.foot_pt.z = blade_fpt_posi_.z();
      df_msg_.crt_pt.x = blade_crt_posi_.x();
      df_msg_.crt_pt.y = blade_crt_posi_.y();
      df_msg_.crt_pt.z = blade_crt_posi_.z();
    }
    else if(stage_mode_.stage_mode % 8 == 2)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_1_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      desire_posi_ = if_traj_end ? tip_side3_posi_[idx_blade] : traj_cur_posi;

      // calculate target yaw
      Eigen::Vector3d vec_to_tip = tip_blade_posi_[idx_blade] - local_posi_;
      updateTargetYaw(vec_to_tip);

      
      // calculate gimbal posi
      desire_gim_vec_ = vec_to_tip;
      last_legal_gim_vec_ = desire_gim_vec_;


      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

      gimbal_keep_yaw_ = false;
    }
    else if(stage_mode_.stage_mode % 8 == 3)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = LineTrajPtr_3_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      if(!if_traj_end)
        desire_posi_ = traj_cur_posi;
      else
        desire_posi_ = root_side3_posi_[idx_blade];

      if(!if_gimbal_end_)
      {
        // if((local_posi_ - tip_side3_posi_[idx_blade]).norm() < tip_wait_dist_[idx_blade])
        // {
        //   desire_gim_vec_ = tip_posi_true_[idx_blade] - local_posi_;
        //   last_legal_gim_vec_ = desire_gim_vec_;  
        // }
        // else
        // {
          // calculate gimbal posi
          double err = (desire_posi_ - local_posi_).norm();
          double vel = sqrt(pow(set_velo_.x, 2) + pow(set_velo_.y, 2) + pow(set_velo_.z, 2));
          double t = err / vel;
          Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - t, traj_vel_track_, if_gimbal_end_);

          df_msg_.err = err;
          df_msg_.vel = vel;
          df_msg_.ttt = t;
          df_msg_.traj_cur_time_minus_t = traj_cur_time - t;

          desire_gim_vec_ = gimbal_tgt - local_posi_;
          last_legal_gim_vec_ = desire_gim_vec_;
        // }
      }
      else
      {
        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;
      }
      
      // calculate target yaw
      Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
      updateTargetYaw(desire_yaw_vec);

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      
      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      gimbal_keep_yaw_ = false;

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = 2;
    }
    else if(stage_mode_.stage_mode % 8 == 4)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_2_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      desire_posi_ = if_traj_end ? (curBladeIsToRight()? root_side4_posi_[idx_blade] : root_side2_posi_[idx_blade]) : traj_cur_posi;

      // calculate target yaw
      Eigen::Vector3d vec_to_tgt;

      if(blade_angle_[idx_blade] > 0)
        vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
      else
        vec_to_tgt = hub_pt_posi_ - local_posi_;

      updateTargetYaw(vec_to_tgt);

      // calculate gimbal posi
      desire_gim_vec_ = hub_pt_posi_ - local_posi_;
      last_legal_gim_vec_ = desire_gim_vec_;  

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

      gimbal_keep_yaw_ = false;
    }
    else if(stage_mode_.stage_mode % 8 == 5)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      bool tmp_gimbal_end = false;

      if(!if_gimbal_beg_)
      {
        desire_posi_ = curBladeIsToRight()? root_side4_posi_[idx_blade] : root_side2_posi_[idx_blade];
        
        Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_gimbal_, tmp_gimbal_end);

        desire_gim_vec_ = gimbal_tgt - local_posi_;
        last_legal_gim_vec_ = desire_gim_vec_;

        // gimbal_keep_yaw_ = true;

        if(traj_cur_time > gimbal_traj_time_[idx_blade])
        {
          if_gimbal_beg_ = true;
          LineTrajPtr_0_[idx_blade] -> clearReturnLimit();
        }
      }
      else
      {
        Eigen::Vector3d traj_cur_posi = curBladeIsToRight()? 
          (LineTrajPtr_4_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - gimbal_traj_time_[idx_blade], traj_vel_track_, if_traj_end)): 
          (LineTrajPtr_2_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - gimbal_traj_time_[idx_blade], traj_vel_track_, if_traj_end));
        
        gimbal_keep_yaw_ = false;

        if(!if_traj_end)
        {
          desire_posi_ = traj_cur_posi;

          // calculate gimbal posi
          double err = (desire_posi_ - local_posi_).norm();
          // double vel = sqrt(pow(set_velo_.x, 2) + pow(set_velo_.y, 2) + pow(set_velo_.z, 2));
          double vel = traj_vel_track_;
          double t = err / vel;
          double tmp_ttt = traj_cur_time - gimbal_traj_time_[idx_blade] + traj_vel_gimbal_ * gimbal_traj_time_[idx_blade] / traj_vel_track_ - t;
          Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(tmp_ttt, traj_vel_track_, tmp_gimbal_end);

          df_msg_.err = err;
          df_msg_.vel = vel;
          df_msg_.ttt = t;
          df_msg_.traj_cur_time_minus_t = tmp_ttt;

          desire_gim_vec_ = gimbal_tgt - local_posi_;
          last_legal_gim_vec_ = desire_gim_vec_;
        }
        else
        {
          desire_posi_ = curBladeIsToRight()? tip_side4_posi_[idx_blade] : tip_side2_posi_[idx_blade];
          desire_gim_vec_ = last_legal_gim_vec_;
        }
      }

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      // calculate target yaw
      Eigen::Vector3d vec_to_tgt;

      if(blade_angle_[idx_blade] > 0)
        vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
      else
        vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;

      // calculate target yaw
      if(curBladeIsNormal(idx_blade))
      {
        Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
        updateTargetYaw(desire_yaw_vec);
      }
      else
      {
        updateTargetYaw(vec_to_tgt);
      }

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      
      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = curBladeIsToRight()? 4 : 3;
    }
    else if(stage_mode_.stage_mode % 8 == 6)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_3_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      desire_posi_ = if_traj_end ? (curBladeIsToRight()? tip_side2_posi_[idx_blade] : tip_side4_posi_[idx_blade]) : traj_cur_posi;

      // calculate target yaw
      Eigen::Vector3d vec_to_tgt;

      if(curBladeIsNormal(idx_blade))
      {
        if(blade_angle_[idx_blade] > 0)
          vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
        else
          vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
      }
      else
      {
        vec_to_tgt = -hub_nor_vec_;
      }

      updateTargetYaw(vec_to_tgt);

      // calculate gimbal posi
      desire_gim_vec_ = tip_blade_posi_[idx_blade] - local_posi_;
      last_legal_gim_vec_ = desire_gim_vec_;  

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
     
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);

      gimbal_keep_yaw_ = false;
    }
    else if(stage_mode_.stage_mode % 8 == 7)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;

      Eigen::Vector3d traj_cur_posi = curBladeIsToRight()? 
        (LineTrajPtr_2_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end)): 
        (LineTrajPtr_4_[idx_blade] -> getPos_SmoothTraj(traj_cur_time, traj_vel_track_, if_traj_end));

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      if(!if_traj_end)
        desire_posi_ = traj_cur_posi;
      else
        desire_posi_ = curBladeIsToRight()? root_side2_posi_[idx_blade] : root_side4_posi_[idx_blade];

      if(!if_gimbal_end_)
      {
        // if((local_posi_ - (curBladeIsToRight()? tip_side2_posi_[idx_blade] : tip_side4_posi_[idx_blade])).norm() < tip_wait_dist_[idx_blade])
        // {
        //   desire_gim_vec_ = tip_posi_true_[idx_blade] - local_posi_;
        //   last_legal_gim_vec_ = desire_gim_vec_;  
        // }
        // else
        // {
          // calculate gimbal posi
          double err = (desire_posi_ - local_posi_).norm();
          double vel = sqrt(pow(set_velo_.x, 2) + pow(set_velo_.y, 2) + pow(set_velo_.z, 2));
          double t = err / vel;
          Eigen::Vector3d gimbal_tgt = LineTrajPtr_0_[idx_blade] -> getPos_SmoothTraj(traj_cur_time - t, traj_vel_track_, if_gimbal_end_);

          df_msg_.err = err;
          df_msg_.vel = vel;
          df_msg_.ttt = t;
          df_msg_.traj_cur_time_minus_t = traj_cur_time - t;

          desire_gim_vec_ = gimbal_tgt - local_posi_;
          last_legal_gim_vec_ = desire_gim_vec_;
        // }
      }
      else
      {
        // calculate gimbal posi
        desire_gim_vec_ = last_legal_gim_vec_;        
      }
      
      // calculate target yaw
      Eigen::Vector3d vec_to_tgt;
      if(curBladeIsNormal(idx_blade))
      {
        if(blade_angle_[idx_blade] > 0)
          vec_to_tgt = root_blade_posi_[idx_blade] - local_posi_;
        else
          vec_to_tgt = tip_blade_posi_[idx_blade] - local_posi_;
      }
      else
      {
        vec_to_tgt = -hub_nor_vec_;
      }

      // calculate target yaw
      Eigen::Vector3d desire_yaw_vec = Eigen::Vector3d(desire_gim_vec_(0), desire_gim_vec_(1), 0).normalized();
      updateTargetYaw(desire_yaw_vec);

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
      // wps ctrl
      wps_servo_publisher_.publish(wps_msg_);
      last_wps_msg_ = wps_msg_;

      gimbal_keep_yaw_ = false;

      track_index_msg_.blade_index = idx_blade + 1;
      track_index_msg_.track_plane = curBladeIsToRight()? 3 : 4;
    }
    else if(stage_mode_.stage_mode % 8 == 0)
    {
      // calculate target posi
      double traj_cur_time = (ros::Time::now() - traj_begin_time_).toSec();
      bool if_traj_end = false;
      Eigen::Vector3d traj_cur_posi = CircleTrajPtr_4_[idx_blade] -> getPosT(traj_cur_time, if_traj_end);

      df_msg_.traj_cur_time = traj_cur_time;
      df_msg_.if_traj_end = if_traj_end;

      desire_posi_ = if_traj_end ? hub_front_posi_ : traj_cur_posi;
      
      // calculate target yaw
      Eigen::Vector3d vec_to_hub = hub_pt_posi_ - local_posi_;
      updateTargetYaw(vec_to_hub);
      
      // calculate gimbal posi
      desire_gim_vec_ = vec_to_hub;
      last_legal_gim_vec_ = desire_gim_vec_;

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);
      
      // wps ctrl
      if(enable_planning_)
        wps_traj_publisher_.publish(wps_msg_);
      else
        wps_servo_publisher_.publish(wps_msg_);
      
      last_wps_msg_ = wps_msg_;

      // yaw ctrl
      err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
      set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
    
      gimbal_keep_yaw_ = false;
    }
  }

  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_DES)
  {
    if(ptcloud_key_.tower_legal)
    {
      Eigen::Vector3d dir_vec = Eigen::Vector3d(0, 0, -1);
      dir_vec = dir_vec * desire_dist_rise_;
      last_dir_vec_vert_ = dir_vec;

      Eigen::Vector3d dist_vec = tower_nor_vec_ * (detect_dist_ - tower_vrt_dst_);

      desire_posi_ = local_posi_ + dir_vec - dist_vec;

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);

      // calculate target yaw
      updateTargetYaw(tower_nor_vec_);

      // calculate gimbal posi
      desire_gim_vec_ = tower_nor_vec_ * track_dist_;
      last_legal_gim_vec_ = desire_gim_vec_;
    }
    else
    {
      desire_posi_ = local_posi_ + last_dir_vec_vert_ ;

      wps_msg_.pose.position.x = desire_posi_(0);
      wps_msg_.pose.position.y = desire_posi_(1);
      wps_msg_.pose.position.z = desire_posi_(2);

      // calculate target yaw
      tgt_yaw_ = last_legal_yaw_;

      // calculate gimbal posi
      desire_gim_vec_ = last_legal_gim_vec_;
    }

    // yaw ctrl
    err_yaw_ = getYawErr(local_euler_(2), tgt_yaw_);
    set_velo_.yaw = ctrl_yaw_.velCtrl(err_yaw_);
    // wps ctrl
    wps_servo_publisher_.publish(wps_msg_);
    last_wps_msg_ = wps_msg_;
  }

  visPtr_ -> visualize_arrow(local_posi_, desire_posi_, arrow_width_ / 3, str_topic_wps_, visualization::orange);
  Eigen::Vector3d ref_vel = Eigen::Vector3d(set_velo_.x, set_velo_.y, set_velo_.z);
  ref_vel = ref_vel * 5.0;
  visPtr_ -> visualize_arrow(local_posi_, local_posi_ + ref_vel, arrow_width_ / 3, str_topic_vel_, visualization::greenblue);
  Eigen::Vector3d ref_yaw = Eigen::Vector3d(desire_yaw_vec_(0), desire_yaw_vec_(1), 0);
  ref_yaw = ref_yaw / ref_yaw.norm() * 5.0;
  visPtr_ -> visualize_arrow(local_posi_, local_posi_ + ref_yaw, arrow_width_ / 3, str_topic_yaw_, visualization::yellow);

  visPtr_ -> visualize_arrow(local_posi_, local_posi_ + desire_gim_vec_, arrow_width_, str_topic_gim_, visualization::purple);

  visPtr_ -> visualize_arrow(local_posi_ + desire_gim_vec_, local_posi_ + desire_gim_vec_ + gim_cpst_vec_, arrow_width_, str_topic_cps_, visualization::blue);

  visPtr_ -> visualize_a_ball(blade_crt_posi_, arrow_width_, str_topic_crt_, visualization::red, 1.0);

  if(arrow_history_)
    if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
      if((ros::Time::now() - history_time_).toSec() > 3.0)
      {
        velo_history_.emplace_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(local_posi_, local_posi_ + ref_vel));
        visPtr_ -> visualize_arrows(velo_history_, str_topic_hst_velo_, arrow_width_, visualization::greenblue);
        
        gimb_history_.emplace_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(local_posi_, local_posi_ + desire_gim_vec_.normalized() * 5.0));
        visPtr_ -> visualize_arrows(gimb_history_, str_topic_hst_gimb_, arrow_width_, visualization::purple);
        
        history_time_ = ros::Time::now();
      }

  key_positions_[0]  = hub_front_posi_;
  key_positions_[1]  = hub_back_posi_;
  key_positions_[2]  = tip_blade_posi_[0];  key_positions_[3]  = tip_blade_posi_[1];  key_positions_[4]  = tip_blade_posi_[2];
  key_positions_[5]  = root_blade_posi_[0]; key_positions_[6]  = root_blade_posi_[1]; key_positions_[7]  = root_blade_posi_[2];
  key_positions_[8]  = tip_side1_posi_[0];  key_positions_[9]  = tip_side1_posi_[1];  key_positions_[10] = tip_side1_posi_[2];
  key_positions_[11] = tip_side3_posi_[0];  key_positions_[12] = tip_side3_posi_[1];  key_positions_[13] = tip_side3_posi_[2];
  key_positions_[14] = tip_side2_posi_[0];  key_positions_[15] = tip_side2_posi_[1];  key_positions_[16] = tip_side2_posi_[2];
  key_positions_[17] = tip_side4_posi_[0];  key_positions_[18] = tip_side4_posi_[1];  key_positions_[19] = tip_side4_posi_[2];
  key_positions_[20] = root_side1_posi_[0]; key_positions_[21] = root_side1_posi_[1]; key_positions_[22] = root_side1_posi_[2]; 
  key_positions_[23] = root_side3_posi_[0]; key_positions_[24] = root_side3_posi_[1]; key_positions_[25] = root_side3_posi_[2]; 
  key_positions_[26] = root_side2_posi_[0]; key_positions_[27] = root_side2_posi_[1]; key_positions_[28] = root_side2_posi_[2]; 
  key_positions_[29] = root_side4_posi_[0]; key_positions_[30] = root_side4_posi_[1]; key_positions_[31] = root_side4_posi_[2]; 
  key_positions_[32] = hub_pt_posi_; key_positions_[33] = tip_posi_true_[0]; key_positions_[34] = tip_posi_true_[1];  key_positions_[35] = tip_posi_true_[2]; 

  visPtr_ -> visualize_balls(key_positions_, arrow_width_ * 5, str_topic_pts_, visualization::blue, 1.0);

  updateLogState();
  updateBladeTargetPoint();
  updateBladeDistance();
  updateTrackIndex();

  // ROS_INFO("keep yaw: %d", gimbal_keep_yaw_);

  df_msg_.traj_begin_time = traj_begin_time_.toSec();
  df_msg_.gimbal_keep_yaw = gimbal_keep_yaw_;
  df_msg_.if_gimbal_beg = if_gimbal_beg_;
  df_msg_.if_gimbal_end = if_gimbal_end_;
  df_msg_.header.stamp = ros::Time::now();
  debug_flight_publisher_.publish(df_msg_);

  return;
}


void FlightNode::futureOccupiedCheck()
{
  // auto tic = std::chrono::steady_clock::now();
  bool if_have_obs = false;

  double step = 0.5;

  Eigen::Vector3d cur_dir = Eigen::Vector3d(set_velo_.x, set_velo_.y, set_velo_.z).normalized();
  
  std::vector<Eigen::Vector3d> vec = std::vector<Eigen::Vector3d>();

  for(double check_offset = 0.0; check_offset < future_occupied_dist_; check_offset += step)
  {
    Eigen::Vector3d check_posi = local_posi_ + cur_dir * check_offset;
    
    insp_msgs::OccupyCheck srv;
    srv.request.pos_x = check_posi.x();
    srv.request.pos_y = check_posi.y();
    srv.request.pos_z = check_posi.z();
    occupied_check_client_.call(srv);

    if(srv.response.if_occupied)
    {
      if_have_obs = true;
    }

    Eigen::Vector3d center_posi = Eigen::Vector3d(srv.response.center_x, srv.response.center_y, srv.response.center_z);

    if(vec.size() == 0 || !vector3dIsTheSame(center_posi, vec.back()))
      vec.push_back(center_posi);
  }

  if(if_have_obs)
  {
    if(if_pre_occupied_)
      consequent_occupied_count_++;
    if_pre_occupied_ = true;
  }
  else
  {
    if_pre_occupied_ = false;
    consequent_occupied_count_ = 0;
  }

  df_msg_.check_obs_cnt = consequent_occupied_count_;

  if(consequent_occupied_count_ > consequent_occupied_thold_)
  {
    // set_velo_.x = 0.0;
    // set_velo_.y = 0.0;
    // set_velo_.z = 0.0;
    // set_velo_.yaw = last_legal_yaw_;
    // emergencyTerminate();
    df_msg_.have_obs = true;
  }
  else
  {
    df_msg_.have_obs = false;
  }

  visPtr_ -> visualize_pointcloud(vec, str_topic_ftr_);

  // auto toc = std::chrono::steady_clock::now();
  // std::cout<<"[flight node]occupied check time cost: "<<(toc-tic).count() * 1e-6<<"ms"<<std::endl;


  return;
}


void FlightNode::sampleTrajPointCallback(const ros::TimerEvent& event)
{
  dist_to_hub_front_ = (local_posi_ - hub_front_posi_).norm();
  
  int idx_blade = next_track_blade_ - 2;

  std::string str_line_traj = "line_traj_" + std::to_string(idx_blade);
  std::string str_circle_traj = "circle_traj_" + std::to_string(idx_blade);


  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1)
    {
      if(dist_to_hub_front_ > dist_enable_sample_track_wps_)
      {
        Eigen::Vector3d pt0, pt1;

        if(!have_sample_first_wp_[idx_blade])
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_ + hub_nor_vec_ * deep_root_offset_;
          else
            pt0 = hub_pt_posi_ + blade_dir_vec_ * dist_enable_sample_track_wps_ + hub_nor_vec_ * deep_root_offset_;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
          root_blade_posi_[idx_blade] = pt0;
          root_side1_posi_[idx_blade] = pt1;

          have_sample_first_wp_[idx_blade] = true;
        }
        else
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_;
          else
            return;

          pt0 += hub_nor_vec_ * getBladeWidthVertical((pt0 - root_blade_posi_[idx_blade]).norm()) / 3.0;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
        }

        int del_num = LineTrajPtr_0_[idx_blade] -> push_back(pt0);
        for(int i = 0; i < del_num; i++)
          pts_traj_0_[idx_blade].pop_back();
        pts_traj_0_[idx_blade].push_back(pt0);

        LineTrajPtr_0_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);

        LineTrajPtr_1_[idx_blade] -> push_back(pt1);
        for(int i = 0; i < del_num; i++)
          pts_traj_1_[idx_blade].pop_back();
        pts_traj_1_[idx_blade].push_back(pt1);

        LineTrajPtr_1_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);
        
        df_msg_.traj_pt0.x = pt0.x();
        df_msg_.traj_pt0.y = pt0.y();
        df_msg_.traj_pt0.z = pt0.z();
      }
    }
    else
    {
      LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);
      LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1)
    {
      if(dist_to_hub_front_ > dist_enable_sample_track_wps_)
      {
        Eigen::Vector3d pt0, pt1, pt3;

        if(!have_sample_first_wp_[idx_blade])
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_ + hub_nor_vec_ * deep_root_offset_;
          else
            pt0 = hub_pt_posi_ + blade_dir_vec_ * dist_enable_sample_track_wps_ + hub_nor_vec_ * deep_root_offset_;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
          pt3 = pt0 + hub_nor_vec_ * track_dist_;
          root_blade_posi_[idx_blade] = pt0;
          root_side1_posi_[idx_blade] = pt1;
          root_side3_posi_[idx_blade] = pt3;

          have_sample_first_wp_[idx_blade] = true;
        }
        else
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_;
          else
            return;

          pt0 += hub_nor_vec_ * getBladeWidthVertical((pt0 - root_blade_posi_[idx_blade]).norm()) / 3.0;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
          pt3 = pt0 + hub_nor_vec_ * track_dist_;
        }
      
        int del_num = LineTrajPtr_0_[idx_blade] -> push_back(pt0);
        for(int i = 0; i < del_num; i++)
          pts_traj_0_[idx_blade].pop_back();
        pts_traj_0_[idx_blade].push_back(pt0);

        LineTrajPtr_0_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);

        LineTrajPtr_1_[idx_blade] -> push_back(pt1);
        for(int i = 0; i < del_num; i++)
          pts_traj_1_[idx_blade].pop_back();
        pts_traj_1_[idx_blade].push_back(pt1);

        LineTrajPtr_1_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);

        for(int i = 0; i < del_num; i++)
          pts_traj_3_[idx_blade].pop_back();
        pts_traj_3_[idx_blade].push_back(pt3);

        df_msg_.traj_pt0.x = pt0.x();
        df_msg_.traj_pt0.y = pt0.y();
        df_msg_.traj_pt0.z = pt0.z();
      }
    }
    else
    {
      LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);
      LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);
      LineTrajPtr_3_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_3", str_line_traj + "_path_3", traj_pt_size_, visualization::green);
      LineTrajPtr_4_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_4", str_line_traj + "_path_4", traj_pt_size_, visualization::green);
      LineTrajPtr_2_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_2", str_line_traj + "_path_2", traj_pt_size_, visualization::green);
      
      CircleTrajPtr_1_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_1", str_circle_traj + "_path_1", traj_pt_size_, visualization::orange);
      CircleTrajPtr_2_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_2", str_circle_traj + "_path_2", traj_pt_size_, visualization::orange);
      CircleTrajPtr_3_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_3", str_circle_traj + "_path_3", traj_pt_size_, visualization::orange);
      CircleTrajPtr_4_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_4", str_circle_traj + "_path_4", traj_pt_size_, visualization::orange);
    }

  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1)
    {
      if(dist_to_hub_front_ > dist_enable_sample_track_wps_)
      {
        Eigen::Vector3d pt0, pt1, pt3;

        if(!have_sample_first_wp_[idx_blade])
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_ + hub_nor_vec_ * deep_root_offset_;
          else
            pt0 = hub_pt_posi_ + blade_dir_vec_ * dist_enable_sample_track_wps_ + hub_nor_vec_ * deep_root_offset_;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
          pt3 = pt0 + hub_nor_vec_ * track_dist_;
          root_blade_posi_[idx_blade] = pt0;
          root_side1_posi_[idx_blade] = pt1;
          root_side3_posi_[idx_blade] = pt3;

          have_sample_first_wp_[idx_blade] = true;
        }
        else
        {
          if(ptcloud_key_.blade_legal)
            pt0 = blade_fpt_posi_;
          else
            return;

          pt0 += hub_nor_vec_ * getBladeWidthVertical((pt0 - root_blade_posi_[idx_blade]).norm()) / 3.0;

          pt1 = pt0 - hub_nor_vec_ * track_dist_;
          pt3 = pt0 + hub_nor_vec_ * track_dist_;
        }
      
        int del_num = LineTrajPtr_0_[idx_blade] -> push_back(pt0);
        for(int i = 0; i < del_num; i++)
          pts_traj_0_[idx_blade].pop_back();
        pts_traj_0_[idx_blade].push_back(pt0);

        LineTrajPtr_0_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);

        LineTrajPtr_1_[idx_blade] -> push_back(pt1);
        for(int i = 0; i < del_num; i++)
          pts_traj_1_[idx_blade].pop_back();
        pts_traj_1_[idx_blade].push_back(pt1);

        LineTrajPtr_1_[idx_blade] -> smooth_v3(8);
        LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);

        for(int i = 0; i < del_num; i++)
          pts_traj_3_[idx_blade].pop_back();
        pts_traj_3_[idx_blade].push_back(pt3);

        df_msg_.traj_pt0.x = pt0.x();
        df_msg_.traj_pt0.y = pt0.y();
        df_msg_.traj_pt0.z = pt0.z();
      }
    }
    else
    {
      LineTrajPtr_0_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_0", str_line_traj + "_path_0", traj_pt_size_, visualization::greenblue);
      LineTrajPtr_1_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_1", str_line_traj + "_path_1", traj_pt_size_, visualization::green);
      LineTrajPtr_3_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_3", str_line_traj + "_path_3", traj_pt_size_, visualization::green);
      LineTrajPtr_4_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_4", str_line_traj + "_path_4", traj_pt_size_, visualization::green);
      LineTrajPtr_2_[idx_blade] -> vis_points_smooth(str_line_traj + "_point_2", str_line_traj + "_path_2", traj_pt_size_, visualization::green);
      
      CircleTrajPtr_1_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_1", str_circle_traj + "_path_1", traj_pt_size_, visualization::orange);
      CircleTrajPtr_2_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_2", str_circle_traj + "_path_2", traj_pt_size_, visualization::orange);
      CircleTrajPtr_3_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_3", str_circle_traj + "_path_3", traj_pt_size_, visualization::orange);
      CircleTrajPtr_4_[idx_blade] -> vis_circle_traj(str_circle_traj + "_point_4", str_circle_traj + "_path_4", traj_pt_size_, visualization::orange);
    }
  }
}


void FlightNode::setGimbalPositionCallback(const ros::TimerEvent& event)
{
  gimbal_set_vec_msg_.is_reset = gimbal_reset_;
  gimbal_set_vec_msg_.keep_yaw = gimbal_keep_yaw_;

  bool do_adjust = false;

  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1 && (hub_front_posi_ - local_posi_).norm() > dist_enable_sample_track_wps_ * 2)
      do_adjust = true;
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1 && (hub_front_posi_ - local_posi_).norm() > dist_enable_sample_track_wps_ * 2)
      do_adjust = true;
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1 && (hub_front_posi_ - local_posi_).norm() > dist_enable_sample_track_wps_ * 2)
      do_adjust = true;
  }

  Eigen::Vector3d gim_vec_norm = (do_adjust) ? (desire_gim_vec_ + gim_cpst_vec_).normalized() : desire_gim_vec_.normalized();

  gimbal_set_vec_msg_.x = gim_vec_norm(0);
  gimbal_set_vec_msg_.y = gim_vec_norm(1);
  gimbal_set_vec_msg_.z = gim_vec_norm(2);

  set_gimbal_publisher_.publish(gimbal_set_vec_msg_);

  gimbal_reset_ = false;
}


void FlightNode::setCameraZoomCallback(const ros::TimerEvent& event)
{
  df_msg_.zoom_wait_cnt = zoom_wait_cnt_;
  df_msg_.focus_wait_cnt = focus_wait_cnt_;
  df_msg_.tip_focus_wait_cnt = tip_focus_wait_cnt_;
  df_msg_.tip_focus_cnt = tip_focus_cnt_;

  bool special = false;
  bool do_zoom = false;
  bool do_focus = false;

  zoom_wait_cnt_ ++;
  if(zoom_wait_cnt_ >= int(gap_zoom_ * 10))
  {
    do_zoom = true;
    zoom_wait_cnt_ = 0;
  }

  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 3)
      special = true;
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 3 || stage_mode_.stage_mode % 8 == 7)
      special = true;
  }

  if(!special)
  {
    focus_wait_cnt_ ++;
    if(focus_wait_cnt_ >= int(gap_focus_ * 10))
    {
      do_focus = true;
      focus_wait_cnt_ = 0;
    }
  }
  else
  {
    if(tip_focus_cnt_ < tip_focus_thold_)
    {
      tip_focus_wait_cnt_ ++;
      if(tip_focus_wait_cnt_ >= int(tip_focus_gap_ * 10))
      {
        do_focus = true;
        tip_focus_wait_cnt_ = 0;
        tip_focus_cnt_ ++;
      }
    }
    else
    {
      focus_wait_cnt_ ++;
      if(focus_wait_cnt_ >= int(gap_focus_ * 10))
      {
        do_focus = true;
        focus_wait_cnt_ = 0;
      }
    }
  }

  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1){
      if(front_zoom_cnt_ > 3){
        do_zoom = false;
      }
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1){
      if(front_zoom_cnt_ > 3){
        do_zoom = false;
      }
    }
  }

  if(do_zoom)
  {
    std_msgs::Float32 msg;
    double zoom_factor = fan_blade_width_ / blade_width_ * zoom_factor_init_;
    camera_zoom_factor_ = (zoom_factor - zoom_factor_init_) * camera_zoom_k_ + zoom_factor_init_;

    msg.data = camera_zoom_factor_;
    camera_set_zoom_para_publisher_.publish(msg);

    if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
    {
      if(stage_mode_.stage_mode % 4 == 1)
      {
        front_zoom_cnt_ ++;
      }
    }
    else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
    {
      if(stage_mode_.stage_mode % 8 == 1)
      {
        front_zoom_cnt_ ++;
      }
    }
  }

  if(do_focus)
  {
    insp_msgs::CameraFocusPoint msg;
    msg.x = 0.5;
    msg.y = 0.5;
    camera_set_focus_para_publisher_.publish(msg);
  }
}


void FlightNode::sendMsgToVisionCallback(const ros::TimerEvent& event)
{
  int idx_blade = next_track_blade_ - 2;
  // blade_width_ = fan_blade_width_;
  double dist_to_root = fan_blade_length_;
  
  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(vector3dIsZero(root_side1_posi_[idx_blade]))
    {
      dist_to_root = 0.0;
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
    else
    {
      Eigen::Vector3d root_to_local = local_posi_ - root_side1_posi_[idx_blade];
      dist_to_root = root_to_local.norm();
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1)
    {
      if(vector3dIsZero(root_side1_posi_[idx_blade]))
      {
        dist_to_root = 0.0;
        blade_width_ = getBladeWidthHorizontal(dist_to_root);
      }
      else
      {
        Eigen::Vector3d root_to_local = local_posi_ - root_side1_posi_[idx_blade];
        dist_to_root = root_to_local.norm();
        blade_width_ = getBladeWidthHorizontal(dist_to_root);
      }
    }
    else if(stage_mode_.stage_mode % 4 == 2)
    {
      dist_to_root = fan_blade_length_;
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 4 == 3)
    {
      if(single_side_  == 1 ||single_side_  == 3)
      {
        Eigen::Vector3d root_to_local = local_posi_ - root_side3_posi_[idx_blade];
        dist_to_root = root_to_local.norm();
        blade_width_ = getBladeWidthHorizontal(dist_to_root);
      }
      else if(single_side_ == 2)
      {
        Eigen::Vector3d root_to_local = local_posi_ - root_side2_posi_[idx_blade];
        dist_to_root = root_to_local.norm();
        blade_width_ = getBladeWidthVertical(dist_to_root);
      }
      else if(single_side_ == 4)
      {
        Eigen::Vector3d root_to_local = local_posi_ - root_side4_posi_[idx_blade];
        dist_to_root = root_to_local.norm();
        blade_width_ = getBladeWidthVertical(dist_to_root);
      }
    }
    else if(stage_mode_.stage_mode % 4 == 0)
    {
      dist_to_root = 0.0;
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1)
    {
      if(vector3dIsZero(root_side1_posi_[idx_blade]))
      {
        dist_to_root = 0.0;
        blade_width_ = getBladeWidthHorizontal(dist_to_root);
      }
      else
      {
        Eigen::Vector3d root_to_local = local_posi_ - root_side1_posi_[idx_blade];
        dist_to_root = root_to_local.norm();
        blade_width_ = getBladeWidthHorizontal(dist_to_root);
      }
    }
    else if(stage_mode_.stage_mode % 8 == 2)
    {
      dist_to_root = fan_blade_length_;
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 3)
    {
      Eigen::Vector3d root_to_local = local_posi_ - root_side3_posi_[idx_blade];
      dist_to_root = root_to_local.norm();
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 4)
    {
      dist_to_root = 0.0;
      blade_width_ = getBladeWidthHorizontal(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 5)
    {
      if(if_gimbal_beg_)
      {
        Eigen::Vector3d root_to_local = local_posi_ - (curBladeIsToRight()? root_side4_posi_[idx_blade] : root_side2_posi_[idx_blade]);
        dist_to_root = root_to_local.norm();
      }
      else
      {
        dist_to_root = 0.0;
      }
      blade_width_ = getBladeWidthVertical(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 6)
    {
      dist_to_root = fan_blade_length_;
      blade_width_ = curBladeIsNormal(idx_blade)? getBladeWidthVertical(dist_to_root) : getBladeWidthInclined(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 7)
    {
      Eigen::Vector3d root_to_local = local_posi_ - (curBladeIsToRight()? root_side2_posi_[idx_blade] : root_side4_posi_[idx_blade]);
      dist_to_root = root_to_local.norm();
      blade_width_ = curBladeIsNormal(idx_blade)? getBladeWidthVertical(dist_to_root) : getBladeWidthInclined(dist_to_root);
    }
    else if(stage_mode_.stage_mode % 8 == 0)
    {
      dist_to_root = 0.0;
      blade_width_ = getBladeWidthVertical(dist_to_root);
    }
  }

  std_msgs::Float32 width_msg;
  double width_k = blade_width_ / fan_blade_width_;
  width_k = std::min(std::max(width_k, 0.3), 1.5);
  width_msg.data = width_k;
  blade_width_publisher_.publish(width_msg);

  insp_msgs::BladeSideState side_msg;
  side_msg.dist_to_hub_front = (local_posi_ - hub_front_posi_).norm();
  side_msg.adjust_thold = dist_enable_sample_track_wps_ * 2;
  side_msg.dist_to_root = dist_to_root;
  side_msg.fan_blade_length = fan_blade_length_;

  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1){side_msg.side = 1;}
    if(curBladeIsNormal(idx_blade)){side_msg.state = 1;}
    else {side_msg.state = 2;}
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1){side_msg.side = 1;}
    else if(stage_mode_.stage_mode % 4 == 3)
    {
      if(single_side_ == 1 || single_side_ == 3)
      {
        side_msg.side = 2;
      }
      else if(single_side_ == 2)
      {
        if(curBladeIsToRight()){side_msg.side = 4;}
        else {side_msg.side = 3;}
      }
      else if(single_side_ == 4)
      {
        if(curBladeIsToRight()){side_msg.side = 3;}
        else {side_msg.side = 4;}
      }
    }
    if(curBladeIsNormal(idx_blade))
    {
      if(curBladeIsToDown(idx_blade)){side_msg.state = 3;}
      else {side_msg.state = 1;}
    }
    else {side_msg.state = 2;}
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1){side_msg.side = 1;}
    else if(stage_mode_.stage_mode % 8 == 3){side_msg.side = 2;}
    else if(stage_mode_.stage_mode % 8 == 5){side_msg.side = 3;}
    else if(stage_mode_.stage_mode % 8 == 7){side_msg.side = 4;}
    else {side_msg.side = 0;}

    if(curBladeIsNormal(idx_blade))
    {
      if(curBladeIsToDown(idx_blade)){side_msg.state = 3;}
      else {side_msg.state = 1;}
    }
    else {side_msg.state = 2;}
  }
  else
  {
    side_msg.side = 0;
    side_msg.state = 0;
  }
  side_msg.header.stamp = ros::Time::now();
  blade_side_state_publisher_.publish(side_msg);
}


void FlightNode::updateLogState()
{
  std_msgs::Int32 msg;

  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
    log_state_ = 1;
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_ADJ)
    log_state_ = 2;
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_LOC)
    log_state_ = 3;
  else if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
    log_state_ = 4;
  else if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
    if(stage_mode_.stage_mode % 2 == 1)
      log_state_ = 4;
    else
      log_state_ = 5;
  else if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
    if(stage_mode_.stage_mode % 2 == 1)
      log_state_ = 4;
    else
      log_state_ = 5;
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_DES)
    log_state_ = 6;
  else
    log_state_ = 0;
  
  msg.data = log_state_;

  log_state_publisher_.publish(msg);
}

void FlightNode::updateBladeDistance()
{
  std_msgs::Float32 msg;
  
  if(log_state_ == 4)
    msg.data = desire_gim_vec_.norm();
  else
    msg.data = 0.0;
  
  blade_distance_publisher_.publish(msg);
}

void FlightNode::updateBladeTargetPoint()
{
  Eigen::Vector3d target_point = desire_gim_vec_ + local_posi_;

  geometry_msgs::Point msg;
  msg.x = target_point(0);
  msg.y = target_point(1);
  msg.z = target_point(2);

  blade_target_point_publisher_.publish(msg);
}

void FlightNode::updateTrackIndex()
{
  track_index_publisher_.publish(track_index_msg_);
}

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------- show control info ----------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------------------------------- */

void FlightNode::showCtrlInfo(){
  
  char *sm_str;
  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_INIT_MODE_INIT)
    sm_str = "INT_INIT";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
    sm_str = "DTC_RISE";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_ADJ)
    sm_str = "DTC_ADJ";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_LOC)
    sm_str = "DTC_LOC";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_1_GO)
    sm_str = "BLD_1_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_1_BK)
    sm_str = "BLD_1_BK";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_2_GO)
    sm_str = "BLD_2_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_2_BK)
    sm_str = "BLD_2_BK";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_3_GO)
    sm_str = "BLD_3_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_3_BK)
    sm_str = "BLD_3_BK";
  
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_GO)
    sm_str = "BLD_1_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_TP)
    sm_str = "BLD_1_TP";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_BK)
    sm_str = "BLD_1_BK";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_RT)
    sm_str = "BLD_1_RT";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_GO)
    sm_str = "BLD_2_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_TP)
    sm_str = "BLD_2_TP";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_BK)
    sm_str = "BLD_2_BK";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_RT)
    sm_str = "BLD_2_RT";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_GO)
    sm_str = "BLD_3_GO";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_TP)
    sm_str = "BLD_3_TP";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_BK)
    sm_str = "BLD_3_BK";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_RT)
    sm_str = "BLD_3_RT";
  
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S1)
    sm_str = "BLD_1_S1";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_13)
    sm_str = "BLD_1_13";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S3)
    sm_str = "BLD_1_S3";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_34)
    sm_str = "BLD_1_34";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S4)
    sm_str = "BLD_1_S4";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_42)
    sm_str = "BLD_1_42";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S2)
    sm_str = "BLD_1_S2";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_20)
    sm_str = "BLD_1_20";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S1)
    sm_str = "BLD_2_S1";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_13)
    sm_str = "BLD_2_13";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S3)
    sm_str = "BLD_2_S3";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_34)
    sm_str = "BLD_2_34";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S4)
    sm_str = "BLD_2_S4";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_42)
    sm_str = "BLD_2_42";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S2)
    sm_str = "BLD_2_S2";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_20)
    sm_str = "BLD_2_20";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S1)
    sm_str = "BLD_3_S1";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_13)
    sm_str = "BLD_3_13";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S3)
    sm_str = "BLD_3_S3";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_34)
    sm_str = "BLD_3_34";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S4)
    sm_str = "BLD_3_S4";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_42)
    sm_str = "BLD_3_42";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S2)
    sm_str = "BLD_3_S2";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_20)
    sm_str = "BLD_3_20";

  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_DES)
    sm_str = "FNS_DES";
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
    sm_str = "FNS_LND";

  else
    sm_str = "ERROR_SM";


  printf("[%s]: ref(%4.2f, %4.2f, %4.2f, %4.2f)\n", sm_str, set_velo_.x, set_velo_.y, set_velo_.z, RAD2DEG(set_velo_.yaw));
  
  return;
}


void FlightNode::emergencyTerminate()
{
  if((ros::Time::now() - last_terminate_req_time_).toSec() > 3.0)
  {
    insp_msgs::EmergencyTerminate srv;
    srv.request.terminate = true;
    emergency_terminate_client_.call(srv);

    ROS_INFO("[flight node]: call to terminate!!!");

    last_terminate_req_time_ = ros::Time::now();
  }

  ROS_WARN("[flight node]: FUTURE OCCUPIED, COLLISION WARNING!!!");
}


/*#################*/
/*                 */
/*  Main Function  */
/*                 */
/*#################*/

int main(int argc, char** argv){
    
  ros::init(argc, argv, "flight_node");
  
  ros::NodeHandle nh;

  FlightNode flight_node(nh);

  ROS_INFO_STREAM("Flight Node is OK !");

  ros::MultiThreadedSpinner spinner(4);

  spinner.spin();

  return 0;
}
