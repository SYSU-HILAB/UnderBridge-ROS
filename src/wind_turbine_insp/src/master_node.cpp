#include "wind_turbine_insp/master_node.h"


using namespace wind_turbine_insp;


/*################*/
/*  Constructor   */
/*################*/

MasterNode::MasterNode(ros::NodeHandle nh): nh_(nh) {

  MasterNode::initVariable();

  MasterNode::initServiceClient();

  MasterNode::initPublisher();
  MasterNode::initSubscriber();

  ros::Duration(0.5).sleep();
  
  MasterNode::initServiceServer();

  ros::Duration(0.5).sleep();

  MasterNode::initTimer();

}

/*##############*/
/*  Destructor  */
/*##############*/

MasterNode::~MasterNode(){

}


bool MasterNode::initVariable(){

  stage_mode_ = insp_msgs::StageMode::STAGE_INIT_MODE_REF;

  last_update_request_time_ = ros::Time::now();
  info_time_ = ros::Time::now();
  local_frame_created_ = false;

  nh_.getParam("master_node/rate_sm_broadcast",   rate_sm_broadcast_);
  nh_.getParam("master_node/rate_odom_broadcast",   rate_odom_broadcast_);
  nh_.getParam("master_node/rate_routine",   rate_routine_);
  nh_.getParam("master_node/rate_task_ctrl",   rate_task_ctrl_);
  nh_.getParam("master_node/rate_camera_exec",   rate_camera_exec_);

  nh_.getParam("master_node/record_delay_start",   record_delay_start_);
  nh_.getParam("master_node/record_delay_end",   record_delay_end_);
  nh_.getParam("master_node/shoot_delay_start",   shoot_delay_start_);
  nh_.getParam("master_node/shoot_delay_end",   shoot_delay_end_);








  nh_.getParam("master_node/interval_shoot_photo",   interval_shoot_photo_);

  if(interval_shoot_photo_ < 2)
    interval_shoot_photo_ = 2;
  

  flight_task_ = 0;
  nh_.getParam("master_node/gim_ctrl_gap",   gim_ctrl_gap_);

  last_req_time_ = ros::Time::now();

  last_yaw_ = 0.0;
  last_pitch_ = 0.0;

  status_camera_recording_ = false;
  status_camera_shooting_ = false;
  signal_record_start_ = 0;
  signal_record_end_ = 0;
  signal_shoot_start_ = 0;
  signal_shoot_end_ = 0;


  global_posi_ = Eigen::Vector3d::Zero();
  global_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  global_euler_ = Eigen::Vector3d::Zero();
  global_velo_ = Eigen::Vector3d::Zero();
  global_gimbal_euler_ = Eigen::Vector3d::Zero();
  global_gimbal_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());

  local_posi_ = Eigen::Vector3d::Zero();
  local_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  local_euler_ = Eigen::Vector3d::Zero();
  local_velo_ = Eigen::Vector3d::Zero();
  local_gimbal_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  local_gimbal_euler_ = Eigen::Vector3d::Zero();

  set_local_velo_ = Eigen::Vector4d::Zero();
  set_local_gimbal_vec_ = Eigen::Vector3d::Zero();
  set_global_velo_ = Eigen::Vector4d::Zero();
  set_global_gimbal_vec_ = Eigen::Vector3d::Zero();
  set_global_gimbal_euler_ = Eigen::Vector3d::Zero();



  // world ENU to local FLU
  R_WorldENU2LocalFLU_  = Eigen::Matrix3d::Identity();
  t_WorldENU2LocalFLU_  = Eigen::Vector3d::Zero();
  Rt_WorldENU2LocalFLU_ = Eigen::Matrix4d::Identity();
  R_LocalFLU2WorldENU_  = Eigen::Matrix3d::Identity();
  t_LocalFLU2WorldENU_  = Eigen::Vector3d::Zero();
  Rt_LocalFLU2WorldENU_ = Eigen::Matrix4d::Identity();













































































  visPtr_ = std::make_shared<visualization::Visualization>(nh_);

  str_topic_world_x_ = "/wind_turbine_insp/world_axis_x";
  str_topic_world_y_ = "/wind_turbine_insp/world_axis_y";
  str_topic_world_z_ = "/wind_turbine_insp/world_axis_z";
  str_topic_mavros_x_ = "/wind_turbine_insp/mavros_axis_x";
  str_topic_mavros_y_ = "/wind_turbine_insp/mavros_axis_y";
  str_topic_mavros_z_ = "/wind_turbine_insp/mavros_axis_z";

  nh_.getParam("master_node/arrow_width",   arrow_width_);

  str_topic_gim_ = "/wind_turbine_insp/marker_cur_gim";

  printf("--------------------\n");
  printf("Loading Params Start\n");

  printf("rate_sm_broadcast: %.2f\n", rate_sm_broadcast_);
  printf("rate_odom_broadcast: %.2f\n", rate_odom_broadcast_);
  printf("rate_routine: %.2f\n", rate_routine_);
  printf("rate_camera_exec: %.2f\n", rate_camera_exec_);
  printf("rate_task_ctrl: %.2f\n", rate_task_ctrl_);
  printf("record_delay_start: %.2f\n", record_delay_start_);
  printf("record_delay_end: %.2f\n", record_delay_end_);
  printf("shoot_delay_start: %.2f\n", shoot_delay_start_);
  printf("shoot_delay_end: %.2f\n", shoot_delay_end_);





  printf("interval_shoot_photo: %d\n", interval_shoot_photo_);
  printf("arrow_width: %.2f\n", arrow_width_);

  printf("gim_ctrl_gap: %.2f\n", gim_ctrl_gap_);


  printf("Loading Params Finish\n");
  printf("---------------------\n");

  return true;
}


/*###################################*/
/* ServiceClient initialize function */
/*###################################*/

bool MasterNode::initServiceClient(){

  mavros_arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");













  ROS_INFO_STREAM("Master node ServiceClient: OK");
}


/*###################################*/
/* ServiceServer initialize function */
/*###################################*/

bool MasterNode::initServiceServer(){

  set_stage_mode_server_             = nh_.advertiseService("/wind_turbine_insp/set_stage_mode",     &MasterNode::setStageModeCallback,      this);
  set_flight_task_server_            = nh_.advertiseService("/wind_turbine_insp/flight_task_control", &MasterNode::setFlightTaskCallback,    this);
  emergency_terminate_server_        = nh_.advertiseService("/wind_turbine_insp/emergency_terminate", &MasterNode::emergencyTerminateCallback, this);

  ROS_INFO_STREAM("Master node ServiceServer: OK");
}


/*###############################*/
/* Publisher initialize function */
/*###############################*/

bool MasterNode::initPublisher(){

  stage_mode_publisher_                 = nh_.advertise<insp_msgs::StageMode>("/wind_turbine_insp/stage_mode", 2);
  local_odom_publisher_                 = nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/local_odom", 2);
  local_gimbal_quat_publisher_          = nh_.advertise<geometry_msgs::QuaternionStamped>("/wind_turbine_insp/local_gimbal_quat", 2);

  debug_drone_odom_enu_pub =  nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/debug_drone_odom_enu", 2);
  debug_gimbal_odom_enu_pub = nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/debug_gimbal_odom_enu", 2);
  debug_gimbal_odom_ned_pub = nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/debug_gimbal_odom_ned", 2);
  debug_drone_odom_flu_pub =  nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/debug_drone_odom_flu", 2);
  debug_gimbal_odom_flu_pub = nh_.advertise<nav_msgs::Odometry>("/wind_turbine_insp/debug_gimbal_odom_flu", 2);
  
  mavros_set_target_velo_publisher_     = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 2);
  gimbal_set_posi_publisher_            = nh_.advertise<mavros_msgs::MountControl>("/mavros/mount_control/command", 2);
  camera_set_zoom_publisher_            = nh_.advertise<std_msgs::Float32>("/wind_turbine_insp/camera_set_zoom_para", 2);
  emergency_terminate_publisher_        = nh_.advertise<std_msgs::Bool>("/wind_turbine_insp/mobile_set_terminate", 2);

  node_status_publisher_                = nh_.advertise<std_msgs::Int32>("/wind_turbine_insp/node_status", 2);
  
  debug_master_publisher_               = nh_.advertise<geometry_msgs::Twist>("/wind_turbine_insp/debug_master", 2);
  
  ROS_INFO_STREAM("Master node Publisher:     OK");
}

/*################################*/
/* Subscriber initialize function */
/*################################*/

bool MasterNode::initSubscriber(){

  mavros_state_subscriber_            = nh_.subscribe<mavros_msgs::State>("/mavros/state", 2, &MasterNode::mavrosStateCallback, this);
  mavros_local_odom_subscriber_       = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 2, &MasterNode::mavrosLocalOdomCallback, this);

  global_gimbal_euler_subscriber_     = nh_.subscribe<sensor_msgs::Imu>("/typhoon_h480/typhoon_h480/gimbal/imu/data", 2, &MasterNode::getGlobalGimbalEulerCallback, this);
  
  set_velo_subscriber_                = nh_.subscribe<insp_msgs::VeloCmd>("/wind_turbine_insp/set_velo", 2, &MasterNode::setVeloCallback, this);

  gimbal_set_vec_subscriber_          = nh_.subscribe<insp_msgs::GimbalSet>("/wind_turbine_insp/set_gimbal_vec", 2, &MasterNode::gimbalSetVecCallback, this);
  camera_set_zoom_subscriber_         = nh_.subscribe<std_msgs::Float32>("/wind_turbine_insp/set_camera_zoom", 2, &MasterNode::cameraSetZoomCallback, this);
  camera_set_focus_point_subscriber_  = nh_.subscribe<insp_msgs::CameraFocusPoint>("/wind_turbine_insp/set_camera_focus", 1, &MasterNode::cameraSetFocusCallback, this);
  camera_record_video_subscriber_     = nh_.subscribe<std_msgs::Bool>("/wind_turbine_insp/camera_record_video", 10, &MasterNode::cameraRecordVideoCallback, this);
  camera_shoot_photo_subscriber_      = nh_.subscribe<std_msgs::Bool>("/wind_turbine_insp/camera_shoot_photo", 10, &MasterNode::cameraShootPhotoCallback, this);



  ROS_INFO_STREAM("Master node Subscriber:    OK");
}


/*################################*/
/*    Timer initialize function   */
/*################################*/

bool MasterNode::initTimer(){
  
  stage_mode_publisher_timer_ = nh_.createTimer(ros::Rate(rate_sm_broadcast_), &MasterNode::stageModePublisherCallback, this);
  routine_timer_ = nh_.createTimer(ros::Rate(rate_routine_), &MasterNode::routineCallback, this);
  local_odom_publisher_timer_ = nh_.createTimer(ros::Rate(rate_odom_broadcast_), &MasterNode::localOdomPublisherCallback, this);
  camera_execute_timer_ = nh_.createTimer(ros::Rate(rate_camera_exec_), &MasterNode::cameraExecuteCallback, this);
  node_status_timer_ = nh_.createTimer(ros::Rate(1.0), &MasterNode::nodeStatusPublisherCallback, this);
  task_execute_timer_ = nh_.createTimer(ros::Rate(rate_task_ctrl_), &MasterNode::taskExecuteCallback, this);

  ROS_INFO_STREAM("Master node Timer:         OK");
}


/*##############################*/
/* Subscriber callback function */
/*##############################*/

void MasterNode::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
  mavros_state_ = *msg;
}
void MasterNode::mavrosLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  global_posi_ = Eigen::Vector3d(msg -> pose.pose.position.x, 
                                 msg -> pose.pose.position.y, 
                                 msg -> pose.pose.position.z);
  global_quat_ = Eigen::Quaterniond(msg -> pose.pose.orientation.w, 
                                    msg -> pose.pose.orientation.x, 
                                    msg -> pose.pose.orientation.y, 
                                    msg -> pose.pose.orientation.z);
  QuatnVecToRPYVec(global_quat_, global_euler_);
  global_velo_ = Eigen::Vector3d(msg -> twist.twist.linear.x, 
                                 msg -> twist.twist.linear.y,
                                 msg -> twist.twist.linear.z);
}
void MasterNode::getGlobalGimbalEulerCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  global_gimbal_quat_ = Eigen::Quaterniond(msg -> orientation.w, msg -> orientation.x, msg -> orientation.y, msg -> orientation.z).normalized();
  
  QuatnVecToRPYVec(global_gimbal_quat_, global_gimbal_euler_);
  global_gimbal_quat_.normalize();
}

void MasterNode::setVeloCallback(const insp_msgs::VeloCmd::ConstPtr& msg)
{
  set_local_velo_ = Eigen::Vector4d(msg -> x, msg -> y, msg -> z, msg -> yaw);
  set_global_velo_.segment(0, 3) = convertVehicleVelocityLocalFLU2WorldENU(set_local_velo_.segment(0, 3));
  set_global_velo_(3) = convertVehicleYawDotLocalFLU2WorldENU(set_local_velo_(3));

  geometry_msgs::Twist global_velo_set_msg;
  global_velo_set_msg.linear.x = set_global_velo_(0);
  global_velo_set_msg.linear.y = set_global_velo_(1);
  global_velo_set_msg.linear.z = set_global_velo_(2);
  global_velo_set_msg.angular.z = set_global_velo_(3);

  mavros_set_target_velo_publisher_.publish(global_velo_set_msg);
}

void MasterNode::gimbalSetVecCallback(const insp_msgs::GimbalSet::ConstPtr& msg)
{
  set_local_gimbal_vec_ = Eigen::Vector3d(msg -> x, msg -> y, msg -> z);
  set_global_gimbal_vec_ = R_WorldENU2LocalFLU_ * set_local_gimbal_vec_;
  Eigen::Matrix3d rotmat_local2body = local_quat_.toRotationMatrix();
  Eigen::Matrix3d rotmat_body2local = rotmat_local2body.inverse();

  Eigen::Vector3d vec_body = rotmat_body2local * set_local_gimbal_vec_;

  Eigen::Matrix3d rotmat_body2gimbal = Eigen::Matrix3d::Zero();
  rotmat_body2gimbal(0, 0) = 1;
  rotmat_body2gimbal(1, 1) = -1;
  rotmat_body2gimbal(2, 2) = -1;
  Eigen::Matrix3d rotmat_gimbal2body = rotmat_body2gimbal.inverse();

  Eigen::Vector3d vec_gimbal = rotmat_gimbal2body * vec_body;

  double roll = 0.0;
  double pitch = -atan2(vec_gimbal.z(), sqrt(pow(vec_gimbal.x(), 2) + pow(vec_gimbal.y(), 2)));
  double yaw = atan2(vec_gimbal.y(), vec_gimbal.x());
  
  if(msg -> keep_yaw)
  {
    yaw = last_yaw_;
    ROS_INFO("pitch: %.2f, KEEPYAW", RAD2DEG(pitch));
  }
  else
  {
    ROS_INFO("pitch: %.2f, normal", RAD2DEG(pitch));
  }

  last_yaw_ = yaw;
  last_pitch_ = pitch;

  mavros_msgs::MountControl mount_ctrl;

  mount_ctrl.header.stamp = ros::Time::now();
  mount_ctrl.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;

  pitch -= DEG2RAD(6.0);
  
  if(msg -> is_reset)
  {
    mount_ctrl.roll  = 0.0;
    mount_ctrl.pitch = 0.0;
    mount_ctrl.yaw   = 0.0;
  }
  else
  {
    mount_ctrl.roll  = std::max(std::min(RAD2DEG(roll),  +90.0),  -90.0);
    mount_ctrl.pitch = std::max(std::min(RAD2DEG(pitch),  +30.0),  -120.0);
    mount_ctrl.yaw   = std::max(std::min(RAD2DEG(yaw), +180.0), -180.0);
  }

  set_global_gimbal_euler_ = Eigen::Vector3d(mount_ctrl.roll, mount_ctrl.pitch, mount_ctrl.yaw);

  gimbal_set_posi_publisher_.publish(mount_ctrl);

  if(msg -> is_reset)
  {
    ROS_INFO("Reset gimbal once!");
  }
}


void MasterNode::cameraSetZoomCallback(const std_msgs::Float32::ConstPtr& msg)
{
  double zoom_factor = msg -> data;
  zoom_factor = std::max(std::min(zoom_factor, 50.0), 1.5);

  std_msgs::Float32 msg_pub;

  msg_pub.data = zoom_factor;

  while(mutex_.test_and_set()) {}
  camera_set_zoom_publisher_.publish(msg_pub);
  ROS_INFO("--- master_node: call to set zoom ---");
  ros::Duration(gim_ctrl_gap_).sleep();
  mutex_.clear();
}


void MasterNode::cameraSetFocusCallback(const insp_msgs::CameraFocusPoint::ConstPtr& msg)
{
  // do focus





  while(mutex_.test_and_set()) {}
  ros::Duration(1.0).sleep();
  ROS_INFO("--- master_node: call to set focus ---");
  ros::Duration(gim_ctrl_gap_).sleep();
  mutex_.clear();
}


void MasterNode::cameraRecordVideoCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("-----------------------------------");
  if(msg -> data)
  {
    signal_record_start_++;
    ROS_INFO("master_node: receive record start");
  }
  else
  {
    signal_record_end_++;
    ROS_INFO("master_node: receive record end");
  }
  ROS_INFO("-----------------------------------");
}

void MasterNode::cameraShootPhotoCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("-----------------------------------");
  if(msg -> data)
  {
    signal_shoot_start_++;
    ROS_INFO("master_node: receive shoot start");
  }
  else
  {
    signal_shoot_end_++;
    ROS_INFO("master_node: receive shoot end");
  }
  ROS_INFO("-----------------------------------");
}

void MasterNode::cameraRecordVideoDelayExec()
{
  dm_msg_.status_camera_recording = status_camera_recording_;
  dm_msg_.signal_record_start = signal_record_start_;
  dm_msg_.signal_record_end = signal_record_end_;
  if(signal_record_start_ > 0)
  {
    if(!status_camera_recording_)
    {
      // delay time
      for(int i = 0; i < int(record_delay_start_); i++)
      {
        ROS_INFO("sleep %d to call", i + 1);
        ros::Duration(1.0).sleep();
      }
      ROS_INFO("sleep %.2f to call", record_delay_start_);
      ros::Duration(record_delay_start_ - int(record_delay_start_)).sleep();
      
      // call to start record
      ROS_INFO("--- master_node: call to start recording ---");








      status_camera_recording_ = true;
      signal_record_start_--;
    }
    else
    {
      // do nothing
      signal_record_start_--;
    }
  }
  if(signal_record_end_ > 0)
  {
    if(status_camera_recording_)
    {
      // delay time
      for(int i = 0; i < int(record_delay_end_); i++)
      {
        ROS_INFO("sleep %d to call", i + 1);
        ros::Duration(1.0).sleep();
      }
      ROS_INFO("sleep %.2f to call", record_delay_end_);
      ros::Duration(record_delay_end_ - int(record_delay_end_)).sleep();
      // call to end record
      ROS_INFO("--- master_node: call to end recording ---");








      status_camera_recording_ = false;
      signal_record_end_--;
    }
    else
    {
      // do nothing
      signal_record_end_--;
    }
  }
}


void MasterNode::cameraShootPhotoDelayExec()
{
  dm_msg_.status_camera_shooting = status_camera_shooting_;
  dm_msg_.signal_shoot_start = signal_shoot_start_;
  dm_msg_.signal_shoot_end = signal_shoot_end_;
  if(signal_shoot_start_ > 0)
  {
    if(!status_camera_shooting_)
    {
      // delay time
      for(int i = 0; i < int(shoot_delay_start_); i++)
      {
        ROS_INFO("sleep %d to call", i + 1);
        ros::Duration(1.0).sleep();
      }
      ROS_INFO("sleep %.2f to call", shoot_delay_start_);
      ros::Duration(shoot_delay_start_ - int(shoot_delay_start_)).sleep();
      
      // call to shoot photo
      ROS_INFO("--- master_node: call to start shooting ---");









      status_camera_shooting_ = true;
      signal_shoot_start_--;
    }
    else
    {
      signal_shoot_start_--;
    }
  }
  if(signal_shoot_end_ > 0)
  {
    if(status_camera_shooting_)
    {
      // delay time
      for(int i = 0; i < int(shoot_delay_end_); i++)
      {
        ROS_INFO("sleep %d to call", i + 1);
        ros::Duration(1.0).sleep();
      }
      ROS_INFO("sleep %.2f to call", shoot_delay_end_);
      ros::Duration(shoot_delay_end_ - int(shoot_delay_end_)).sleep();
      
      // call to shoot photo
      ROS_INFO("--- master_node: call to end shooting ---");







      status_camera_shooting_ = false;
      signal_shoot_end_--;
    }
    else
    {
      // do nothing
      signal_shoot_end_--;
    }
  }
}


















/*##################################*/
/* Service Server Callback function */
/*##################################*/

bool MasterNode::setStageModeCallback(insp_msgs::SetStageMode::Request& request, insp_msgs::SetStageMode::Response& response){

  if(request.source == insp_msgs::SetStageMode::Request::SOURCE_ADMIN)
  {
    stage_mode_ = request.new_stage_mode;
    response.result = true;
    ROS_INFO("master_node: Recv stage_mode update order, force set to %d", stage_mode_);
    last_update_request_time_ = ros::Time::now();
  }
  else if(request.source == insp_msgs::SetStageMode::Request::SOURCE_CODE)
  {
    ros::Time cur_time = ros::Time::now();
    
    if((cur_time - last_update_request_time_).toSec() < 5.0)
    {
      response.result = false;
      if((cur_time - info_time_).toSec() > 1.0){
        ROS_INFO("master_node: Recv stage_mode update req to %d, Reject, req illegal", request.new_stage_mode);
        info_time_ = ros::Time::now();
      }
    }
    else{
      stage_mode_ = request.new_stage_mode;
      response.result = true;
      ROS_INFO("master_node: Recv stage_mode update req, Accept, set to %d", stage_mode_);
      last_update_request_time_ = ros::Time::now();

      if(stage_mode_ == insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
      {
        ros::Duration(1.0).sleep();
        system("gnome-terminal -- bash -c 'rosnode kill $(rosnode list) | grep record_1'");
        ros::Duration(1.0).sleep();
        ros::shutdown();
      }
    }
  }

  return response.result;
}


bool MasterNode::setFlightTaskCallback(insp_msgs::FlightTaskControl::Request& request, insp_msgs::FlightTaskControl::Response& response){
  
  switch(request.task)
  {
    case insp_msgs::FlightTaskControl::Request::TASK_TAKEOFF:
    {
      flight_task_ = insp_msgs::FlightTaskControl::Request::TASK_TAKEOFF;

      if(local_posi_.z() < 2.0){
        response.result = false;
      }else{
        response.result = true;
        ROS_INFO_STREAM("master_node: vehicle take off to 2m.");
        flight_task_ = 0;
      }

      break;
    }
    case insp_msgs::FlightTaskControl::Request::TASK_LAND:
    {
      flight_task_ = insp_msgs::FlightTaskControl::Request::TASK_LAND;

      if(local_posi_.z() > 0.1){
        response.result = false;
      }else{
        response.result = true;
        ROS_INFO_STREAM("master_node: vehicle land.");
        flight_task_ = 0;
      }
      break;
    }
    
    default:
    {
      ROS_INFO_STREAM("master_node: Flight Task Command: No Recognized Command!!!");
      response.result = false;
      break;
    }
  }

  return response.result;
}

bool MasterNode::emergencyTerminateCallback(insp_msgs::EmergencyTerminate::Request& request, insp_msgs::EmergencyTerminate::Response& response)
{
  std_msgs::Bool msg;
  msg.data = request.terminate;
  emergency_terminate_publisher_.publish(msg);
  ROS_INFO("[master node]: call to terminate!!!");
  response.result = true;
  return response.result;
}

/*###################################*/
/*      Timer Callback function      */
/*###################################*/

void MasterNode::stageModePublisherCallback(const ros::TimerEvent& event)
{
  insp_msgs::StageMode msg;
  msg.stage_mode = stage_mode_;
  stage_mode_publisher_.publish(msg);
}

void MasterNode::routineCallback(const ros::TimerEvent& event){
  
  if(stage_mode_ == insp_msgs::StageMode::STAGE_INIT_MODE_REF){
    createLocalFrame();


    stage_mode_ = insp_msgs::StageMode::STAGE_INIT_MODE_INIT;
    ROS_INFO("master_node: position reference set, stage mode set to %d", stage_mode_);
  }

  Eigen::Vector3d origin_posi = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d world_x = Eigen::Vector3d(1, 0, 0);
  Eigen::Vector3d world_y = Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d world_z = Eigen::Vector3d(0, 0, 1);
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + world_x * 50, arrow_width_ * 3, str_topic_world_x_, visualization::red);
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + world_y * 50, arrow_width_ * 3, str_topic_world_y_, visualization::green);
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + world_z * 50, arrow_width_ * 3, str_topic_world_z_, visualization::blue);
  Eigen::Vector3d mavros_x = R_WorldENU2LocalFLU_ * world_x;
  Eigen::Vector3d mavros_y = R_WorldENU2LocalFLU_ * world_y;
  Eigen::Vector3d mavros_z = R_WorldENU2LocalFLU_ * world_z;
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + mavros_x * 100, arrow_width_, str_topic_mavros_x_, visualization::red);
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + mavros_y * 100, arrow_width_, str_topic_mavros_y_, visualization::green);
  visPtr_ -> visualize_arrow(origin_posi, origin_posi + mavros_z * 100, arrow_width_, str_topic_mavros_z_, visualization::blue);

}


void MasterNode::localOdomPublisherCallback(const ros::TimerEvent& event){

  if(local_frame_created_)
  {

    local_posi_ = convertVehiclePositionWorldENU2LocalFLU(global_posi_);
    local_quat_ = convertVehicleAttitudeWorldENU2LocalFLU(global_quat_);
    QuatnVecToRPYVec(local_quat_, local_euler_);
    local_velo_ = convertVehicleVelocityWorldENU2LocalFLU(global_velo_);
    local_gimbal_quat_ = convertGimbalAttitudeWorldENU2LocalFLU(global_gimbal_quat_);
    QuatnVecToRPYVec(local_gimbal_quat_, local_gimbal_euler_);
  }
  else
  {
    global_posi_ = Eigen::Vector3d::Zero();
    local_posi_ = Eigen::Vector3d::Zero();
    local_euler_ = Eigen::Vector3d::Zero();
    local_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    local_velo_ = Eigen::Vector3d::Zero();
    local_gimbal_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    local_gimbal_euler_ = Eigen::Vector3d::Zero();
  }

  nav_msgs::Odometry local_odom_msg;

  local_odom_msg.header.stamp = ros::Time::now();
  local_odom_msg.header.frame_id = "world";
  local_odom_msg.child_frame_id = "body";

  local_odom_msg.pose.pose.position.x = local_posi_.x();
  local_odom_msg.pose.pose.position.y = local_posi_.y();
  local_odom_msg.pose.pose.position.z = local_posi_.z();

  local_odom_msg.pose.pose.orientation.w = local_quat_.w();
  local_odom_msg.pose.pose.orientation.x = local_quat_.x();
  local_odom_msg.pose.pose.orientation.y = local_quat_.y();
  local_odom_msg.pose.pose.orientation.z = local_quat_.z();

  local_odom_msg.twist.twist.linear.x = local_velo_.x();
  local_odom_msg.twist.twist.linear.y = local_velo_.y();
  local_odom_msg.twist.twist.linear.z = local_velo_.z();

  local_odom_publisher_.publish(local_odom_msg);

/* ---------------------------------------------------------------------------------- */

  nav_msgs::Odometry drone_odom_enu;
  drone_odom_enu.header.stamp = ros::Time::now();
  drone_odom_enu.header.frame_id = "enu";
  drone_odom_enu.child_frame_id = "drone";
  drone_odom_enu.pose.pose.position.x = global_posi_.x();
  drone_odom_enu.pose.pose.position.y = global_posi_.y();
  drone_odom_enu.pose.pose.position.z = global_posi_.z();
  drone_odom_enu.pose.pose.orientation.w = global_quat_.w();
  drone_odom_enu.pose.pose.orientation.x = global_quat_.x();
  drone_odom_enu.pose.pose.orientation.y = global_quat_.y();
  drone_odom_enu.pose.pose.orientation.z = global_quat_.z();
  drone_odom_enu.twist.twist.linear.x = (global_velo_).x();
  drone_odom_enu.twist.twist.linear.y = (global_velo_).y();
  drone_odom_enu.twist.twist.linear.z = (global_velo_).z();
  debug_drone_odom_enu_pub.publish(drone_odom_enu);



  // NOTE : Vis the odom
  nav_msgs::Odometry gim_odom_enu;
  gim_odom_enu.header.stamp = ros::Time::now();
  gim_odom_enu.header.frame_id = "enu";
  gim_odom_enu.child_frame_id = "gimbal";
  gim_odom_enu.pose.pose.position.x = global_posi_.x();
  gim_odom_enu.pose.pose.position.y = global_posi_.y();
  gim_odom_enu.pose.pose.position.z = global_posi_.z();
  gim_odom_enu.pose.pose.orientation.w = global_gimbal_quat_.w();
  gim_odom_enu.pose.pose.orientation.x = global_gimbal_quat_.x();
  gim_odom_enu.pose.pose.orientation.y = global_gimbal_quat_.y();
  gim_odom_enu.pose.pose.orientation.z = global_gimbal_quat_.z();
  gim_odom_enu.twist.twist.linear.x = (global_velo_).x();
  gim_odom_enu.twist.twist.linear.y = (global_velo_).y();
  gim_odom_enu.twist.twist.linear.z = (global_velo_).z();
  debug_gimbal_odom_enu_pub.publish(gim_odom_enu);

  nav_msgs::Odometry drone_odom_flu;
  drone_odom_flu.header.stamp = ros::Time::now();
  drone_odom_flu.header.frame_id = "flu";
  drone_odom_flu.child_frame_id = "drone";
  drone_odom_flu.pose.pose.position.x = local_posi_.x();
  drone_odom_flu.pose.pose.position.y = local_posi_.y();
  drone_odom_flu.pose.pose.position.z = local_posi_.z();
  drone_odom_flu.pose.pose.orientation.w = local_quat_.w();
  drone_odom_flu.pose.pose.orientation.x = local_quat_.x();
  drone_odom_flu.pose.pose.orientation.y = local_quat_.y();
  drone_odom_flu.pose.pose.orientation.z = local_quat_.z();
  drone_odom_flu.twist.twist.linear.x = local_velo_.x();
  drone_odom_flu.twist.twist.linear.y = local_velo_.y();
  drone_odom_flu.twist.twist.linear.z = local_velo_.z();
  debug_drone_odom_flu_pub.publish(drone_odom_flu);

  nav_msgs::Odometry gim_odom_flu;
  gim_odom_flu.header.stamp = ros::Time::now();
  gim_odom_flu.header.frame_id = "flu";
  gim_odom_flu.child_frame_id = "gimbal";
  gim_odom_flu.pose.pose.position.x = local_posi_.x();
  gim_odom_flu.pose.pose.position.y = local_posi_.y();
  gim_odom_flu.pose.pose.position.z = local_posi_.z();
  gim_odom_flu.pose.pose.orientation.w = local_gimbal_quat_.w();
  gim_odom_flu.pose.pose.orientation.x = local_gimbal_quat_.x();
  gim_odom_flu.pose.pose.orientation.y = local_gimbal_quat_.y();
  gim_odom_flu.pose.pose.orientation.z = local_gimbal_quat_.z();
  gim_odom_flu.twist.twist.linear.x = local_velo_.x();
  gim_odom_flu.twist.twist.linear.y = local_velo_.y();
  gim_odom_flu.twist.twist.linear.z = local_velo_.z();
  debug_gimbal_odom_flu_pub.publish(gim_odom_flu);

  nav_msgs::Odometry gim_ned_odom;
  gim_ned_odom.header.stamp = ros::Time::now();
  gim_ned_odom.header.frame_id = "ned";
  gim_ned_odom.child_frame_id = "gimbal";
  gim_ned_odom.pose.pose.orientation.w = global_gimbal_quat_.normalized().w();
  gim_ned_odom.pose.pose.orientation.x = global_gimbal_quat_.normalized().x();
  gim_ned_odom.pose.pose.orientation.y = global_gimbal_quat_.normalized().y();
  gim_ned_odom.pose.pose.orientation.z = global_gimbal_quat_.normalized().z();
  debug_gimbal_odom_ned_pub.publish(gim_ned_odom);
  
  /* ---------------------------------------------------------------------------------- */

  geometry_msgs::QuaternionStamped local_gimbal_quat_msg;

  local_gimbal_quat_msg.header.stamp = ros::Time::now();
  local_gimbal_quat_msg.header.frame_id = "world";

  local_gimbal_quat_msg.quaternion.w = local_gimbal_quat_.w();
  local_gimbal_quat_msg.quaternion.x = local_gimbal_quat_.x();
  local_gimbal_quat_msg.quaternion.y = local_gimbal_quat_.y();
  local_gimbal_quat_msg.quaternion.z = local_gimbal_quat_.z();

  local_gimbal_quat_publisher_.publish(local_gimbal_quat_msg);

  updateDebugMasterMsg();
  debug_master_publisher_.publish(dm_msg_);


  Eigen::Vector3d vec_cur = local_gimbal_quat_.normalized().toRotationMatrix() * Eigen::Vector3d(1, 0, 0);
  visPtr_ -> visualize_arrow(local_posi_, local_posi_ + vec_cur * 20, arrow_width_, str_topic_gim_, visualization::yellow);
  
  return;
}


void MasterNode::cameraExecuteCallback(const ros::TimerEvent& event)
{
  cameraRecordVideoDelayExec();
  cameraShootPhotoDelayExec();
}


void MasterNode::nodeStatusPublisherCallback(const ros::TimerEvent& event)
{
  std_msgs::Int32 msg;
  msg.data = 3;
  node_status_publisher_.publish(msg);
}


void MasterNode::taskExecuteCallback(const ros::TimerEvent& event){

  switch (flight_task_)
  {
    case insp_msgs::FlightTaskControl::Request::TASK_TAKEOFF:
    { 
      mavros_msgs::SetMode mavros_set_mode;
      mavros_set_mode.request.custom_mode = "OFFBOARD";

      mavros_msgs::CommandBool mavros_arm_cmd;
      mavros_arm_cmd.request.value = true;

      geometry_msgs::Twist mavros_target_velo;
      mavros_target_velo.linear.x = 0;
      mavros_target_velo.linear.y = 0;
      mavros_target_velo.linear.z = 0.5;

      if(mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_req_time_ > ros::Duration(3.0))){
        
        if(mavros_set_mode_client_.call(mavros_set_mode) && mavros_set_mode.response.mode_sent){
          ROS_INFO_STREAM("master_node: MAVROS: mode OFFBOARD enabled");
        }
        last_req_time_ = ros::Time::now();

      }else{
        
        if(!mavros_state_.armed && (ros::Time::now() - last_req_time_ > ros::Duration(3.0))){
          
          if(mavros_arming_client_.call(mavros_arm_cmd) && mavros_arm_cmd.response.success){
            ROS_INFO_STREAM("master_node: MAVROS: vehicle armed");
          }
          last_req_time_ = ros::Time::now();
        }
      }
      mavros_set_target_velo_publisher_.publish(mavros_target_velo);

      break;
    }
    case insp_msgs::FlightTaskControl::Request::TASK_LAND:
    {

      mavros_msgs::SetMode mavros_set_mode;
      mavros_set_mode.request.custom_mode = "AUTO.LAND";

      if(mavros_state_.mode != "AUTO.LAND" && (ros::Time::now() - last_req_time_ > ros::Duration(3.0))){
        
        if(mavros_set_mode_client_.call(mavros_set_mode) && mavros_set_mode.response.mode_sent){
          ROS_INFO_STREAM("master_node: MAVROS: mode AUTO.LAND enabled");
        }

        last_req_time_ = ros::Time::now();
      }
      
      break;
    }

    default:
      break;
  }

}




/* ################################################ */
/* ----------- frame transform function ----------- */
/* ################################################ */

void MasterNode::createLocalFrame(){




  // world ENU to local FLU
  ROS_INFO("cur euler: (%.2f, %.2f, %.2f)", RAD2DEG(global_euler_(0)), RAD2DEG(global_euler_(1)), RAD2DEG(global_euler_(2)));
  Eigen::Vector3d ref_euler = Eigen::Vector3d(0, 0, global_euler_(2));
  Eigen::Quaterniond ref_quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  RPYVecToQuatnVec(ref_euler, ref_quat);
  R_WorldENU2LocalFLU_ = ref_quat.normalized().toRotationMatrix();
  t_WorldENU2LocalFLU_ = local_posi_;

  R_LocalFLU2WorldENU_ = R_WorldENU2LocalFLU_.inverse();
  t_LocalFLU2WorldENU_ = R_WorldENU2LocalFLU_.inverse() * (-t_WorldENU2LocalFLU_);

  Rt_WorldENU2LocalFLU_.block(0, 0, 3, 3) = R_WorldENU2LocalFLU_;
  Rt_WorldENU2LocalFLU_.block(0, 3, 3, 1) = t_WorldENU2LocalFLU_;

  Rt_LocalFLU2WorldENU_.block(0, 0, 3, 3) = R_LocalFLU2WorldENU_;
  Rt_LocalFLU2WorldENU_.block(0, 3, 3, 1) = t_LocalFLU2WorldENU_;



















  local_frame_created_ = true;

  ROS_INFO_STREAM("[master node]: LOCAL FRAME CREATED!!!");

  return;
}











































Eigen::Vector3d MasterNode::convertVehiclePositionWorldENU2LocalFLU(Eigen::Vector3d posi_enu)
{
  Eigen::Vector4d homo_posi_enu = Eigen::Vector4d(posi_enu(0), posi_enu(1), posi_enu(2), 1);
  Eigen::Vector4d homo_posi_flu = Rt_LocalFLU2WorldENU_ * homo_posi_enu;
  Eigen::Vector3d posi_flu = Eigen::Vector3d(homo_posi_flu(0), homo_posi_flu(1), homo_posi_flu(2));
  return posi_flu;
}

Eigen::Quaterniond MasterNode::convertVehicleAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu)
{
  Eigen::Quaterniond quat_flu = Eigen::Quaterniond(R_LocalFLU2WorldENU_).normalized() * quat_enu;
  return quat_flu.normalized();
}

Eigen::Vector3d MasterNode::convertVehicleVelocityWorldENU2LocalFLU(Eigen::Vector3d velo_enu)
{
  Eigen::Vector3d velo_flu = R_LocalFLU2WorldENU_ * velo_enu;
  return velo_flu;
}
// convert gimbal attitude from world ENU to local FLU
Eigen::Quaterniond MasterNode::convertGimbalAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu)
{
  Eigen::Quaterniond quat_flu = Eigen::Quaterniond(R_LocalFLU2WorldENU_).normalized() * quat_enu;
  return quat_flu.normalized();
}

/* data transform to control */
// convert vehicle velo from local FLU to world ENU
Eigen::Vector3d MasterNode::convertVehicleVelocityLocalFLU2WorldENU(Eigen::Vector3d velo_flu)
{
  Eigen::Vector3d velo_enu = R_WorldENU2LocalFLU_ * velo_flu;
  return velo_enu;
}
// convert vehicle yaw dot from local FLU to world ENU
double MasterNode::convertVehicleYawDotLocalFLU2WorldENU(double yaw_flu)
{
  Eigen::Vector3d euler_flu = Eigen::Vector3d(0, 0, yaw_flu);
  Eigen::Vector3d euler_enu = R_WorldENU2LocalFLU_ * euler_flu;
  double yaw_enu = euler_enu(2);
  return yaw_enu;
}








void MasterNode::updateDebugMasterMsg()
{
  dm_msg_.header.stamp = ros::Time::now();





  dm_msg_.get_world_posi_enu.x = global_posi_.x();
  dm_msg_.get_world_posi_enu.y = global_posi_.y();
  dm_msg_.get_world_posi_enu.z = global_posi_.z();

  // dm_msg_.get_world_quat_enu.w = global_quat_.w();
  // dm_msg_.get_world_quat_enu.x = global_quat_.x();
  // dm_msg_.get_world_quat_enu.y = global_quat_.y();
  // dm_msg_.get_world_quat_enu.z = global_quat_.z();

  dm_msg_.get_world_euler_enu.x = RAD2DEG(global_euler_.x());
  dm_msg_.get_world_euler_enu.y = RAD2DEG(global_euler_.y());
  dm_msg_.get_world_euler_enu.z = RAD2DEG(global_euler_.z());

  dm_msg_.get_world_velo_ned.x = global_velo_.x();
  dm_msg_.get_world_velo_ned.y = global_velo_.y();
  dm_msg_.get_world_velo_ned.z = global_velo_.z();

  dm_msg_.get_world_gim_euler_ned.x = RAD2DEG(global_gimbal_euler_.x());
  dm_msg_.get_world_gim_euler_ned.y = RAD2DEG(global_gimbal_euler_.y());
  dm_msg_.get_world_gim_euler_ned.z = RAD2DEG(global_gimbal_euler_.z());

  // dm_msg_.get_world_gim_quat_ned.w = global_gimbal_quat_.w();
  // dm_msg_.get_world_gim_quat_ned.x = global_gimbal_quat_.x();
  // dm_msg_.get_world_gim_quat_ned.y = global_gimbal_quat_.y();
  // dm_msg_.get_world_gim_quat_ned.z = global_gimbal_quat_.z();

  dm_msg_.get_local_posi_flu.x = local_posi_.x();
  dm_msg_.get_local_posi_flu.y = local_posi_.y();
  dm_msg_.get_local_posi_flu.z = local_posi_.z();

  // dm_msg_.get_local_quat_flu.w = local_quat_.w();
  // dm_msg_.get_local_quat_flu.x = local_quat_.x();
  // dm_msg_.get_local_quat_flu.y = local_quat_.y();
  // dm_msg_.get_local_quat_flu.z = local_quat_.z();

  dm_msg_.get_local_euler_flu.x = RAD2DEG(local_euler_.x());
  dm_msg_.get_local_euler_flu.y = RAD2DEG(local_euler_.y());
  dm_msg_.get_local_euler_flu.z = RAD2DEG(local_euler_.z());

  dm_msg_.get_local_velo_flu.x = local_velo_.x();
  dm_msg_.get_local_velo_flu.y = local_velo_.y();
  dm_msg_.get_local_velo_flu.z = local_velo_.z();

  // dm_msg_.get_local_gim_quat_flu.w = local_gimbal_quat_.w();
  // dm_msg_.get_local_gim_quat_flu.x = local_gimbal_quat_.x();
  // dm_msg_.get_local_gim_quat_flu.y = local_gimbal_quat_.y();
  // dm_msg_.get_local_gim_quat_flu.z = local_gimbal_quat_.z();

  dm_msg_.get_local_gim_euler_flu.x = RAD2DEG(local_gimbal_euler_.x());
  dm_msg_.get_local_gim_euler_flu.y = RAD2DEG(local_gimbal_euler_.y());
  dm_msg_.get_local_gim_euler_flu.z = RAD2DEG(local_gimbal_euler_.z());

  dm_msg_.set_local_velo_flu.x = set_local_velo_(0);
  dm_msg_.set_local_velo_flu.y = set_local_velo_(1);
  dm_msg_.set_local_velo_flu.z = set_local_velo_(2);
  dm_msg_.set_local_yaw_dot_flu = set_local_velo_(3);

  dm_msg_.set_local_gim_vector_flu.x = set_local_gimbal_vec_.x();
  dm_msg_.set_local_gim_vector_flu.y = set_local_gimbal_vec_.y();
  dm_msg_.set_local_gim_vector_flu.z = set_local_gimbal_vec_.z();

  dm_msg_.set_world_velo_neu.x = set_global_velo_(0);
  dm_msg_.set_world_velo_neu.y = set_global_velo_(1);
  dm_msg_.set_world_velo_neu.z = set_global_velo_(2);
  dm_msg_.set_world_yaw_dot_neu = set_global_velo_(3);

  dm_msg_.set_world_gim_vector_ned.x = set_global_gimbal_vec_.x();
  dm_msg_.set_world_gim_vector_ned.y = set_global_gimbal_vec_.y();
  dm_msg_.set_world_gim_vector_ned.z = set_global_gimbal_vec_.z();

  dm_msg_.set_world_gim_euler_ned.x = set_global_gimbal_euler_.x();
  dm_msg_.set_world_gim_euler_ned.y = set_global_gimbal_euler_.y();
  dm_msg_.set_world_gim_euler_ned.z = set_global_gimbal_euler_.z();

  return;
}



/*#################*/
/*                 */
/*  Main Function  */
/*                 */
/*#################*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "master_node");

  ROS_INFO_STREAM("Master node initialize start, waiting...");

  ros::NodeHandle nh;
  
  MasterNode master_node(nh);

  ROS_INFO_STREAM("Master node initialize finish");
  ROS_INFO_STREAM("Master node is Ready !!!");

  ros::MultiThreadedSpinner spinner(6);

  spinner.spin();

  return 0;
}