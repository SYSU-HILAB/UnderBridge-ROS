#include "wind_turbine_insp/record_node.h"


using namespace wind_turbine_insp;


RecordNode::RecordNode(ros::NodeHandle nh): nh_(nh)
{
  initVariable();
  initSubscriber();
  initTimer();
}

RecordNode::~RecordNode()
{

}

bool RecordNode::initVariable()
{
  nh_.getParam("record_node/rate_record",   rate_record_);
  nh_.getParam("record_node/filename",   filename_);


  std::string time_string = getTimeString();
  std::string log_path = "/home/hilab/wind_turbine_inspection/log/";
  std::string log_filename_str = log_path + filename_ + "_" + time_string + ".txt";
  const char *log_filename_p = log_filename_str.c_str();

  ROS_INFO("log file name: %s", log_filename_p);

  pFile_ = std::fopen(log_filename_p, "w");
  

  std::fprintf(pFile_, "time_stamp(s), longitude(deg), latitude(deg), altitude(m), position_x(m), position_y(m), position_z(m), attitude_roll(deg), attitude_pitch(deg), attitude_yaw(deg), velocity_x(m/s), velocity_y(m/s), velocity_z(m/s), gimbal_roll(deg), gimbal_pitch(deg), gimbal_yaw(deg), focal_len(mm), distance_to_blade(m), current_state\n");

  log_record_ = true;
  
  sigIntHandler_.sa_handler = shutDownHandler;
  sigemptyset(&sigIntHandler_.sa_mask);
  sigIntHandler_.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler_, NULL);

  local_posi_ = Eigen::Vector3d::Zero();
  local_atti_ = Eigen::Vector3d::Zero();
  local_velo_ = Eigen::Vector3d::Zero();
  gimbal_atti_ = Eigen::Vector3d::Zero();
  focal_len_ = 0.0;
  blade_dist_ = 0.0;
  log_state_ = 0;

  printf("--------------------\n");
  printf("Loading Params Start\n");

  printf("rate_record: %.2f\n", rate_record_);

  printf("Loading Params Finish\n");
  printf("---------------------\n");

  return true;
}

bool RecordNode::initSubscriber()
{
  stage_mode_subscriber_ = nh_.subscribe<insp_msgs::StageMode>("/wind_turbine_insp/stage_mode", 2, &RecordNode::getStageModeCallback, this);

  global_lla_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>("/dji_osdk_ros/rtk_position", 2, &RecordNode::getGlobalLLACallback, this);
  local_odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/wind_turbine_insp/local_odom", 2, &RecordNode::getLocalOdomCallback, this);
  global_gimbal_angle_subscriber_ = nh_.subscribe<geometry_msgs::Vector3Stamped>("/dji_osdk_ros/global_gimbal_angle", 2, &RecordNode::getGlobalGimbalAngleCallback, this);
  camera_zoom_subscriber_ = nh_.subscribe<std_msgs::Float32>("/wind_turbine_insp/set_camera_zoom", 2, &RecordNode::getCameraZoomCallback, this);
  blade_distance_subscriber_ = nh_.subscribe<std_msgs::Float32>("/wind_turbine_insp/blade_distance", 2, &RecordNode::getBladeDistCallback, this);
  log_state_subscriber_ = nh_.subscribe<std_msgs::Int32>("/wind_turbine_insp/log_state", 2, &RecordNode::getLogStateCallback, this);
  
  node_status_publisher_ = nh_.advertise<std_msgs::Int32>("/wind_turbine_insp/node_status", 2);
}

bool RecordNode::initTimer()
{
  record_log_timer_ = nh_.createTimer(ros::Rate(rate_record_), &RecordNode::recordLogCallback, this);
  node_status_timer_ = nh_.createTimer(ros::Rate(1.0), &RecordNode::nodeStatusPublisherCallback, this);

}

void RecordNode::getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg)
{
  stage_mode_ = *msg;
}


void RecordNode::getGlobalLLACallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  global_lla_ = *msg;
}

void RecordNode::getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  local_odom_ = *msg;
  PointMsgToXYZVec(local_odom_.pose.pose.position, local_posi_);
  QuatnMsgToRPYVec(local_odom_.pose.pose.orientation, local_atti_);
  VectorMsgToEigen(local_odom_.twist.twist.linear, local_velo_);
}


void RecordNode::getGlobalGimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  global_gimbal_angle_ = *msg;
  VectorMsgToEigen(global_gimbal_angle_.vector, gimbal_atti_);
}

void RecordNode::getCameraZoomCallback(const std_msgs::Float32::ConstPtr& msg)
{
  zoom_factor_ = *msg;
  focal_len_ = zoom_factor_.data * ZF2FL_P1 + ZF2FL_P2;
}

void RecordNode::getBladeDistCallback(const std_msgs::Float32::ConstPtr& msg)
{
  blade_distance_ = *msg;
  blade_dist_ = blade_distance_.data;
}

void RecordNode::getLogStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
  log_state_ = msg -> data;
}

void RecordNode::recordLogCallback(const ros::TimerEvent & event)
{
  if(log_record_)
  {
    if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
    {
      log_record_ = false;
      ROS_INFO("vehicle auto land, stop recording log!");
      std::fclose(pFile_);
      ROS_INFO("write log file finish!");
      ros::shutdown();
    }
    else if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
    {
      std::fprintf(pFile_, 
                  "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.3f, %.3f, %d\n",
                  ros::Time::now().toSec(),
                  global_lla_.longitude, 
                  global_lla_.latitude, 
                  global_lla_.altitude,
                  local_posi_(0), 
                  local_posi_(1), 
                  local_posi_(2),
                  RAD2DEG(local_atti_(0)), 
                  RAD2DEG(local_atti_(1)), 
                  RAD2DEG(local_atti_(2)),
                  local_velo_(0), 
                  local_velo_(1), 
                  local_velo_(2),
                  gimbal_atti_(1), 
                  gimbal_atti_(0), 
                  gimbal_atti_(2), 
                  focal_len_ / 10, 
                  blade_dist_, 
                  log_state_);
      ROS_INFO("record once!");
    }
  }

}

void RecordNode::nodeStatusPublisherCallback(const ros::TimerEvent& event)
{
  std_msgs::Int32 msg;
  msg.data = 9;
  node_status_publisher_.publish(msg);
}

std::string RecordNode::getTimeString()
{
  time_t cur_time = ros::Time::now().toSec();
  struct tm* timeinfo;
  timeinfo = localtime(&cur_time);
  char time_buffer[128];
  strftime(time_buffer, sizeof(time_buffer),"%Y%m%d%H%M%S", timeinfo);
  std::string time_string = time_buffer;
  return time_string;
}




/*#################*/
/*                 */
/*  Main Function  */
/*                 */
/*#################*/

int main(int argc, char** argv){
    
  ros::init(argc, argv, "record_node");

  ros::NodeHandle nh;

  RecordNode record_node(nh);

  ROS_INFO_STREAM("Record Node is OK !");

  ros::MultiThreadedSpinner spinner(4);

  spinner.spin();

  return 0;
}