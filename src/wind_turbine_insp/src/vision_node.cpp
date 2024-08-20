#include "wind_turbine_insp/vision_node.h"

using namespace wind_turbine_insp;



VisionNode::VisionNode(ros::NodeHandle nh): nh_(nh) {

  initVariable();

  initSubscriber();

  initPublisher();



  initTimer();





}
  

VisionNode::~VisionNode(){

}

bool VisionNode::initVariable()
{
  nh_.getParam("vision_node/photo_intr_fx",   photo_intr_fx_);
  nh_.getParam("vision_node/photo_intr_fy",   photo_intr_fy_);
  nh_.getParam("vision_node/photo_intr_cx",   photo_intr_cx_);
  nh_.getParam("vision_node/photo_intr_cy",   photo_intr_cy_);

  nh_.getParam("vision_node/video_intr_fx",   video_intr_fx_);
  nh_.getParam("vision_node/video_intr_fy",   video_intr_fy_);
  nh_.getParam("vision_node/video_intr_cx",   video_intr_cx_);
  nh_.getParam("vision_node/video_intr_cy",   video_intr_cy_);

  nh_.getParam("vision_node/rate_image_proc",   rate_image_proc_);
  nh_.getParam("vision_node/rate_adjust_gim",   rate_adjust_gim_);
  nh_.getParam("vision_node/rate_set_camera_ev",   rate_set_camera_ev_);
  nh_.getParam("vision_node/rate_shoot_interval",   rate_shoot_interval_);

  nh_.getParam("vision_node/video_record_mode",   video_record_mode_);
  nh_.getParam("vision_node/ev_mode",   ev_mode_);

  nh_.getParam("vision_node/msg_text_scale",   msg_text_scale_);
  nh_.getParam("vision_node/msg_text_thick",   msg_text_thick_);

  nh_.getParam("vision_node/enable_line",   enable_line_);
  nh_.getParam("vision_node/canny_min",   canny_min_);
  nh_.getParam("vision_node/canny_max",   canny_max_);
  nh_.getParam("vision_node/hough_thold",   hough_thold_);
  nh_.getParam("vision_node/min_line_len",   min_line_len_);
  nh_.getParam("vision_node/max_line_gap",   max_line_gap_);
  nh_.getParam("vision_node/max_line_num",   max_line_num_);

  nh_.getParam("vision_node/line_detect_err",   line_detect_err_);
  nh_.getParam("vision_node/compensate_k",   compensate_k_);
  nh_.getParam("vision_node/focus_len_ori",   focus_len_ori_);
  nh_.getParam("vision_node/set_adj_limit",   set_adj_limit_);

  intr_ = Eigen::Matrix3d::Zero();

  if(video_record_mode_ == 2)
  {
    image_width_ = 1920;
    image_height_ = 1080;

    intr_(0, 0) = photo_intr_fx_;
    intr_(1, 1) = photo_intr_fy_;
    intr_(0, 2) = photo_intr_cx_ * image_width_;
    intr_(1, 2) = photo_intr_cy_ * image_height_;
  }
  else
  {
    image_width_ = 1920;
    image_height_ = 1080;

    intr_(0, 0) = video_intr_fx_;
    intr_(1, 1) = video_intr_fy_;
    intr_(0, 2) = video_intr_cx_ * image_width_;
    intr_(1, 2) = video_intr_cy_ * image_height_;
  }

  intr_(2, 2) = 1;

  extr_ = Eigen::MatrixXd::Zero(3, 4);

  std::cout << "------------------------------" << std::endl;
  std::cout << intr_ << std::endl;
  std::cout << "------------------------------" << std::endl;



  stage_mode_.stage_mode = insp_msgs::StageMode::STAGE_INIT_MODE_INIT;

  local_posi_ = Eigen::Vector3d::Zero();

  camera_zoom_factor_ = 2.0;

  local_gimbal_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());

  target_pt3d_ = Eigen::Vector3d::Zero();
  target_pt2d_ = Eigen::Vector2i(int(image_width_ / 2), int(image_height_ / 2));

  nh_.getParam("vision_node/area_radius_ori",   area_radius_ori_);
  nh_.getParam("vision_node/area_mean_max",   area_mean_max_);
  nh_.getParam("vision_node/area_mean_min",   area_mean_min_);
  nh_.getParam("vision_node/mean_offset",   mean_offset_);
  nh_.getParam("vision_node/adjust_k",   adjust_k_);

  image_zoom_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
  image_draw_point_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
  image_resize_ = cv::Mat::zeros(cv::Size(image_width_ * 0.25, image_height_ * 0.25), CV_8UC3);
  image_draw_line_ = cv::Mat::zeros(cv::Size(image_width_ * 0.25, image_height_ * 0.25), CV_8UC3);

  target_point_legal_ = false;

  target_area_mean_ = 0;
  target_area_stddev_ = 0;

  area_status_ = 0;

  last_get_image_time_ = ros::Time::now();

  status_recording = false;
  status_shooting_ = false;

  plines_ = std::vector<cv::Vec4f>();
  plines_theta_ = std::vector<cv::Vec4f>();
  plines_edge_ = std::vector<cv::Vec4f>();
  clines_ = std::vector<cv::Vec4f>();

  blade_angle_ = 0;

  x_base_vec_ = Eigen::Vector3d::Zero();
  y_base_vec_ = Eigen::Vector3d::Zero();

  save_idx_ = 0;
  adjust_mat_ = cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, cv::Scalar(1, 1, 1)) * adjust_k_;
  last_adj_val_ = 0;

  printf("--------------------\n");
  printf("Loading Params Start\n");

  printf("video_intr_fx: %.2f\n", video_intr_fx_);
  printf("video_intr_fy: %.2f\n", video_intr_fy_);
  printf("video_intr_cx: %.2f\n", video_intr_cx_);
  printf("video_intr_cy: %.2f\n", video_intr_cy_);
  printf("photo_intr_fx: %.2f\n", photo_intr_fx_);
  printf("photo_intr_fy: %.2f\n", photo_intr_fy_);
  printf("photo_intr_cx: %.2f\n", photo_intr_cx_);
  printf("photo_intr_cy: %.2f\n", photo_intr_cy_);
  printf("rate_image_proc: %.2f\n", rate_image_proc_);
  printf("rate_adjust_gim: %.2f\n", rate_adjust_gim_);
  printf("rate_set_camera_ev: %.2f\n", rate_set_camera_ev_);
  printf("rate_shoot_interval: %.2f\n", rate_shoot_interval_);
  printf("video_record_mode: %d\n", video_record_mode_);
  
  printf("area_radius_ori: %d\n", area_radius_ori_);
  printf("area_mean_min: %.2f\n", area_mean_min_);
  printf("area_mean_max: %.2f\n", area_mean_max_);
  printf("mean_offset: %.2f\n", mean_offset_);
  printf("adjust_k: %d\n", adjust_k_);
  printf("msg_text_scale: %.2f\n", msg_text_scale_);
  printf("msg_text_thick: %d\n", msg_text_thick_);
  printf("enable_line: %d\n", enable_line_);

  printf("canny_min: %.2f\n", canny_min_);
  printf("canny_max: %.2f\n", canny_max_);
  printf("hough_thold: %d\n", hough_thold_);
  printf("min_line_len: %.2f\n", min_line_len_);
  printf("max_line_gap: %.2f\n", max_line_gap_);
  printf("max_line_num: %d\n", max_line_num_);
  printf("line_detect_err: %.2f\n", line_detect_err_);
  printf("compensate_k: %.2f\n", compensate_k_);
  printf("set_adj_limit: %d\n", set_adj_limit_);

  printf("Loading Params Finish\n");
  printf("---------------------\n");

  ROS_INFO_STREAM("Vision_Node: init Variable");

  return true;
}

bool VisionNode::initTimer()
{
  image_process_timer_ = nh_.createTimer(ros::Rate(rate_image_proc_), &VisionNode::imageProcessCallback, this);
  gimbal_adjust_timer_ = nh_.createTimer(ros::Rate(rate_adjust_gim_), &VisionNode::gimbalAdjustCallback, this);
  set_camera_ev_timer_ = nh_.createTimer(ros::Rate(rate_set_camera_ev_), &VisionNode::setCameraEvCallback, this);
  camera_shoot_action_timer_ = nh_.createTimer(ros::Rate(rate_shoot_interval_), &VisionNode::cameraShootActionCallback, this);
  ROS_INFO_STREAM("Vision_Node: init Timer");
}


bool VisionNode::initSubscriber()
{
  stage_mode_subscriber_              = nh_.subscribe<insp_msgs::StageMode>("/wind_turbine_insp/stage_mode", 2, &VisionNode::getStageModeCallback, this);
  gimbal_image_subscriber_            = nh_.subscribe<sensor_msgs::Image>("/wind_turbine_insp/gimbal_image", 2, &VisionNode::getGimbalImageCallback, this);
  camera_set_zoom_subscriber_         = nh_.subscribe<std_msgs::Float32>("/wind_turbine_insp/camera_set_zoom_para", 2, &VisionNode::getCameraZoomParaCallback, this);
  camera_record_video_subscriber_     = nh_.subscribe<std_msgs::Bool>("/wind_turbine_insp/camera_record_video", 100, &VisionNode::cameraRecordVideoCallback, this);
  camera_shoot_photo_subscriber_      = nh_.subscribe<std_msgs::Bool>("/wind_turbine_insp/camera_shoot_photo", 2, &VisionNode::cameraShootPhotoCallback, this);
  local_gimbal_quat_subscriber_       = nh_.subscribe<geometry_msgs::QuaternionStamped>("/wind_turbine_insp/local_gimbal_quat", 2, &VisionNode::localGimbalQuatCallback, this);
  local_odom_subscriber_              = nh_.subscribe<nav_msgs::Odometry>("/wind_turbine_insp/local_odom", 2, &VisionNode::getLocalOdomCallback, this);
  target_pt3d_subscriber_             = nh_.subscribe<geometry_msgs::Point>("/wind_turbine_insp/blade_target_point", 2, &VisionNode::getBladeTatgetPointCallback, this);
  blade_width_subscriber_             = nh_.subscribe<std_msgs::Float32>("/wind_turbine_insp/blade_width", 2, &VisionNode::getBladeWidthCallback, this);
  blade_side_state_subscriber_        = nh_.subscribe<insp_msgs::BladeSideState>("/wind_turbine_insp/blade_side_state", 2, &VisionNode::getBladeSideStateCallback, this);
  get_log_state_subscriber_           = nh_.subscribe<std_msgs::Int32>("/wind_turbine_insp/log_state", 2, &VisionNode::getLogStateCallback, this);
  ptcloud_key_subscriber_             = nh_.subscribe<insp_msgs::PtCloudKey>("/wind_turbine_insp/ptcloud_key", 2, &VisionNode::getPtCloudKeyCallback, this);

  ROS_INFO_STREAM("Vision_Node: init Subscriber");

  return true;
}


bool VisionNode::initPublisher()
{
  point_image_publisher_ = nh_.advertise<sensor_msgs::Image>("/wind_turbine_insp/gimbal_image_zoom", 2);
  line_image_publisher_   = nh_.advertise<sensor_msgs::Image>("/wind_turbine_insp/gimbal_image_line", 2);
  gimbal_compensate_publisher_ = nh_.advertise<insp_msgs::GimbalCompensate>("/wind_turbine_insp/gimbal_compensate", 2);
  
  debug_vision_publisher_ = nh_.advertise<insp_msgs::DebugVision>("/wind_turbine_insp/debug_vision", 2);

  ROS_INFO_STREAM("Vision_Node: init Publisher");

  return true;
}





















void VisionNode::getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg)
{
  stage_mode_ = *msg;
  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
  {
    ros::Duration(1.0).sleep();
    ros::shutdown();
  }
}



void VisionNode::getGimbalImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if((ros::Time::now() - last_get_image_time_).toSec() > 0.5)
  {
    cv_bridge::CvImagePtr cv_ptr;

    int data_len = (msg -> height) * (msg -> step);

    try{
      sensor_msgs::Image tmp_msg = *msg;
      tmp_msg.data.resize(data_len);
      cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8); 
    }
    catch(cv_bridge::Exception& e){
      ROS_INFO_STREAM(e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr -> image;

    image_width_ = cv_img.cols;
    image_height_ = cv_img.rows;

    if(last_adj_val_ > 0)
      cv_img += last_adj_val_ * adjust_mat_;
    else if(last_adj_val_ < 0)
      cv_img -= -last_adj_val_ * adjust_mat_;

    imageDigitalZoom(cv_img, image_zoom_, focus_len_ori_ * camera_zoom_factor_);
    imageDrawMessage(image_zoom_, image_draw_point_);

    sensor_msgs::ImagePtr msg_ptr_zoom = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_draw_point_).toImageMsg();
    (*msg_ptr_zoom).data.resize(data_len);
    point_image_publisher_.publish(*msg_ptr_zoom);

    cv::resize(image_zoom_, image_resize_, cv::Size(image_width_ * 0.25, image_height_ * 0.25));
    imageDrawLines(image_resize_, image_draw_line_);

    sensor_msgs::ImagePtr msg_ptr_resize = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_draw_line_).toImageMsg();
    (*msg_ptr_resize).data.resize(data_len * 0.25 * 0.25);
    line_image_publisher_.publish(*msg_ptr_resize);

    last_get_image_time_ = ros::Time::now();
  }
}

void VisionNode::getCameraZoomParaCallback(const std_msgs::Float32::ConstPtr &msg)
{
  camera_zoom_factor_ = msg -> data;
}

void VisionNode::cameraRecordVideoCallback(const std_msgs::Bool::ConstPtr& msg)
{
  status_recording = msg -> data;
}

void VisionNode::cameraShootPhotoCallback(const std_msgs::Bool::ConstPtr& msg)
{
  status_shooting_ = msg -> data;
}

void VisionNode::localGimbalQuatCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  local_gimbal_quat_ = Eigen::Quaterniond(msg -> quaternion.w, 
                                          msg -> quaternion.x, 
                                          msg -> quaternion.y, 
                                          msg -> quaternion.z);
}

void VisionNode::getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  PointMsgToXYZVec((msg -> pose).pose.position, local_posi_);
}

void VisionNode::getBladeTatgetPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  target_pt3d_(0) = msg -> x;
  target_pt3d_(1) = msg -> y;
  target_pt3d_(2) = msg -> z;
}
void VisionNode::getBladeWidthCallback(const std_msgs::Float32::ConstPtr& msg)
{
  if(log_state_ == 4)
    area_radius_ = int(area_radius_ori_ * msg -> data);
  else
    area_radius_ = int(area_radius_ori_ * 0.5);
}
void VisionNode::getBladeSideStateCallback(const insp_msgs::BladeSideState::ConstPtr& msg)
{
  blade_side_state_ = *msg;
}
void VisionNode::getLogStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
  log_state_ = msg -> data;
}
void VisionNode::getPtCloudKeyCallback(const insp_msgs::PtCloudKey::ConstPtr& msg)
{
  hub_nor_vec_ = Eigen::Vector3d(msg -> hub_vx,   msg -> hub_vy,   msg -> hub_vz).normalized();
  Eigen::Vector3d blade_dir_vec = Eigen::Vector3d(msg -> blade_vx, msg -> blade_vy, msg -> blade_vz).normalized();

  x_base_vec_ = hub_nor_vec_.cross(Eigen::Vector3d(0, 0, 1)).normalized();
  y_base_vec_ = x_base_vec_.cross(hub_nor_vec_).normalized();
  blade_angle_ = RAD2DEG(atan2(blade_dir_vec.dot(y_base_vec_), blade_dir_vec.dot(x_base_vec_)));
  // printf("<blade angle>: %.2f\n", blade_angle_);
}

void VisionNode::imageProcessCallback(const ros::TimerEvent& event)
{
  if(log_state_ == 1 || log_state_ == 4)
  {
    // printf("(%5d)   (%8.2f, %8.2f, %6.2f)   (%6.2f, %6.2f, %6.2f, %6.2f)   (%6.2f)\n", target_pt2d_(0), intr_(0, 0), intr_(0, 1), intr_(0, 2), extr_(0, 0), extr_(0, 1), extr_(0, 2), extr_(0, 3), target_pt3d_(0));
    // printf("(%5d) = (%8.2f, %8.2f, %6.2f) * (%6.2f, %6.2f, %6.2f, %6.2f) * (%6.2f)\n", target_pt2d_(1), intr_(1, 0), intr_(1, 1), intr_(1, 2), extr_(1, 0), extr_(1, 1), extr_(1, 2), extr_(1, 3), target_pt3d_(1));
    // printf("(%5d)   (%8.2f, %8.2f, %6.2f)   (%6.2f, %6.2f, %6.2f, %6.2f)   (%6.2f)\n",              1 , intr_(2, 0), intr_(2, 1), intr_(2, 2), extr_(2, 0), extr_(2, 1), extr_(2, 2), extr_(2, 3), target_pt3d_(2));

    if(target_point_legal_)
    {
      cv::Mat image_gray;
      cv::cvtColor(image_zoom_, image_gray, cv::COLOR_BGR2GRAY);

      cv::Mat mean_mat, stddev_mat;
      cv::Mat mask_mat = cv::Mat::zeros(image_gray.size(), image_gray.type());
      cv::circle(mask_mat, cv::Point(target_pt2d_(0), target_pt2d_(1)), area_radius_, cv::Scalar(1), -1);

      cv::meanStdDev(image_gray, mean_mat, stddev_mat, mask_mat);

      target_area_mean_ = mean_mat.at<double>(0, 0);
      target_area_stddev_ = stddev_mat.at<double>(0, 0);

      double cur_area_max = area_mean_max_;
      double cur_area_min = area_mean_min_;

      if(blade_side_state_.side == 1)
      {
        cur_area_max = area_mean_max_ - mean_offset_;
        cur_area_min = area_mean_min_ - mean_offset_;
      }
      else if(blade_side_state_.side == 2)
      {
        cur_area_max = area_mean_max_ - mean_offset_;
        cur_area_min = area_mean_min_ - mean_offset_;
      }
      else if(blade_side_state_.side == 3)
      {
        cur_area_max = area_mean_max_ - mean_offset_ * 2;
        cur_area_min = area_mean_min_ - mean_offset_ * 2;
      }
      else if(blade_side_state_.side == 4)
      {
        if(blade_side_state_.state == 1)
        {
          cur_area_max = area_mean_max_ + mean_offset_;
          cur_area_min = area_mean_min_ + mean_offset_;
        }
        else if(blade_side_state_.state == 2)
        {
          cur_area_max = area_mean_max_ + mean_offset_ * 4;
          cur_area_min = area_mean_min_ + mean_offset_ * 4;
        }
      }
      else
      {
        cur_area_max = area_mean_max_;
        cur_area_min = area_mean_min_;
      }

      if((blade_side_state_.side != 3) && (blade_side_state_.dist_to_root > blade_side_state_.fan_blade_length * 9.0 / 10.0))
      {
        area_status_ = 0;
      }
      else
      {
        if(target_area_mean_ > cur_area_max)
          area_status_ = 1;
        else if(target_area_mean_ < cur_area_min)
          area_status_ = -1;
        else
          area_status_ = 0;
      }
      dv_msg_.pt2d_x = target_pt2d_(0);
      dv_msg_.pt2d_y = target_pt2d_(1);
      dv_msg_.area_mean = target_area_mean_;
      dv_msg_.area_stddev = target_area_stddev_;
      dv_msg_.max_mean = cur_area_max;
      dv_msg_.min_mean = cur_area_min;
    }
    else
    {
      area_status_ = 0;
      dv_msg_.pt2d_x = -1;
      dv_msg_.pt2d_y = -1;
      dv_msg_.area_mean = -1;
      dv_msg_.area_stddev = -1;
      dv_msg_.max_mean = area_mean_max_;
      dv_msg_.min_mean = area_mean_min_;
    }

    last_adj_val_ -= area_status_;

    if(blade_side_state_.side == 4 && blade_side_state_.state != 1)
    {
      if(blade_side_state_.dist_to_root >= blade_side_state_.fan_blade_length * 1.0 / 4.0)
        last_adj_val_ = std::max(+int(set_adj_limit_ * 1.7), std::min(+int(set_adj_limit_ * 2.1), last_adj_val_));
      else
        last_adj_val_ = std::max(+int(set_adj_limit_ * 0.5), std::min(+int(set_adj_limit_ * 1.0), last_adj_val_));
    }
    else if(blade_side_state_.side == 3 && blade_side_state_.state == 2)
    {
      last_adj_val_ = std::max(-int(set_adj_limit_ * 1.0), std::min(-int(set_adj_limit_ * 0.5), last_adj_val_));
    }
    else
    {
      last_adj_val_ = std::max(-set_adj_limit_, std::min(+set_adj_limit_, last_adj_val_));
    }
  }
  else
  {
    area_status_ = 0;
    last_adj_val_ = 0;
    dv_msg_.pt2d_x = -2;
    dv_msg_.pt2d_y = -2;
    dv_msg_.area_mean = -2;
    dv_msg_.area_stddev = -2;
    dv_msg_.max_mean = area_mean_max_;
    dv_msg_.min_mean = area_mean_min_;
  }
  
  dv_msg_.pt3d_x = target_pt3d_(0);
  dv_msg_.pt3d_y = target_pt3d_(1);
  dv_msg_.pt3d_z = target_pt3d_(2);

  dv_msg_.area_radius = area_radius_;
  dv_msg_.area_status = area_status_;
  dv_msg_.last_adj_val = last_adj_val_;
  dv_msg_.dist_to_root = blade_side_state_.dist_to_root;
  dv_msg_.header.stamp = ros::Time::now();
  debug_vision_publisher_.publish(dv_msg_);

  return;
}

void VisionNode::gimbalAdjustCallback(const ros::TimerEvent& event)
{
  bool do_adjust = false;
  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
  {
    if(stage_mode_.stage_mode % 2 == 1 && blade_side_state_.dist_to_hub_front > blade_side_state_.adjust_thold)
      do_adjust = true;
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_2 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
  {
    if(stage_mode_.stage_mode % 4 == 1 && blade_side_state_.dist_to_hub_front > blade_side_state_.adjust_thold)
      do_adjust = true;
  }
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_4 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    if(stage_mode_.stage_mode % 8 == 1 && blade_side_state_.dist_to_hub_front > blade_side_state_.adjust_thold)
      do_adjust = true;
  }

  // if(abs(blade_angle_ - 90) < 45 || abs(blade_angle_ - (-90)) < 45)
  //   adjust_vertical_ = false;
  // else
  //   adjust_vertical_ = true;

  if(enable_line_ && do_adjust)
  {
    imageDetectLine(image_resize_, plines_, canny_min_, canny_max_, hough_thold_, min_line_len_, max_line_gap_, max_line_num_);
    selectBladeLine();
    ROS_INFO("do adjust!");
  }

}


void VisionNode::setCameraEvCallback(const ros::TimerEvent &event)
{
  



}

void VisionNode::cameraShootActionCallback(const ros::TimerEvent& event)
{
  if(status_shooting_)
  {
    std::string name_pre = "/home/hilab/img_save/wti_zoom_image_";
    std::string name_suc = ".jpg";
    cv::Mat image_save;
    cv::cvtColor(image_zoom_, image_save, cv::COLOR_BGR2RGB);
    cv::imwrite((name_pre) + std::to_string(save_idx_) + name_suc, image_save);
    save_idx_++;
  }
}

void VisionNode::imageDigitalZoom(cv::Mat image, cv::Mat& image_crop, double focal_len_dst){
  
  int rect_width = int(image_width_ * focus_len_ori_ / focal_len_dst);
  int rect_height = int(rect_width * image_height_ / image_width_);

  int left_up_y = int((image_height_ - rect_height) / 2);
  int left_up_x = int((image_width_ - rect_width) / 2);

  cv::Range range_y = cv::Range(left_up_y, left_up_y + rect_height);
  cv::Range range_x = cv::Range(left_up_x, left_up_x + rect_width);

  cv::resize(image(range_y, range_x), image_crop, cv::Size(image_width_, image_height_));

  return;
}


void VisionNode::imageDrawMessage(cv::Mat image_crop, cv::Mat& image_draw)
{
  image_draw = image_crop.clone();

  std::string str_zoom = D2S(camera_zoom_factor_, 2) + "x";

  projectTargetPoint();

  cv::Point left_up = cv::Point(int(image_width_ * 0.10), int(image_height_ * 0.86));
  std::string str_msg;

  if(log_state_ == 1 || log_state_ == 4)
  {
    if(target_pt2d_(0) > 0 && target_pt2d_(0) < image_width_ - 1 && target_pt2d_(1) > 0 && target_pt2d_(1) < image_height_ - 1)
    {
      target_point_legal_ = true;
      // draw the circle
      cv::circle(image_draw, cv::Point(target_pt2d_(0), target_pt2d_(1)), area_radius_, cv::Scalar(255, 165, 0), -1);
      // set the msg
      std::string str_adj = (area_status_ == 1)? "dn": ((area_status_ == -1)? "up": "no");
      str_msg = str_zoom + ", ev: " + D2S(last_adj_val_ / 10.0, 1) + ", target: (" + D2S(target_area_mean_, 2) + ", " + D2S(target_area_stddev_, 2) + ")  " + "adjust: " + str_adj;
    }
    else
    {
      target_point_legal_ = false;
      // draw the arrow
      double k = (image_height_ / 2.0 - target_pt2d_(1)) / (image_width_ / 2.0 - target_pt2d_(0));
      double b = target_pt2d_(1) - k * target_pt2d_(0);
      double a = image_height_ * 1.0 / image_width_;
      double x, y;
      if(target_pt2d_(1) < a * target_pt2d_(0))
        if(target_pt2d_(1) < -a * target_pt2d_(0) + image_height_)
        {
          y = 10;
          x = (y - b) / k;
        }
        else
        {
          x = image_width_ - 10;
          y = k * x + b;
        }
      else
        if(target_pt2d_(1) < -a * target_pt2d_(0) + image_height_)
        {
          x = 10;
          y = k * x + b;
        }
        else
        {
          y = image_height_ - 10;
          x = (y - b) / k;
        }
      double xx = (image_width_ / 2 + x) / 2.0;
      double yy = (image_height_ / 2 + y) / 2.0;
      cv::arrowedLine(image_draw, cv::Point(int(xx), int(yy)), cv::Point(int(x), int(y)), cv::Scalar(255, 165, 0), 30, 8, 0, 0.3);
      // set the msg
      str_msg = str_zoom + ", ev: " + D2S(last_adj_val_ / 10.0, 1) + ", target: illegal";
    }
  }
  else
  {
    target_point_legal_ = false;
    // set the msg
    str_msg = str_zoom + ", ev: " + D2S(last_adj_val_ / 10.0, 1) + ", target invalid";
  }
  
  // std::cout << str_msg << std::endl;

  int text_msg_baseline;
  cv::Size text_msg_size = cv::getTextSize(str_msg, cv::FONT_HERSHEY_SIMPLEX, msg_text_scale_, msg_text_thick_, &text_msg_baseline);
  Eigen::Vector2d bound = Eigen::Vector2d(image_height_ * 0.02, image_height_ * 0.02);
  cv::rectangle(image_draw, cv::Rect(int(left_up.x - bound(0)), int(left_up.y - bound(1)), int(text_msg_size.width + bound(0) * 2), int(text_msg_size.height + bound(1) * 2)), cv::Scalar(255, 255, 255), -1);
  cv::putText(image_draw, str_msg, cv::Point(left_up.x, left_up.y + text_msg_size.height), cv::FONT_HERSHEY_SIMPLEX, msg_text_scale_, cv::Scalar(0, 0, 0), msg_text_thick_, cv::LINE_8, false);

  return;
}


void VisionNode::projectTargetPoint()
{
  Eigen::Matrix3d R_W2G = local_gimbal_quat_.toRotationMatrix();
  Eigen::Vector3d t_W2G = local_posi_;

  Eigen::Matrix3d R_G2W = R_W2G.transpose().eval();
  Eigen::Vector3d t_G2W = R_W2G.transpose().eval() * (-t_W2G);
  
  Eigen::Matrix3d R_C2G;
  R_C2G << 0, -1,  0, 
           0,  0, -1, 
           1,  0,  0;
  Eigen::Vector3d t_C2G = Eigen::Vector3d(0, 0, 0);


  Eigen::Matrix4d T_C2G = Eigen::Matrix4d::Zero();
  T_C2G.block(0, 0, 3, 3) = R_C2G;
  T_C2G.block(0, 3, 3, 1) = t_C2G;
  T_C2G(3, 3) = 1;

  Eigen::Matrix4d T_G2W = Eigen::Matrix4d::Zero();
  T_G2W.block(0, 0, 3, 3) = R_G2W;
  T_G2W.block(0, 3, 3, 1) = t_G2W;
  T_G2W(3, 3) = 1;

  Eigen::Matrix4d T_C2W = T_C2G * T_G2W;
  extr_ = T_C2W.block(0, 0, 3, 4);


  Eigen::Vector4d p_world = Eigen::Vector4d::Ones();
  p_world.block(0, 0, 3, 1) = target_pt3d_;

  if(video_record_mode_ == 2)
  {
    intr_(0, 0) = photo_intr_fx_ * camera_zoom_factor_;
    intr_(1, 1) = photo_intr_fy_ * camera_zoom_factor_;
    intr_(0, 2) = photo_intr_cx_ * image_width_;
    intr_(1, 2) = photo_intr_cy_ * image_height_;
  }
  else
  {
    intr_(0, 0) = video_intr_fx_ * camera_zoom_factor_;
    intr_(1, 1) = video_intr_fy_ * camera_zoom_factor_;
    intr_(0, 2) = video_intr_cx_ * image_width_;
    intr_(1, 2) = video_intr_cy_ * image_height_;
  }

  Eigen::Vector3d p_image = intr_ * extr_ * p_world;
  p_image = p_image / p_image(2);

  target_pt2d_(0) = int(p_image(0));
  target_pt2d_(1) = int(p_image(1));

  return;
}


void VisionNode::imageDetectLine(cv::Mat image, std::vector<cv::Vec4f> &plines, double canny_min, double canny_max, 
                 int thold, double min_line_len, double max_line_gap, int max_line_num)
{
  cv::Mat image_proc = image.clone();
  cv::cvtColor(image, image_proc, cv::COLOR_BGR2GRAY);

  cv::GaussianBlur(image_proc, image_proc, cv::Size(3, 3), 0, 0);

  cv::Canny(image_proc, image_proc, canny_min, canny_max, 3);
  
  //                            rho,    theta, bthreshold, min_line_len, max_line_gap
  cv::HoughLinesP(image_proc, plines, 1, CV_PI / 180, thold, min_line_len, max_line_gap);

  return;
}

void VisionNode::selectBladeLine()
{
  if(plines_.size() < 2)
  {
    ROS_WARN("[vision node]: detect line cnt < 2!!!");
    return;
  }

  plines_theta_ = std::vector<cv::Vec4f>();
  plines_edge_ = std::vector<cv::Vec4f>();
  clines_ = std::vector<cv::Vec4f>();

  for(int i = 0; i < plines_.size(); i++)
  {
    double theta1 = RAD2DEG(atan2(-(plines_[i][1] - plines_[i][3]), plines_[i][0] - plines_[i][2]));
    double theta2 = RAD2DEG(atan2(-(plines_[i][3] - plines_[i][1]), plines_[i][2] - plines_[i][0]));

    if(((theta1 > blade_angle_ - line_detect_err_) && (theta1 < blade_angle_ + line_detect_err_))
    || ((theta2 > blade_angle_ - line_detect_err_) && (theta2 < blade_angle_ + line_detect_err_)))
    {
      plines_theta_.push_back(plines_[i]);
    }
  }

  if(plines_theta_.size() < 2)
  {
    ROS_WARN("[vision node]: theta line cnt < 2!!!");
    return;
  }

  double x0 = 0;
  double y0 = (blade_angle_ > 90 || (blade_angle_ < 0 && blade_angle_ > -90)) ? (image_resize_.rows) : (0);
  double a, b, c; 
  double cx = image_resize_.cols / 2, cy = image_resize_.rows / 2;

  std::vector<double> plines_dc;

  for(int i = 0; i < plines_theta_.size(); i++){
    a = plines_theta_[i][1] - plines_theta_[i][3], b = plines_theta_[i][2] - plines_theta_[i][0], c = plines_theta_[i][0] * plines_theta_[i][3] - plines_theta_[i][1] * plines_theta_[i][2];
    plines_dc.push_back(abs(a * x0 + b * y0 + c) / (sqrt(a * a + b * b) + 1e-9));
  }
  
  int max_idx = distance(plines_dc.begin(), max_element(plines_dc.begin(), plines_dc.end()));
  int min_idx = distance(plines_dc.begin(), min_element(plines_dc.begin(), plines_dc.end()));

  plines_edge_.push_back(plines_theta_[max_idx]);
  plines_edge_.push_back(plines_theta_[min_idx]);

  cv::Vec4f cline = cv::Vec4f((plines_edge_[0][0] + plines_edge_[1][0]) / 2, (plines_edge_[0][1] + plines_edge_[1][1]) / 2, (plines_edge_[0][2] + plines_edge_[1][2]) / 2, (plines_edge_[0][3] + plines_edge_[1][3]) / 2);
  clines_.push_back(cline);

  a = cline[1] - cline[3], b = cline[2] - cline[0], c = cline[0] * cline[3] - cline[1] * cline[2];
  
  // vertical
  // line up, point down, dc > 0
  // line down, point up, dc < 0

  // horizontal
  // line left, point right, dc > 0
  // line right, point left, dc < 0
  
  double dc = abs(a * cx + b * cy + c) / sqrt(a * a + b * b);
  
  dc = (-(a / b) * cx - (c / b) > cy) ? -dc : dc;

  if(dc < -image_resize_.cols || dc > image_resize_.cols)
  {
    ROS_WARN("[vision node]: center line to center point illegal!!!");
  }

  Eigen::Vector3d blade_vec = x_base_vec_ * cos(DEG2RAD(blade_angle_)) + y_base_vec_ * sin(DEG2RAD(blade_angle_));
  Eigen::Vector3d adj_vec = hub_nor_vec_.cross(blade_vec).normalized();
  
  adj_vec = (adj_vec.dot(y_base_vec_) > 0) ? (adj_vec) : (-adj_vec);
  adj_vec = adj_vec * dc * compensate_k_ / camera_zoom_factor_;

  printf("[vision node]: dc: %.2f, adj: %.2f\n", dc, adj_vec.norm());

  dv_msg_.dc = dc;
  dv_msg_.adj = adj_vec.norm();

  insp_msgs::GimbalCompensate msg;
  msg.compensate_vec.x = adj_vec.x();
  msg.compensate_vec.y = adj_vec.y();
  msg.compensate_vec.z = adj_vec.z();
  
  gimbal_compensate_publisher_.publish(msg);
}

void VisionNode::imageDrawLines(cv::Mat image_resize, cv::Mat& image_draw)
{
  image_draw = image_resize.clone();

  for(int i = 0; i < plines_.size(); i++)
    cv::line(image_draw, cv::Point(plines_[i][0], plines_[i][1]), cv::Point(plines_[i][2], plines_[i][3]), cv::Scalar(0, 255, 0), 10, cv::LINE_8, 0);
  
  for(int i = 0; i < plines_edge_.size(); i++)
    cv::line(image_draw, cv::Point(plines_edge_[i][0], plines_edge_[i][1]), cv::Point(plines_edge_[i][2], plines_edge_[i][3]), cv::Scalar(255, 165, 0), 10, cv::LINE_8, 0);
  
  for(int i = 0; i < clines_.size(); i++)
    cv::line(image_draw, cv::Point(clines_[i][0], clines_[i][1]), cv::Point(clines_[i][2], clines_[i][3]), cv::Scalar(128, 0, 128), 10, cv::LINE_8, 0);
  
  cv::circle(image_draw, cv::Point(image_resize_.cols / 2, image_resize_.rows / 2), 20, cv::Scalar(128, 0, 128), -1, cv::LINE_8, 0);
  return;
}


std::string VisionNode::D2S(double val, int digits)
{
  digits = std::max(0, std::min(7, digits));

  std::string ret;

  double positive_val = abs(val);

  if(digits != 0)
  {
    double pow_val = pow(10, digits);
    double big_val = positive_val * pow_val;
    int big_int_part = int(big_val);
    double res_val = big_int_part / pow_val;

    int int_part = int(res_val);
    int dec_part = int((res_val - int_part) * pow_val);

    ret = std::to_string(int_part) + "." + std::to_string(dec_part);
  }
  else
  {
    int int_part = int(positive_val);
    ret = std::to_string(int_part);
  }

  if(val < 0)
    ret = "-" + ret;
  
  return ret;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  
  ros::NodeHandle nh;

  VisionNode vision_node(nh);

  ROS_INFO_STREAM("vision_node Node is OK !");

  ros::spin();

  return 0;

}