#include "wind_turbine_insp/sensing_node.h"
#include <chrono>

using namespace wind_turbine_insp;


SensingNode::SensingNode(ros::NodeHandle nh): nh_(nh) {

  initVariable();

  initServiceClient();
  initSubscriber();

  ros::Duration(1.0).sleep();

  initPublisher();

  initServiceServer();

  initTimer();
}

SensingNode::~SensingNode(){

}

bool SensingNode::initSubscriber()
{

  stage_mode_subscriber_ = nh_.subscribe<insp_msgs::StageMode>("/wind_turbine_insp/stage_mode", 2, &SensingNode::getStageModeCallback, this);
  ptcloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/wind_turbine_insp/forward_ptcloud", 2, &SensingNode::getPtcloudCallback, this);
  local_odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/wind_turbine_insp/local_odom", 2, &SensingNode::getLocalOdomCallback, this);
  
  ROS_INFO_STREAM("Sensing_Node: init Subscriber");

  return true;

}

bool SensingNode::initServiceClient(){

  blade_vector_detect_client_ = nh_.serviceClient<insp_msgs::BladeVectorDetect>("/detection/blade_vec_detect");
  blade_tip_check_client_ = nh_.serviceClient<insp_msgs::BladeTipCheck>("/detection/blade_tip_check");

  ROS_INFO_STREAM("Sensing_Node: init ServiceClient");

  return true;
}

bool SensingNode::initTimer()
{
  ptcloud_proc_timer_ = nh_.createTimer(ros::Rate(rate_ptcloud_proc_), &SensingNode::ptcloudProcCallback, this);
  node_status_timer_ = nh_.createTimer(ros::Rate(1.0), &SensingNode::nodeStatusPublisherCallback, this);

  ROS_INFO_STREAM("Sensing_Node: init Timer");

  return true;
}


bool SensingNode::initServiceServer()
{
  true_blade_tip_server_ = nh_.advertiseService("/wind_turbine_insp/blade_tip_query", &SensingNode::trueTipPointCallback, this);

  ROS_INFO_STREAM("Sensing_Node: init ServiveServer");

  return true;
}


bool SensingNode::initPublisher()
{
  ptcloud_feature_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/wind_turbine_insp/ptcloud_feature", 2);

  ptcloud_key_publisher_ = nh_.advertise<insp_msgs::PtCloudKey>("/wind_turbine_insp/ptcloud_key", 2);

  ptcloud_remove_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/wind_turbine_insp/ptcloud_remove", 2);
  ptcloud_sum_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/wind_turbine_insp/ptcloud_sum", 2);
  node_status_publisher_ = nh_.advertise<std_msgs::Int32>("/wind_turbine_insp/node_status", 2);

  ROS_INFO_STREAM("Sensing_Node: init Subscriber");

  return true;

}

bool SensingNode::initVariable()
{

  success_count_ = 0;

  nh_.getParam("sensing_node/fan_tower_height",   fan_tower_height_);
  nh_.getParam("sensing_node/fan_blade_length",   fan_blade_length_);
  nh_.getParam("sensing_node/fan_neck_depth",   fan_neck_depth_);
  nh_.getParam("sensing_node/fan_tower_radius",   fan_tower_radius_);
  nh_.getParam("sensing_node/fan_blade_pitch",   fan_blade_pitch_);

  nh_.getParam("sensing_node/start_blade",   start_blade_);

  nh_.getParam("sensing_node/rate_ptcloud_proc",   rate_ptcloud_proc_);

  nh_.getParam("sensing_node/detect_dist",   detect_dist_);
  nh_.getParam("sensing_node/track_dist",   track_dist_);

  visPtr_ = std::make_shared<visualization::Visualization>(nh_);

  nh_.getParam("sensing_node/tower_remove_radius",   tower_remove_radius_);
  nh_.getParam("sensing_node/tower_top_adjust_k",   tower_top_adjust_k_);

  nh_.getParam("sensing_node/blade_dir_err",   blade_dir_err_);
  nh_.getParam("sensing_node/blade_window_size_near",   blade_window_size_near_);
  nh_.getParam("sensing_node/blade_window_size_far",   blade_window_size_far_);
  nh_.getParam("sensing_node/plane_window_size",   plane_window_size_);

  nh_.getParam("sensing_node/voxel_leaf_size",   voxel_leaf_size_);
  
  nh_.getParam("sensing_node/count_angle_detect",   count_angle_detect_);
  nh_.getParam("sensing_node/weight_hub_posi_loc",   weight_hub_posi_loc_);

  nh_.getParam("sensing_node/arrow_width",   arrow_width_);
  
  nh_.getParam("sensing_node/enable_hub_vec_z",   enable_hub_vec_z_);

  nh_.getParam("sensing_node/thold_tip_check_hit_pt",   thold_tip_check_hit_pt_);
  nh_.getParam("sensing_node/thold_tip_check_sum",   thold_tip_check_sum_);

  nh_.getParam("sensing_node/thold_change_tower_height",   thold_change_tower_height_);
  nh_.getParam("sensing_node/change_k",   change_k_);
  nh_.getParam("sensing_node/detect_circle_radius",   detect_circle_radius_);
  nh_.getParam("sensing_node/search_blade_radius",   search_blade_radius_);
  nh_.getParam("sensing_node/search_tip_number",   search_tip_number_);

  coeff_line_tower.init(6);
  coeff_hub.init(3);
  coeff_plane.init(4);
  coeff_line_blade.init(6);
 
  if_update_hub_xyz_in_loc_ = false;

  last_detect_trigger_time = ros::Time::now();

  for(int i = 0; i < 6; i++)
    have_save_angle_vec[i] = false;
  for(int i = 0; i < 6; i++)
    have_clear_coeff_blade[i] = false;
  for(int i = 0; i < 6; i++)
    have_send_first_legal_key[i] = 0;
  for(int i = 0; i < 6; i++)
    have_send_origin_blade_dir[i] = false;
  for(int i = 0; i < 2; i++)
    have_clear_coeff_tower[i] = false;

  have_enter_hub_esti_ = false;

  str_topic_dir_ = "/wind_turbine_insp/marker_dir";
  str_topic_nor_ = "/wind_turbine_insp/marker_nor";
  str_topic_vtc_ = "/wind_turbine_insp/marker_vtc";
  str_topic_pln_ = "/wind_turbine_insp/marker_pln";
  str_topic_tow_ = "/wind_turbine_insp/marker_tow";
  str_topic_sca_ = "/wind_turbine_insp/marker_sca";

  ptcloud_key_.header.frame_id = "world";

  tree.reset(new pcl::search::KdTree<PCT> ());
  cloud.reset(new pcl::PointCloud<PCT> ());
  cloud_normals.reset(new pcl::PointCloud<pcl::Normal>());
  
  coefficients_plane.reset(new pcl::ModelCoefficients ());
  coefficients_line_blade.reset(new pcl::ModelCoefficients ());
  coefficients_line_tower.reset(new pcl::ModelCoefficients ());
  
  inliers_plane.reset(new pcl::PointIndices ());
  inliers_line_blade.reset(new pcl::PointIndices ());
  inliers_line_tower.reset(new pcl::PointIndices ());
  
  cloud_plane.reset(new pcl::PointCloud<PCT> ());
  cloud_line_blade.reset(new pcl::PointCloud<PCT> ());
  cloud_line_tower.reset(new pcl::PointCloud<PCT> ());
  cloud_blade_sum.reset(new pcl::PointCloud<PCT> ());
  
  if_save_pcd_ = false;
  pcd_save_name_ = "/home/hilab/pcdfiles/ptcloud_";

  kdtree = pcl::KdTreeFLANN<PCT>();

  arrow_dir_beg_ = Eigen::Vector3d::Zero();
  arrow_dir_end_ = Eigen::Vector3d::Zero();
  arrow_nor_beg_ = Eigen::Vector3d::Zero();
  arrow_nor_end_ = Eigen::Vector3d::Zero();
  arrow_vtc_beg_ = Eigen::Vector3d::Zero();
  arrow_vtc_end_ = Eigen::Vector3d::Zero();

  num_fan_tip_check_hit_pt_none_ = 0;
  if_fan_tip_arrive_ = false;

  send_blade_detect_count_ = 0;
  change_up_ = true;

  correct_foot_posi_ = Eigen::Vector3d(0, 0, 0);

  // moderate violet red: (199, 21, 133)
  // brown: (139, 26, 26)
  // chocolate shit: (255, 127, 36)
  // midium slate blue(123, 104, 238)
  // pink (255, 105, 180)

  printf("--------------------\n");
  printf("Loading Params Start\n");

  printf("fan_tower_height: %.2f\n", fan_tower_height_);
  printf("fan_blade_length: %.2f\n", fan_blade_length_);
  printf("fan_neck_depth: %.2f\n", fan_neck_depth_);
  printf("fan_tower_radius: %.2f\n", fan_tower_radius_);
  printf("fan_blade_pitch: %.2f\n", fan_blade_pitch_);
  printf("start_blade: %d\n", start_blade_);
  printf("rate_ptcloud_proc: %.2f\n", rate_ptcloud_proc_);
  printf("detect_dist: %.2f\n", detect_dist_);
  printf("track_dist: %.2f\n", track_dist_);
  printf("tower_remove_radius: %.2f\n", tower_remove_radius_);
  printf("tower_top_adjust_k: %.2f\n", tower_top_adjust_k_);
  printf("blade_dir_err: %.2f\n", blade_dir_err_);
  printf("blade_window_size_near: %d\n", blade_window_size_near_);
  printf("blade_window_size_far: %d\n", blade_window_size_far_);
  printf("plane_window_size: %d\n", plane_window_size_);
  printf("voxel_leaf_size: %.2f\n", voxel_leaf_size_);
  printf("count_angle_detect: %.d\n", count_angle_detect_);
  printf("weight_hub_posi_loc: %.2f\n", weight_hub_posi_loc_);
  printf("arrow_width: %.2f\n", arrow_width_);
  printf("enable_hub_vec_z: %d\n", enable_hub_vec_z_);
  printf("thold_tip_check_hit_pt: %d\n", thold_tip_check_hit_pt_);
  printf("thold_tip_check_sum: %d\n", thold_tip_check_sum_);
  printf("thold_change_tower_height: %d\n", thold_change_tower_height_);
  printf("change_k: %.2f\n", change_k_);
  printf("detect_circle_radius: %.2f\n", detect_circle_radius_);
  printf("search_blade_radius: %.2f\n", search_blade_radius_);
  printf("search_tip_number: %d\n", search_tip_number_);

  printf("Loading Params Finish\n");
  printf("---------------------\n");

  ROS_INFO_STREAM("Sensing_Node: init Variable");

  return true;

}





void SensingNode::VoxelFilter()
{
  pcl::VoxelGrid<PCT> sor;
  sor.setInputCloud(cloud_blade_sum);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*cloud_blade_sum);
}

void SensingNode::RemoveTowerPtcloud()
{
  // ROS_INFO("check 1: %ld", cloud -> size());
  double remove_height = (coeff_hub.getNum() > 0)? (coeff_hub[2]) : (fan_tower_height_);

  double tower_x = (remove_height * tower_top_adjust_k_ - coeff_line_tower[2]) * coeff_line_tower[3] / coeff_line_tower[5] + coeff_line_tower[0];
  double tower_y = (remove_height * tower_top_adjust_k_ - coeff_line_tower[2]) * coeff_line_tower[4] / coeff_line_tower[5] + coeff_line_tower[1];
  tower_x += coeff_plane[0] * fan_tower_radius_;
  tower_y += coeff_plane[1] * fan_tower_radius_;

  pcl::PointCloud<PCT>::Ptr tmp_cloud (new pcl::PointCloud<PCT>);
  for(int i = 0; i < cloud -> size(); i++)
  {
    if(cloud -> points[i].z < remove_height && (pow(cloud -> points[i].x - tower_x, 2) + pow(cloud -> points[i].y - tower_y, 2) < pow(tower_remove_radius_, 2)))
      continue;
    else
      tmp_cloud -> push_back(cloud -> points[i]);
  }
  *cloud = *tmp_cloud;
  // ROS_INFO("check 2: %ld", cloud -> size());

}


void SensingNode::RemoveOutlier()
{
  pcl::StatisticalOutlierRemoval<PCT> sor;
  sor.setInputCloud(cloud_blade_sum);
  sor.setMeanK(30);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_blade_sum);
}

// 计算点云每个点的法向量
void SensingNode::EstimatePointNormals()
{
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (30);
  ne.compute (*cloud_normals); 
}

// **********************************************************
// --------------------   Extract Plane   -------------------
// **********************************************************

bool SensingNode::ExtractPlaneObjects()
{
  seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.2);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(5.0);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_plane, *coefficients_plane);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	extract.filter(*cloud_plane);

  if (cloud_plane->points.empty ()){
    std::cerr << "Can't detect plane." << std::endl;
    return false;
  }
  else
  {
    // std::cerr << "PointCloud plane: " << cloud_plane->points.size () << " points." << std::endl;
  }

  sensor_msgs::PointCloud2 tmp_msg;
  pcl::toROSMsg(*cloud_plane, tmp_msg);
  tmp_msg.header.frame_id = "world";
  tmp_msg.header.stamp = ros::Time::now();

  ptcloud_feature_publisher_.publish(tmp_msg);

  return true;

}


// **********************************************************
// --------------------   Extract Line   --------------------
// **********************************************************

bool SensingNode::ExtractBladeLineObjects()
{
  seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setNormalDistanceWeight(0.2);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(3.0);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_line_blade, *coefficients_line_blade);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers_line_blade);
	extract.setNegative(false);
	extract.filter(*cloud_line_blade);

  if (cloud_line_blade->points.empty ()){
    std::cerr << "Can't detect line ." << std::endl;
    return false;
  }
  else
    std::cerr << "PointCloud line: " << cloud_line_blade->points.size () << " points." << std::endl;

  sensor_msgs::PointCloud2 tmp_msg;
  pcl::toROSMsg(*cloud_line_blade, tmp_msg);
  tmp_msg.header.frame_id = "world";
  tmp_msg.header.stamp = ros::Time::now();
  
  ptcloud_feature_publisher_.publish(tmp_msg);
  return true;

}


bool SensingNode::ExtractTowerLineObjects()
{
  seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setNormalDistanceWeight(0.2);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(4.0);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_line_tower, *coefficients_line_tower);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers_line_tower);
	extract.setNegative(false);
	extract.filter(*cloud_line_tower);

  if (cloud_line_tower->points.empty ()){
    std::cerr << "Can't detect line ." << std::endl;
    return false;
  }
  else
    std::cerr << "PointCloud line: " << cloud_line_tower->points.size () << " points." << std::endl;


  sensor_msgs::PointCloud2 tmp_msg;
  pcl::toROSMsg(*cloud_line_tower, tmp_msg);
  tmp_msg.header.frame_id = "world";
  tmp_msg.header.stamp = ros::Time::now();
  
  ptcloud_feature_publisher_.publish(tmp_msg);

  return true;

}


Eigen::Vector3d SensingNode::getCorrectFootPoint(Eigen::Vector3d fpt, double radius)
{
  kdtree.setInputCloud(cloud_blade_sum);
  
  std::vector<int> indices;
  std::vector<float> distances;

  PCT point;
  point.x = fpt.x();
  point.y = fpt.y();
  point.z = fpt.z();

  kdtree.radiusSearch(point, radius, indices, distances);

  if(indices.size() == 0)
  {
    return fpt;
  }

  Eigen::Vector3d ret = Eigen::Vector3d(0, 0, 0);
  for(int i = 0; i < indices.size(); i++)
  {
    ret += Eigen::Vector3d(cloud_blade_sum -> points[indices[i]].x, cloud_blade_sum -> points[indices[i]].y, cloud_blade_sum -> points[indices[i]].z);
  }
  ret /= indices.size();

  last_crt_posi_ = ret;

  return ret;
}

Eigen::Vector3d SensingNode::getNearestPointsMean(Eigen::Vector3d pt, int n)
{
  kdtree.setInputCloud(cloud_blade_sum);

  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  PCT point;
  point.x = pt.x();
  point.y = pt.y();
  point.z = pt.z();

  kdtree.nearestKSearch(point, n, pointIdxNKNSearch, pointNKNSquaredDistance);

  Eigen::Vector3d ret = Eigen::Vector3d(0, 0, 0);

  for (int i = 0; i < n; ++i)
  {
    ret += Eigen::Vector3d(cloud_blade_sum -> points[pointIdxNKNSearch[i]].x, cloud_blade_sum -> points[pointIdxNKNSearch[i]].y, cloud_blade_sum -> points[pointIdxNKNSearch[i]].z);
  }

  ret /= n;

  return ret;
}


void SensingNode::getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg){
  stage_mode_ = *msg;
  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_LAND)
  {
    ros::Duration(1.0).sleep();
    ros::shutdown();
  }
}

void SensingNode::getPtcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  cloud_ros = *msg;
}

void SensingNode::getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  PointMsgToXYZVec((*msg).pose.pose.position, local_posi_);
}


void SensingNode::ptcloudProcCallback(const ros::TimerEvent& event)
{
  pcl::fromROSMsg(cloud_ros, *cloud);  

  if(cloud -> size() < 1)
  {
    if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_1_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_2_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_1_MODE_BLD_3_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_1_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_2_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_2_MODE_BLD_3_GO 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_1_S1 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_2_S1 
    || stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_TRK_4_MODE_BLD_3_S1)
    {
      Eigen::Vector3d hub_posi = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
      Eigen::Vector3d hub_nor_vec = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
      Eigen::Vector3d hub_front_posi = hub_posi - hub_nor_vec * track_dist_;
      double dist_to_hub_front = (local_posi_ - hub_front_posi).norm();
      if(dist_to_hub_front > fan_blade_length_ * 2 / 3)
      {

        ptcloud_key_.blade_legal = false;
        // calculate dist and vec from vehicle to blade
        Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]) - local_posi_;
        Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_blade[3], coeff_line_blade[4], coeff_line_blade[5]);
        Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
        Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);

        double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
        if(abs(tmp_err_angle) > M_PI / 2){
          tmp_cross_cross = -tmp_cross_cross;
        }

        double blade_dist = tmp_cross.norm() / tmp_vec_ln.norm();
        Eigen::Vector3d vert_vec = tmp_cross_cross / tmp_cross_cross.norm();

        Eigen::Vector3d foot_posi = local_posi_ + vert_vec * blade_dist;

        ROS_INFO("dist(line angle illegal): %.2f", blade_dist);

        ptcloud_key_.blade_x = coeff_line_blade[0];
        ptcloud_key_.blade_y = coeff_line_blade[1];
        ptcloud_key_.blade_z = coeff_line_blade[2];
        ptcloud_key_.blade_vx = coeff_line_blade[3];
        ptcloud_key_.blade_vy = coeff_line_blade[4];
        ptcloud_key_.blade_vz = coeff_line_blade[5];
        ptcloud_key_.blade_dist = blade_dist;
        ptcloud_key_.blade_nx = vert_vec(0);
        ptcloud_key_.blade_ny = vert_vec(1);
        ptcloud_key_.blade_nz = vert_vec(2);
        ptcloud_key_.blade_fx = foot_posi(0);
        ptcloud_key_.blade_fy = foot_posi(1);
        ptcloud_key_.blade_fz = foot_posi(2);
        ptcloud_key_.blade_cx = foot_posi(0);
        ptcloud_key_.blade_cy = foot_posi(1);
        ptcloud_key_.blade_cz = foot_posi(2);

        // ***************************************************
        
        ROS_INFO("send tip check request(no ptcloud)");

        insp_msgs::BladeTipCheck blade_tip_check_srv;


        blade_tip_check_srv.request.start_pt_x = ptcloud_key_.blade_fx;
        blade_tip_check_srv.request.start_pt_y = ptcloud_key_.blade_fy;
        blade_tip_check_srv.request.start_pt_z = ptcloud_key_.blade_fz;



        blade_tip_check_srv.request.vec_x = ptcloud_key_.blade_vx;
        blade_tip_check_srv.request.vec_y = ptcloud_key_.blade_vy;
        blade_tip_check_srv.request.vec_z = ptcloud_key_.blade_vz;

        blade_tip_check_srv.request.radius = 1.0;
        blade_tip_check_srv.request.max_empty_depth = 5.0;

        if(blade_tip_check_client_.call(blade_tip_check_srv) && blade_tip_check_srv.response.success)
        {
          if(blade_tip_check_srv.response.hit_points_num < thold_tip_check_hit_pt_)
          {
            num_fan_tip_check_hit_pt_none_ ++;
            ROS_INFO("arrive blade tip once! ");
          }
          else
          {
            ptcloud_key_.tip_x = blade_tip_check_srv.response.end_pt_x;
            ptcloud_key_.tip_y = blade_tip_check_srv.response.end_pt_y;
            ptcloud_key_.tip_z = blade_tip_check_srv.response.end_pt_z;
            num_fan_tip_check_hit_pt_none_ = 0;
          }
          if(num_fan_tip_check_hit_pt_none_ >= thold_tip_check_sum_)
          {
            ptcloud_key_.tip_arrive = true;
            ROS_INFO("arrive blade tip confirm, enter next stage mode!");
            num_fan_tip_check_hit_pt_none_ = 0;
          }
        }
      }
      // ***************************************************

    }

    ptcloud_key_.header.stamp = ros::Time::now();
    ptcloud_key_publisher_.publish(ptcloud_key_);

    ptcloud_key_.tip_arrive = false;

    return;
  }

  // RemoveOutlier();

  if(stage_mode_.stage_mode >= insp_msgs::StageMode::STAGE_DTC_MODE_ADJ && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    RemoveTowerPtcloud();

    sensor_msgs::PointCloud2 tmp_msg;
    pcl::toROSMsg(*cloud, tmp_msg);
    tmp_msg.header.frame_id = "world";
    tmp_msg.header.stamp = ros::Time::now();
    ptcloud_remove_publisher_.publish(tmp_msg);
  }

  EstimatePointNormals();
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_RISE)
  {

    if(!have_clear_coeff_tower[0])
    {
      coeff_line_tower.clearVal();
      have_clear_coeff_tower[0] = true;
    }

    bool have_ln = ExtractTowerLineObjects();
    
    if(!have_ln) // if no detect line
    {
      ptcloud_key_.tower_legal = false;
    }
    else  // if detect line
    {
      // confirm the tower dir vector is down -> up
      Eigen::Vector3d dv_ori = Eigen::Vector3d(0, 0, 1);
      Eigen::Vector3d dv_now = Eigen::Vector3d( coefficients_line_tower -> values[3], 
                                                coefficients_line_tower -> values[4], 
                                                coefficients_line_tower -> values[5]);

      double aa_angle = acos(dv_ori.dot(dv_now) / (dv_ori.norm() * dv_now.norm()));
      if(abs(aa_angle) > M_PI / 2)
      {
        coefficients_line_tower -> values[3] = -coefficients_line_tower -> values[3];
        coefficients_line_tower -> values[4] = -coefficients_line_tower -> values[4];
        coefficients_line_tower -> values[5] = -coefficients_line_tower -> values[5];
        dv_now = -dv_now;
      }

      // judge if the tower dir vector is legal
      aa_angle = acos(dv_ori.dot(dv_now) / (dv_ori.norm() * dv_now.norm()));

      if(abs(aa_angle) > DEG2RAD(20.0)) // if illegal line, update key msg
      {
        ptcloud_key_.tower_legal = false;
      }
      else // if legal line, add avg, update key msg
      {
        coeff_line_tower.addValAvg(coefficients_line_tower -> values);

        // vector: local_position -> point in tower line
        Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_tower[0], coeff_line_tower[1], coeff_line_tower[2]) - local_posi_;
        // vector: tower line vector, down -> up
        Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_tower[3], coeff_line_tower[4], coeff_line_tower[5]);
        // vector: vertical vector
        Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
        Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);
        // confirm vertical vector is local_position -> foot_point
        double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
        if(abs(tmp_err_angle) > M_PI / 2){
          tmp_cross_cross = -tmp_cross_cross;
        }

        double tower_dist = tmp_cross.norm() / tmp_vec_ln.norm();
        Eigen::Vector3d tower_vec = tmp_cross_cross / tmp_cross_cross.norm();
        
        ROS_INFO("dist, vec: %.2f, (%.2f, %.2f, %.2f)", tower_dist, tower_vec(0), tower_vec(1), tower_vec(2));

        ptcloud_key_.tower_legal = true;

        ptcloud_key_.tower_x = coeff_line_tower[0];
        ptcloud_key_.tower_y = coeff_line_tower[1];
        ptcloud_key_.tower_z = coeff_line_tower[2];
        ptcloud_key_.tower_vx = coeff_line_tower[3];
        ptcloud_key_.tower_vy = coeff_line_tower[4];
        ptcloud_key_.tower_vz = coeff_line_tower[5];
        ptcloud_key_.tower_dist = tower_dist;
        ptcloud_key_.tower_nx = tower_vec(0);
        ptcloud_key_.tower_ny = tower_vec(1);
        ptcloud_key_.tower_nz = tower_vec(2);

      }

    }

    // update rviz
    arrow_dir_beg_ = Eigen::Vector3d(coeff_line_tower[0], coeff_line_tower[1], coeff_line_tower[2]);
    arrow_dir_end_ = Eigen::Vector3d(coeff_line_tower[0] + coeff_line_tower[3] * detect_dist_, 
                                     coeff_line_tower[1] + coeff_line_tower[4] * detect_dist_, 
                                     coeff_line_tower[2] + coeff_line_tower[5] * detect_dist_);

    arrow_nor_beg_ = local_posi_;
    arrow_nor_end_ = Eigen::Vector3d(local_posi_(0) + ptcloud_key_.tower_nx * detect_dist_, 
                                     local_posi_(1) + ptcloud_key_.tower_ny * detect_dist_, 
                                     local_posi_(2) + ptcloud_key_.tower_nz * detect_dist_);

    arrow_vtc_beg_ = Eigen::Vector3d::Zero();
    arrow_vtc_end_ = Eigen::Vector3d::Zero();

    // pub key
    ptcloud_key_.header.stamp = ros::Time::now();  
    ptcloud_key_publisher_.publish(ptcloud_key_);


  }
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  else if (stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_ADJ)
  {
    ptcloud_key_.ris_to_adj = true;

    bool have_plane = ExtractPlaneObjects();

    if(!have_plane) // if no detect plane, update key msg
    {
      ptcloud_key_.hub_legal = false;
    }
    else // if detect plane
    {
      if(abs(coefficients_plane -> values[2]) > 0.7) // if illegal plane, update key msg
      {
        ptcloud_key_.hub_legal = false;
        ROS_INFO("illegal plane: > 0.7: %.2f", coefficients_plane -> values[2]);
      }
      else // if leagl plane, add avg, update key msg
      {
        double tower_x = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vx / ptcloud_key_.tower_vz + ptcloud_key_.tower_x;
        double tower_y = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vy / ptcloud_key_.tower_vz + ptcloud_key_.tower_y;
        tower_x += coeff_plane[0] * fan_tower_radius_;
        tower_y += coeff_plane[1] * fan_tower_radius_;

        // confirm vec local_position -> hub
        Eigen::Vector3d vec_now = Eigen::Vector3d(coefficients_plane -> values[0], coefficients_plane -> values[1], coefficients_plane -> values[2]);
        Eigen::Vector3d vec_hub = Eigen::Vector3d(tower_x, tower_y, fan_tower_height_) - local_posi_;
        double angle_tmp = acos(vec_now.dot(vec_hub) / (vec_now.norm() * vec_hub.norm()));
        if(abs(angle_tmp) > M_PI / 2){
          vec_now = -vec_now;
          coefficients_plane -> values[0] = -coefficients_plane -> values[0];
          coefficients_plane -> values[1] = -coefficients_plane -> values[1];
          coefficients_plane -> values[2] = -coefficients_plane -> values[2];
        }

        if(coeff_plane.getNum() > plane_window_size_)
          coeff_plane.setNum(plane_window_size_);

        coeff_plane.addValAvg(coefficients_plane -> values); 

        Eigen::Vector3d tower_posi = Eigen::Vector3d(tower_x, tower_y, fan_tower_height_);
        Eigen::Vector3d vec_tower_to_hub = -Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
        Eigen::Vector3d hub_posi = tower_posi + vec_tower_to_hub / vec_tower_to_hub.norm() * fan_neck_depth_;

        if(!have_enter_hub_esti_)
          coeff_hub.addValAvg(std::vector<float>({hub_posi(0), hub_posi(1), hub_posi(2)}));

        ptcloud_key_.hub_legal = true;

        if(enable_hub_vec_z_)
        {
          ptcloud_key_.hub_vx = coeff_plane[0];
          ptcloud_key_.hub_vy = coeff_plane[1];
          ptcloud_key_.hub_vz = coeff_plane[2];
        }
        else
        {
          Eigen::Vector2d hub_vec_2d = Eigen::Vector2d(coeff_plane[0], coeff_plane[1]).normalized() * cos(DEG2RAD(fan_blade_pitch_));
          double hub_vec_z = sin(DEG2RAD(fan_blade_pitch_));
          ptcloud_key_.hub_vx = hub_vec_2d(0);
          ptcloud_key_.hub_vy = hub_vec_2d(1);
          ptcloud_key_.hub_vz = -hub_vec_z;
        }

        ptcloud_key_.hub_x = coeff_hub[0];
        ptcloud_key_.hub_y = coeff_hub[1];
        ptcloud_key_.hub_z = coeff_hub[2];
      }
      
    }

    // update rviz plane
    // Eigen::Vector3d vec_now = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
    // Eigen::Vector3d vec_now = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], 0);
    Eigen::Vector3d vec_now = Eigen::Vector3d(ptcloud_key_.hub_vx, ptcloud_key_.hub_vy, ptcloud_key_.hub_vz);
    double aa_angle = acos(Eigen::Vector3d(1, 0, 0).dot(vec_now));
    Eigen::Vector3d aa_axis = Eigen::Vector3d(1, 0, 0).cross(vec_now);
    aa_axis.normalize();
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(aa_angle, aa_axis);
    Eigen::Quaterniond quat = Eigen::Quaterniond(aa);
    quat.normalize();

    // visualize plane
    Eigen::Vector3d cube_center = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
    Eigen::Vector3d cube_scale = Eigen::Vector3d(0.1, 50, 50);
    visPtr_ -> visualize_a_cube(cube_center, cube_scale, quat, str_topic_pln_, visualization::red, 0.5);

    // update rviz
    arrow_dir_beg_ = Eigen::Vector3d::Zero();
    arrow_dir_end_ = Eigen::Vector3d::Zero();

    arrow_nor_beg_ = local_posi_;
    arrow_nor_end_ = Eigen::Vector3d(local_posi_(0) + ptcloud_key_.hub_vx * track_dist_, 
                                     local_posi_(1) + ptcloud_key_.hub_vy * track_dist_, 
                                     local_posi_(2) + ptcloud_key_.hub_vz * track_dist_);
    
    arrow_vtc_beg_ = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
    arrow_vtc_end_ = Eigen::Vector3d(coeff_hub[0] - ptcloud_key_.hub_vx * track_dist_, 
                                     coeff_hub[1] - ptcloud_key_.hub_vy * track_dist_, 
                                     coeff_hub[2] - ptcloud_key_.hub_vz * track_dist_);

    // if update stage mode
    ROS_INFO("detect plane number: %d", coeff_plane.getNum());
    if(coeff_plane.getNum() > 20)
      ptcloud_key_.adj_to_loc = true;
    
    ptcloud_key_.blade_fx = coeff_hub[0];
    ptcloud_key_.blade_fy = coeff_hub[1];
    ptcloud_key_.blade_fz = coeff_hub[2];
    ptcloud_key_.blade_cx = coeff_hub[0];
    ptcloud_key_.blade_cy = coeff_hub[1];
    ptcloud_key_.blade_cz = coeff_hub[2];

    // pub key msg
    ptcloud_key_.header.stamp = ros::Time::now();
    ptcloud_key_publisher_.publish(ptcloud_key_);


  }
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_DTC_MODE_LOC)
  {
    have_enter_hub_esti_ = true;

    if(!if_update_hub_xyz_in_loc_)
    {
      int num_ori_hub_posi_esti = int(count_angle_detect_ / weight_hub_posi_loc_ * (1 - weight_hub_posi_loc_)) + 1;
      coeff_hub.setNum(num_ori_hub_posi_esti);
      if_update_hub_xyz_in_loc_ = true;
    }

    ptcloud_key_.ris_to_adj = false;

    bool have_plane = ExtractPlaneObjects();

    if(!have_plane) // if no detect plane, update key msg
    {
      ptcloud_key_.hub_legal = false;
    }
    else // if detect plane
    {
      if(abs(coefficients_plane -> values[2]) > 0.5) // if illegal plane, update key msg
      {
        ptcloud_key_.hub_legal = false;
      }
      else // if leagl plane, add avg, update key msg
      {
        double tower_x = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vx / ptcloud_key_.tower_vz + ptcloud_key_.tower_x;
        double tower_y = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vy / ptcloud_key_.tower_vz + ptcloud_key_.tower_y;
        tower_x += coeff_plane[0] * fan_tower_radius_;
        tower_y += coeff_plane[1] * fan_tower_radius_;

        // confirm vec local_position -> hub
        Eigen::Vector3d vec_now = Eigen::Vector3d(coefficients_plane -> values[0], coefficients_plane -> values[1], coefficients_plane -> values[2]);
        Eigen::Vector3d vec_hub = Eigen::Vector3d(tower_x, tower_y, fan_tower_height_) - local_posi_;
        double angle_tmp = acos(vec_now.dot(vec_hub) / (vec_now.norm() * vec_hub.norm()));
        if(abs(angle_tmp) > M_PI / 2){
          vec_now = -vec_now;
          coefficients_plane -> values[0] = -coefficients_plane -> values[0];
          coefficients_plane -> values[1] = -coefficients_plane -> values[1];
          coefficients_plane -> values[2] = -coefficients_plane -> values[2];
        }

        if(coeff_plane.getNum() > plane_window_size_)
          coeff_plane.setNum(plane_window_size_);
        
        // coeff_plane.addValAvg(coefficients_plane -> values); 

        // Eigen::Vector3d tower_posi = Eigen::Vector3d(tower_x, tower_y, fan_tower_height_);
        // Eigen::Vector3d vec_tower_to_hub = -Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
        // Eigen::Vector3d hub_posi = tower_posi + vec_tower_to_hub / vec_tower_to_hub.norm() * fan_neck_depth_;

        // coeff_hub.addValAvg(std::vector<float>({hub_posi(0), hub_posi(1), hub_posi(2)}));

        ptcloud_key_.hub_legal = true;

        if(enable_hub_vec_z_)
        {
          ptcloud_key_.hub_vx = coeff_plane[0];
          ptcloud_key_.hub_vy = coeff_plane[1];
          ptcloud_key_.hub_vz = coeff_plane[2];
        }
        else
        {
          Eigen::Vector2d hub_vec_2d = Eigen::Vector2d(coeff_plane[0], coeff_plane[1]).normalized() * cos(DEG2RAD(fan_blade_pitch_));
          double hub_vec_z = sin(DEG2RAD(fan_blade_pitch_));
          ptcloud_key_.hub_vx = hub_vec_2d(0);
          ptcloud_key_.hub_vy = hub_vec_2d(1);
          ptcloud_key_.hub_vz = -hub_vec_z;
        }

        ptcloud_key_.hub_x = coeff_hub[0];
        ptcloud_key_.hub_y = coeff_hub[1];
        ptcloud_key_.hub_z = coeff_hub[2];
      }

    }


    // update rviz plane
    // Eigen::Vector3d vec_now = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
    // Eigen::Vector3d vec_now = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], 0);
    Eigen::Vector3d vec_now = Eigen::Vector3d(ptcloud_key_.hub_vx, ptcloud_key_.hub_vy, ptcloud_key_.hub_vz);
    double aa_angle = acos(Eigen::Vector3d(1, 0, 0).dot(vec_now));
    Eigen::Vector3d aa_axis = Eigen::Vector3d(1, 0, 0).cross(vec_now);
    aa_axis.normalize();
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(aa_angle, aa_axis);
    Eigen::Quaterniond quat = Eigen::Quaterniond(aa);
    quat.normalize();

    // visualize plane
    Eigen::Vector3d cube_center = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
    Eigen::Vector3d cube_scale = Eigen::Vector3d(0.1, 50, 50);
    visPtr_ -> visualize_a_cube(cube_center, cube_scale, quat, str_topic_pln_, visualization::red, 0.5);

    // update rviz
    arrow_dir_beg_ = Eigen::Vector3d::Zero();
    arrow_dir_end_ = Eigen::Vector3d::Zero();

    arrow_nor_beg_ = local_posi_;
    arrow_nor_end_ = Eigen::Vector3d(local_posi_(0) + ptcloud_key_.hub_vx * track_dist_, 
                                     local_posi_(1) + ptcloud_key_.hub_vy * track_dist_, 
                                     local_posi_(2) + ptcloud_key_.hub_vz * track_dist_);

    arrow_vtc_beg_ = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
    arrow_vtc_end_ = Eigen::Vector3d(coeff_hub[0] - ptcloud_key_.hub_vx * track_dist_, 
                                     coeff_hub[1] - ptcloud_key_.hub_vy * track_dist_, 
                                     coeff_hub[2] - ptcloud_key_.hub_vz * track_dist_);


    if((ros::Time::now() - last_detect_trigger_time).toSec() > 1.0)
    {
      insp_msgs::BladeVectorDetect blade_vector_detect_srv;

      if(send_blade_detect_count_ > thold_change_tower_height_)
      {
        if(coeff_hub[2] >= fan_tower_height_ + 13.0)
          change_up_ = false;
        if(coeff_hub[2] <= fan_tower_height_ - 3.0)
          change_up_ = true;

        if(change_up_)
          coeff_hub.plusOneVal(2, change_k_);
        else
          coeff_hub.plusOneVal(2, -change_k_);
          
        send_blade_detect_count_ = 0;
        // success_count_ = 0;
      }
      ROS_INFO("[sensing node]: current detect height: %.2f", coeff_hub[2]);

      blade_vector_detect_srv.request.hub_x = ptcloud_key_.hub_x;
      blade_vector_detect_srv.request.hub_y = ptcloud_key_.hub_y;
      blade_vector_detect_srv.request.hub_z = ptcloud_key_.hub_z;

      blade_vector_detect_srv.request.hub_vx = ptcloud_key_.hub_vx;
      blade_vector_detect_srv.request.hub_vy = ptcloud_key_.hub_vy;
      blade_vector_detect_srv.request.hub_vz = ptcloud_key_.hub_vz;

      blade_vector_detect_srv.request.use_custom_radius = true;
      blade_vector_detect_srv.request.circle_radius = detect_circle_radius_;

      if(abs(local_posi_(2) - coeff_hub[2]) < detect_circle_radius_)
        send_blade_detect_count_ ++;

      if(blade_vector_detect_client_.call(blade_vector_detect_srv) && blade_vector_detect_srv.response.success)
      {
        pre_blade_vec[0] =  Eigen::Vector3d(blade_vector_detect_srv.response.blade_0_vx, 
                                            blade_vector_detect_srv.response.blade_0_vy, 
                                            blade_vector_detect_srv.response.blade_0_vz).normalized();
        pre_blade_vec[1] =  Eigen::Vector3d(blade_vector_detect_srv.response.blade_1_vx, 
                                            blade_vector_detect_srv.response.blade_1_vy, 
                                            blade_vector_detect_srv.response.blade_1_vz).normalized();
        pre_blade_vec[2] =  Eigen::Vector3d(blade_vector_detect_srv.response.blade_2_vx, 
                                            blade_vector_detect_srv.response.blade_2_vy, 
                                            blade_vector_detect_srv.response.blade_2_vz).normalized();
        coeff_hub.addValAvg(std::vector<float>({blade_vector_detect_srv.response.hub_x, 
                                                blade_vector_detect_srv.response.hub_y,
                                                blade_vector_detect_srv.response.hub_z}));
        ROS_INFO("vector detect 0: (%.2f, %.2f, %.2f)", pre_blade_vec[0](0), pre_blade_vec[0](1), pre_blade_vec[0](2));
        ROS_INFO("vector detect 1: (%.2f, %.2f, %.2f)", pre_blade_vec[1](0), pre_blade_vec[1](1), pre_blade_vec[1](2));
        ROS_INFO("vector detect 2: (%.2f, %.2f, %.2f)", pre_blade_vec[2](0), pre_blade_vec[2](1), pre_blade_vec[2](2));
        ROS_INFO("hub posi detect: (%.2f, %.2f, %.2f)", coeff_hub[0], coeff_hub[1], coeff_hub[2]);

        success_count_ ++;
      }
      last_detect_trigger_time = ros::Time::now();
    }
    ROS_INFO("success count: %d / %d", success_count_, count_angle_detect_);
    if(success_count_ > count_angle_detect_)
    {
      ptcloud_key_.loc_dtc_fns = true;
      success_count_ = 0;
    }

    ptcloud_key_.header.stamp = ros::Time::now();
    ptcloud_key_publisher_.publish(ptcloud_key_);

  }
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  else if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1 && stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_FNS)
  {
    int mode_num_per_blade;
    int idx_have_k;
    int alter;

    if(stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_2)
    {
      mode_num_per_blade = 2;
      idx_have_k = 1;
      alter = insp_msgs::StageMode::STAGE_TRK_1;
    }
    else if(stage_mode_.stage_mode < insp_msgs::StageMode::STAGE_TRK_4)
    {
      mode_num_per_blade = 4;
      idx_have_k = 2;
      alter = insp_msgs::StageMode::STAGE_TRK_2;
    }
    else
    {
      mode_num_per_blade = 8;
      idx_have_k = 4;
      alter = insp_msgs::StageMode::STAGE_TRK_4;
    }

    if(stage_mode_.stage_mode % mode_num_per_blade == 1)
    {
      ptcloud_key_.adj_to_loc = false;

      int idx_have = (stage_mode_.stage_mode - alter - 1) / idx_have_k;

      int idx_pre;
      if(start_blade_ == 3)
        idx_pre = (idx_have == 4) ? (1) : (idx_have);
      else if(start_blade_ == 2)
        idx_pre = (idx_have == 0) ? (1) : ((idx_have == 2) ? (0) : (2));
      else
        idx_pre = (5 - idx_have) / 2;

      int vec_sign = pow(-1, idx_have);

      // ROS_INFO("check: (%d, %d, %d)", idx_have, idx_pre, vec_sign);

      if(!have_clear_coeff_blade[idx_have])
      {
        coeff_line_blade.clearVal();
        have_clear_coeff_blade[idx_have] = true;
      }
      if(!have_save_angle_vec[idx_have])
      {
        coeff_line_blade.addValAvg(std::vector<float>({ptcloud_key_.hub_x, ptcloud_key_.hub_y, ptcloud_key_.hub_z, 
                                                      vec_sign * pre_blade_vec[idx_pre](0), vec_sign * pre_blade_vec[idx_pre](1), vec_sign * pre_blade_vec[idx_pre](2)}));
        have_save_angle_vec[idx_have] = true;
      }
      if(have_send_first_legal_key[idx_have] < 5)
      {
        Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]) - local_posi_;
        Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_blade[3], coeff_line_blade[4], coeff_line_blade[5]);
        Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
        Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);
        double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
        if(abs(tmp_err_angle) > M_PI / 2){
          tmp_cross_cross = -tmp_cross_cross;
        }

        double blade_dist = tmp_cross.norm() / tmp_vec_ln.norm();
        Eigen::Vector3d vert_vec = tmp_cross_cross / tmp_cross_cross.norm();

        Eigen::Vector3d foot_posi = local_posi_ + vert_vec * blade_dist;

        // update key msg
        ptcloud_key_.blade_legal = true;

        ptcloud_key_.blade_x = coeff_line_blade[0];
        ptcloud_key_.blade_y = coeff_line_blade[1];
        ptcloud_key_.blade_z = coeff_line_blade[2];
        ptcloud_key_.blade_vx = coeff_line_blade[3];
        ptcloud_key_.blade_vy = coeff_line_blade[4];
        ptcloud_key_.blade_vz = coeff_line_blade[5];
        ptcloud_key_.blade_dist = blade_dist;
        ptcloud_key_.blade_nx = vert_vec(0);
        ptcloud_key_.blade_ny = vert_vec(1);
        ptcloud_key_.blade_nz = vert_vec(2);
        ptcloud_key_.blade_fx = foot_posi(0);
        ptcloud_key_.blade_fy = foot_posi(1);
        ptcloud_key_.blade_fz = foot_posi(2);
        ptcloud_key_.blade_cx = foot_posi(0);
        ptcloud_key_.blade_cy = foot_posi(1);
        ptcloud_key_.blade_cz = foot_posi(2);

        ptcloud_key_.tip_arrive = false;

        have_send_first_legal_key[idx_have]++;
      }
      else  // main control
      {
        *cloud_blade_sum += *cloud;
        VoxelFilter();
        sensor_msgs::PointCloud2 sum_msg;
        pcl::toROSMsg(*cloud_blade_sum, sum_msg);
        sum_msg.header.frame_id = "world";
        sum_msg.header.stamp = ros::Time::now();
        ptcloud_sum_publisher_.publish(sum_msg);

        bool have_ln = ExtractBladeLineObjects();

        if(!have_ln) // if no detect line
        {
          ptcloud_key_.blade_legal = false;

          // calculate dist and vec from vehicle to blade
          Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]) - local_posi_;
          Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_blade[3], coeff_line_blade[4], coeff_line_blade[5]);
          Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
          Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);

          double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
          if(abs(tmp_err_angle) > M_PI / 2){
            tmp_cross_cross = -tmp_cross_cross;
          }

          double blade_dist = tmp_cross.norm() / tmp_vec_ln.norm();
          Eigen::Vector3d vert_vec = tmp_cross_cross / tmp_cross_cross.norm();

          Eigen::Vector3d foot_posi = local_posi_ + vert_vec * blade_dist;

          ROS_INFO("dist(no detect line): %.2f", blade_dist);

          // update key msg
          ptcloud_key_.blade_x = coeff_line_blade[0];
          ptcloud_key_.blade_y = coeff_line_blade[1];
          ptcloud_key_.blade_z = coeff_line_blade[2];
          ptcloud_key_.blade_vx = coeff_line_blade[3];
          ptcloud_key_.blade_vy = coeff_line_blade[4];
          ptcloud_key_.blade_vz = coeff_line_blade[5];
          ptcloud_key_.blade_dist = blade_dist;
          ptcloud_key_.blade_nx = vert_vec(0);
          ptcloud_key_.blade_ny = vert_vec(1);
          ptcloud_key_.blade_nz = vert_vec(2);
          ptcloud_key_.blade_fx = foot_posi(0);
          ptcloud_key_.blade_fy = foot_posi(1);
          ptcloud_key_.blade_fz = foot_posi(2);
          ptcloud_key_.blade_cx = foot_posi(0);
          ptcloud_key_.blade_cy = foot_posi(1);
          ptcloud_key_.blade_cz = foot_posi(2);
        }
        else // if detect line
        {
          // calculate if legal line
          Eigen::Vector3d now_vec = Eigen::Vector3d(coefficients_line_blade -> values[3], coefficients_line_blade -> values[4], coefficients_line_blade -> values[5]);
          Eigen::Vector3d tmp_pt_line = Eigen::Vector3d(coefficients_line_blade -> values[0], coefficients_line_blade -> values[1], coefficients_line_blade -> values[2]);
          Eigen::Vector3d tmp_to_hub = Eigen::Vector3d(ptcloud_key_.hub_x, ptcloud_key_.hub_y, ptcloud_key_.hub_z) - tmp_pt_line;
          double tmp_angle = acos(now_vec.dot(tmp_to_hub) / (now_vec.norm() * tmp_to_hub.norm()));
          if(tmp_angle < M_PI / 2)
          {
            coefficients_line_blade -> values[3] = coefficients_line_blade -> values[3] * (-vec_sign);
            coefficients_line_blade -> values[4] = coefficients_line_blade -> values[4] * (-vec_sign);
            coefficients_line_blade -> values[5] = coefficients_line_blade -> values[5] * (-vec_sign);
            now_vec = Eigen::Vector3d(coefficients_line_blade -> values[3], coefficients_line_blade -> values[4], coefficients_line_blade -> values[5]);
          }
          else
          {
            coefficients_line_blade -> values[3] = coefficients_line_blade -> values[3] * (vec_sign);
            coefficients_line_blade -> values[4] = coefficients_line_blade -> values[4] * (vec_sign);
            coefficients_line_blade -> values[5] = coefficients_line_blade -> values[5] * (vec_sign);
            now_vec = Eigen::Vector3d(coefficients_line_blade -> values[3], coefficients_line_blade -> values[4], coefficients_line_blade -> values[5]);
          }
          
          double err_angle = acos(now_vec.dot(pre_blade_vec[idx_pre] * vec_sign) / (now_vec.norm() * (vec_sign * pre_blade_vec[idx_pre]).norm()));

          if(abs(err_angle) > DEG2RAD(blade_dir_err_)) // if illegal line, update key msg
          {
            ptcloud_key_.blade_legal = false;
            // calculate dist and vec from vehicle to blade
            Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]) - local_posi_;
            Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_blade[3], coeff_line_blade[4], coeff_line_blade[5]);
            Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
            Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);

            double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
            if(abs(tmp_err_angle) > M_PI / 2){
              tmp_cross_cross = -tmp_cross_cross;
            }

            double blade_dist = tmp_cross.norm() / tmp_vec_ln.norm();
            Eigen::Vector3d vert_vec = tmp_cross_cross / tmp_cross_cross.norm();

            Eigen::Vector3d foot_posi = local_posi_ + vert_vec * blade_dist;

            ROS_INFO("dist(line angle illegal): %.2f", blade_dist);

            ptcloud_key_.blade_x = coeff_line_blade[0];
            ptcloud_key_.blade_y = coeff_line_blade[1];
            ptcloud_key_.blade_z = coeff_line_blade[2];
            ptcloud_key_.blade_vx = coeff_line_blade[3];
            ptcloud_key_.blade_vy = coeff_line_blade[4];
            ptcloud_key_.blade_vz = coeff_line_blade[5];
            ptcloud_key_.blade_dist = blade_dist;
            ptcloud_key_.blade_nx = vert_vec(0);
            ptcloud_key_.blade_ny = vert_vec(1);
            ptcloud_key_.blade_nz = vert_vec(2);
            ptcloud_key_.blade_fx = foot_posi(0);
            ptcloud_key_.blade_fy = foot_posi(1);
            ptcloud_key_.blade_fz = foot_posi(2);
            ptcloud_key_.blade_cx = foot_posi(0);
            ptcloud_key_.blade_cy = foot_posi(1);
            ptcloud_key_.blade_cz = foot_posi(2);
          }
          else // if legal line, add avg, update key msg
          {
            Eigen::Vector3d hub_posi = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
            Eigen::Vector3d hub_nor_vec = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
            Eigen::Vector3d hub_front_posi = hub_posi - hub_nor_vec * track_dist_;
            double dist_to_hub_front = (local_posi_ - hub_front_posi).norm();
            if(dist_to_hub_front > (fan_blade_length_ * 1 / 3))
            {
              if(coeff_line_blade.getNum() > blade_window_size_far_)
                coeff_line_blade.setNum(blade_window_size_far_);
            }
            else
            {
              if(coeff_line_blade.getNum() > blade_window_size_near_)
                coeff_line_blade.setNum(blade_window_size_near_);
            }

            coeff_line_blade.addValAvg(coefficients_line_blade -> values);

            // calculate dist and vec from vehicle to blade
            Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]) - local_posi_;
            Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_blade[3], coeff_line_blade[4], coeff_line_blade[5]);
            Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
            Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);

            double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
            if(abs(tmp_err_angle) > M_PI / 2){
              tmp_cross_cross = -tmp_cross_cross;
            }

            double blade_dist = tmp_cross.norm() / tmp_vec_ln.norm();
            Eigen::Vector3d vert_vec = tmp_cross_cross / tmp_cross_cross.norm();

            Eigen::Vector3d foot_posi = local_posi_ + vert_vec * blade_dist;

            correct_foot_posi_ = getCorrectFootPoint(foot_posi, search_blade_radius_);

            ROS_INFO("dist(line angle legal): %.2f", blade_dist);

            // update key msg
            ptcloud_key_.blade_legal = true;

            ptcloud_key_.blade_x = coeff_line_blade[0];
            ptcloud_key_.blade_y = coeff_line_blade[1];
            ptcloud_key_.blade_z = coeff_line_blade[2];
            ptcloud_key_.blade_vx = coeff_line_blade[3];
            ptcloud_key_.blade_vy = coeff_line_blade[4];
            ptcloud_key_.blade_vz = coeff_line_blade[5];
            ptcloud_key_.blade_dist = blade_dist;
            ptcloud_key_.blade_nx = vert_vec(0);
            ptcloud_key_.blade_ny = vert_vec(1);
            ptcloud_key_.blade_nz = vert_vec(2);
            ptcloud_key_.blade_fx = foot_posi(0);
            ptcloud_key_.blade_fy = foot_posi(1);
            ptcloud_key_.blade_fz = foot_posi(2);
            ptcloud_key_.blade_cx = correct_foot_posi_(0);
            ptcloud_key_.blade_cy = correct_foot_posi_(1);
            ptcloud_key_.blade_cz = correct_foot_posi_(2);
          }
        }
      }
      
      Eigen::Vector3d hub_posi = Eigen::Vector3d(coeff_hub[0], coeff_hub[1], coeff_hub[2]);
      Eigen::Vector3d hub_nor_vec = Eigen::Vector3d(coeff_plane[0], coeff_plane[1], coeff_plane[2]);
      Eigen::Vector3d hub_front_posi = hub_posi - hub_nor_vec * track_dist_;
      double dist_to_hub_front = (local_posi_ - hub_front_posi).norm();
      
      if(dist_to_hub_front > fan_blade_length_ * 1 / 3 && !have_send_origin_blade_dir[idx_have])
      {
        ptcloud_key_.blade_ox = coeff_line_blade[3];
        ptcloud_key_.blade_oy = coeff_line_blade[4];
        ptcloud_key_.blade_oz = coeff_line_blade[5];
        have_send_origin_blade_dir[idx_have] = true;
      }

      if(dist_to_hub_front > fan_blade_length_ * 2 / 3)
      {

        // ***************************************************
        
        ROS_INFO("send tip check request(normal)");

        insp_msgs::BladeTipCheck blade_tip_check_srv;

        blade_tip_check_srv.request.start_pt_x = ptcloud_key_.blade_fx;
        blade_tip_check_srv.request.start_pt_y = ptcloud_key_.blade_fy;
        blade_tip_check_srv.request.start_pt_z = ptcloud_key_.blade_fz;


        blade_tip_check_srv.request.vec_x = ptcloud_key_.blade_vx;
        blade_tip_check_srv.request.vec_y = ptcloud_key_.blade_vy;
        blade_tip_check_srv.request.vec_z = ptcloud_key_.blade_vz;

        blade_tip_check_srv.request.radius = 1.0;
        blade_tip_check_srv.request.max_empty_depth = 5.0;

        if(blade_tip_check_client_.call(blade_tip_check_srv) && blade_tip_check_srv.response.success)
        {
          if(blade_tip_check_srv.response.hit_points_num < thold_tip_check_hit_pt_)
          {
            num_fan_tip_check_hit_pt_none_ ++;
            ROS_INFO("arrive blade tip once! ");
          }
          else
          {
            ptcloud_key_.tip_x = blade_tip_check_srv.response.end_pt_x;
            ptcloud_key_.tip_y = blade_tip_check_srv.response.end_pt_y;
            ptcloud_key_.tip_z = blade_tip_check_srv.response.end_pt_z;
            num_fan_tip_check_hit_pt_none_ = 0;
          }
          if(num_fan_tip_check_hit_pt_none_ >= thold_tip_check_sum_)
          {
            ptcloud_key_.tip_arrive = true;
            ROS_INFO("arrive blade tip confirm, enter next stage mode!");
            num_fan_tip_check_hit_pt_none_ = 0;
          }
        }

        // ***************************************************
      }

      // update rviz
      arrow_dir_beg_ = Eigen::Vector3d(coeff_line_blade[0], coeff_line_blade[1], coeff_line_blade[2]);
      arrow_dir_end_ = Eigen::Vector3d(coeff_line_blade[0] + coeff_line_blade[3] * track_dist_, 
                                       coeff_line_blade[1] + coeff_line_blade[4] * track_dist_, 
                                       coeff_line_blade[2] + coeff_line_blade[5] * track_dist_);

      arrow_nor_beg_ = local_posi_;
      arrow_nor_end_ = Eigen::Vector3d(ptcloud_key_.blade_fx, ptcloud_key_.blade_fy, ptcloud_key_.blade_fz);
      
      arrow_vtc_beg_ = arrow_nor_end_;    
      arrow_vtc_end_ = Eigen::Vector3d(arrow_vtc_beg_(0) - ptcloud_key_.hub_vx * track_dist_, 
                                       arrow_vtc_beg_(1) - ptcloud_key_.hub_vy * track_dist_, 
                                       arrow_vtc_beg_(2) - ptcloud_key_.hub_vz * track_dist_);


      // pub key msg
      ptcloud_key_.header.stamp = ros::Time::now();
      ptcloud_key_publisher_.publish(ptcloud_key_);

      ptcloud_key_.tip_arrive = false;
    }
    else
    {
      ptcloud_key_.adj_to_loc = false;

      arrow_dir_beg_ = Eigen::Vector3d::Zero();
      arrow_dir_end_ = Eigen::Vector3d::Zero();
      arrow_nor_beg_ = Eigen::Vector3d::Zero();
      arrow_nor_end_ = Eigen::Vector3d::Zero();
      arrow_vtc_beg_ = Eigen::Vector3d::Zero();
      arrow_vtc_end_ = Eigen::Vector3d::Zero();

    }
    
  }
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------
  else if(stage_mode_.stage_mode == insp_msgs::StageMode::STAGE_FNS_MODE_DES)
  {
    // if(!if_save_pcd_)
    // {
    //   if_save_pcd_ = true;
    //   std::string save_name = pcd_save_name_ + std::to_string(ros::Time::now().sec);
    //   pcl::io::savePCDFile(save_name, *cloud_blade_sum, true);
    // }

    if(!have_clear_coeff_tower[1])
    {
      coeff_line_tower.clearVal();
      have_clear_coeff_tower[1] = true;
    }

    bool have_ln = ExtractTowerLineObjects();
    
    if(!have_ln) // if no detect line
    {
      ptcloud_key_.tower_legal = false;
    }
    else  // if detect line
    {
      Eigen::Vector3d dv_ori = Eigen::Vector3d(0, 0, -1);
      Eigen::Vector3d dv_now = Eigen::Vector3d( coefficients_line_tower -> values[3], 
                                                coefficients_line_tower -> values[4], 
                                                coefficients_line_tower -> values[5]);

      double aa_angle = acos(dv_ori.dot(dv_now) / (dv_ori.norm() * dv_now.norm()));
      if(abs(aa_angle) > M_PI / 2)
      {
        coefficients_line_tower -> values[3] = -coefficients_line_tower -> values[3];
        coefficients_line_tower -> values[4] = -coefficients_line_tower -> values[4];
        coefficients_line_tower -> values[5] = -coefficients_line_tower -> values[5];
        dv_now = -dv_now;
      }

      aa_angle = acos(dv_ori.dot(dv_now) / (dv_ori.norm() * dv_now.norm()));

      if(abs(aa_angle) > DEG2RAD(20.0)) // if line cylinder, update key msg
      {
        ptcloud_key_.tower_legal = false;
      }
      else // if legal line, add avg, update key msg
      {
        coeff_line_tower.addValAvg(coefficients_line_tower -> values);

        Eigen::Vector3d tmp_vec_ab = Eigen::Vector3d(coeff_line_tower[0], coeff_line_tower[1], coeff_line_tower[2]) - local_posi_;
        Eigen::Vector3d tmp_vec_ln = Eigen::Vector3d(coeff_line_tower[3], coeff_line_tower[4], coeff_line_tower[5]);
        Eigen::Vector3d tmp_cross = tmp_vec_ab.cross(tmp_vec_ln);
        Eigen::Vector3d tmp_cross_cross = tmp_cross.cross(tmp_vec_ln);

        double tmp_err_angle = acos(tmp_cross_cross.dot(tmp_vec_ab) / (tmp_cross_cross.norm() * tmp_vec_ab.norm()));
        if(abs(tmp_err_angle) > M_PI / 2){
          tmp_cross_cross = -tmp_cross_cross;
        }

        double tower_dist = tmp_cross.norm() / tmp_vec_ln.norm();
        Eigen::Vector3d tower_vec = tmp_cross_cross / tmp_cross_cross.norm();
        
        ROS_INFO("dist, vec: %.2f, (%.2f, %.2f, %.2f)", tower_dist, tower_vec(0), tower_vec(1), tower_vec(2));

        ptcloud_key_.tower_legal = true;

        ptcloud_key_.tower_x = coeff_line_tower[0];
        ptcloud_key_.tower_y = coeff_line_tower[1];
        ptcloud_key_.tower_z = coeff_line_tower[2];
        ptcloud_key_.tower_vx = coeff_line_tower[3];
        ptcloud_key_.tower_vy = coeff_line_tower[4];
        ptcloud_key_.tower_vz = coeff_line_tower[5];
        ptcloud_key_.tower_dist = tower_dist;
        ptcloud_key_.tower_nx = tower_vec(0);
        ptcloud_key_.tower_ny = tower_vec(1);
        ptcloud_key_.tower_nz = tower_vec(2);

      }

    }

    // update rviz line
    arrow_dir_beg_ = Eigen::Vector3d(coeff_line_tower[0], coeff_line_tower[1], coeff_line_tower[2]);
    arrow_dir_end_ = Eigen::Vector3d(coeff_line_tower[0] + coeff_line_tower[3] * detect_dist_, 
                                                    coeff_line_tower[1] + coeff_line_tower[4] * detect_dist_, 
                                                    coeff_line_tower[2] + coeff_line_tower[5] * detect_dist_);

    arrow_nor_beg_ = local_posi_;
    arrow_nor_end_ = Eigen::Vector3d(local_posi_(0) + ptcloud_key_.tower_nx * detect_dist_, 
                                                    local_posi_(1) + ptcloud_key_.tower_ny * detect_dist_, 
                                                    local_posi_(2) + ptcloud_key_.tower_nz * detect_dist_);

    arrow_vtc_beg_ = Eigen::Vector3d::Zero();
    arrow_vtc_end_ = Eigen::Vector3d::Zero();

    // pub key
    ptcloud_key_.header.stamp = ros::Time::now();
    ptcloud_key_publisher_.publish(ptcloud_key_);


  }

  visPtr_ -> visualize_arrow(arrow_dir_beg_, arrow_dir_end_, arrow_width_, str_topic_dir_, visualization::red);
  visPtr_ -> visualize_arrow(arrow_nor_beg_, arrow_nor_end_, arrow_width_, str_topic_nor_, visualization::blue);
  visPtr_ -> visualize_arrow(arrow_vtc_beg_, arrow_vtc_end_, arrow_width_, str_topic_vtc_, visualization::green);
  Eigen::Vector3d ball_center = Eigen::Vector3d(ptcloud_key_.blade_fx, ptcloud_key_.blade_fy, ptcloud_key_.blade_fz);
  visPtr_ -> visualize_a_ball(ball_center, search_blade_radius_, str_topic_sca_, visualization::yellow, 0.5);

  double tower_x = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vx / ptcloud_key_.tower_vz + ptcloud_key_.tower_x;
  double tower_y = (fan_tower_height_ * tower_top_adjust_k_ - ptcloud_key_.tower_z) * ptcloud_key_.tower_vy / ptcloud_key_.tower_vz + ptcloud_key_.tower_y;
  tower_x += coeff_plane[0] * fan_tower_radius_;
  tower_y += coeff_plane[1] * fan_tower_radius_;

  double vis_tower_height_ = (coeff_hub.getNum() > 0)? (coeff_hub[2]) : (fan_tower_height_);
  visPtr_ -> visualize_cylinder(Eigen::Vector3d(tower_x, tower_y, fan_tower_height_ / 2), 
  Eigen::Quaterniond(1, 0, 0, 0), 
  Eigen::Vector3d(tower_remove_radius_ * 2, tower_remove_radius_ * 2, vis_tower_height_), 
  str_topic_tow_, visualization::green, 0.5);
  
  if(stage_mode_.stage_mode > insp_msgs::StageMode::STAGE_TRK_1)
    visPtr_ -> visualize_arrow(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), arrow_width_, str_topic_pln_, visualization::green);

}

void SensingNode::nodeStatusPublisherCallback(const ros::TimerEvent& event)
{
  std_msgs::Int32 msg;
  msg.data = 8;
  node_status_publisher_.publish(msg);
}


bool SensingNode::trueTipPointCallback(insp_msgs::BladeTipQuery::Request& request, insp_msgs::BladeTipQuery::Response& response)
{
  auto tic = std::chrono::steady_clock::now();

  if(cloud -> size() < 3)
  {
    response.true_tip.x = last_crt_posi_.x();
    response.true_tip.y = last_crt_posi_.y();
    response.true_tip.z = last_crt_posi_.z();
    ROS_INFO("enter < 3");
  }
  else
  {
    Eigen::Vector3d tip_fake = Eigen::Vector3d(request.fake_tip.x, request.fake_tip.y, request.fake_tip.z);
    Eigen::Vector3d tip_true = getNearestPointsMean(tip_fake, search_tip_number_);
    response.true_tip.x = tip_true.x();
    response.true_tip.y = tip_true.y();
    response.true_tip.z = tip_true.z();
    ROS_INFO("enter > 3 and search nearest points");
  }
  auto toc = std::chrono::steady_clock::now();
  std::cout<<"[detection]blade vector detect time cost in one service response: "<<(toc-tic).count() * 1e-6<<"ms"<<std::endl;

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensing_node");
  
  ros::NodeHandle nh;

  SensingNode sensing_node(nh);

  ROS_INFO_STREAM("sensing_node Node is OK !");

  ros::spin();

  return 0;

}