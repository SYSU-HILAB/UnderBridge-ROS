#ifndef __SENSING_NODE_H__
#define __SENSING_NODE_H__


#include <ros/ros.h>
#include <Eigen/Eigen>

/* snesor_msgs */
#include <sensor_msgs/PointCloud2.h>
/* geometry_msgs */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
/* wind_turbine_insp msgs*/
#include <insp_msgs/StageMode.h>
#include <insp_msgs/BladeVectorDetect.h>
#include <insp_msgs/BladeTipCheck.h>
#include <insp_msgs/PtCloudKey.h>
#include <insp_msgs/BladeTipQuery.h>

#include <nav_msgs/Odometry.h>

#include <insp_msgs/msg_tran.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <math.h>


#include <visualization/visualization.hpp>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB PCT;


namespace wind_turbine_insp{

  class AvgValNd
  {
    private:
    
      int dim;
      std::vector<float> val;
      int num;
    
    public:

      AvgValNd() {};

      AvgValNd(int dim_)
      {
        dim = dim_;
        val.resize(dim);
        std::fill(val.begin(), val.end(), 0.0);
        num = 0;
      }

      ~AvgValNd() {};

      inline void init(int dim_)
      {
        dim = dim_;
        val.resize(dim);
        std::fill(val.begin(), val.end(), 0.0);
        num = 0;
      }

      inline void addValAvg(std::vector<float> new_val)
      {        
        for(int i = 0; i < dim; i++)
        {
          if(std::isnan(new_val[i]))
            continue;
          else
            val[i] = (val[i] * num + new_val[i]) / (num + 1);
        }

        num++;
        
        return;
      }

      inline double operator[] (int i){
        return val[i];
      }

      inline int getNum(){
        return num;
      }

      inline void clearVal(){
        std::fill(val.begin(), val.end(), 0.0);
        num = 0;
      }

      inline void setNum(int n){
        num = n;
      }

      inline void plusOneVal(int idx, double plus){
        val[idx] += plus;
      }
  };




  class SensingNode{

    public:

      SensingNode(ros::NodeHandle nh);
      ~SensingNode();

      bool initVariable();

      bool initServiceClient();
      bool initSubscriber();
      bool initTimer();
      bool initServiceServer();
      bool initPublisher();

    protected:

      ros::Subscriber stage_mode_subscriber_;
      ros::Subscriber ptcloud_subscriber_;
      ros::Subscriber local_odom_subscriber_;

      ros::Publisher ptcloud_feature_publisher_;
      ros::Publisher ptcloud_remove_publisher_;
      ros::Publisher ptcloud_sum_publisher_;
      ros::Publisher ptcloud_key_publisher_;

      ros::Publisher node_status_publisher_;

      ros::ServiceClient blade_vector_detect_client_;
      ros::ServiceClient blade_tip_check_client_;

      ros::ServiceServer true_blade_tip_server_;

      ros::Timer ptcloud_proc_timer_;
      ros::Timer node_status_timer_;

    protected:

      void getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg);
      void getPtcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
      void getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void ptcloudProcCallback(const ros::TimerEvent& event);
      void nodeStatusPublisherCallback(const ros::TimerEvent& event);

      bool trueTipPointCallback(insp_msgs::BladeTipQuery::Request& request, insp_msgs::BladeTipQuery::Response& response);

      void VoxelFilter();
      void RemoveTowerPtcloud();
      void RemoveOutlier();
      void EstimatePointNormals();


      bool ExtractPlaneObjects();
      bool ExtractBladeLineObjects();
      bool ExtractTowerLineObjects();

      Eigen::Vector3d getCorrectFootPoint(Eigen::Vector3d fpt, double radius);
      Eigen::Vector3d getNearestPointsMean(Eigen::Vector3d pt, int n);

    private:

      ros::NodeHandle                              nh_;

      double                                       rate_ptcloud_proc_;

      // fan physical param
      double                                       fan_tower_height_;
      double                                       fan_blade_length_;
      double                                       fan_neck_depth_;
      double                                       fan_tower_radius_;
      double                                       fan_blade_pitch_;

      int                                          start_blade_;
      
      // dist
      double                                       track_dist_;
      double                                       detect_dist_;

      double                                       voxel_leaf_size_;

      double                                       tower_remove_radius_;
      double                                       tower_top_adjust_k_;

      double                                       blade_dir_err_;
      int                                          blade_window_size_near_;
      int                                          blade_window_size_far_;
      int                                          plane_window_size_;

      int                                          count_angle_detect_;

      double                                       weight_hub_posi_loc_;

      double                                       arrow_width_;

      bool                                         enable_hub_vec_z_;

      int                                          thold_tip_check_hit_pt_;
      int                                          thold_tip_check_sum_;

      int success_count_;


      AvgValNd coeff_line_tower;
      // hub position pt, (pt.x, pt.y, pt.z)
      AvgValNd coeff_hub;
      // Ax + By + Cz + D = 0, (A, B, C, D)
      AvgValNd coeff_plane;
      // pt in line, vec of line, (pt.x, pt,y, pt.z, vec.x, vec.y, vec.z)
      AvgValNd coeff_line_blade;


      // visualization
      std::shared_ptr<visualization::Visualization> visPtr_;

      insp_msgs::PtCloudKey ptcloud_key_;

      insp_msgs::StageMode stage_mode_;



      ros::Time last_detect_trigger_time;

      bool have_save_angle_vec[6];
      bool have_clear_coeff_blade[6];
      int have_send_first_legal_key[6];
      bool have_send_origin_blade_dir[6];
      bool have_clear_coeff_tower[2];
      bool have_enter_hub_esti_;

      Eigen::Vector3d pre_blade_vec[3];

      int              num_fan_tip_check_hit_pt_none_;
      bool             if_fan_tip_arrive_;


      std::string str_topic_dir_;
      std::string str_topic_nor_;
      std::string str_topic_vtc_;
      std::string str_topic_pln_;
      std::string str_topic_tow_;
      std::string str_topic_sca_;


      // 点云每个点的法向量提取
      pcl::NormalEstimation<PCT, pcl::Normal> ne;
      // 从每个点的法向量提取物体
      pcl::SACSegmentationFromNormals<PCT, pcl::Normal> seg;
      // 点云抽取
      pcl::ExtractIndices<PCT> extract;
      // 抽取切片
      pcl::ExtractIndices<pcl::Normal> extract_normals;
      // 搜索树
      pcl::search::KdTree<PCT>::Ptr tree;

      bool if_update_hub_xyz_in_loc_;

      // ros原始点云
      sensor_msgs::PointCloud2 cloud_ros;

      // ros转换的pcl点云
      pcl::PointCloud<PCT>::Ptr cloud;
      // 点云每个点的法向量
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
      // 圆柱和平面参数
      pcl::ModelCoefficients::Ptr coefficients_plane;
      pcl::ModelCoefficients::Ptr coefficients_line_blade;
      pcl::ModelCoefficients::Ptr coefficients_line_tower;

      // 切片圆柱和平面点云
      pcl::PointIndices::Ptr inliers_plane;
      pcl::PointIndices::Ptr inliers_line_blade;
      pcl::PointIndices::Ptr inliers_line_tower;

      // 圆柱和平面点云
      pcl::PointCloud<PCT>::Ptr cloud_plane;
      pcl::PointCloud<PCT>::Ptr cloud_line_blade;
      pcl::PointCloud<PCT>::Ptr cloud_line_tower;

      pcl::PointCloud<PCT>::Ptr cloud_blade_sum;
      
      bool if_save_pcd_;
      std::string pcd_save_name_;

      pcl::KdTreeFLANN<PCT> kdtree;

      Eigen::Vector3d local_posi_;

      // visualization arrow
      Eigen::Vector3d arrow_dir_beg_;
      Eigen::Vector3d arrow_dir_end_;
      Eigen::Vector3d arrow_nor_beg_;
      Eigen::Vector3d arrow_nor_end_;
      Eigen::Vector3d arrow_vtc_beg_;
      Eigen::Vector3d arrow_vtc_end_;

      int send_blade_detect_count_;
      int thold_change_tower_height_;
      
      double change_k_;
      bool change_up_;

      double detect_circle_radius_;

      Eigen::Vector3d correct_foot_posi_;
      double search_blade_radius_;

      Eigen::Vector3d last_crt_posi_;

      int search_tip_number_;
  };

}


#endif