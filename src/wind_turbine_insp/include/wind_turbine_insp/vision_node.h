#ifndef __VISION_NODE_H__
#define __VISION_NODE_H__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>





/* std_msgs */
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
/* sensor_msgs */
#include <sensor_msgs/Image.h>
/* geometry_msgs */
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/QuaternionStamped.h>
/* nav_msgs */
#include <nav_msgs/Odometry.h>
/* wind_turbine_insp msgs */
#include <insp_msgs/StageMode.h>
#include <insp_msgs/msg_tran.h>
#include <insp_msgs/DebugVision.h>
#include <insp_msgs/BladeSideState.h>
#include <insp_msgs/PtCloudKey.h>
#include <insp_msgs/GimbalCompensate.h>


namespace wind_turbine_insp{

  #define PI (double)3.14159265358979323846
  #define DEG2RAD(DEG) ((DEG) * (PI) / (180.0))
  #define RAD2DEG(RAD) ((RAD) * (180.0) / (PI))

  class VisionNode{

    public:

      VisionNode(ros::NodeHandle nh);
      ~VisionNode();

      bool initVariable();

      bool initPublisher();
      bool initSubscriber();


      bool initTimer();




    protected:

      ros::Subscriber stage_mode_subscriber_;
      ros::Subscriber gimbal_image_subscriber_;
      ros::Subscriber camera_set_zoom_subscriber_;
      ros::Subscriber camera_record_video_subscriber_;
      ros::Subscriber camera_shoot_photo_subscriber_;

      ros::Subscriber local_gimbal_quat_subscriber_;
      ros::Subscriber local_odom_subscriber_;
      ros::Subscriber target_pt3d_subscriber_;
      ros::Subscriber blade_width_subscriber_;
      ros::Subscriber blade_side_state_subscriber_;
      ros::Subscriber get_log_state_subscriber_;
      ros::Subscriber ptcloud_key_subscriber_;

      ros::Publisher point_image_publisher_;
      ros::Publisher line_image_publisher_;
      ros::Publisher gimbal_compensate_publisher_;

      ros::Publisher debug_vision_publisher_;




      ros::Timer image_process_timer_;
      ros::Timer gimbal_adjust_timer_;
      ros::Timer set_camera_ev_timer_;
      ros::Timer camera_shoot_action_timer_;

    protected:

      void getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg);
      void getGimbalImageCallback(const sensor_msgs::Image::ConstPtr& msg);
      void getCameraZoomParaCallback(const std_msgs::Float32::ConstPtr &msg);
      void cameraRecordVideoCallback(const std_msgs::Bool::ConstPtr& msg);
      void cameraShootPhotoCallback(const std_msgs::Bool::ConstPtr& msg);
      void localGimbalQuatCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
      void getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void getBladeTatgetPointCallback(const geometry_msgs::Point::ConstPtr& msg);
      void getBladeWidthCallback(const std_msgs::Float32::ConstPtr& msg);
      void getBladeSideStateCallback(const insp_msgs::BladeSideState::ConstPtr& msg);
      void getLogStateCallback(const std_msgs::Int32::ConstPtr& msg);
      void getPtCloudKeyCallback(const insp_msgs::PtCloudKey::ConstPtr& msg);

      void imageProcessCallback(const ros::TimerEvent& event);
      void gimbalAdjustCallback(const ros::TimerEvent& event);
      void setCameraEvCallback(const ros::TimerEvent &event);
      void cameraShootActionCallback(const ros::TimerEvent &event);

      void imageDigitalZoom(cv::Mat image, cv::Mat& image_crop, double focal_len_dst);
      void imageDrawMessage(cv::Mat image_crop, cv::Mat& image_draw);

      void projectTargetPoint();

      void imageDetectLine(cv::Mat image, std::vector<cv::Vec4f> &plines, double canny_min=40, double canny_max=120, 
                           int thold=30, double min_line_len=60, double max_line_gap=20, int max_line_num=10);
      void selectBladeLine();
      void imageDrawLines(cv::Mat image_resize, cv::Mat& image_draw);

      std::string D2S(double val, int digits);


    private:

      ros::NodeHandle              nh_;

      double                       photo_intr_fx_;
      double                       photo_intr_fy_;
      double                       photo_intr_cx_;
      double                       photo_intr_cy_;

      double                       video_intr_fx_;
      double                       video_intr_fy_;
      double                       video_intr_cx_;
      double                       video_intr_cy_;

      double                       rate_image_proc_;
      double                       rate_adjust_gim_;
      double                       rate_set_camera_ev_;
      double                       rate_shoot_interval_;

      int                          video_record_mode_;
      int                          ev_mode_;

      int                          image_width_;
      int                          image_height_;

      Eigen::Matrix3d              intr_;
      Eigen::MatrixXd              extr_;

      int                          set_ev_;

      // callback update value

      insp_msgs::StageMode         stage_mode_;

      Eigen::Vector3d              local_posi_;

      double                       camera_zoom_factor_;

      Eigen::Quaterniond           local_gimbal_quat_;

      int                          log_state_;

      Eigen::Vector3d              target_pt3d_;
      Eigen::Vector2i              target_pt2d_;

      int                          area_radius_ori_;
      int                          area_radius_;
      double                       area_mean_max_;
      double                       area_mean_min_;
      double                       mean_offset_;
      int                          adjust_k_;

      cv::Mat                      image_zoom_;
      cv::Mat                      image_draw_point_;
      cv::Mat                      image_resize_;
      cv::Mat                      image_draw_line_;

      bool                         target_point_legal_;

      double                       target_area_mean_;
      double                       target_area_stddev_;

      int                          area_status_;

      ros::Time                    last_get_image_time_;

      double                       msg_text_scale_;
      int                          msg_text_thick_;

      double                       line_detect_err_;

      insp_msgs::BladeSideState    blade_side_state_;

      int                          set_adj_limit_;

      bool                         enable_line_;

      std::vector<cv::Vec4f>       plines_;
      std::vector<cv::Vec4f>       plines_theta_;
      std::vector<cv::Vec4f>       plines_edge_;
      std::vector<cv::Vec4f>       clines_;
      
      double                       canny_min_;
      double                       canny_max_;
      int                          hough_thold_;
      double                       min_line_len_;
      double                       max_line_gap_;
      int                          max_line_num_;

      double                       blade_angle_;

      Eigen::Vector3d              x_base_vec_;
      Eigen::Vector3d              y_base_vec_;
      Eigen::Vector3d              hub_nor_vec_;

      double                       compensate_k_;

      insp_msgs::DebugVision       dv_msg_;

      double                       focus_len_ori_;

      cv::Mat                      adjust_mat_;

      bool                         status_recording;
      bool                         status_shooting_;

      int                          last_adj_val_;

      int                          save_idx_;

  };


}

#endif