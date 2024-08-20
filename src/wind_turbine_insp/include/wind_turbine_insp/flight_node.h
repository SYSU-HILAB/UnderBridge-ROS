#ifndef __FLIGHT_NODE_H__
#define __FLIGHT_NODE_H__


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>

/* std_msgs */
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
/* geometry_msgs */
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
/* sensor_msgs */
#include <sensor_msgs/PointCloud2.h>
/* nav_msgs */
#include <nav_msgs/Odometry.h>
/* wind_turbine_insp msgs*/
#include <insp_msgs/DebugFlight.h>
#include <insp_msgs/StageMode.h>
#include <insp_msgs/VeloCmd.h>
#include <insp_msgs/PtCloudKey.h>
#include <insp_msgs/TrackIndex.h>
#include <insp_msgs/GimbalSet.h>
#include <insp_msgs/OccupyCheck.h>
#include <insp_msgs/BladeSideState.h>
#include <insp_msgs/CameraFocusPoint.h>

/* wind_turbine_insp srvs*/
#include <insp_msgs/SetStageMode.h>
#include <insp_msgs/FlightTaskControl.h>
#include <insp_msgs/EmergencyTerminate.h>
#include <insp_msgs/GimbalCompensate.h>
#include <insp_msgs/FootPointCorrect.h>
#include <insp_msgs/BladeTipQuery.h>







#include <px4_ctrl/pid_ctrl.h>
#include <insp_msgs/msg_tran.h>
#include <traj_utils/line_traj.h>
#include <traj_utils/circle_traj.h>
#include <visualization/visualization.hpp>
#include <chrono>


namespace wind_turbine_insp{

  #define PI (double)3.14159265358979323846
  #define DEG2RAD(DEG) ((DEG) * (PI) / (180.0))
  #define RAD2DEG(RAD) ((RAD) * (180.0) / (PI))

  #define MIN_FOCALLEN_REAL (double)(6.83)
  #define MIN_FOCALLEN_EQU  (double)(31.7)
  #define MAX_FOV_HOR_HALF (double)(33.3)

  #define CMOS_HALF_WIDTH (double)(MIN_FOCALLEN_REAL * tan(DEG2RAD(MAX_FOV_HOR_HALF)))
  #define CMOS_HALF_HEIGHT (double)(CMOS_HALF_WIDTH * 3 / 4)
  
  #define FOCALLEN_REAL_TO_EQU (double)(MIN_FOCALLEN_EQU / MIN_FOCALLEN_REAL)
  #define ZOOM_TO_FOCALLEN_EQU (double)(23.71)
  #define ZOOM_TO_FOCALLEN_REAL (double)(ZOOM_TO_FOCALLEN_EQU / FOCALLEN_REAL_TO_EQU)

  #define GET_TARGET_VIEW(DIST, ZOOM) (2.0 * (DIST * CMOS_HALF_HEIGHT) / (ZOOM * ZOOM_TO_FOCALLEN_REAL))

  class FlightNode{

    public:

      FlightNode(ros::NodeHandle nh);
      ~FlightNode();

      bool initVariable();

      bool initServiceClient();
      bool initSubscriber();
      bool initTimer();
      bool initServiceServer();
      bool initPublisher();



    protected:

      /* Service Client */

      ros::ServiceClient set_stage_mode_client_;

      ros::ServiceClient flight_task_control_client_;


      ros::ServiceClient occupied_check_client_;
      ros::ServiceClient emergency_terminate_client_;
      ros::ServiceClient correct_foot_point_client_;
      ros::ServiceClient true_tip_point_client_;

      /* Timer */
      ros::Timer update_stage_mode_timer_;
      ros::Timer servo_control_timer_;
      ros::Timer sample_traj_point_timer_;
      ros::Timer set_gimbal_position_timer_;
      ros::Timer set_camera_zoom_timer_;
      ros::Timer send_msg_to_vision_timer_;

      ros::Timer node_status_timer_;

      /* Subscriber */
      ros::Subscriber stage_mode_subscriber_;

      ros::Subscriber local_odom_subscriber_;

      ros::Subscriber ptcloud_key_subscriber_;
      ros::Subscriber traj_velo_subscriber_;
      ros::Subscriber gimbal_compensate_subscriber_;

      /* Publisher */
      ros::Publisher set_velo_publisher_;

      ros::Publisher wps_servo_publisher_;
      ros::Publisher wps_traj_publisher_;

      ros::Publisher send_force_hover_signal_publisher_;

      ros::Publisher set_gimbal_publisher_;

      ros::Publisher camera_set_zoom_para_publisher_;
      ros::Publisher camera_set_focus_para_publisher_;
      ros::Publisher camera_record_video_publisher_;

      ros::Publisher camera_shoot_photo_publisher_;



      ros::Publisher blade_distance_publisher_;

      ros::Publisher blade_target_point_publisher_;

      ros::Publisher blade_width_publisher_;

      ros::Publisher blade_side_state_publisher_;

      ros::Publisher log_state_publisher_;

      ros::Publisher node_status_publisher_;

      ros::Publisher track_index_publisher_;

      ros::Publisher debug_flight_publisher_;

    protected:

      /* Timer Callback Function */
      void updateStageModeCallback(const ros::TimerEvent& event);
      void servoControlCallback(const ros::TimerEvent& event);
      void sampleTrajPointCallback(const ros::TimerEvent& event);
      void setGimbalPositionCallback(const ros::TimerEvent& event);
      void setCameraZoomCallback(const ros::TimerEvent& event);
      void sendMsgToVisionCallback(const ros::TimerEvent& event);

      void nodeStatusPublisherCallback(const ros::TimerEvent& event);

      /* Subscriber Callback Function */
      void getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg);
      
      void getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void getPtcloudKeyCallback(const insp_msgs::PtCloudKey::ConstPtr& msg);

      void trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg);

      void gimbalCompensateCallback(const insp_msgs::GimbalCompensate::ConstPtr& msg);

      /* Control function */
      void vehicleXYZCtrl();
      void futureOccupiedCheck();

      void showCtrlInfo();

      void ForceHoverFromUp(bool hover);

      double getYawErr(double cur, double& tgt, double k=0.6);
      void updateTargetYaw(Eigen::Vector3d vec_to_tgt);

      Eigen::Vector3d getCirclePoint3D(Eigen::Vector3d center, double radius, Eigen::Vector3d nor_vec, double t_cur, double omega);


      bool curBladeIsToRight();
      bool curBladeIsNormal(int idx_blade);
      bool curBladeIsToDown(int idx_blade);

      bool enoughSlowDown(double velo_thold);
      bool enoughYawAlign(double yaw_thold);

      Eigen::Vector3d getHrzRotAxisBladeTip();
      Eigen::Vector3d getVtcRotAxisBladeTip(int idx_blade);

      Eigen::Vector3d getHfRotAxisBladeHub();
      
      Eigen::Vector3d getUpRotAxisBladeHub();
      Eigen::Vector3d getDnRotAxisBladeHub(int idx_blade);

      Eigen::Vector3d getUpQuatRotAxisBladeTip();
      Eigen::Vector3d getUpQuatRotAxisBladeHub();

      Eigen::Vector3d getDnQuatRotAxisBladeTip(int idx_blade);
      Eigen::Vector3d getDnQuatRotAxisBladeHub(int idx_blade);

      void cameraRecordVideo(bool start);
      void cameraShootPhoto(bool start);

      double getShootInterval();

      double getBladeWidthHorizontal(double dist_to_root);
      double getBladeWidthVertical(double dist_to_root);
      double getBladeWidthInclined(double dist_to_root);

      bool vector3dIsZero(Eigen::Vector3d vec);
      bool vector3dIsTheSame(Eigen::Vector3d vec1, Eigen::Vector3d vec2);

      void updateLogState();
      void updateBladeDistance();
      void updateBladeTargetPoint(); 
      void updateTrackIndex();

      void emergencyTerminate();

    private:

      ros::NodeHandle                              nh_;

      int                                          track_mode_;
      int                                          single_side_;
      bool                                         enable_planning_;

      // fan physical param
      double                                       fan_tower_height_;
      double                                       fan_blade_length_;
      double                                       fan_blade_width_;

      // Timer rate 
      double                                       rate_sm_update_;
      double                                       rate_flight_ctrl_;
      double                                       rate_wps_sample_;
      double                                       rate_set_gim_posi_;
      double                                       rate_set_camera_zoom_;
      double                                       rate_msg_to_vision_;


      // flag
      bool                                         flag_if_takeoff_;
      bool                                         flag_if_land_;

      // time
      ros::Time                                    last_info_time_;
      ros::Time                                    loc_beg_time_;
      ros::Time                                    traj_begin_time_;

      // msg & srv
      insp_msgs::SetStageMode                      set_stage_mode_;

      insp_msgs::VeloCmd                           set_velo_;
      
      insp_msgs::PtCloudKey                        ptcloud_key_;
      
      insp_msgs::StageMode                         stage_mode_;

      geometry_msgs::PoseStamped                   wps_msg_;
      
      insp_msgs::FlightTaskControl                 flight_task_control;



      // yaw ctrl
      px4_ctrl::PIDctrl                            ctrl_yaw_;

      double                                       kp_yaw_;
      double                                       ki_yaw_;
      double                                       kd_yaw_;
      double                                       mv_yaw_;
      double                                       ma_yaw_;

      bool                                         yaw_ctrl_in_deg_;

      
      double                                       err_yaw_;
      double                                       tgt_yaw_;


      // telemetry
      Eigen::Vector3d                              local_posi_;
      Eigen::Vector3d                              local_euler_;
      Eigen::Vector3d                              local_velo_;

      // gimbal
      Eigen::Vector3d                              desire_gim_vec_;
      Eigen::Vector3d                              gim_cpst_vec_;
      insp_msgs::GimbalSet                         gimbal_set_vec_msg_;

      // last control value
      double                                       last_legal_yaw_;
      geometry_msgs::PoseStamped                   last_wps_msg_;

      Eigen::Vector3d                              last_dir_vec_vert_;
      Eigen::Vector3d                              last_dir_vec_track_;

      // dist
      double                                       track_dist_;
      double                                       detect_dist_;

      double                                       desire_dist_rise_;
      double                                       desire_dist_track_;

      double                                       loc_rotate_radius_;
      double                                       loc_rotate_period_;

      Eigen::Vector3d                              desire_posi_;
      Eigen::Vector2d                              desire_yaw_vec_;
      Eigen::Vector3d                              last_legal_yaw_vec_;
      Eigen::Vector3d                              last_legal_gim_vec_;

      // visualization
      std::shared_ptr<visualization::Visualization>    visPtr_;
    
      double                                       arrow_width_;

      bool                                         arrow_history_;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>  velo_history_;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>  gimb_history_;
      ros::Time                                    history_time_;

      std::string                                  str_topic_hst_velo_;
      std::string                                  str_topic_hst_gimb_;
      
      std::string                                  str_topic_wps_;
      std::string                                  str_topic_vel_;
      std::string                                  str_topic_yaw_;
      std::string                                  str_topic_gim_;
      std::string                                  str_topic_pts_;
      std::string                                  str_topic_ftr_;
      std::string                                  str_topic_cps_;
      std::string                                  str_topic_crt_;

      double                                       traj_pt_size_;

      // key position hub and tip
      Eigen::Vector3d                              hub_front_posi_;
      Eigen::Vector3d                              hub_back_posi_;

      Eigen::Vector3d                              tip_side1_posi_[3];
      Eigen::Vector3d                              tip_side3_posi_[3];
      Eigen::Vector3d                              tip_side4_posi_[3];
      Eigen::Vector3d                              tip_side2_posi_[3];

      Eigen::Vector3d                              tip_up_posi_[3];
      Eigen::Vector3d                              tip_down_posi_[3];

      Eigen::Vector3d                              root_side1_posi_[3];
      Eigen::Vector3d                              root_side3_posi_[3];
      Eigen::Vector3d                              root_side4_posi_[3];
      Eigen::Vector3d                              root_side2_posi_[3];

      Eigen::Vector3d                              root_up_posi_[3];
      Eigen::Vector3d                              root_down_posi_[3];

      Eigen::Vector3d                              tip_blade_posi_[3];
      Eigen::Vector3d                              root_blade_posi_[3];

      std::vector<Eigen::Vector3d>                 key_positions_;

      int                                          next_track_blade_;

      double                                       dist_to_hub_front_;

      // blade tip check
      bool                                         if_fan_tip_arrive_;
      bool                                         if_save_tip_front_posi_;

      double                                       thold_arrive_target_;
      double                                       thold_keep_yaw_dist_;

      double                                       thold_update_yaw_;

      double                                       thold_enough_vel_slow_;
      double                                       thold_enough_yaw_accu_;

      double                                       dist_enable_sample_track_wps_;
      double                                       deep_root_offset_;

      double                                       blade_angle_[3];

      double                                       zoom_factor_init_;
      double                                       camera_zoom_k_;
      double                                       camera_zoom_factor_;






      bool                                         if_emergency_save_tip_;

      bool                                         if_gimbal_end_;
      bool                                         if_gimbal_beg_;
      double                                       gimbal_traj_time_[3];

      double                                       hub_root_dist_[3];



      int                                          log_state_;
      
      // line traj
      std::shared_ptr<line_traj::LineTraj> LineTrajPtr_0_[3];
      std::shared_ptr<line_traj::LineTraj> LineTrajPtr_1_[3];
      std::shared_ptr<line_traj::LineTraj> LineTrajPtr_2_[3];
      std::shared_ptr<line_traj::LineTraj> LineTrajPtr_3_[3];
      std::shared_ptr<line_traj::LineTraj> LineTrajPtr_4_[3];

      std::vector<Eigen::Vector3d> pts_traj_0_[3];
      std::vector<Eigen::Vector3d> pts_traj_1_[3];
      std::vector<Eigen::Vector3d> pts_traj_2_[3];
      std::vector<Eigen::Vector3d> pts_traj_3_[3];
      std::vector<Eigen::Vector3d> pts_traj_4_[3];

      std::shared_ptr<circle_traj::CircleTraj> CircleTrajPtr_1_[3];
      std::shared_ptr<circle_traj::CircleTraj> CircleTrajPtr_2_[3];
      std::shared_ptr<circle_traj::CircleTraj> CircleTrajPtr_3_[3];
      std::shared_ptr<circle_traj::CircleTraj> CircleTrajPtr_4_[3];


      Eigen::Vector3d                              tower_pt_posi_;
      Eigen::Vector3d                              tower_dir_vec_;
      double                                       tower_vrt_dst_;
      Eigen::Vector3d                              tower_nor_vec_;
      Eigen::Vector3d                              hub_pt_posi_;
      Eigen::Vector3d                              hub_nor_vec_;
      Eigen::Vector3d                              blade_pt_posi_;
      Eigen::Vector3d                              blade_dir_vec_;
      double                                       blade_vrt_dst_;
      Eigen::Vector3d                              blade_nor_vec_;
      Eigen::Vector3d                              blade_fpt_posi_;
      Eigen::Vector3d                              blade_ori_vec_;
      Eigen::Vector3d                              blade_crt_posi_;

      double                                       traj_vel_track_;
      double                                       traj_vel_turn_;
      double                                       traj_vel_gimbal_;
      double                                       traj_delay_time_;
      double                                       traj_down_angle_;

      ros::Duration                                traj_delay_duration_;

      int                                          video_record_mode_;
      bool                                         if_include_tower_;

      bool                                         gimbal_reset_;
      bool                                         gimbal_keep_yaw_;

      bool                                         have_sample_first_wp_[3];

      double                                       shoot_photo_coverage_;
      ros::Time                                    last_shoot_time_;
  
      insp_msgs::TrackIndex                        track_index_msg_;

      bool                                         enable_emergency_terminal_;
      double                                       future_occupied_dist_;
      int                                          consequent_occupied_thold_;
      int                                          consequent_occupied_count_;
      bool                                         if_pre_occupied_;
      ros::Time                                    last_terminate_req_time_;

      double                                       blade_width_;

      int                                          zoom_wait_cnt_;
      int                                          focus_wait_cnt_;
      int                                          tip_focus_wait_cnt_;

      int                                          tip_focus_cnt_;
      int                                          tip_focus_thold_;
      double                                       tip_focus_gap_;

      double                                       gap_focus_;
      double                                       gap_zoom_;

      double                                       focus_time_root_;
      double                                       focus_time_tip_;
      double                                       zoom_time_root_;
      double                                       zoom_time_tip_;

      insp_msgs::DebugFlight                       df_msg_;

      bool                                         enable_ptcloud_correct_;
      Eigen::Vector3d                              correct_posi_;

      Eigen::Vector3d                              tip_posi_true_[3];
      double                                       tip_wait_dist_[3];

      int                                          front_zoom_cnt_;
  };
}

#endif