#ifndef __MASTER_NODE_H__
#define __MASTER_NODE_H__


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <atomic>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
/* std_msgs */
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
/* sensor_msgs */
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
/* nav_msgs */
#include <nav_msgs/Odometry.h>
/* geometry_msgs */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <insp_msgs/msg_tran.h>
/* wind_turbine_insp msgs */
#include <insp_msgs/StageMode.h>
#include <insp_msgs/VeloCmd.h>
#include <insp_msgs/GimbalSet.h>
#include <insp_msgs/DebugMaster.h>
#include <insp_msgs/CameraFocusPoint.h>
/* wind_turbine_insp srvs */
#include <insp_msgs/SetStageMode.h>
#include <insp_msgs/FlightTaskControl.h>
#include <insp_msgs/EmergencyTerminate.h>
#include <visualization/visualization.hpp>
/* mavros_msgs */
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MountControl.h>









namespace wind_turbine_insp{

  #define PI (double)3.14159265358979323846
  #define R_EARTH (double)6378137.0

  #define DEG2RAD(DEG) ((DEG) * (PI) / (180.0))
  #define RAD2DEG(RAD) ((RAD) * (180.0) / (PI))

  class MasterNode{

    public:

      MasterNode(ros::NodeHandle nh);
      ~MasterNode();
      
      bool initVariable();

      bool initServiceServer();
      bool initServiceClient();

      bool initPublisher();
      bool initSubscriber();

      bool initTimer();


    protected:

      /* ---------------------- services server ---------------------- */
      ros::ServiceServer set_stage_mode_server_;
      ros::ServiceServer set_flight_task_server_;
      ros::ServiceServer emergency_terminate_server_;

      /* ---------------------- services clients ---------------------- */  
      ros::ServiceClient mavros_arming_client_;
      ros::ServiceClient mavros_set_mode_client_;
      











      /* ---------------------- publisher ---------------------- */
      ros::Publisher stage_mode_publisher_;
      ros::Publisher local_odom_publisher_;
      ros::Publisher local_gimbal_quat_publisher_;
      
      ros::Publisher debug_drone_odom_enu_pub;
      ros::Publisher debug_gimbal_odom_enu_pub;
      ros::Publisher debug_gimbal_odom_ned_pub;
      ros::Publisher debug_drone_odom_flu_pub;
      ros::Publisher debug_gimbal_odom_flu_pub;

      ros::Publisher mavros_set_target_velo_publisher_;
      ros::Publisher gimbal_set_posi_publisher_;
      ros::Publisher camera_set_zoom_publisher_;
      ros::Publisher emergency_terminate_publisher_;

      ros::Publisher node_status_publisher_;

      ros::Publisher debug_master_publisher_;

      /* ---------------------- subscriber ---------------------- */  

      ros::Subscriber mavros_state_subscriber_;
      ros::Subscriber mavros_local_odom_subscriber_;
      
      ros::Subscriber global_gimbal_euler_subscriber_;

      ros::Subscriber set_velo_subscriber_;

      ros::Subscriber gimbal_set_vec_subscriber_;

      ros::Subscriber camera_set_zoom_subscriber_;
      ros::Subscriber camera_set_focus_point_subscriber_;
      ros::Subscriber camera_record_video_subscriber_;

      ros::Subscriber camera_shoot_photo_subscriber_;



      /* ---------------------- Timer ---------------------- */
      ros::Timer stage_mode_publisher_timer_;
      ros::Timer routine_timer_;
      ros::Timer local_odom_publisher_timer_;
      ros::Timer camera_execute_timer_;
      ros::Timer node_status_timer_;
      ros::Timer task_execute_timer_;

    protected:

      /* ----------- ServiceServer callback function ----------- */
      bool setStageModeCallback(insp_msgs::SetStageMode::Request& request, insp_msgs::SetStageMode::Response& response);
      bool setFlightTaskCallback(insp_msgs::FlightTaskControl::Request& request, insp_msgs::FlightTaskControl::Response& response);
      bool emergencyTerminateCallback(insp_msgs::EmergencyTerminate::Request& request, insp_msgs::EmergencyTerminate::Response& response);
      
      /* ----------- Subscriber callback function ----------- */
      void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
      void mavrosLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      void getGlobalGimbalEulerCallback(const sensor_msgs::Imu::ConstPtr& msg);

      void setVeloCallback(const insp_msgs::VeloCmd::ConstPtr& msg);

      void gimbalSetVecCallback(const insp_msgs::GimbalSet::ConstPtr& msg);

      void cameraSetZoomCallback(const std_msgs::Float32::ConstPtr& msg);
      void cameraSetFocusCallback(const insp_msgs::CameraFocusPoint::ConstPtr& msg);

      void cameraRecordVideoCallback(const std_msgs::Bool::ConstPtr& msg);
      void cameraShootPhotoCallback(const std_msgs::Bool::ConstPtr& msg);
      void cameraRecordVideoDelayExec();
      void cameraShootPhotoDelayExec();

      

      /* ----------- Timer callback function ----------- */
      void stageModePublisherCallback(const ros::TimerEvent& event);
      void routineCallback(const ros::TimerEvent& event);
      void localOdomPublisherCallback(const ros::TimerEvent& event);
      void cameraExecuteCallback(const ros::TimerEvent& event);
      void nodeStatusPublisherCallback(const ros::TimerEvent& event);
      void taskExecuteCallback(const ros::TimerEvent& event);

      /* ----------- frame transform function ----------- */
      void createLocalFrame();



      /* data transform from sensor */


      // convert vehicle position from world ENU to local FLU
      Eigen::Vector3d convertVehiclePositionWorldENU2LocalFLU(Eigen::Vector3d posi_enu);
      // convert vehicle attitude from world ENU to local FLU
      Eigen::Quaterniond convertVehicleAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu);
      // convert vehicle velocity from world ENU to local FLU
      Eigen::Vector3d convertVehicleVelocityWorldENU2LocalFLU(Eigen::Vector3d velo_enu);
      // convert gimbal attitude from world ENU to local FLU
      Eigen::Quaterniond convertGimbalAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu);

      /* data transform to control */
      // convert vehicle velo from local FLU to world ENU
      Eigen::Vector3d convertVehicleVelocityLocalFLU2WorldENU(Eigen::Vector3d velo_flu);
      // convert vehicle yaw dot from local FLU to world ENU
      double convertVehicleYawDotLocalFLU2WorldENU(double yaw_flu);



      void updateDebugMasterMsg();

    private:

      ros::NodeHandle                             nh_;

      std::atomic_flag                            mutex_ = ATOMIC_FLAG_INIT;
      double                                      gim_ctrl_gap_;

      double                                      rate_sm_broadcast_;
      double                                      rate_odom_broadcast_;
      double                                      rate_routine_;
      double                                      rate_camera_exec_;
      double                                      rate_task_ctrl_;








      int                                         stage_mode_;
      ros::Time                                   last_update_request_time_;
      ros::Time                                   info_time_;

      bool                                        local_frame_created_;

      mavros_msgs::State                          mavros_state_;

      // global data

      Eigen::Vector3d                             global_posi_;
      Eigen::Quaterniond                          global_quat_;
      Eigen::Vector3d                             global_euler_;
      Eigen::Vector3d                             global_velo_;
      Eigen::Vector3d                             global_gimbal_euler_;
      Eigen::Quaterniond                          global_gimbal_quat_;

      // local data
      Eigen::Vector3d                             local_posi_;
      Eigen::Quaterniond                          local_quat_;
      Eigen::Vector3d                             local_euler_;
      Eigen::Vector3d                             local_velo_;
      Eigen::Quaterniond                          local_gimbal_quat_;
      Eigen::Vector3d                             local_gimbal_euler_;

      // set data
      Eigen::Vector4d                             set_local_velo_;
      Eigen::Vector3d                             set_local_gimbal_vec_;

      Eigen::Vector4d                             set_global_velo_;
      Eigen::Vector3d                             set_global_gimbal_vec_;
      Eigen::Vector3d                             set_global_gimbal_euler_;

      // R and t

    
      // world ENU to local FLU
      Eigen::Matrix3d                             R_WorldENU2LocalFLU_;
      Eigen::Vector3d                             t_WorldENU2LocalFLU_;
      Eigen::Matrix4d                             Rt_WorldENU2LocalFLU_;
      Eigen::Matrix3d                             R_LocalFLU2WorldENU_;
      Eigen::Vector3d                             t_LocalFLU2WorldENU_;
      Eigen::Matrix4d                             Rt_LocalFLU2WorldENU_;































      int                                         flight_task_;
      ros::Time                                   last_req_time_;
      

      double                                      last_yaw_;
      double                                      last_pitch_;



      bool                                        status_camera_recording_;
      bool                                        status_camera_shooting_;

      int                                         signal_record_start_;
      int                                         signal_record_end_;
      int                                         signal_shoot_start_;
      int                                         signal_shoot_end_;

      double                                      record_delay_start_;
      double                                      record_delay_end_;
      double                                      shoot_delay_start_;
      double                                      shoot_delay_end_;

      int                                         interval_shoot_photo_;



      std::shared_ptr<visualization::Visualization>    visPtr_;

      std::string                                 str_topic_world_x_;
      std::string                                 str_topic_world_y_;
      std::string                                 str_topic_world_z_;
      std::string                                 str_topic_mavros_x_;
      std::string                                 str_topic_mavros_y_;
      std::string                                 str_topic_mavros_z_;

      std::string                                 str_topic_gim_;

      double                                      arrow_width_;

      insp_msgs::DebugMaster                      dm_msg_;

  };
}


#endif

