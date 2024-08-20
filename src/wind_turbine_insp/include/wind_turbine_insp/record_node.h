#ifndef __RECORD_NODE_H__
#define __RECORD_NODE_H__


#include <ros/ros.h>
#include <Eigen/Eigen>

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <signal.h>

#include <insp_msgs/msg_tran.h>
#include <insp_msgs/StageMode.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>


namespace wind_turbine_insp{

  #define ZF2FL_P1 237.10
  #define ZF2FL_P2 0.1326

  #define PI (double)3.14159265358979323846

  #define DEG2RAD(DEG) ((DEG) * (PI) / (180.0))
  #define RAD2DEG(RAD) ((RAD) * (180.0) / (PI))

  struct sigaction                            sigIntHandler_;
  
  std::FILE *                                 pFile_;

  bool                                        log_record_;

  void shutDownHandler(int s)
  {
    ROS_INFO("Caught signal %d", s);

    if(log_record_)
    {
      log_record_ = false;
      ros::Duration(1.0).sleep();
      std::fclose(pFile_);
    }

    ros::shutdown();
  }

  class RecordNode{

    public:

      RecordNode(ros::NodeHandle nh);
      ~RecordNode();

      bool initVariable();

      bool initSubscriber();

      bool initTimer();


    protected:

      ros::Subscriber stage_mode_subscriber_;
      // global longitute, latitude, altitude
      ros::Subscriber global_lla_subscriber_;
      // local position, attitude, velocity
      ros::Subscriber local_odom_subscriber_;
      // gimbal angle
      ros::Subscriber global_gimbal_angle_subscriber_;
      // camera zoom
      ros::Subscriber camera_zoom_subscriber_;
      // blade distance
      ros::Subscriber blade_distance_subscriber_;
      // log state
      ros::Subscriber log_state_subscriber_;

      ros::Timer record_log_timer_;
      ros::Timer node_status_timer_;

      ros::Publisher node_status_publisher_;


    protected:

      void getStageModeCallback(const insp_msgs::StageMode::ConstPtr& msg);
      void getGlobalLLACallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void getLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void getGlobalGimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
      void getCameraZoomCallback(const std_msgs::Float32::ConstPtr& msg);
      void getBladeDistCallback(const std_msgs::Float32::ConstPtr& msg);
      void getLogStateCallback(const std_msgs::Int32::ConstPtr& msg);

      void recordLogCallback(const ros::TimerEvent & event);
      void nodeStatusPublisherCallback(const ros::TimerEvent& event);

      std::string getTimeString();
    
    private:

      ros::NodeHandle                             nh_;


      double                                      rate_record_;
      std::string                                 filename_;

      insp_msgs::StageMode                        stage_mode_;

      sensor_msgs::NavSatFix                      global_lla_;
      nav_msgs::Odometry                          local_odom_;
      geometry_msgs::Vector3Stamped               global_gimbal_angle_;
      std_msgs::Float32                           zoom_factor_;
      std_msgs::Float32                           blade_distance_;

      Eigen::Vector3d                             local_posi_;
      Eigen::Vector3d                             local_atti_;
      Eigen::Vector3d                             local_velo_;
      Eigen::Vector3d                             gimbal_atti_;
      double                                      focal_len_;
      double                                      blade_dist_;
      int                                         log_state_;


  };



}

#endif