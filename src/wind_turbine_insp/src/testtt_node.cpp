#include <ros/ros.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/QuaternionStamped.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "testtt_node");
  ros::NodeHandle nh;

  std::vector<Eigen::Vector3d> vec;
  vec.clear();
  // vec.push_back(Eigen::Vector3d(0, 0, 1));
  std::cout << vec.empty() << std::endl;
  std::cout << vec.front() << std::endl;
  std::cout << vec.back() << std::endl;

  // Eigen::Matrix3d rotA2B;
  // Eigen::Matrix3d rotB2A;

  // geometry_msgs::QuaternionStamped global_quat_;
  // global_quat_.quaternion.w = 0.906;
  // global_quat_.quaternion.x = 0;
  // global_quat_.quaternion.y = 0;
  // global_quat_.quaternion.z = 0.422;

  // Eigen::Quaterniond cur_quat = Eigen::Quaterniond(global_quat_.quaternion.w, 
  //                                                  global_quat_.quaternion.x, 
  //                                                  global_quat_.quaternion.y, 
  //                                                  global_quat_.quaternion.z).normalized();
  // Eigen::Vector3d cur_euler = cur_quat.toRotationMatrix().eulerAngles(2, 1, 0);
  // Eigen::Vector3d ref_euler = Eigen::Vector3d(0, 0, cur_euler(0));

  // Eigen::AngleAxisd a_r1(Eigen::AngleAxisd(ref_euler(0), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd a_p1(Eigen::AngleAxisd(ref_euler(1), Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd a_y1(Eigen::AngleAxisd(ref_euler(2), Eigen::Vector3d::UnitZ()));

  // rotA2B = a_y1 * a_p1 * a_r1;

  // rotB2A = rotA2B.inverse();

  // std::cout << rotA2B << std::endl;
  // std::cout << std::endl;
  // std::cout << rotB2A << std::endl;


  // Eigen::Vector3d euler_flu = Eigen::Vector3d(0, 0, 10.0 / 180.0 * 3.14);

  // Eigen::AngleAxisd a_r2(Eigen::AngleAxisd(euler_flu(0), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd a_p2(Eigen::AngleAxisd(euler_flu(1), Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd a_y2(Eigen::AngleAxisd(euler_flu(2), Eigen::Vector3d::UnitZ()));

  // Eigen::Matrix3d rotmat_flu;
  // rotmat_flu = a_y2 * a_p2 * a_r2;

  // std::cout << rotmat_flu << std::endl;


  // Eigen::Matrix3d rotmat_enu = rotA2B * rotmat_flu;

  // std::cout << rotmat_enu << std::endl;

  // Eigen::Vector3d euler_enu = rotmat_enu.eulerAngles(2, 1, 0);

  // std::cout << euler_enu << std::endl;

  

  // double yaw_enu = euler_enu(0);

  // ROS_INFO("(yaw_flu, yaw_enu): %.2f, %.2f", euler_flu(2) / 3.14 * 180.0, yaw_enu / 3.14 * 180.0);


  return 0;
}





// Eigen::Vector3d rotvec_axis = (vec_origin.cross(vec_gimbal)).normalized();
//   double rotvec_angle = acos(vec_origin.dot(vec_gimbal) / (vec_origin.norm() * vec_gimbal.norm()));
//   Eigen::AngleAxisd rotvec = Eigen::AngleAxisd(rotvec_angle, rotvec_axis);
//   Eigen::Vector3d euler_gimbal = rotvec.toRotationMatrix().eulerAngles(2, 1, 0);

//   double roll = euler_gimbal(2), pitch = euler_gimbal(1), yaw = euler_gimbal(0);






// int main(int argc, char ** argv)
// {
//   ros::init(argc, argv, "testtt_node");
//   ros::NodeHandle nh;

//   Eigen::Matrix3d rotA2B;
//   Eigen::Matrix3d rotB2A;

//   geometry_msgs::QuaternionStamped global_quat_;
//   global_quat_.quaternion.w = 0.415;
//   global_quat_.quaternion.x = -0.606;
//   global_quat_.quaternion.y = -0.453;
//   global_quat_.quaternion.z = -0.504;

//   Eigen::Quaterniond cur_quat = Eigen::Quaterniond(global_quat_.quaternion.w, 
//                                                    global_quat_.quaternion.x, 
//                                                    global_quat_.quaternion.y, 
//                                                    global_quat_.quaternion.z).normalized();
//   Eigen::Vector3d cur_euler = cur_quat.toRotationMatrix().eulerAngles(2, 1, 0);
//   Eigen::Vector3d ref_euler = Eigen::Vector3d(0, 0, cur_euler(0));

//   Eigen::AngleAxisd a_r(Eigen::AngleAxisd(ref_euler(0), Eigen::Vector3d::UnitX()));
//   Eigen::AngleAxisd a_p(Eigen::AngleAxisd(ref_euler(1), Eigen::Vector3d::UnitY()));
//   Eigen::AngleAxisd a_y(Eigen::AngleAxisd(ref_euler(2), Eigen::Vector3d::UnitZ()));

//   rotA2B = a_y * a_p * a_r;

//   rotB2A = rotA2B.inverse();

//   std::cout << rotA2B << std::endl;
//   std::cout << std::endl;
//   std::cout << rotB2A << std::endl;

//   Eigen::Vector3d vec;

//   tf::Quaternion quat;
//   tf::quaternionMsgToTF(global_quat_.quaternion, quat);
//   tf::Matrix3x3(quat).getRPY(vec(0), vec(1), vec(2));

//   std::cout << ref_euler << std::endl;

//   std::cout << vec << std::endl;

//   return 0;
// }





















    // local_posi_.point.x += +(rotor_dist_ver_ / 2.0 + gimbal_center_dist_) * cos(rpy_cur(2)) - (rotor_dist_hor_ / 2.0) * sin(rpy_cur(2));
    // local_posi_.point.y += +(rotor_dist_ver_ / 2.0 + gimbal_center_dist_) * sin(rpy_cur(2)) + (rotor_dist_hor_ / 2.0) * cos(rpy_cur(2));
    // local_posi_.point.z -= gimbal_rtk_hei_;


  //   double yaw = 0.0, pitch = 0.0, x_new = 0.0;

  // if(msg -> keep_yaw)
  // {
  //   yaw = last_yaw_;

  //   if(vec_gimbal(0) > 0 && vec_gimbal(0) < 1e-5)
  //     vec_gimbal(0) = 1e-5;
  //   else if(vec_gimbal(0) < 0 && vec_gimbal(0) > -1e-5)
  //     vec_gimbal(0) = -1e-5;

  //   // double yaw_now = atan2(vec_gimbal(1), vec_gimbal(0));

  //   x_new = vec_gimbal(0) * cos(-yaw) - vec_gimbal(1) * sin(-yaw);

  //   // ROS_INFO("yaw yaw: %.2f", RAD2DEG(abs(yaw - yaw_now)));

  //   // if(abs(yaw - yaw_now) > PI / 2)
  //   // pitch = -(PI - atan2(vec_gimbal(2), x_new));
  //   // else
  //   pitch = -atan2(vec_gimbal(2), x_new);

  //   ROS_INFO("pitch: %.2f, KEEPYAW", RAD2DEG(pitch));

  // }
  // else
  // {
  //   if(vec_gimbal(0) > 0 && vec_gimbal(0) < 1e-5)
  //     vec_gimbal(0) = 1e-5;
  //   else if(vec_gimbal(0) < 0 && vec_gimbal(0) > -1e-5)
  //     vec_gimbal(0) = -1e-5;

  //   yaw = atan2(vec_gimbal(1), vec_gimbal(0));

  //   x_new = vec_gimbal(0) * cos(-yaw) - vec_gimbal(1) * sin(-yaw);

  //   if(x_new > 0 && x_new < 1e-5)
  //     x_new = 1e-5;
  //   else if(x_new < 0 && x_new > -1e-5)
  //     x_new = -1e-5;

  //   pitch = -atan2(vec_gimbal(2), x_new);

  //   if(pitch < -DEG2RAD(80.0))
  //   {
  //     ROS_INFO("pitch: %.2f, BIGGER than 80", RAD2DEG(pitch));
  //     yaw = last_yaw_;
  //   }else
  //   {
  //     ROS_INFO("pitch: %.2f, normal", RAD2DEG(pitch));
  //   }
  // }


  // ROS_INFO("euler: (roll, %.2f, %.2f)", RAD2DEG(pitch), RAD2DEG(yaw));

  // last_yaw_ = yaw;
  // last_pitch_ = pitch;

  // mavros_msgs::MountControl mount_ctrl;

  // mount_ctrl.header.stamp = ros::Time::now();
  // mount_ctrl.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;

  // pitch -= DEG2RAD(8.0);
  
  // if(msg -> is_reset)
  // {
  //   mount_ctrl.roll  = 0.0;
  //   mount_ctrl.pitch = 0.0;
  //   mount_ctrl.yaw   = 0.0;
  // }
  // else
  // {
  //   mount_ctrl.roll  = std::max(std::min(0.0,  +90.0),  -90.0);
  //   mount_ctrl.pitch = std::max(std::min(RAD2DEG(pitch),  +30.0),  -120.0);
  //   mount_ctrl.yaw   = std::max(std::min(RAD2DEG(yaw), +180.0), -180.0);
  // }

  // gimbal_set_posi_publisher_.publish(mount_ctrl);

  // if(msg -> is_reset)
  // {
  //   ROS_INFO("Reset gimbal once!");
  // }


//   double MasterNode::convertVehicleYawDotLocalFLU2WorldENU(double yaw_flu)
// {
//   Eigen::Vector3d euler_flu = Eigen::Vector3d(0, 0, yaw_flu);

//   // Eigen::AngleAxisd a_r(Eigen::AngleAxisd(euler_flu(0), Eigen::Vector3d::UnitX()));
//   // Eigen::AngleAxisd a_p(Eigen::AngleAxisd(euler_flu(1), Eigen::Vector3d::UnitY()));
//   // Eigen::AngleAxisd a_y(Eigen::AngleAxisd(euler_flu(2), Eigen::Vector3d::UnitZ()));

//   // Eigen::Matrix3d rotmat_flu;
//   // rotmat_flu = a_y * a_p * a_r;

//   // Eigen::Matrix3d rotmat_enu = R_WorldENU2LocalFLU_ * rotmat_flu;
//   Eigen::Vector3d euler_enu = R_WorldENU2LocalFLU_ * euler_flu;

  
//   // Eigen::Vector3d euler_enu = rotmat_enu.eulerAngles(2, 1, 0);

//   // double yaw_enu = euler_enu(0);
//     double yaw_enu = euler_enu(2);


//   return yaw_enu;
// }
