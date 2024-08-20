#include "insp_msgs/msg_tran.h"

namespace wind_turbine_insp{


  /* ------------------------------------------------------------ PoseStamped ------------------------------------------------------------ */

  void PointMsgToXYZVec(geometry_msgs::Point msg, Eigen::Vector3d& vec){
    
    vec(0) = msg.x;
    vec(1) = msg.y;
    vec(2) = msg.z;

    return;  
  }


  void QuatnMsgToRPYVec(geometry_msgs::Quaternion msg, Eigen::Vector3d& vec){

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
    tf::Matrix3x3(quat).getRPY(vec(0), vec(1), vec(2));

    return;
  }


  void QuatnVecToRPYVec(Eigen::Quaterniond qtn, Eigen::Vector3d& vec)
  {
    tf::Quaternion quat(qtn.x(), qtn.y(), qtn.z(), qtn.w());
    tf::Matrix3x3(quat).getRPY(vec(0), vec(1), vec(2));
    return;
  }


  void RPYVecToQuatnVec(Eigen::Vector3d vec, Eigen::Quaterniond& qtn)
  {
    tf::Quaternion quat;
    quat.setRPY(vec(0), vec(1), vec(2));
    qtn = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
    return;
  }


  void VectorEigenToMsg(Eigen::Vector3d vec, geometry_msgs::Vector3& msg){

    tf::vectorEigenToMsg(vec, msg);

    return;

  }

  void VectorMsgToEigen(geometry_msgs::Vector3 msg, Eigen::Vector3d& vec){

    tf::vectorMsgToEigen(msg, vec);

    return;

  }

}