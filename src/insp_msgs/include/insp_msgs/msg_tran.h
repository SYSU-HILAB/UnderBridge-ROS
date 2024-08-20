#ifndef __MSG_TRAN_H__
#define __MSG_TRAN_H__


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>


namespace wind_turbine_insp {

  /* ------------------------------------------------------------ PoseStamped ------------------------------------------------------------ */

  void PointMsgToXYZVec(geometry_msgs::Point msg, Eigen::Vector3d& vec);

  void QuatnMsgToRPYVec(geometry_msgs::Quaternion msg, Eigen::Vector3d& vec);

  void QuatnVecToRPYVec(Eigen::Quaterniond qtn, Eigen::Vector3d& vec);

  void RPYVecToQuatnVec(Eigen::Vector3d vec, Eigen::Quaterniond& msg);

  /* ------------------------------------------------------------ PoseStamped ------------------------------------------------------------ */

  void VectorEigenToMsg(Eigen::Vector3d vec, geometry_msgs::Vector3& msg);

  void VectorMsgToEigen(geometry_msgs::Vector3 msg, Eigen::Vector3d& vec);

}

#endif