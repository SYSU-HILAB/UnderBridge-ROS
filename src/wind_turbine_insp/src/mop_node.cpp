#include <ros/ros.h>
#include <std_msgs/Int32.h>

bool RosNodeStatus[12];
ros::Time RosNodeHeartbeatTime[12];

double timeOut = 3.0;

void ROSnodeLaunchStatusSubCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int node_idx = msg -> data;
  RosNodeHeartbeatTime[node_idx] = ros::Time::now();
}

void updateROSNodeStatus()
{
  RosNodeHeartbeatTime[0] = ros::Time::now();
  RosNodeHeartbeatTime[1] = ros::Time::now();
  RosNodeHeartbeatTime[2] = ros::Time::now();
  
  for(int i = 0; i < 12; i++)
  {
    if((ros::Time::now() - RosNodeHeartbeatTime[i]).toSec() < timeOut)
      RosNodeStatus[i] = true;
    else
      RosNodeStatus[i] = false;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "mop_node");
  ros::NodeHandle nh;

  ros::Subscriber ros_node_status_sub = nh.subscribe<std_msgs::Int32>("/wind_turbine_insp/node_status", 10, &ROSnodeLaunchStatusSubCallback);

  for(int i = 0; i < 12; i++)
  {
    RosNodeStatus[i] = false;
    RosNodeHeartbeatTime[i] = ros::Time::now() - ros::Duration(timeOut);
  }

  ROS_INFO("launch mop_node");

	while (ros::ok())
	{
    updateROSNodeStatus();

    printf("node status: [ ");
    for(int i = 0; i < 12; i++)
      printf("%d ", RosNodeStatus[i]);
    printf("]\n");

		ros::spinOnce();
		ros::Duration(0.2).sleep();
	}
	
  return 0;
}
