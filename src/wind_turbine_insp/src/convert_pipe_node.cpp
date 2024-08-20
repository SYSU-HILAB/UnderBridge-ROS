#include "wind_turbine_insp/convert_pipe_node.h"
using namespace class_convert_pipe;


#define MSGKEY 1000

/*################*/
/* Message struct */
/*################*/
struct pipeMessageBuffer{
    long mType; // msg type
    double data[7]; // calocity xyz, quat wxyz
};


/*################*/
/*  Constructor   */
/*################*/
ConvertPipeNode::ConvertPipeNode(ros::NodeHandle nh): nh_(nh) {

  initVariable();

  initServiceClient();
  initSubscriber();

  ros::Duration(1.0).sleep();

  initServiceServer();
  initPublisher();

  initTimer();
}
 
/*##############*/
/*  Destructor  */
/*##############*/

ConvertPipeNode::~ConvertPipeNode(){
}

bool ConvertPipeNode::initVariable(){

  printf("Loading Params Finish\n");
  printf("---------------------\n");

  

  ROS_INFO_STREAM("ConvertPipeNode: init Variable");


  return true;
}

/*###################################*/
/* Subscriber initialize function */
/*###################################*/
bool ConvertPipeNode::initSubscriber(){
  
  traj_velo_subscriber_          = nh_.subscribe<geometry_msgs::Twist>("/wind_turbine_insp/traj_velo_ctrl", 2, &ConvertPipeNode::trajVeloCtrlCallback, this);

  ROS_INFO_STREAM("ConvertPipeNode: init Subscriber");

  return true;
}


/*###################################*/
/* ServiceClient initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceClient(){


  ROS_INFO_STREAM("Flight_node: init ServiceClient");

  return true;
}

/*###################################*/
/* ServiceServer initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceServer(){
  
  ROS_INFO_STREAM("Flight_node: init ServiceServer");
}

/*###################################*/
/*   Publisher initialize function   */
/*###################################*/
bool ConvertPipeNode::initPublisher(){
    set_velo_publisher_                  = nh_.advertise<insp_msgs::VeloCmd>("/wind_turbine_insp/set_velo", 2);
    wps_servo_publisher_                 = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_common", 2);
    wps_traj_publisher_                  = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_minco", 1);
    

    ROS_INFO_STREAM("Flight_node: init Publisher");

    return true;
}

/*################################*/
/*    Timer initialize function   */
/*################################*/

bool ConvertPipeNode::initTimer(){

    ROS_INFO_STREAM("Flight_node: init timer");

    return true;
}

/*###############################*/
/*  subscriber Callback function */
/*################################*/
void ConvertPipeNode::trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg){
    set_velo_.x = msg -> linear.x;
    set_velo_.y = msg -> linear.y;
    set_velo_.z = msg -> linear.z;
}



/*###############################*/
/*  subscriber Callback function */
/*################################*/
void ConvertPipeNode::convertMsg2PipeCallback(const ros::TimerEvent& event){
    set_velo_publisher_.publish(set_velo_);

    if(ros::Time::now() - last_info_time_ > ros::Duration(3.0))
    {
        ROS_WARN("[Convert Pipe Node]:Timeout for 3s.");
    } else {

    }

    last_info_time_ = ros::Time::now();
    return;
}

/*#################*/
/*                 */
/*  Main Function  */
/*                 */
/*#################*/

int main(int argc, char** argv){
    // init ros node
    ros::init(argc, argv, "convert_pipe_node");
    ros::NodeHandle nh;
    ConvertPipeNode convert_pipe_node(nh);
    ROS_INFO_STREAM("Convert Pipe Node is OK !");
    ros::MultiThreadedSpinner spinner(1);
    spinner.spin(); 

    // init msg pipe

    return 0;
}