cmake_minimum_required(VERSION 3.0.2)
project(insp_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  mavros_msgs
  eigen_conversions
)




## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DebugFlight.msg
  DebugMaster.msg
  DebugVision.msg
  PtCloudKey.msg
  StageMode.msg
  TrackIndex.msg
  VeloCmd.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FlightTaskControl.srv
  FootPointCorrect.srv
  SetStageMode.srv
  OccupyCheck.srv
  EmergencyTerminate.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(

  INCLUDE_DIRS 
    include

  LIBRARIES 
    ${PROJECT_NAME}

  CATKIN_DEPENDS 
    roscpp 
    rospy
    std_msgs
    geometry_msgs
    message_runtime

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
  src/msg_tran.cpp
)


target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies( ${PROJECT_NAME}
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}

  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
)
