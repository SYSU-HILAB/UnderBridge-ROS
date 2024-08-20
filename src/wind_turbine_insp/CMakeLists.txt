cmake_minimum_required(VERSION 3.0.2)
project(wind_turbine_insp)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
add_definitions(-w)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.3.0 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

  std_msgs
  mavros_msgs
  geometry_msgs
  nav_msgs

  tf

  pcl_conversions
  pcl_ros
  eigen_conversions

  insp_msgs
  px4_ctrl
  traj_utils

  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations -Wno-deprecated-declarations ")


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS

    roscpp
    rospy

    std_msgs
    mavros_msgs
    geometry_msgs

    insp_msgs
    px4_ctrl
    traj_utils

    
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

################################
###        master_node        ##
################################

# add_executable(master_node
#   src/master_node.cpp
# )
# target_link_libraries(master_node
#   ${catkin_LIBRARIES}
# )
##  add_dependencies( ${PROJECT_NAME}
##    # ${PROJECT_NAME}

##    # ${${PROJECT_NAME}_EXPORTED_TARGETS} 
##  )



################################
###        flight_node        ##
################################

# add_executable(flight_node
#   src/flight_node.cpp
# )
# target_link_libraries(flight_node
#   ${catkin_LIBRARIES}
#   px4_ctrl
# )
##  add_dependencies( ${PROJECT_NAME}
##    # ${PROJECT_NAME}

##    # ${${PROJECT_NAME}_EXPORTED_TARGETS} 
##  )


################################
###        sensing_node       ##
################################

# add_executable(sensing_node
#   src/sensing_node.cpp
# )
# target_link_libraries(sensing_node
#   ${catkin_LIBRARIES}
# )
##  add_dependencies( ${PROJECT_NAME}
##   # ${PROJECT_NAME}
##    # ${catkin_EXPORTED_TARGETS}

##    # ${${PROJECT_NAME}_EXPORTED_TARGETS} 
##  )

################################
###        vision_node        ##
################################

# add_executable(vision_node
#   src/vision_node.cpp
# )
# target_link_libraries(vision_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )
##  add_dependencies( ${PROJECT_NAME}
##    # ${PROJECT_NAME}
##    # ${catkin_EXPORTED_TARGETS}

##    # ${${PROJECT_NAME}_EXPORTED_TARGETS} 
##  )


################################
###        record_node        ##
################################

# add_executable(record_node
#   src/record_node.cpp
# )

# target_link_libraries(record_node
#   ${catkin_LIBRARIES}
#   ${DJIOSDK_LIBRARIES}
# )

# add_executable(mop_node
#   src/mop_node.cpp
# )

# target_link_libraries(mop_node
#   ${catkin_LIBRARIES}
#   ${DJIOSDK_LIBRARIES}
# )

# add_executable(testtt_node
#   src/testtt_node.cpp
# )

# target_link_libraries(testtt_node
#   ${catkin_LIBRARIES}
#   ${DJIOSDK_LIBRARIES}
# )


################################
###   convert_pipe_node       ##
################################

add_executable(convert_pipe_node
  src/convert_pipe_node.cpp
)
target_link_libraries(convert_pipe_node
  ${catkin_LIBRARIES}
)
