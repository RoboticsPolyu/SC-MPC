cmake_minimum_required(VERSION 2.8.3)
project(jpcm)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 
find_package(GTSAM 4 REQUIRED)
find_package(yaml-cpp)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include/${PROJECT_NAME}
  include
  GTSAMIntegration
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(jpcm_lib 
            GTSAMIntegration/factors.cpp
            GTSAMIntegration/GTSAMUtils.cpp
            GTSAMIntegration/JPCM.cpp
            GTSAMIntegration/Marginalization.cpp    
            src/controller.cpp
            src/PX4CtrlParam.cpp
            src/input.cpp)

target_link_libraries(jpcm_lib PUBLIC ${catkin_LIBRARIES} gtsam gtsam_unstable ${YAML_CPP_LIBRARIES})

add_executable(gps_imu        
  src/gps_imu.cpp
  app/gps_imu_node.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
)

add_dependencies(gps_imu quadrotor_msgs geometry_msgs)

target_link_libraries(gps_imu
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

add_executable(odom_node        
  app/odom_node.cpp
  src/PX4CtrlParam.cpp
)

add_dependencies(odom_node quadrotor_msgs geometry_msgs)

target_link_libraries(odom_node
  gtsam
  gtsam_unstable
  jpcm_lib
)

add_executable(JPCM_node 
  app/JPCM_node.cpp
  src/PX4CtrlFSM.cpp
)

add_dependencies(JPCM_node quadrotor_msgs)

target_link_libraries(JPCM_node
  gtsam 
  gtsam_unstable
  jpcm_lib
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_jpcm test/test_jpcm.cpp)

add_dependencies(test_jpcm quadrotor_msgs geometry_msgs)

target_link_libraries(test_jpcm
  gtsam
  gtsam_unstable
  jpcm_lib
)

add_executable(FakeGPS_IMU_fusion_node app/FakeGPS_IMU_fusion_node.cpp)

add_dependencies(FakeGPS_IMU_fusion_node quadrotor_msgs geometry_msgs)

target_link_libraries(FakeGPS_IMU_fusion_node
  gtsam
  gtsam_unstable
  jpcm_lib
)


add_executable(Wall_in_proximity_run app/Wall_in_proximity_run.cpp)

add_dependencies(Wall_in_proximity_run quadrotor_msgs geometry_msgs)

target_link_libraries(Wall_in_proximity_run
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)