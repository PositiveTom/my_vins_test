cmake_minimum_required(VERSION 3.0.2)
project(visual_hendend)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visual_hendend
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(featureExtractor
  src/ORBextractor.cc
)
target_include_directories(featureExtractor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(visualheadend_node src/visualheadend_node.cpp)
target_include_directories(visualheadend_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(visualheadend_node PUBLIC
  featureExtractor
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# 创建估计器库
set(EstimatorFile 
  src/estimator.cc 
  src/parameters.cc 
  src/feature_manager.cc
  src/preIntegration.cc
  src/image_frame.cc
  src/visualization.cc
  src/initial_sfm.cc
)
add_library(EstimatorLib ${EstimatorFile})
target_include_directories(EstimatorLib PUBLIC
  ${PROJECT_SOURCE_DIR}/../my_utils/mythirdparty/NLP/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(EstimatorLib PUBLIC
  /home/zjj/MyCode/myVinsPrj/devel/lib/libNLP_LIB.so
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(estimator_node src/estimator_node.cpp)
target_include_directories(estimator_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(estimator_node PUBLIC
  EstimatorLib 
  ${catkin_LIBRARIES}
)
