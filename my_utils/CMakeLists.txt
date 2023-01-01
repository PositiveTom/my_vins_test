cmake_minimum_required(VERSION 3.0.2)
project(my_utils)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf
)
find_package(OpenCV 3.4.9 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

catkin_package()

# add_subdirectory(mythirdparty/Realsense-Operator)
add_subdirectory(mythirdparty/NLP)
add_subdirectory(mythirdparty/MyOrbExtractor)
add_subdirectory(mythirdparty/CameraModel)

include_directories(
  ${Sophus_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_check_time src/my_check_time.cpp)
add_executable(my_calibration src/my_calibration.cpp)
add_executable(sample_photo src/my_samplephoto.cpp)
add_executable(my_visualhendendtest src/my_visualhendendtest.cpp)
add_executable(my_smoothproperty src/my_smoothproperty.cpp)
# add_executable(my_sampleRealsense src/my_sampleRealsense.cpp)
# add_executable(my_sample_cam_imu src/my_sample_cam_imu.cpp )
add_executable(my_NLPTest src/my_NLPTest.cpp)

target_link_libraries(my_visualhendendtest
  OrbExtractor
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
target_include_directories(my_visualhendendtest PUBLIC
  mythirdparty/MyOrbExtractor/include
)

target_link_libraries(sample_photo
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
target_link_libraries(my_check_time
  ${catkin_LIBRARIES}
)
target_link_libraries(my_calibration
  NLP_LIB
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
)
target_link_libraries(my_smoothproperty
  ${Sophus_LIBRARIES}
  ${catkin_LIBRARIES}
)
# target_link_libraries(my_sampleRealsense
#   MyRealsenseLIB
#   ${catkin_LIBRARIES}
# )
target_link_libraries(my_NLPTest
  NLP_LIB
  ${catkin_LIBRARIES}
)

target_include_directories(my_calibration PUBLIC
  mythirdparty/NLP/inlude
  ${catkin_INCLUDE_DIRS}
)

# target_include_directories(my_sampleRealsense PUBLIC
#   mythirdparty/Realsense-Operator/include
#   ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(my_sample_cam_imu
#   MyRealsenseLIB
#   ${catkin_LIBRARIES}
# )
# target_include_directories(my_sample_cam_imu PUBLIC
#   mythirdparty/Realsense-Operator/include
#   ${catkin_INCLUDE_DIRS}
# )
target_include_directories(my_NLPTest PUBLIC
  mythirdparty/NLP/inlude
  ${catkin_INCLUDE_DIRS}
)