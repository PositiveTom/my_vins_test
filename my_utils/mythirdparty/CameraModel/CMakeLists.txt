

set(CameraFile src/mCamera.cc src/PinholeCamera.cc)

find_package(OpenCV 3.4.9 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(MyCamera SHARED ${CameraFile})
target_include_directories(MyCamera PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(MyCamera PUBLIC
    ${OpenCV_DIRS}
)