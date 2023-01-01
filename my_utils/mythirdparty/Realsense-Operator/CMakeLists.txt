

message(STATUS "************MyRealsenseLIB*************")

find_package(realsense2 REQUIRED)
find_package(OpenCV 3.4.9 REQUIRED)

# message(STATUS ${realsense2_INCLUDE_DIR})
# message(STATUS "************MyRealsenseLIB*************")
# set(MyRealsenseFile src/ImuSample.cc)


add_library(MyRealsenseLIB src/ImuSample.cc)

target_include_directories(MyRealsenseLIB PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(MyRealsenseLIB
    ${OpenCV_LIBS}
    ${catkin_INCLUDE_DIRS}
    ${realsense2_LIBRARY}
)
