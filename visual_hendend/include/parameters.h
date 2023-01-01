#pragma once
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;

extern Eigen::Matrix3d RCI;
extern Eigen::Vector3d TCI;
extern double TD;
extern std::string IMU_TOPIC_NAME;
const int WINDOW_SIZE = 11; //  滑动窗口的尺寸(包含的图像帧数)
extern double N_A;
extern double B_N_A;
extern double N_G;
extern double B_N_G;
extern double MIN_PARALLAX;

void readParameters();


// #endif
