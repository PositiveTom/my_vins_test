#pragma once
#include <ros/ros.h>
#include <string>

extern std::string IMAGE_TOPIC;
extern std::string INTRINSIC_ADDRESS;
extern int FREQ;
extern bool EQUALIZE;
extern bool PUB_THIS_FRAME;
extern int MAX_CNT;
extern int ROW;
extern int COL;
extern double F_THRESHOLD;
extern int MIN_DIST;
/**
 * @brief 读取图像话题名, 相机内参文件地址
 */ 
void readParameters();