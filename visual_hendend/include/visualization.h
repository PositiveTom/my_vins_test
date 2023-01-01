#ifndef VISUALIZATION_H
#define VISUALIZATION_H
// #pragma once

/****可视化工具****/

#include <ros/ros.h>
#include "estimator.h"
#include <iostream>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

class Estimator;// 这句代码,告诉编译器先不要报错, 这是一个类, 或许目前还没有看见,但是接下来会有的
using namespace std;

extern ros::Publisher point_cloud;


/**
 * @brief 发布特征管理器里面的 特征点集合 容器
 * 
 */ 
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);


void debug(const Estimator &estimator);

//  注册发布者, 专门发布估计器和VIO运行得到的数据
void registerPub(ros::NodeHandle& n);


#endif