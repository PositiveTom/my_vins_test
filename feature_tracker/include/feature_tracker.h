#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include "mCamera.h"
#include "PinholeCamera.h"
#include "tic_toc.h"
#include "parameters.h"
#include <Eigen/Eigen>
#include <map>

using namespace std;

#define CLAHE_Debug false       // debug查看直方图均衡化的图像与原始图像的对比
#define SHI_TOMASIDebug false
#define maskDebug false          // 观察掩膜
#define showTrack true          // 展示跟踪效果
#define FeatureTrackerDebug true     // 展示一些数据信息的debug,这个最直观

class FeatureTracker
{
private:
    /* data */
public:
    FeatureTracker(/* args */);
    //  读取相机内参
    void readIntrinsicParameter(const std::string& intrinsic_path);
    //  读取图像 处理核心进程
    void readImage(const cv::Mat &_img, double _cur_time);
    //  给cur_pts容器添加暂时新增的harris特征点, 预处理特征点id, 预处理以及被看到的次数
    void addPoints();
    //  特征点去畸变
    void undistortedPoints();
    //  利用RANSCAC本质矩阵的方法拒绝掉一些光流跟踪的特征点
    void rejectWithF();
    //  避免提取重复的角点,设置一个mask掩膜,告诉系统不要提取光流交点附近的角点
    //  同时对提取到的特征点进行一个排序,出现次数越多的特征点,排在容器的前面
    void setMask();
    //  给特征点赋予id值
    bool updateID(ulong i);
public:
    //  内参矩阵
    cv::Mat intrinsicMatrix_;
    //  内参矩阵的系数
    double fx, fy, u0, v0;
    //  畸变系数
    double k1, k2, p1, p2;

    //  相机实例化对象
    std::shared_ptr<mCamera> camera_;

    //  当前时间戳
    double cur_time_;
    //  上一帧图像的时间
    double prev_time_;
    //  存储当前帧无畸变特征点的容器, 存放特征点的齐次相机坐标
    std::vector<cv::Point2f> cur_un_pts, prev_un_pts;
    //  存放 特征点 齐次相机坐标, 以及对应在整个slam流程中的id号
    std::map<int, cv::Point2f> cur_un_pts_map;
    //  上一帧图像的容器, 与 cur_un_pts_map 的含义相同
    std::map<int, cv::Point2f> prev_un_pts_map;
    //  当前有效无畸变特征点的速度
    std::vector<cv::Point2f> pts_velocity;

    /****注意:以下的当前含义都是只针对于readImage函数而言****/

    //  上上帧图像
    cv::Mat prev_img;
    //  上帧图像
    cv::Mat cur_img;
    //  当前帧图像
    cv::Mat forw_img;
    //  存放当前帧图像的特征点容器
    std::vector<cv::Point2f> forw_pts;
    //  存放当前帧图像的特征点id
    std::vector<int> ids;
    //  存放当前帧的特征点总共被看到的次数
    std::vector<int> track_cnt;
    //  存放上一帧图像的特征点容器
    std::vector<cv::Point2f> cur_pts;
    //  存放上上帧图像的特征点的容器
    std::vector<cv::Point2f> prev_pts;

    /****harris特征提取相关参数****/

    //  掩膜
    cv::Mat mask;
    //  暂时新增的harris角点
    std::vector<cv::Point2f> n_pts;

    static int n_id;
};





#endif