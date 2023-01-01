#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

/****特征管理器****/
/****只记录滑动窗口内的特征点****/

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <map>
#include <vector>
#include <list>
#include "parameters.h"

using namespace std;

//  专门管理每个特征点的类, 
//  每帧图像上的每个特征点都对应这样的一个类, 
//  这意味着世界坐标系下同一个特征点可以有多个这样的类, 特征点可以被多个图像帧看见
class FeaturePerFrame
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeaturePerFrame() = delete; // 禁用默认构造函数
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& feature)
    {
        xy1.x() = feature(0);
        xy1.y() = feature(1);
        xy1.z() = feature(2);

        uv.x() = feature(3);
        uv.y() = feature(4);

        velocity[0] = feature(5);
        velocity[1] = feature(6);
    }

public:
    //  特征点齐次相机坐标
    Eigen::Vector3d xy1;
    //  特征点像素坐标(带畸变的)
    Eigen::Vector2d uv;
    //  特征点的光流速度
    Eigen::Vector2d velocity;

};

//  管理 特征点 相关帧的类, 因为同一个特征点可以被不同帧看见, 
//  每个特征点, 都有唯一的这样一个类
class FeaturePerId
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeaturePerId() = delete; // 禁用默认构造函数

    /**
     * @param _feature_id 特征点的id
     * @param _start_frame 特征点在滑动窗口中的开始帧id
     * 
     */ 
    FeaturePerId(int _feature_id, int _start_frame)
    {
        feature_id_ = _feature_id;
        start_frame_ = _start_frame;

        estimated_depth = -1.;
        used_num = 0;

        solver_flag = false;
    }
    //  返回结束帧索引
    int endFrame();

public:
    //  特征点的id (整个slam系统里面的id)
    int feature_id_;
    //  开始帧id, 滑动窗口内的id
    int start_frame_;
    //  特征点的深度值, 定义在第一个看到此特征点的关键帧上
    double estimated_depth;
    //  看到此特征点的相关帧
    std::vector<FeaturePerFrame> feature_per_frame;
    //  看到此特征点的帧数
    int used_num;
    //  是否三角化成功
    bool solver_flag;

};

//  专门管理一帧图像上所有特征点集合的类, 称为特征管理器类
class FeatureManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureManager();
    //  设置相机 到 imu的旋转矩阵
    void setRic(Eigen::Matrix3d &ric);
    //  设置是否为关键帧
    bool addFeatureCheckParallax
    (int frame_count,const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    //  得到 目前 滑动窗口内 符合特定要求的 特征点的 个数, 这里的值是逐渐递增的, 在滑动窗口帧数从0到11的过程中
    int getFeatureCount();
    //  得到两帧图像的视差, 倒数第二帧 和 倒数第三 帧之间的视差, 每次只计算一个特征点的视差
    double compensatedParallax2(const FeaturePerId& feature, int frame_count);
    //  由于边缘化了最老帧, 因此要对特征管理器中的特征更新,//TODO 具体的更新是针对哪些参数呢?
    //  主要参数: (1)特征点的开始帧, (2)以及特征点的共视帧(决定了其是否存在)
    //  估计器初始化阶段的滑动窗口更新, 还没有估计深度,此时
    void removeBack();
    //  得到 图像帧 frame_count_l 和 frame_count_r 之间的共视特征点 集合
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> 
    getCorresponding(int frame_count_l, int frame_count_r);
    //  由于边缘化了次新帧, 对特征点产生的影响, 这里的frame_count表明告诉特征管理器, 为边缘化的是frame_count-1帧
    void removeFront(int frame_count);

public:
    //  相机 到 imu 的旋转矩阵
    Eigen::Matrix3d ric_;
    //  存储 滑动窗口内所有 特征点的双向链表, 保证唯一性, 使用 FeaturePerId 类
    std::list<FeaturePerId> features_;
};


#endif