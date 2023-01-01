#ifndef INITIAL_SFM_H
#define INITIAL_SFM_H

/****管理初始化过程中SfM的类****/

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>
#include "estimator.h"
#include <tf/transform_broadcaster.h>

using namespace std;

// 在估计器初始化的过程中, 每个被纳入到 特征管理器中 feature 容器的特征点, 都有这样一个类
struct SfmFeature
{
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //  是否三角化
    bool state;
    //  特征点的id
    int id;
    //  特征点 在 第一帧图像中的 齐次相机坐标
    Eigen::Vector3d position;
    //  特征点的深度值
    double depth;
    //  其他图像帧看到此特征点时的齐次相机坐标
    std::vector<std::pair<int, Eigen::Vector2d>> observation;
};

//  全局sfm, 恢复滑动窗口内帧与帧之间的位姿, 和特征点相机坐标
class GlobalSfM
{
public:
    GlobalSfM()
    {
        feature_nums = 0;
    }
    /**
     * @brief 估计器初始化阶段的全局SfM
     * @param WINDOW_SIZE 滑动窗口数
     * @param Q 指向存储滑动窗口旋转矩阵数组的指针
     * @param T 指向存储滑动窗口平移向量数组的指针
     * @param l 参考帧在滑动窗口中的索引
     * @param R_cl_cur 当前帧到参考帧的旋转矩阵
     * @param T_cl_cur 当前帧到参考帧的平移向量
     * @param sfm_f 参与sfm的特征点
     * @param sfm_track_points sfm_f中成功三角化的特征点
    */  
    bool construct(int WINDOW_SIZE, Eigen::Quaterniond *Q, Eigen::Vector3d *T, int l,
    const Eigen::Matrix3d &R_cl_cur, const Eigen::Vector3d &T_cl_cur, 
    std::vector<SfmFeature> &sfm_f, std::map<int, Eigen::Vector3d> &sfm_track_points);

    /**
     * @brief 两帧进行三角化,得到特征点的世界坐标
     * @param frame0 图像帧的索引frame0
     * @param frame1 图像帧的索引frame1
     * @param Pose0 图像帧frame0的位姿
     * @param Pose1 图像帧frame1的位姿
     * @param sfm_f 等待三角化的特征点容器
     */ 
    void triangulateTwoFrames(int frame0, const Eigen::Matrix<double, 3, 4> &Pose0,
        int frame1, const Eigen::Matrix<double, 3, 4> &Pose1, std::vector<SfmFeature> &sfm_f);
    /**
     * @brief  三角化函数  
     *  (\bar{x_1} (T_1_w)_{row3} - (T_1_w)_{row1}) Pw = 0
     *  (\bar{y_1} (T_1_w)_{row3} - (T_1_w)_{row2}) Pw = 0
     *  (\bar{x_2} (T_2_w)_{row2} - (T_2_w)_{row1}) Pw = 0
     *  (\bar{y_2} (T_2_w)_{row2} - (T_2_w)_{row2}) Pw = 0
     */
    bool triangulatePoint(const Eigen::Matrix<double,3, 4> &Pose0, const Eigen::Matrix<double,3,4> &Pose1, 
        const Eigen::Vector2d &point0, const Eigen::Vector2d &point1, Eigen::Vector3d &Pw);
    //  求解相机帧的世界坐标
    bool solverFrameByPnP(Eigen::Matrix3d &R_lPLus1_cl, Eigen::Vector3d &T_lPlus1_cl, 
        int frame_id, std::vector<SfmFeature> &sfm_f);


public:
    //  特征管理器中的特征数
    int feature_nums;
};



#endif