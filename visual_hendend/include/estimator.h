#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include "parameters.h"
#include "feature_manager.h"
#include "preIntegration.h"
#include <map>
#include <vector>
#include <std_msgs/Header.h>
#include "image_frame.h"
#include "visualization.h"
#include "tic_toc.h"
#include "initial_sfm.h"

using namespace std;

/****需要待定调整的参数****/
//  [1]. 视差 TODO:

#define MEASUREMENT_Debug false  // 测量数据debug  这个非常重要, 日后值得好好总结
#define SfMDebug true

class Estimator
{

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Estimator();
    //  清除状态程序
    void clearState();
    //  设置初始参数
    void setParameter();
    //  处理IMU数据, 对IMU数据进行预积分, 得到预积分量, 每次处理一帧IMU数据
    void processIMU(const double dt, const Eigen::Vector3d& body_acc,const Eigen::Vector3d& body_gyro);
    //  处理图像特征数据, 每次处理一帧图像数据
    void processImage(
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
        const std_msgs::Header &header);
    //  估计器初始化, 最重要的一个步骤之一
    bool initialStructure();
    //  找到当前帧对应的参考帧,并且计算出相对位姿
    bool relativePose(Eigen::Matrix3d &R_cl_cur, Eigen::Vector3d &T_cl_cur, int &l);
    //  滑动窗口, 更新滑动窗口, // TODO 这个函数指明了哪些变量是滑动窗口内依赖的变量
    //  只有在滑动窗口满了之后才会进行边缘化操作
    void slideWindow();
    //  TODO 在估计器初始化阶段 更新特征点的开始帧, 删除marg掉的相关帧, 以及特征点是否存在, 以及在VIO阶段额外更新特征点的相机坐标, 深度
    void slideWindowOld();
    //  由于边缘化了次新帧, 因此导致的特征点更新
    //  之所以不需要更新深度信息, 是因为满足求深度的特征点 必须是稳定的特征点, 但凡开始帧位于WINDOW_SIZE-1, 都没有必要对齐三角化
    void slideWindowNew();
    //  求解帧与帧之间的相对位姿, 对极几何的方法   P_1 t^ R_1_2 P_2 = 0
    bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& corres,
        Eigen::Matrix3d &relativeR, Eigen::Vector3d &relativeT);
public:

    /****外参****/
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;


    /****特征管理器****/
    FeatureManager f_manager;


    //  时间偏移, t_imu = t_cam + td;
    double td;
    //  第一帧IMU信息的标志
    bool first_imu;
    //  每一次预积分的初始加速度 (选用中值积分)
    Eigen::Vector3d acc0;
    //  每一次预积分的初始角速度, 只当作预积分初始化使用, 当作整个预积分流程的第一帧imu数据, 而不会参与过程中的积分
    Eigen::Vector3d gyro0;


    /****预积分量类, 采取滑动窗口的方法进行优化, 因此只实时优化WINDOW_SIZE帧预积分量****/ 
    //  Note: 估计器只记录关键帧的预积分量, 但是每个图像帧都对应预积分量, 
    //        因此这里创建了另外一个变量all_image_frame来存储这些非关键帧以及关键帧所有的图像帧信息以及其预积分量
    
    // 创建WINDOW_SIZE个指向PreIntegration类的指针, 这种方法避免了类实例化
    PreIntegration *pre_integration[WINDOW_SIZE];    


    /****滑动窗口相关参数****/
    //  滑动窗口 中 关键帧 的索引
    int frame_count;
    //  滑动窗口 中 关键帧 的时间头信息
    std_msgs::Header Headers[WINDOW_SIZE];
    //  滑动窗口 中 关键帧 的旋转矩阵 , 滑动窗口旋转预积分
    Eigen::Matrix3d Rs[WINDOW_SIZE];
    //  滑动窗口 中 关键帧 的平移向量 , 滑动窗口位置预积分
    Eigen::Vector3d Ps[WINDOW_SIZE];
    //  滑动窗口 速度预积分
    Eigen::Vector3d Vs[WINDOW_SIZE];
    //  滑动窗口 中 对应的IMU原始数据
    std::vector<double> dt_buf[WINDOW_SIZE];
    std::vector<Eigen::Vector3d> acc_buf[WINDOW_SIZE];
    std::vector<Eigen::Vector3d> gyro_buf[WINDOW_SIZE];


    /****待优化变量****/
    //  滑动窗口中, 每个预积分量, 对应的 加速度项随机游走
    Eigen::Vector3d ba[WINDOW_SIZE];
    //  滑动窗口中, 每个预积分量, 对应的 角速度项随机游走
    Eigen::Vector3d bg[WINDOW_SIZE];


    /****边缘化参数****/
    enum class MarginalizationFlag
    {
        MARGIN_OLD,
        MARGIN_SECOND_NEW
    };
    MarginalizationFlag marginalization_flag; // 边缘化标志
    //  保存当前时刻边缘化的帧的旋转矩阵
    Eigen::Matrix3d back_R0;
    Eigen::Vector3d back_P0;
    //  统计slam系统边缘了多少老帧
    int sum_of_back;
    //  统计slam系统边缘了多少新帧
    int sum_of_new;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR        
    };
    //  当前是位于估计器初始化,还是VIO
    bool solver_flag;


    /****图像帧信息****/
    //  存储所有图像帧的容器,之所以键值用时间戳, 是因为我们没有对全局图像设置索引id
    std::map<double, ImageFrame> all_image_frame;
    //  临时预积分量类, 专门用来计算 并且 转移 所有图像帧 的预积分量
    PreIntegration *tmp_pre_integration;
};



#endif