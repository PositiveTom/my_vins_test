#ifndef IMAGE_FRAME_H
#define IMAGE_FRAME_H

/****管理图像帧的类****/
//Note: 图像帧不一定都是关键帧, 但是每个图像帧必然有对应的预积分量

#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Eigen>
#include "preIntegration.h"

using namespace std;

class ImageFrame
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageFrame(
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& image, double t
    )
    {
        features = image;   
        t_ = t;

        is_keyframe = false;
        if(pre_integration != nullptr)
            pre_integration = nullptr;
    }

public:
    //  图像帧上的特征点信息
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> features;
    //  图像帧的时间戳
    double t_;
    //  是否是关键帧
    bool is_keyframe;
    //  对应的预积分量指针
    PreIntegration *pre_integration;

};


#endif