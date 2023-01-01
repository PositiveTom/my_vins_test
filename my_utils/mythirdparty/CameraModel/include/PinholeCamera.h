#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include "mCamera.h"
#include <iostream>
#include <Eigen/Eigen>

using namespace std;

class PinholeCamera : public mCamera
{
public:
    PinholeCamera
    (double k1, double k2, double p1, double p2,
    double fx, double fy, double cx, double cy, bool m_noDistortion);

    //  把像素坐标投影到z轴为1的齐次相机坐标
    void liftProjective(Eigen::Vector2d &uv, Eigen::Vector3d &xy);

private:
    double fx_, fy_, cx_, cy_;
    //  相机的畸变系数
    double k1_, k2_, p1_, p2_;
    //  相机内参矩阵的逆对应的系数
    double m_inv_K11_, m_inv_K13_, m_inv_K22_, m_inv_K23_;
    //  是否无畸变
    bool m_noDistortion_;
};

#endif