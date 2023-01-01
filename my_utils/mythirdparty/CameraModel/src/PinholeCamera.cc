#include "PinholeCamera.h"

PinholeCamera::PinholeCamera
(double k1, double k2, double p1, double p2,
 double fx, double fy, double cx, double cy, bool m_noDistortion)
{
    m_noDistortion_ = m_noDistortion;

    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;

    m_inv_K11_ = 1. / fx_;
    m_inv_K22_ = 1. / fy_;
    m_inv_K13_ = - cx_ / fx_;
    m_inv_K23_ = - cy_ / fy_;

    k1_ = k1;
    k2_ = k2;
    p1_ = p1;
    p2_ = p2;
}

void PinholeCamera::liftProjective(Eigen::Vector2d &uv, Eigen::Vector3d &xy)
{
    //  有畸变的像素坐标
    double mx_d = 0., my_d = 0.;

    //  [1]. 先把像素坐标投影到图像坐标
    mx_d = m_inv_K11_ * uv[0] + m_inv_K13_;
    my_d = m_inv_K22_ * uv[1] + m_inv_K23_;

    //  [2]. 查看是否图像有畸变
    if(m_noDistortion_)
    {
        //  [3] 如果没有畸变
        xy << mx_d, my_d, 1.;
        return;
    }
    else
    {
        //  [3] 如果有畸变, 通过不动点迭代方程去畸变

        //  [4]. 给不动点方程赋初值
        //  无畸变图像坐标  
        double un_mx_d = mx_d, un_my_d = my_d;
        //  无畸变图像坐标的模长平方
        double r2 = 0., x2 = 0., y2 = 0., r4 = 0.;
        //  迭代8次的畸变模型
        int n = 8;
        
        for(int i=1; i<n ;i++)
        {   
            //  [5]. 计算模长平方量
            x2 = un_mx_d * un_mx_d;
            y2 = un_my_d * un_my_d;
            r2 = x2 + y2;
            r4 = r2 * r2;

            //  [6]. 迭代
            un_mx_d = mx_d - (k1_ * r2 + k2_ * r4) * un_mx_d - 2 * p1_ * un_my_d - p2_ * (r2 + 2 * x2);
            un_my_d = my_d - (k1_ * r2 + k2_ * r4) * un_my_d - 2 * p1_ * un_mx_d - p2_ * (r2 + 2 * y2);
        }
        xy << un_mx_d, un_my_d, 1.;
        return;
    }
}