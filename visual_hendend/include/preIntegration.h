#ifndef PREINTEGRATION_H
#define PREINTEGRATION_H

/****预积分类****/
#include <Eigen/Eigen>
#include <parameters.h>
#include <vector>

using namespace std;

class PreIntegration
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PreIntegration() = delete; //   禁用默认构造函数

    /**
     * @brief 预积分量初始化
     * @param acc0 预积分量的初始加速度, 基于body系
     * @param gyro0 预积分量的初始角速度
     * @param ba 预积分量的 加速度随机游走项, 也是一个实时优化更新的参数
     * @param bg 预积分量的 角速度随机游走项, 也是一个实时优化更新的参数
     */ 
    PreIntegration(const Eigen::Vector3d &acc0,  
                   const Eigen::Vector3d &gyro0,
                   const Eigen::Vector3d &ba,
                   const Eigen::Vector3d &bg);
    //  预积分function
    void Integral(double dt, const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyro1);
    //  中值积分
    void midPropagation
    (double dt, const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyro0,
     const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyro1,
     const Eigen::Vector3d &alpha, const Eigen::Vector3d &beta,
     const Eigen::Quaterniond &q, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg,
     Eigen::Vector3d &result_alpha, Eigen::Vector3d &result_beta,
     Eigen::Quaterniond &result_q, Eigen::Vector3d &result_ba, Eigen::Vector3d &result_bg);

public:
    //  预积分量的初始加速度
    Eigen::Vector3d acc0_;
    //  预积分量的初始角速度
    Eigen::Vector3d gyro0_;
    //  预积分量对应的加速度随机游走项
    Eigen::Vector3d ba_;
    //  预积分量对应的角速度随机游走项
    Eigen::Vector3d bg_;
    //  此预积分量中, 第1帧imu数据的加速度数据, 也就是下文原始imu数据的acc0, 这里是单独保存的
    const Eigen::Vector3d linearized_acc_;
    //  此预积分量中, 第1帧imu数据的陀螺仪数据
    const Eigen::Vector3d linearized_gyro_;
    //  计算预积分量时的时间
    double sum_dt;

    /****预积分量****/
    Eigen::Vector3d alpha_bi_bj_;   // 位置预积分量
    Eigen::Vector3d beta_bi_bj_;    // 速度预积分量
    Eigen::Quaterniond q_bi_bj_;    // 旋转预积分量

    /*
     * 计算预积分量残差的测量置信度所需要的量(通过递推表达式进行计算)
     *      X_{k+1} = G_k * X_k + F_k * Y
     *          ...
     *      X_{j} = G * X_{i} + F * Y
     */
    double n_a_;    //  加速度计随机噪声标准差  m/s^2  !!!注意这里的单位
    double n_g_;    //  陀螺仪随机噪声标准差    rad/s
    double b_n_a_;  //  加速度随机游走项的导数的标准差  m/s^2
    double b_n_g_;  //  陀螺仪随机游走项的导数的标准差  rad/s
    Eigen::Matrix<double, 18, 18> y_var_; //  高斯随机向量Y的协方差矩阵
    Eigen::Matrix<double, 15, 15> x_covariance_; //  imu测量残差协方差矩阵
    Eigen::Matrix<double, 15, 15> G_; // 满足 X_{j} = G * X_{i} + F * Y 的G矩阵, 也可以称为雅可比矩阵, 只不过这里的雅可比只用作计算置信度
    Eigen::Matrix<double, 15, 18> F_; // 满足 X_{j} = G * X_{i} + F * Y 的F矩阵, 也可以称为对高斯白噪声的雅可比矩阵, 只不过这里的雅可比只用作计算置信度

    /****存储imu的原始数据****/
    /****acc0 acc1 acc2 acc3 acc4 ...****/
    /****   dt1  dt2  dt3  dt4 ....****/
    std::vector<Eigen::Vector3d> acc_buf; // 预积分内部的加速度原始数据, 存储如上所示的acc1, acc2, acc3 ..., acc0给acc0_了
    std::vector<Eigen::Vector3d> gyro_buf; // 陀螺仪原始数据
    std::vector<double> dt_buf; // 积分时间间隔原始数据
};



#endif