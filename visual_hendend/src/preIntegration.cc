#include "preIntegration.h"

static Eigen::Matrix3d AntiSkewMatrix(const Eigen::Vector3d &p)
{
    Eigen::Matrix3d pMatrix;
    pMatrix << 0, -p(2), p(1),
              p(2), 0, -p(0), 
            -p(1), p(0), 0;
    return pMatrix;
}

PreIntegration::PreIntegration
(const Eigen::Vector3d &acc0,  const Eigen::Vector3d &gyro0,
 const Eigen::Vector3d &ba,    const Eigen::Vector3d &bg):
 linearized_acc_(acc0), linearized_gyro_{gyro0}  // TODO const类型的特殊初始化
{
    //  [1]. 给预积分量的初始加速度, bias随机游走项初始化
    acc0_ = acc0;
    gyro0_ = gyro0;
    ba_ = ba;
    bg_ = bg;
    sum_dt = 0.;

    //  [2]. 预积分量初始化, 即 alpha_bi_bi_ = 0, beta_bi_bi_ = 0, q_bi_bi = [1 0 0 0]
    alpha_bi_bj_.setZero();
    beta_bi_bj_.setZero();
    q_bi_bj_.setIdentity();
    
    //  [3]. IMU的内参模型参数, 用于计算预积分量, 以及预积分量的置信度
    n_a_ = N_A;
    n_g_ = N_G;
    b_n_a_ = B_N_A;
    b_n_g_ = B_N_G;

    //  [4]. 给高斯随机向量协方差矩阵赋值
    y_var_.block<3,3>(0,0) = (n_a_ * n_a_) * Eigen::Matrix3d::Identity(); // n_k_a_
    y_var_.block<3,3>(3,3) = (n_g_ * n_g_) * Eigen::Matrix3d::Identity(); // n_k_g_
    y_var_.block<3,3>(6,6) = (n_a_ * n_a_) * Eigen::Matrix3d::Identity(); // n_{k+1}_a_
    y_var_.block<3,3>(9,9) = (n_g_ * n_g_) * Eigen::Matrix3d::Identity(); // n_{k+1}_g_
    y_var_.block<3,3>(12,12) = (b_n_a_ * b_n_a_) * Eigen::Matrix3d::Identity(); // n_b_a_
    y_var_.block<3,3>(15,15) = (b_n_g_ * b_n_g_) * Eigen::Matrix3d::Identity(); // n_b_g_

    //  [5]. 给计算预积分量残差的递推式的相关量初始化   
    x_covariance_.setZero();
    F_.setIdentity();
}

void PreIntegration::Integral
(double dt, const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyro1)
{
    //  [1]. 添加原始imu数据, 因为一开始的bias是不准的, 后续优化了之后需要重新积分
    dt_buf.push_back(dt);
    acc_buf.push_back(acc1);
    gyro_buf.push_back(gyro1);

    //  [2]. 运行中值积分 (这里可以用其他的积分类型)
    Eigen::Vector3d result_alpha_;
    Eigen::Vector3d result_beta_;
    Eigen::Quaterniond result_q_;
    Eigen::Vector3d result_ba_;
    Eigen::Vector3d result_bg_;
    midPropagation
    (dt, acc0_, gyro0_, acc1, gyro1, alpha_bi_bj_, beta_bi_bj_, q_bi_bj_,
     ba_, bg_, result_alpha_, result_beta_, result_q_, result_ba_, result_bg_);

    //  [3]. 更新预积分量
    alpha_bi_bj_ = result_alpha_;
    beta_bi_bj_ = result_beta_;
    q_bi_bj_ = result_q_;
    ba_ = result_ba_;
    bg_ = result_bg_;

    //  [4]. 注意旋转预积分量进行归一化
    q_bi_bj_.normalize();
    sum_dt += dt;

    //  [5]. 初始加速度和角速度置换,为下一次积分作准备
    acc0_ = acc1;
    gyro0_ = gyro1;
}   

void PreIntegration::midPropagation
(   double dt, const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyro0,
    const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyro1,
    const Eigen::Vector3d &alpha, const Eigen::Vector3d &beta,
    const Eigen::Quaterniond &q, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg,
    Eigen::Vector3d &result_alpha, Eigen::Vector3d &result_beta,
    Eigen::Quaterniond &result_q, Eigen::Vector3d &result_ba, Eigen::Vector3d &result_bg)
{
    //  [1]. 计算 \bar{w}
    Eigen::Vector3d gyro_mean_0 = 0.5 * ( gyro0 - bg + gyro1 - bg );
    //  [2]. 计算 q^{b_i}_{b_{k+1}} = q^{b_i}_{b_k} * temp_q; 旋转预积分量递推公式
    Eigen::Quaterniond temp_q(1, 0.5 * gyro_mean_0(0) * dt, 0.5 * gyro_mean_0(1) * dt, 0.5 * gyro_mean_0(2) * dt);
    result_q =  q * temp_q;
    //  [3]. 计算 \bar{a} = \frac{1}{2} (q^{b_i}_{b_k} (a^{b_k} - b_{b_k}^a) + q^{b_i}_{b_{k+1}} (a^{b_{k+1}} - b_{b_{k+1}}^a) )
    Eigen::Vector3d acc_mean_0 = 0.5 * ( q * (acc0 - ba ) + result_q * (acc1 - ba) );
    //  [4]. 计算 \beta^{b_i}_{b_{k+1}} = \beta^{b_i}_{b_k} + \bar{a} dt 
    result_beta = beta + acc_mean_0 * dt;
    //  [5]. 计算 \alpha^{b_i}_{b_{k+1}}
    result_alpha = alpha + beta * dt + 0.5 * acc_mean_0 * dt * dt;
    result_ba = ba;
    result_bg = bg;

    //  [6]. 更新 计算置信度的G矩阵和F矩阵
    Eigen::MatrixXd G_temp = Eigen::Matrix<double, 15, 15>::Zero();
    /****\delta \theta^{b_i}_{b_{k+1}}****/
    G_temp.block<3,3>(3,3) = Eigen::Matrix3d::Identity() - AntiSkewMatrix(gyro_mean_0) * dt; // \delta \theta^{b_i}_{b_k}
    G_temp.block<3,3>(3,12) = -1.0 * Eigen::Matrix3d::Identity() * dt; // \delta bg

    /****\delta \alpha^{b_i}_{b_{k+1}}****/
    G_temp.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3); // \delta \alpha^{b_i}_{b_k}
    Eigen::Vector3d ak = acc0 - ba;
    Eigen::Vector3d ak_plus_1 = acc1 - ba;
    G_temp.block<3,3>(0,3) = -0.25 * 
    (q * AntiSkewMatrix(ak) + result_q * AntiSkewMatrix(ak_plus_1) * 
    ( Eigen::Matrix3d::Identity() - AntiSkewMatrix(gyro_mean_0) * dt ) ) * dt * dt; // \delta \theta^{b_i}_{b_k}
    G_temp.block<3,3>(0,6) = Eigen::Matrix3d::Identity() * dt; // \delta \beta^{b_i}_{b_k}
    G_temp.block<3,3>(0,9) = -0.25 * ( q.toRotationMatrix() + result_q.toRotationMatrix() ) * dt * dt;  // \delta ba
    G_temp.block<3,3>(0,12) = 0.25 * result_q.toRotationMatrix() * AntiSkewMatrix(ak_plus_1) * dt * dt * dt; // \delta bg

    /****\delta \beta^{b_i}_{b_{k+1}}****/
    G_temp.block<3,3>(6,3) = -0.5 * 
    ( q * AntiSkewMatrix(ak) + result_q * AntiSkewMatrix(ak_plus_1) * 
    ( Eigen::Matrix3d::Identity() - AntiSkewMatrix(gyro_mean_0) * dt ) ) * dt; // \delta \theta
    G_temp.block<3,3>(6,6) = Eigen::Matrix3d::Identity(); // \delta \beta
    G_temp.block<3,3>(6,9) = -0.5 * ( q.toRotationMatrix() + result_q.toRotationMatrix() ) * dt; // \delta ba
    G_temp.block<3,3>(6,12) = 0.5 * result_q.toRotationMatrix() * AntiSkewMatrix(ak_plus_1) * dt * dt;

    /****\delta ba****/
    G_temp.block<3,3>(9,9) = Eigen::Matrix3d::Identity();

    /****\delta bg****/
    G_temp.block<3,3>(12,12) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd F_temp = Eigen::Matrix<double, 15, 18>::Zero();
    /****\delta \alpha****/
    F_temp.block<3,3>(0,0) = 0.25 * q.toRotationMatrix() * dt * dt;
    F_temp.block<3,3>(0,3) = -0.125 * result_q.toRotationMatrix() * AntiSkewMatrix(ak_plus_1) * dt * dt * dt;
    F_temp.block<3,3>(0,6) = 0.25 * result_q.toRotationMatrix() * dt * dt;
    F_temp.block<3,3>(0,9) = F_temp.block<3,3>(0,3);

    /****\delta \theta****/
    F_temp.block<3,3>(3,3) = 0.5 * Eigen::Matrix3d::Identity() * dt;
    F_temp.block<3,3>(3,9) = 0.5 * Eigen::Matrix3d::Identity() * dt;

    /****\delta \beta****/
    F_temp.block<3,3>(6,0) = 0.5 * q.toRotationMatrix() * dt;
    F_temp.block<3,3>(6,3) = -0.25 * result_q.toRotationMatrix() * AntiSkewMatrix(ak_plus_1) * dt * dt;
    F_temp.block<3,3>(6,6) = 0.5 * result_q.toRotationMatrix() * dt;
    F_temp.block<3,3>(6,9) = F_temp.block<3,3>(6,3);

    /****\delta ba****/
    F_temp.block<3,3>(9,12) = Eigen::Matrix3d::Identity() * dt;

    /****\delta bg****/
    F_temp.block<3,3>(12,15) = Eigen::Matrix3d::Identity() * dt;

    //  [7]. 递推计算测量残差协方差矩阵
    x_covariance_ = G_temp * x_covariance_ * G_temp.transpose() + F_temp * y_var_ * F_temp.transpose();

    //  [8]. 计算雅可比矩阵
    F_ = G_temp * F_;
}
