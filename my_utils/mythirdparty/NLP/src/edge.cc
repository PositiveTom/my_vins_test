#include "edge.h"

unsigned long global_edge_id = 0;

Edge::Edge
(int residual_dimension, int num_verticals,
const vector<string> &verticies_name)
{
    //  [1]. 初始化残差维度以及申请内存
    residuals_.resize(residual_dimension, 1);
    //  [2]. 初始化包含顶点的名称
    if(!verticies_name.empty())
        verticies_types_ = verticies_name;
    //  [3]. 初始存储雅可比矩阵的容器
    jacobians_.resize(num_verticals);
    //  [3]. 初始化残差id
    id_ = global_edge_id++;
    //  [4]. 初始化信息矩阵维度以及申请内存
    information_.resize(residual_dimension, residual_dimension);
    information_.setIdentity();
    //  [5]. 初始化损失函数类指针为空
    lossfunction_ = nullptr;
}

double Edge::LossItem() const
{
    return residuals_.transpose() * information_ * residuals_;
}

double Edge::RobustLoss() const
{
    double e2 = LossItem();
    if(lossfunction_)
    {
        Eigen::Vector3d rho;
        lossfunction_->Compute(e2, rho);
        return rho[0];
    }
    return e2;
}

void Edge::RobustInfo(double &drho, Matxx &info) const
{
    //  [1]. 如果损失函数类存在
    if(lossfunction_)
    {
        //  [2]. 计算 损失项s
        double e2 = LossItem();
        Eigen::Vector3d rho;
        //  [3]. 计算 rho(s), rho', rho''
        lossfunction_->Compute(e2, rho);
        //  [4]. 计算带权重的误差
        Vecx weight_err = sqrt_information_ * residuals_;
        //  [5]. 构造鲁棒信息矩阵
        Matxx robust_info(information_.rows(), information_.cols());
        robust_info.setIdentity();
        robust_info *= rho[1];
        //  TODO 这里为了保证鲁棒损失函数的值为正
        if(rho[1] + 2 * rho[2] * e2 > 0.)
        {
            robust_info += 2 * rho[2] * weight_err * weight_err.transpose();
        }
        info = sqrt_information_.transpose() * robust_info * sqrt_information_;
        drho = rho[1];
    }
    else
    {
        drho = 1.0;
        info = information_;
    }
}