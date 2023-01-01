#ifndef EDGE_H
#define EDGE_H

#include <iostream>
#include <vector>
#include "eigen_types.h"
#include "vertex.h"
#include <string>
#include <memory>
#include "loss_function.h"

using namespace std;

//  残差 也可以称为 边
class Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //  使用explict禁止隐式类型转换
    /**
     * @param residual_dimension 残差的维度
     * @param num_verticals 顶点的数量
     * @param verticls_name 顶点的类型名
     */ 
    explicit Edge(int residual_dimension, int num_verticals,
                  const vector<string> &verticls_name = vector<string>());

    // virtual ~Edge();

    //  计算残差, 申明为纯虚函数,决定了这个类为抽象类
    virtual void ComputeResidual() = 0;
    //  计算雅可比, 同上
    virtual void ComputeJacobians() = 0;
    //  返回边的类型, 同上的意义
    virtual std::string TypeInfo() const = 0;
    //  为边设置顶点
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices)
    {
        vertices_ =  vertices;
        return true;
    }
    //  返回边的id
    unsigned long Id(){return id_;}
    //  返回雅可比矩阵
    std::vector<Matxx> Jacobians() const { return jacobians_; }
    //  返回顶点
    std::vector<std::shared_ptr<Vertex>> Verticies(){return vertices_;};
    //  返回信息矩阵
    Matxx Information()const{return information_;}
    //  设置带鲁棒核的信息矩阵
    void RobustInfo(double &drho, Matxx &info) const;
    //  计算 f^T W f 损失项, 用于计算 rho(s) 中的s
    double LossItem() const;
    //  计算 rho(s)
    double RobustLoss() const;
    //  设置信息矩阵 和 其1/2的信息矩阵
    void SetInformation(const Matxx &information)
    {
        information_ = information;
        //  使用LLT分解 cholesky分解
        sqrt_information_ = Eigen::LLT<Matxx>(information_).matrixL().transpose();
    }
    //  返回残差
    Vecx Residual()const {return residuals_;}

    //  TODO: 这个函数有问题???
    ulong ResidualDim() const { return residuals_.rows();}

protected:
    //  残差
    Vecx residuals_;
    //  顶点类型,用于debug
    vector<string> verticies_types_;
    //  残差块对应的雅可比集, 每个雅可比的维度是: 残差维数 x 顶点维数
    vector<Matxx> jacobians_; 
    //  残差id
    unsigned long id_;
    //  信息矩阵
    Matxx information_;
    //  信息矩阵的一半, 用LLT分解替代
    Matxx sqrt_information_;
    //  残差对应的顶点, 容器存放顶点类的共享指针
    vector<shared_ptr<Vertex>> vertices_;
    //  指向损失函数类的指针
    LossFunction *lossfunction_;

};

#endif