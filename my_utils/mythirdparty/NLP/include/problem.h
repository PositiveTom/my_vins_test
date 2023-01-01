#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <map>
#include <memory>
#include "vertex.h"
#include "edge.h"
#include "tic_toc.h"
#include <iomanip>
#include <typeinfo>

using namespace std;

typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::map<unsigned long, std::shared_ptr<Edge>> HashMap;

//  顶点的含义等于优化变量
//  边的含义等于残差
class Problem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum class ProblemType
    {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };

    enum class OptimizeType
    {
        LM,
        DogLeg
    };


    Problem(ProblemType problemType);
    //  添加顶点
    bool AddVertex(std::shared_ptr<Vertex> vertex);
    //  添加边
    bool AddEdge(shared_ptr<Edge> edge);
    /**
     * @brief 问题求解
     * @param iterations 算法的求解迭代次数
     */
    bool Solve(int iterations);
    //  使用DogLeg进行求解
    bool SolveWithDogLeg(int iterations);

private:
    //  统计优化变量的维数
    void SetOrdering();
    //  构造海森矩阵
    void MakeHessian();
    //  LM算法初始化
    void ComputeLambdaInitLM();
    //  为hessian矩阵添加阻尼因子
    void AddLambdatoHessianLM();
    //  移出阻尼因子,为尝试下一个阻尼因子作准备, 因为每一次求解的线性方程组增量不一定可行,因此要去除得到下一次
    void RemoveLambdaHessianLM();
    //  求解线性方程组 Hx=b, //TODO: 牵涉到求解Ax=b的方法原理以及优劣总结
    void SolveLinearSystem();
    //  更新状态变量
    void UpdateStates();
    //  判断这一次的更新是否是好的更新
    bool IsGoodStepInLM();
    //  把参数退回到上一次的参数
    void RollbackStates();
    //  狗腿法初始化
    void DogLegInit();
    //  计算雅可比和hessian矩阵
    void MakeJacoAndHessi();
    //  计算增量
    void CaculateHdl();
    //  判断hdl_是否是好的并且更新信赖域半径
    bool IsGoodStepInDogLeg();

private:
    //  表示此问题的类型
    ProblemType problemType_;
    //  存储边缘化的顶点的容器
    HashVertex verticies_marg_;
    //  存储所有的顶点的容器
    HashVertex verticies_;
    //  存储所有边的容器
    HashMap edges_;
    //  所有待优化变量的维数
    unsigned long ordering_generic_;
    //  hessian矩阵
    Matxx Hessian_;
    //  b矩阵
    Vecx b_;
    //  求解Hx=b得到的增量
    Vecx delta_x_;
    //  求解hessian矩阵花费的时间
    double t_hessian_cost_ = 0.0;
    //  当前优化问题的损失函数值
    double currentChi_;
    //  LM算法退出的终止条件之一, 目标函数下降的数值
    double stopThresholdLM_;
    //  LM算法阻尼因子的初值
    double currentLambda_;
    //  Nilsen策略里面的 v    void DogLegInit();
    double ni_;
    //  jaco矩阵
    Matxx Jaco_;

    //  Dogleg 参数
    //  信赖域半径
    double Delta_; 
    //  Dogleg增量
    Vecx hdl_;
    //  hdl_选的方向
    ulong flag_;
    //
    Vecx hgn_, hsd_;
    //
    double alpha_, beta_;
};


#endif