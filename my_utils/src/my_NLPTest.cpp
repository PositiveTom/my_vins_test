#include "problem.h"
#include <Eigen/Core>
#include <iostream>
#include <random>
#include <vector>
#include <string>
#include "edge.h"
#include "vertex.h"

using namespace std;

class CurveFittingVertex:public Vertex
{
public:
    CurveFittingVertex(): Vertex(3) {}
    virtual std::string TypeInfo() const{return "abc";}
};

class CurveFittingEdge:public Edge
{
public:
    CurveFittingEdge(double x, double y): Edge(1, 1, vector<string>{"abc"})
    {
        x_ = x;
        y_ = y;
    }
    virtual void ComputeResidual() override
    {
        Vecx abc = vertices_[0]->Parameters();
        residuals_(0) = std::exp(abc(0) * x_ * x_ + abc(1) * x_ + abc(2)) - y_;
    }

    virtual void ComputeJacobians() override
    {
        Vecx abc = vertices_[0]->Parameters();
        double val = abc(0) * x_ * x_ + abc(1) * x_ + abc(2);
        Eigen::Matrix<double, 1, 3> jaco;
        jaco << exp(val) * x_ * x_, exp(val) * x_, exp(val);
        jacobians_[0] = jaco;
    }

    virtual std::string TypeInfo() const
    {
        return "CurveFitting Edge";
    }

private:
    double x_;
    double y_;
};

int main()
{
    //  16 个字节 = 64 位
    // cout << EIGEN_MAX_ALIGN_BYTES << endl;

    //  [1]. 产生测量值
    double a = 2., b = 3., c = 1.;
    int N = 200;
    double sigma = 1.;
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0., sigma);
    vector<double> x(N, 0);
    vector<double> y_mse(N, 0);
    for(int i=0; i<N; i++)
    {
        x[i] = -1. + 0.01 * i;
        y_mse[i] = std::exp(a*x[i]*x[i] + b*x[i] + c) + noise(generator);
    }

    //  [2]. 创建问题
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);

    //  [3]. 创建顶点并添加顶点
    Eigen::Vector3d abc(0, 0, 0);
    std::shared_ptr<CurveFittingVertex> abcvertex = std::make_shared<CurveFittingVertex>();
    abcvertex->SetParameters(abc);
    abcvertex->SetOrderingId(0);
    problem.AddVertex(abcvertex);
    
    //  [4]. 添加残差
    for(int i=0; i<x.size(); i++)
    {
        std::shared_ptr<CurveFittingEdge> fitEdge(new CurveFittingEdge(x[i], y_mse[i]));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(abcvertex);
        fitEdge->SetVertex(edge_vertex);
        problem.AddEdge(fitEdge);
    }

    // problem.Solve(30);
    problem.SolveWithDogLeg(30);
    cout << "abc:" << endl << abcvertex->Parameters().transpose() << endl;

    return 0;
}
