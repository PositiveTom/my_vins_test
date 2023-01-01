#ifndef VERTEX_H
#define VERTEX_H

#include <iostream>
#include "eigen_types.h"
#include <string>

class Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //  explicit 禁止隐式类型转换, 即禁止这种用法: Vertex mvertex = 6;
    /**
     * @param num_dimension 顶点自身的维度
     * @param local_dimension 实际维度
     */ 
    explicit Vertex(int num_dimension, int local_dimension = -1);

    // virtual ~Vertex();

    //  返回顶点的名称, 这个=0决定了这个类是抽象类,派生类必须得实现这个类
    virtual std::string TypeInfo() const = 0;

    //  设置顶点的参数值
    void SetParameters(const Vecx& param){parameters_ = param;}
    //  返回顶点对应的id号
    unsigned long Id()const {return id_;}
    //  返回顶点的维数
    int LocalDimension();
    //  设置顶点的状态(固定与否)
    void SetFixed(bool fixed = true)
    {
        fixed_ = fixed;
    }
    //  返回顶点目前的状态(是否固定)
    bool IsFixed(){return fixed_;}
    //  返回此顶点的Id
    unsigned long OrderingId(){return ordering_id_;}
    //  保存顶点的参数值
    void BackUpParameters(){parameters_backup_ = parameters_;}
    //  顶点参数值更新
    void Plus(const Vecx &delta){parameters_ += delta;}
    //  参数回退
    void RollBackParameters();
    //  在problem中的id
    void SetOrderingId(int id) { ordering_id_ = id; };
    //  返回参数值
    Vecx Parameters(){return parameters_;}
private:
    //  待优化变量的取值
    Vecx parameters_;
    //  上一次的优化变量的值
    Vecx parameters_backup_;
    //  顶点的维数(待优化变量的维数)
    int local_dimension_;
    //  顶点的id
    unsigned long id_;
    //  顶点是否固定
    bool fixed_ = false;
    //  problem中排序后的id, 用于寻找对应的雅可比块,带有维度信息
    unsigned long ordering_id_ = 0;
};


#endif