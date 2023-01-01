#include "vertex.h"

unsigned long global_vertex_id = 0;

Vertex::Vertex
(int num_dimension, int local_dimension)
{
    //  动态Matrix声明维度, 在堆上申请内存
    parameters_.resize(num_dimension, 1);
    local_dimension_ = local_dimension>0 ? local_dimension : num_dimension;
    //  声明顶点的id
    id_ = global_vertex_id++;
}

int Vertex::LocalDimension()
{
    return local_dimension_;
}

void Vertex::RollBackParameters()
{
    parameters_ = parameters_backup_;
}
