#include "problem.h"

template<typename T>
void ShowDebug(T first)
{
    if(typeid(first) == typeid(double))
        std::cout << std::setw(12) << std::setprecision(4) << std::fixed << std::left << first << ' ';
    else 
        std::cout << std::setw(12) << std::fixed << std::left << first << ' ';
}

template<typename T, typename... Args>
void ShowDebug(T first, Args... args)
{
    if(typeid(first) == typeid(double))
        std::cout << std::setw(12) << std::setprecision(4) << std::fixed << std::left << first << ' ';
    else 
        std::cout << std::setw(12) << std::fixed << std::left << first << ' ';
    ShowDebug(args...);
}

Problem::Problem
(ProblemType problemType):problemType_(problemType)
{
    verticies_marg_.clear();
}

bool Problem::AddVertex
(std::shared_ptr<Vertex> vertex)
{
    //  [1]. 先查找顶点容器有没有这个顶点,有的话就返回
    if(verticies_.find(vertex->Id()) != verticies_.end())
    {
        //  进入这里, 表示重复添加了顶点
        return false;
    }
    //  [2]. 没有顶点的话就添加
    else
    {
        //  进入这里,表示顶点容器里面不存在这个顶点, 因此要添加这个顶点
        verticies_.insert(pair<unsigned long,shared_ptr<Vertex>>(vertex->Id(), vertex));
    }
    return true;
}

bool Problem::AddEdge(shared_ptr<Edge> edge)
{
    //  [1]. 先查找边容器内有没有这个边, 有的话就返回
    if(edges_.find(edge->Id()) != edges_.end())
        return false;
    //  [2]. 没有的话就添加
    else
        edges_.insert(pair<unsigned long, shared_ptr<Edge>>(edge->Id(), edge));
    return true;
}

void Problem::AddLambdatoHessianLM()
{
    ulong size = Hessian_.cols();
    for(ulong i=0; i<size; i++)
    {
        Hessian_(i,i) += currentLambda_;
    }
}

void Problem::RemoveLambdaHessianLM()
{
    ulong size = Hessian_.cols();
    for(ulong i=0; i<size; i++)
    {
        Hessian_(i,i) -= currentLambda_;
    }
}

bool Problem::Solve(int iterations)
{
    //  [1]. 先检查残差块和待优化变量是否都有
    if(edges_.size() == 0 || verticies_.size() == 0)
    {
        std::cerr << "\n cannot solve problem without edges or verticies" << std::endl;
        return false;
    }
    //  [2]. 计时开始
    TicToc t_solve;
    //  [3]. 统计优化变量的维数, 为构建hessian矩阵做准备
    SetOrdering();
    //  [4]. 构造hessian矩阵
    MakeHessian();
    //  [5]. LM算法初始化
    ComputeLambdaInitLM();
    //  [6]. LM算法迭代求解开始
    bool stop = false;  // 是否停止迭代
    int iter = 0;       // 迭代次数
    double last_chi_ = 1e20;
    std::cout << "********************iteration begin********************" << endl;
    for(;!stop && (iter < iterations);)
    {
        bool oneStepSuccess = false;    //  当前步是否迭代成功
        //  记录每次一次寻找增量过程中,尝试了几个阻尼因子
        int false_cnt = 0;
        //  [7].  尝试不同的阻尼因子值,直到找到合适的迭代向量
        std::cout << std::endl;
        cout << "----------------found delta x at " <<iter<<" times----------------" << endl;
        ShowDebug("scale", "curChi", "curLambda", "ni_", "result");
        for(;!oneStepSuccess;)
        {
            //  [8]. 尝试一个阻尼因子
            AddLambdatoHessianLM();
            //  [9]. 进行线性方程组的求解,得到一次的迭代步长
            SolveLinearSystem();
            //  [10]. 移除阻尼因子
            RemoveLambdaHessianLM();

            //  [11]. 更新状态变量
            UpdateStates();
            //  [12]. 判断这一次的更新是否是好的更新
            oneStepSuccess = IsGoodStepInLM();

            if(oneStepSuccess)
            {
                //  [13]. 接收本次更新,再新线性化点重新构建hessian矩阵
                MakeHessian();
                false_cnt = 0;
            }
            else
            {
                false_cnt++;
                //  [13]. 拒绝本次更新,退回到上一次的参数
                RollbackStates();
            }
        }
        iter++;
        //  [14]. 前后两次的误差变化特别小,就终止
        if(last_chi_ - currentChi_ < 1e-5)
        {
            cout << endl << "iter end!" << endl;
            stop = true;
        }
        last_chi_ = currentChi_;
    }
    std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
    std::cout << "makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
    t_hessian_cost_ = 0;
    return true;
}

void Problem::RollbackStates()
{
    for(auto &vertex:verticies_)
    {
        vertex.second->RollBackParameters();
    }
}


bool Problem::IsGoodStepInLM()
{
    //  [1]. 计算理论目标函数的下降值 L(x) - L(x+\Delta x)
    double scale = 0.0;
    scale = 0.5 * delta_x_.transpose() * (currentLambda_ * delta_x_ + b_ );
    scale += 1e-10; // 确保非零

    //  [2]. 计算F(x+\Delta x)
    double tempChi = 0.0;
    for(auto &edge:edges_)
    {
        edge.second->ComputeResidual();
        tempChi += edge.second->RobustLoss();
    }
    tempChi *= 0.5;

    //  [3]. 计算比例因子
    double rho = (currentChi_ - tempChi) / scale;
    double result = false;
    //  [4]. 阻尼因子更新策略 [Nielsen策略]
    double alpha = 0;
    double scaleFactor = 0;
    if(rho>0 && isfinite(tempChi))
    {
        alpha = 1. - pow((2 * rho - 1), 3);
        alpha = min(alpha, 2./3.); // TODO 你这里做了改动
        scaleFactor = max(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        result =  true;
    }
    else
    {
        currentLambda_ *= ni_;
        ni_ *= 2;
        result =  false;
    }
    std::cout << std::endl;
    ShowDebug(scale, currentChi_, currentLambda_, ni_, result);
    // std::cout << tempChi;
    return result;
}

void Problem::UpdateStates()
{
    for(auto &vertex: verticies_)
    {
        //  [1]. 先记录原来的参数值
        vertex.second->BackUpParameters();
        //  [2]. 得到这个顶点在 \Delta x 中的id号 和 维度
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        Vecx delta = delta_x_.segment(idx, dim);
        //  [3]. 参数更新
        vertex.second->Plus(delta);
    }
}

void Problem::SolveLinearSystem()
{
    delta_x_ = Hessian_.ldlt().solve(b_);
}

void Problem::ComputeLambdaInitLM()
{
    //  [1]. 计算迭代的终止条件1, 损失函数值要下降到原来的 1e-10
    currentChi_ = 0.0;
    for(auto &edge:edges_)
    {
        currentChi_ += edge.second->RobustLoss();
    }
    currentChi_ *= 0.5;
    stopThresholdLM_ = 1e-10 * currentChi_;

    //  [2]. 计算阻尼因子的初值, 选择hessian矩阵对角线元素的绝对值的最大值的1e-5倍当作初值
    currentLambda_ = -1;
    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    for(ulong i=0; i<size; i++)
    {
        maxDiagonal = max(fabs(Hessian_(i,i)), maxDiagonal);
    }
    maxDiagonal = min(5e10, maxDiagonal);
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;

    //  [3]. 
    ni_ = 2.;
}

void Problem::SetOrdering()
{
    //  [1]. 每次进来都重新计数
    ordering_generic_ = 0;
    //  [2]. 统计所有待优化变量的维数
    for(auto &vertic : verticies_)
    {
        ordering_generic_ += vertic.second->LocalDimension();
    }
}

void Problem::MakeHessian()
{
    TicToc t_h;
    //  [1]. 得到hessian矩阵的维数
    unsigned long size = ordering_generic_;
    //  [2]. 为hessian矩阵申请内存和初始化
    Matxx H(Matxx::Zero(size, size));
    //  [3]. 为b矩阵申请内存和初始化
    Vecx b(Vecx::Zero(size, 1));
    //  [4]. 遍历每个残差,计算雅可比,并构造hessian矩阵
    for(auto &edge : edges_)
    {
        //  [5]. 计算残差的残差和雅可比矩阵
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        //  [6]. 得到此残差中各自存放雅可比矩阵和顶点的容器
        auto jacobians = edge.second->Jacobians();
        auto vertices = edge.second->Verticies();
        assert(jacobians.size() == vertices.size());

        //  [7]. 遍历存放雅可比矩阵的容器
        for(size_t i=0; i<vertices.size(); i++)
        {
            //  [8]. 依次取出对应的雅可比矩阵和顶点, 如果顶点固定,则不用优化这个变量
            auto v_i = vertices[i];
            if(v_i->IsFixed()) continue;
            auto jacobian_i = jacobians[i];
            //  [9]. 
            unsigned long index_i = v_i->OrderingId(); // TODO: 这里的值还没有指定
            unsigned long dim_i = v_i->LocalDimension();

            //  [10]. 设置了鲁棒核,要修改对应的信息矩阵, 以及计算对应的 rho'(用于构造鲁棒残差)
            double drho;
            Matxx robustInfo(edge.second->Information().rows(), edge.second->Information().cols());
            edge.second->RobustInfo(drho, robustInfo);

            //  [11]. 计算 J^T * W
            Matxx JtW = jacobian_i.transpose() * robustInfo;

            //  [12]. 难理解点: 开始把残差块的的Hi填入到整个非线性最小二乘问题的H矩阵
            for(size_t j=0; j<vertices.size(); j++)
            {
                auto v_j = vertices[j];
                if(v_j->IsFixed()) continue;
                auto jacobian_j = jacobians[j];

                unsigned long index_j = v_j->OrderingId();
                unsigned long dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);

                //  [13]. 构造hessian矩阵
                Matxx Hi = JtW * jacobian_j;

                //  [14]. 把Hi填入到H矩阵中去
                H.block(index_i, index_j, dim_i, dim_j).noalias() += Hi;

                //  [15]. 填补对称位置
                if(i!=j)
                {
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += Hi.transpose();
                }
            }
            //  [16]. 填补b矩阵
            b.segment(index_i, dim_i).noalias() -= 
                drho * jacobian_i.transpose() * edge.second->Information() * edge.second->Residual();
        }
    }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();
}

void Problem::MakeJacoAndHessi()
{
    TicToc t_h;
    //  [1]. 得到hessian矩阵的维数
    unsigned long size = ordering_generic_;
    //  [2]. 为hessian矩阵申请内存和初始化
    Matxx H(Matxx::Zero(size, size));
    ulong residual_dims = 2;
    Matxx Jaco(Matxx::Zero(residual_dims,size));
    //  [3]. 为b矩阵申请内存和初始化
    Vecx b(Vecx::Zero(size, 1));
    //  [4]. 遍历每个残差,计算雅可比,并构造hessian矩阵
    for(auto &edge : edges_)
    {
        //  [5]. 计算残差的残差和雅可比矩阵
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        //  [6]. 得到此残差中各自存放雅可比矩阵和顶点的容器
        auto jacobians = edge.second->Jacobians();
        auto vertices = edge.second->Verticies();
        assert(jacobians.size() == vertices.size());

        //  [7]. 遍历存放雅可比矩阵的容器
        for(size_t i=0; i<vertices.size(); i++)
        {
            //  [8]. 依次取出对应的雅可比矩阵和顶点, 如果顶点固定,则不用优化这个变量
            auto v_i = vertices[i];
            if(v_i->IsFixed()) continue;
            auto jacobian_i = jacobians[i];
            //  [9]. 
            unsigned long index_i = v_i->OrderingId(); // TODO: 这里的值还没有指定
            unsigned long dim_i = v_i->LocalDimension();

            //  [10]. 设置了鲁棒核,要修改对应的信息矩阵, 以及计算对应的 rho'(用于构造鲁棒残差)
            double drho;
            Matxx robustInfo(edge.second->Information().rows(), edge.second->Information().cols());
            edge.second->RobustInfo(drho, robustInfo);

            //  填入雅可比矩阵
            Jaco.block(0, index_i, residual_dims, dim_i) += jacobian_i;

            //  [11]. 计算 J^T * W
            Matxx JtW = jacobian_i.transpose() * robustInfo;

            //  [12]. 难理解点: 开始把残差块的的Hi填入到整个非线性最小二乘问题的H矩阵
            for(size_t j=0; j<vertices.size(); j++)
            {
                auto v_j = vertices[j];
                if(v_j->IsFixed()) continue;
                auto jacobian_j = jacobians[j];

                unsigned long index_j = v_j->OrderingId();
                unsigned long dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);

                //  [13]. 构造hessian矩阵
                Matxx Hi = JtW * jacobian_j;

                //  [14]. 把Hi填入到H矩阵中去
                H.block(index_i, index_j, dim_i, dim_j).noalias() += Hi;

                //  [15]. 填补对称位置
                if(i!=j)
                {
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += Hi.transpose();
                }
            }
            //  [16]. 填补b矩阵
            b.segment(index_i, dim_i).noalias() -= 
                drho * jacobian_i.transpose() * edge.second->Information() * edge.second->Residual();
        }
    }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();
    Jaco_ = Jaco;
}

void Problem::CaculateHdl()
{
    //  [1]. 计算最速下降增量

    //  [1.1]. 计算最速下降方向
    Vecx hsd = - b_;

    //  [1.2]. 计算最速下降步长
    double hsd_norm = hsd.norm();
    Vecx J_hsd = Jaco_ * hsd;
    double J_hsd_norm = J_hsd.norm();
    double alpha = hsd_norm * hsd_norm / (J_hsd_norm * J_hsd_norm);

    hsd *= alpha;
    hsd_norm = hsd.norm();

    //  [2]. 计算高斯牛顿方向
    Vecx hgn = Hessian_.ldlt().solve(b_);

    //  [3]. 计算hdl
    double hgn_norm = hgn.norm();
    double beta = 0.;
    if(hgn_norm <= Delta_)
    {
        hdl_ = hgn;
        flag_ = 1;
    }
    else if(hsd_norm >= Delta_)
    {
        hdl_ = Delta_/hsd_norm * hsd;
        flag_ = 2;
    }
    else
    {
        Vecx a = hsd;
        Vecx b = hgn;

        double b_a_norm = (b -a).norm();
        double b_a_norm2 = b_a_norm * b_a_norm;

        double c = a.dot(b-a);

        double a_norm = a.norm();
        double a_norm2 = a_norm * a_norm;
        double a2_Delta2 = a_norm2 - Delta_ * Delta_;

        beta = ((-c + sqrt(c*c + b_a_norm2 * (-a2_Delta2)))/b_a_norm2 );

        hdl_ = a + beta * (b - a);
        flag_ = 3;
    }
    hsd_ = hsd;
    hgn_ = hgn;
    alpha_ = alpha;
    beta_ = beta;
}

bool Problem::IsGoodStepInDogLeg()
{
    double scale = 0.;

    //  [1]. 计算 L(0) - L(hdl) = scale
    Vecx g = - hsd_;
    switch(flag_)
    {
    case 1:
        scale = currentChi_;
        break;
    case 2:
        scale = Delta_ * (2 * hsd_.norm() - Delta_ ) / (2*alpha_);
        break;
    case 3:
        double g_norm = g.norm();
        scale = 0.5 * alpha_ * ( 1 - beta_ ) * ( 1 - beta_ ) * g_norm * g_norm
                + beta_ * ( 2 - beta_ ) * currentChi_;
        break;
    // default:
    //     break;
    }

    //  [2]. 计算F(x+hdl)
    double tempChi_ = 0.;
    for(auto &edge:edges_)
    {
        edge.second->ComputeResidual();
        tempChi_ += edge.second->RobustLoss();
    }
    tempChi_ *= 0.5;
    //  563229.51772594824
    //  [3]. 计算比例因子 rho
    double rho = ( currentChi_ - tempChi_ ) / scale;

    double result = false;
    if(rho < 0 )
    {
        result = false;
    }
    else
    {
        //  [4]. 更新信赖域
        if(rho < 0.25)
        {
            Delta_ /= 2;
        }
        else if ( rho > 0.75)
        {
            Delta_ = max(Delta_, 3 * hdl_.norm() );
        }
        result = true;
        currentChi_ = tempChi_;
    }
    //        ShowDebug("scale", "curChi", "beta", "Delta", "flag", "result");
    std::cout << std::endl;
    ShowDebug(scale, currentChi_, beta_, Delta_, flag_, result);
    return result;
}

void Problem::DogLegInit()
{
    for(auto &edge:edges_)
    {
        currentChi_ += edge.second->RobustLoss();
    }
    currentChi_ *= 0.5;
    
    //  信赖域半径初始化
    Delta_ = 10.0;
}

bool Problem::SolveWithDogLeg(int iterations)
{
    //  [1]. 先检查残差块和待优化变量是否都有
    if(edges_.size() == 0 || verticies_.size() == 0)
    {
        std::cerr << "\n cannot solve problem without edges or verticies" << std::endl;
        return false;
    }
    //  [2]. 计时开始
    TicToc t_solve;
    //  [3]. 统计优化变量的维数, 为构建hessian矩阵做准备
    SetOrdering();
    //  [4]. 计算 hessian矩阵
    MakeJacoAndHessi();
    //  [5]. DogLeg初始化
    DogLegInit();

    //  [6]. 开始迭代
    double last_chi_ = 1e20;
    bool stop = false;
    ulong iter = 0;
    ulong false_cnt = 0;
    std::cout << "********************DogLeg iteration begin**************" << endl;
    for(;(!stop) && (iter<iterations); iter++)
    {
        //  当前步是否迭代成功
        bool oneStepSuccess = false;
        std::cout << std::endl;
        cout << "----------------found delta x at " <<iter<<" times----------------" << endl;
        ShowDebug("scale", "curChi", "beta", "Delta", "flag", "result");
        for(;!oneStepSuccess;)
        {   
            //  [7]. 计算hdl
            CaculateHdl();

            //  [8]. 更新状态量
            // cout << hdl_ << endl;
            delta_x_ = hdl_;
            UpdateStates();

            //  [9]. 判断更新是否合理
            oneStepSuccess = IsGoodStepInDogLeg();
            if(oneStepSuccess)
            {
                //  [10]. 如果合理,则接收本次更新, 并在新线性化点构建hessian矩阵 和 jaco矩阵
                MakeJacoAndHessi();
            }
            else
            {
                //  [10]. 如果不合理,则回滚
                RollbackStates();
            }
            false_cnt++;
        }
        if(last_chi_ - currentChi_ < 1e-5)
        {
            cout << endl << "iter end!" << endl;
            stop = true;
        }
        last_chi_ = currentChi_;

    }
    std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
    std::cout << "makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
    t_hessian_cost_ = 0;
    return true;
}