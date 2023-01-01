
// my zhang calibration
#include "my_calibration.h"
#include "NLP.h"

//  Hyperparameters 超参数
bool debug = true;

//  TODO Note: 一旦通过线性最小二乘SVD求解，就一定会丢掉一个尺度因子!!!

my_calibr::my_calibr(ros::NodeHandle& n)
{
    n.getParam("/my_calibration/CALIBRATION_IMAGE_PATH", m_CALIBRATION_IMAGE_PATH);
    n.getParam("/my_calibration/points_per_row", m_points_per_row);
    n.getParam("/my_calibration/points_per_col", m_points_per_col);
    n.getParam("/my_calibration/square_size", m_square_size);
    // ROS_INFO("points_per_row:%d", points_per_row);
    k0 = 0;
    k1 = 0;
    readPhotosPath(m_CALIBRATION_IMAGE_PATH);
}

void showstr(std::string& data )
{
    std::cout << data << std::endl;
}

//  reference: https://blog.csdn.net/qq_40250056/article/details/114832692#:~:text=Linux%E7%B3%BB%E7%BB%9F%20C%2FC%2B%2B%E8%8E%B7%E5%8F%96%E5%BD%93%E5%89%8D%E6%96%87%E4%BB%B6%E5%A4%B9%E8%B7%AF%E5%BE%84%E5%92%8C%E6%96%87%E4%BB%B6%E5%90%8D%201%20%E4%B8%80%E3%80%81%E6%96%87%E4%BB%B6%E5%A4%B9%E8%B7%AF%E5%BE%84%E8%8E%B7%E5%8F%96%202%201.1%20%E5%A4%B4%E6%96%87%E4%BB%B6%201.2,2.3%20%E6%89%93%E5%BC%80%E6%96%87%E4%BB%B6%E7%9B%AE%E5%BD%95opendir%202.4%20%E8%AF%BB%E5%8F%96%E6%96%87%E4%BB%B6%E4%BF%A1%E6%81%AF%202.4%20%E5%85%B3%E9%97%AD%E6%96%87%E4%BB%B6%E7%9B%AE%E5%BD%95%205%20%E4%B8%89%E3%80%81%E5%85%B6%E4%BB%96%E5%85%B3%E4%BA%8E%E7%9B%AE%E5%BD%95%E6%93%8D%E4%BD%9C%E7%9A%84%E5%87%BD%E6%95%B0
void my_calibr::readPhotosPath(std::string& imgsPath)
{
    std::vector<int> numset;

    //  1. 打开目录流 <sys/types.h>
    DIR* dirStream;
    dirStream = opendir(imgsPath.c_str());

    //  2. 读取文件信息 <dirent.h>
    struct dirent* dirInfo;
    while( (dirInfo = readdir(dirStream)) != 0)
    {
        // 文件类型: https://www.cnblogs.com/frisk/p/11508205.html
        if(static_cast<int>(dirInfo->d_type) == 8)
        {
            // std::cout << "file: " <<imgsPath << "/"  << dirInfo->d_name << std::endl;
            // std::cout << static_cast<int>(dirInfo->d_type) << std::endl;
            std::string num_str(dirInfo->d_name);
            int index = num_str.find(".");
            num_str.erase(index);
            std::istringstream ss(num_str);
            int num_int;
            ss >> num_int;
            // ROS_INFO("%d", num_int);
            numset.push_back(num_int);

            m_files_path.push_back(imgsPath+"/"+dirInfo->d_name);
        }
    }
    //  3. 关闭目录流
    closedir(dirStream);
    
    //  4. 对文件进行排序
    std::vector<std::string> temp;
    temp.resize(m_files_path.size());
    int j = 0;
    for(auto& i: numset)
    {
        temp[i-1] = m_files_path[j++];
    }
    m_files_path.clear();
    m_files_path = temp;
    std::for_each(m_files_path.begin(), m_files_path.end(), showstr);
}

//  棋盘角点检测
void my_calibr::cornerDectection()
{        
    if( m_files_path.empty() && (m_points_per_row == m_points_per_col) )
    {
        ROS_ERROR("There are no photos in Set");
        ros::shutdown();
        return;
    }
    //  TODO
    //  世界坐标，从行开始分配, 搞了大半天， 
    //  原来世界坐标分配错误了， 果然，只要有一个细节没有弄懂，一定会出错
    //  不是你错了，而是c++的opencv检测角点顺序不同于python
    //  也不是这里的问题，而是 findChessboardCorners 决定了顺序
    //  findChessboardCorners 先保证 cv::Size(m_points_per_col, m_points_per_row)
    //  每一行有 m_points_per_col 个特征点

    wordlCordinate XY1;
    XY1.reserve(m_points_per_col*m_points_per_row);
    for( int i=0; i<m_points_per_col; i++)
    {
        for(int j=0; j<m_points_per_row; j++)
        {
            cv::Point3d w(j*m_square_size, i*m_square_size, 1);
            XY1.push_back(w);
        }
    }

    int i = 0;
    for(auto &imgPath: m_files_path)
    {

        //  [1]. 读取图片
        cv::Mat image = cv::imread(imgPath, cv::COLOR_BGR2GRAY);

        //  [2]. 角点检测, 内部自动收缩了内存
        std::vector<cv::Point2f> corners;
        bool ret = cv::findChessboardCorners
                (image, cv::Size(m_points_per_row, m_points_per_col), corners);
        if(false) // TODO
        {
            myShowPhoto("photo1", image, 0);
        }

        if(ret)
        {
            std::vector<cv::Point2f> subcorners(corners);
            //  [3].使得像素达到亚像素级别 TODO: 亚像素检测出来竟然可能更加差
            cv::TermCriteria criteria = cv::TermCriteria(
                cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER,
                30,
                0.001
            );
            cv::cornerSubPix(image, subcorners, cv::Size(11,11), cv::Size(-1,-1), criteria);

            // [4]. 为角点分配对应的世界坐标
            mypair cors(subcorners, XY1);
            m_corre[i++] = cors;
            if(false) //TODO
            {
                cv::Mat image1 = image.clone();
                cv::Mat image1_color;
                cv::cvtColor(image1, image1_color, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners
                    (image1_color, cv::Size(m_points_per_row, m_points_per_col), corners, ret);
                
                std::string text = 
                "(" + std::to_string(int(XY1[1].x)) + "," +std::to_string(int(XY1[1].y)) + ")";
                cv::putText(image1_color, text, corners[1], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 0) );
                myShowPhoto("image1_color", image1_color, 0);

                // cv::Mat image2 = image.clone();
                // cv::Mat image2_color;
                // cv::cvtColor(image2, image2_color, cv::COLOR_GRAY2BGR);
                // cv::drawChessboardCorners
                //     (image2_color, cv::Size(m_points_per_row, m_points_per_col), subcorners, ret);
                // myShowPhoto("image2_color", image2_color, 0);                
            }

        }
        // ROS_INFO("%d", i);
    }
}

//  计算单应矩阵的初值
void my_calibr::caculateHomoInit()
{
    // ROS_INFO("m_corre:%d", m_corre.size());
    if(m_corre.size() < 3) // TODO 至少需要3张
    {
        ROS_ERROR("The photos is not enough!");
        ros::shutdown();
        return;
    }

    std::map< int, mypair > m_corre_copy = m_corre;

    cv::namedWindow("test", cv::WINDOW_NORMAL);
    for(int i=0; i<m_corre_copy.size(); i++ )
    {
        //  [1]. 依次取出每张图片对应的 2d-3d 点集合
        mypair points = m_corre_copy[i];
        Eigen::Matrix<double, 3, 3> H, NuInverse, Nx;

        //  [2]. 计算归一化矩阵 同时 计算归一化坐标, 用于提升数值稳定性
        caculateNormalizedMatrix(points, NuInverse, Nx);
        if(false)
        {
            std::cout << "Nu:" << NuInverse << std::endl;
            std::cout << "Nx:" << Nx << std::endl;
            std::cout << "***************" << std::endl;
            cv::waitKey(0);
        }

        Eigen::Matrix3d H_;
        //  [3]. 利用线性最小二乘SVD计算伪单应性矩阵, 属于DLT方法
        caculateHWithSVD(points, H_);

        //  [4]. 计算反归一化之前的单应矩阵
        H = NuInverse * H_ * Nx;
        if(false)
        {
            std::cout << "H_:" << H_ << std::endl;
            std::cout << "H:" << H << std::endl;
        }
        H = H / H(2,2);
        homoSets.push_back(H);
    }
    homoSets.swap(homoSets);
}

//  虽然写出来了，但是存在重大的问题！！！
void my_calibr::caculateHWithSVD(my_calibr::mypair& points, Eigen::Matrix3d& H_)
{
    Eigen::Matrix<double, Eigen::Dynamic, 9> A;
    A.resize(points.first.size()*2, 9);
    A.setZero();

    pixelCordinate::iterator pixelptr = points.first.begin();
    wordlCordinate::iterator worldptr = points.second.begin();
    int row = 0;
    for(; pixelptr!=points.first.end() ;pixelptr++, worldptr++, row+=2)
    {
        Eigen::Matrix<double, 1, 9> row1, row2;
        row1 << -worldptr->x, -worldptr->y, -1, 
            0, 0, 0, pixelptr->x*worldptr->x, pixelptr->x*worldptr->y, pixelptr->x;

        row2 << 0, 0, 0, -worldptr->x, -worldptr->y, -1,
            pixelptr->y*worldptr->x, pixelptr->y*worldptr->y, pixelptr->y;

        A.block<1,9>(row, 0) = row1;
        A.block<1,9>(row+1,0) = row2;
        // A += row1.transpose() * row1;
        // A += row2.transpose() * row2;
        if(false)
        {   
            // ROS_INFO("hello");
            std::cout << A << std::endl;
            cv::waitKey(0);
        }
    }
    // std::cout << A << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 1> Vright = svd.matrixV().rightCols(1);

    if(false)
    {
        std::cout << "Vright:" << Vright << std::endl;
        std::cout << "H_:" << svd.singularValues() << std::endl;
        // std::cout << Vright << std::endl;
    }
    
    H_ << Vright[0], Vright[1], Vright[2], Vright[3], Vright[4], Vright[5], Vright[6],
            Vright[7], Vright[8];

    // Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> 
    if(false)
    {
        std::cout << H_ << std::endl;
        std::abort();
        cv::waitKey(0);
    }
}

//  计算归一化矩阵 的 同时计算归一化坐标
void my_calibr::caculateNormalizedMatrix
(my_calibr::mypair& points, Eigen::Matrix3d& NuInverse, Eigen::Matrix3d& Nx)
{
    double su, sv, sX, sY;
    double sigmau2, sigmav2, sigmaX2,sigmaY2;
    int N = points.first.size();
    double meanu, meanX, meanv, meanY;

    pixelCordinate pixelpoints = points.first;
    wordlCordinate worldpoints = points.second;

    //  计算均值
    for(int i=0; i<N; i++)
    {
        meanu += 1.0 / double(N) * pixelpoints[i].x;
        meanv += 1.0 / double(N) * pixelpoints[i].y;

        meanX += 1.0 / double(N) * worldpoints[i].x;
        meanY += 1.0 / double(N) * worldpoints[i].y;
    }

    //  计算方差
    for(int i=0; i<N; i++ )
    {
        sigmau2 += 1.0 / double(N) * pow(meanu - pixelpoints[i].x ,2) ;
        sigmav2 += 1.0 / double(N) * pow(meanv - pixelpoints[i].y ,2) ;

        sigmaX2 += 1.0 / double(N) * pow(meanX - worldpoints[i].x ,2) ; 
        sigmaY2 += 1.0 / double(N) * pow(meanY - worldpoints[i].y ,2) ;        
    }
    if(false)
    {
        cv::namedWindow("sigma");
        ROS_INFO("sigmaX2:%lf", sigmaX2);
        cv::waitKey(0);
    }

    pixelCordinate::iterator pixelptr = points.first.begin();
    wordlCordinate::iterator worldptr = points.second.begin();
    const double num = sqrt(2);
    // TODO: 阿西吧，这里写错了，错把 sigmav2 写成了 sigmau2
    double sigmau = sqrt(sigmau2), sigmav = sqrt(sigmav2);
    double sigmaX = sqrt(sigmaX2), sigmaY = sqrt(sigmaY2);
    for(int i=0; i<N; i++)
    {
        pixelptr->x = (pixelptr->x - meanu ) / sigmau * num;
        pixelptr->y = (pixelptr->y - meanv ) / sigmav * num;

        worldptr->x = (worldptr->x - meanX ) / sigmaX * num;
        worldptr->y = (worldptr->y - meanY ) / sigmaY * num;

        pixelptr++;
        worldptr++;
    }
    su = sqrt(2.0 / sigmau2 );
    sv = sqrt(2.0 / sigmav2);
    sX = sqrt(2.0 / sigmaX2);
    sY = sqrt(2.0 / sigmaY2);

    // ROS_INFO("%lf", meanu);
    // ROS_INFO("%lf", meanv);
    // ROS_INFO("%lf", meanX);
    // ROS_INFO("%lf", meanY);
    // NuInverse << su, 0, -su*meanu, 0, sv, -sv*meanv, 0, 0, 1;
    NuInverse << 1.0/su, 0, meanu, 0, 1.0/sv, meanv, 0, 0, 1;
    Nx << sX, 0, -sX*meanX, 0, sY, -sY*meanY, 0, 0, 1;
}

void my_calibr::myShowPhoto(const char* name, cv::Mat& image, int time)
{
    cv::namedWindow(std::string(name), cv::WINDOW_NORMAL);
    cv::imshow(name, image);
    cv::waitKey(time); 
}

bool my_calibr::nonlinearOptimizeH() // TODO: failed
{   
    //  [1]. 实例化Problem对象
    ceres::Problem problem;

    //  [1.5]. 添加鲁棒核函数
    ceres::LossFunction* myloss = new ceres::HuberLoss(0.5);

    //  [2]. 创建参数块
    if(homoSets.size() < 3)
    {
        ROS_ERROR("There is no enough homo_matrix");
        ros::shutdown();
    }
    //  内存--地址(内存)--地址
    double param[homoSets.size()][9];

    for(int i=0; i<homoSets.size(); i++)
    {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> H_ = homoSets[i];
        if(false)
        {
            std::cout << "H_:" << H_ << std::endl;
            std::cout << "homoSets[i]:" << homoSets[i] << std::endl;
            double *p1 = H_.data();
            double *p2 = homoSets[i].data();
            std::cout << "*(p1+1)" << *(p1+1) <<  p1[2] << std::endl;
            std::cout << "*(p2+1)" << *(p2+1) << std::endl;
            cv::waitKey(0);
        }
        double *p = H_.data();
        for(int j=0; j<9; j++ )
        {
            param[i][j] = *(p+j);
        }
        problem.AddParameterBlock((*(param+i)), 9);
    }
    if(false)
    {
        // wrong
        std::cout << "*(p1)" << *(*(param+1)) << std::endl;
        std::cout << "*(p1+1)" <<*(*(param+1)+1) << std::endl;
        std::cout << "*(p1+1)" <<*(*(param+2)+1) << std::endl;
    }

    //  [3]. 依次创建残差块
    for(int i=0; i<homoSets.size(); i++ )
    {
        mypair data = m_corre[i];
        pixelCordinate::iterator pielptr = data.first.begin();
        wordlCordinate::iterator worldptr = data.second.begin();
        for(int j=0 ; pielptr!=data.first.end(); pielptr++, worldptr++, j++ )
        {
            //  [3.1] 创建代价函数
            ceres::CostFunction * myfunction = 
                new myCostFunctor(worldptr->x, worldptr->y, pielptr->x, pielptr->y);

            if(false)
            {
                double residuals[2];
                double jacobians[1][18];
                myCostFunctor* mcost = new myCostFunctor(worldptr->x, worldptr->y, pielptr->x, pielptr->y);
                double **m_param = new double *[1];
                m_param[0] = *(param+i);
                mcost->check(m_param, residuals, jacobians);
                // ros::shutdown();
                if(j == 53 && i == 2)
                {
                    std::abort();
                }
            }
            //  [3.2] 添加残差块
            problem.AddResidualBlock(myfunction, NULL, (*(param+i)));
        }
    }
    //  [4]. 创建求解器设置器 TODO: 如果更细致理解这个求解器？
    ceres::Solver::Options option;
    option.max_num_iterations = 30;
    option.minimizer_type = ceres::TRUST_REGION;
    option.trust_region_strategy_type = ceres::DOGLEG;
    // option.dogleg_type =ceres::SUBSPACE_DOGLEG;
    // option.linear_solver_type = ceres::DENSE_QR;
    // option.preconditioner_type = ceres::JACOBI;
    option.minimizer_progress_to_stdout = true;
    // option.minimizer_type = ceres::LINE_SEARCH;
    
    std::string error;
    if(!option.IsValid(&error))
    {
        ROS_ERROR("option isvalid");
        std::abort();
    }
    //  [5]. 创建求解器
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    
    ROS_INFO("final_cost:%lf", summary.final_cost);
    ROS_INFO("initial_cost:%lf", summary.final_cost);

    if(summary.final_cost < summary.initial_cost)
        return true;
    else
        return false;
}

//  要么 CostFunction 没有评估和填充所有请求的残差和雅可比值，要么在雅可比计算期间生成了一个非有限值（nan/无限）
//  对于每个参数块，参数的值打印在第一列中，雅可比值打印在相应的残差下。如果参数块保持不变，则相应的雅可比矩阵被打印为“未计算”。
//  如果请求了雅可比/残差数组的条目，但未被用户代码写入，则由“未初始化”指示。这是一个错误。残差或雅可比值评估
//  到 Inf 或 NaN 也是一个错误
// Parameter Block 0, size: 9

//     0.856131 |           -0 Uninitialized 
//      2.46043 |           -0 Uninitialized 
//     -301.145 |     -1.45784 Uninitialized 
//      2.91292 | Uninitialized           -0 
//      0.52387 | Uninitialized           -0 
//     -283.454 | Uninitialized     -1.45784 
//   0.00322438 |           -0           -0 
// -0.000224314 |           -0           -0 
//    -0.685946 |      -2.1253      -2.1253
//  上面这种问题是由于jacobi矩阵没有完全初始化

//  W1129 10:42:35.106639  9340 trust_region_minimizer.cc:729] Step failed to evaluate. Treating it as a step with infinite cost
//  通常还会遇到上面这种问题

// homo1:    2.68279    0.201296     243.964
//   -0.207887     3.36459     92.0397
// -0.00137782 0.000498025           1
// Ceres Solver Report: Iterations: 3, Initial cost: 2.115809e+01, Final cost: 2.066528e+01, Termination: CONVERGENCE
// homo2:    2.70629    0.209921     243.686
//   -0.199012     3.37641     91.7754
// -0.00133286 0.000521512    0.999685

class HomoVertex :public Vertex
{
public:
    HomoVertex() : Vertex(9) {}
    virtual std::string TypeInfo() const {return "homo";}
};

class HomoEdge :public Edge
{
public:
    HomoEdge(double u, double v, double X, double Y):Edge(2, 1, std::vector<std::string>{"HomoEdge"})
    {
        u_ = u;
        v_ = v;
        X_ = X;
        Y_ = Y;
    }
    virtual void ComputeResidual()
    {
        Vecx h = vertices_[0]->Parameters();
        double su = h(0) * X_ + h(1) * Y_ + h(2);
        double sv = h(3) * X_ + h(4) * Y_ + h(5);
        double s = h(6) * X_ + h(7) * Y_ + h(8);

        residuals_(0) = su / s - u_;
        residuals_(1) = sv / s - v_;
    }
    virtual void ComputeJacobians()
    {
        Vecx h = vertices_[0]->Parameters();
        double su = h(0) * X_ + h(1) * Y_ + h(2);
        double sv = h(3) * X_ + h(4) * Y_ + h(5);
        double s = h(6) * X_ + h(7) * Y_ + h(8);

        Eigen::Matrix<double, 2, 9> jaco;
        jaco.setZero();
        jaco << X_/s, Y_/s, 1./s, 0., 0., 0., -su*X_/(s*s), -su*Y_/(s*s), -su/(s*s),
                0., 0., 0., X_/s, Y_/s, 1./s, -sv*X_/(s*s), -sv*Y_/(s*s), -sv/(s*s); 
        jacobians_[0] = jaco;
    }
    virtual std::string TypeInfo() const
    {
        return "homoEdge";
    }
private:
    double u_, v_, X_, Y_;
};


void my_calibr::myNLPoptimized()
{
    for(int i=0; i<homoSets.size(); i++)
    {
        //  [1]. 创建问题
        Problem problem(Problem::ProblemType::GENERIC_PROBLEM);

        //  [2]. 创建顶点以及设置顶点初值(带优化变量)
        std::shared_ptr<HomoVertex> homovertex(new HomoVertex());
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> homo = homoSets[i];
        Eigen::Map<Eigen::Matrix<double, 9, 1>> param(homo.data());
        homovertex->SetParameters(param);

        //  [3]. 添加顶点
        problem.AddVertex(homovertex);

        mypair *points = &m_corre[i];
        for(int j=0; j<points->first.size(); j++)
        {
            //  [4]. 创建残差以及设置残差的观测值和输入
            std::shared_ptr<HomoEdge> homoedge = std::make_shared<HomoEdge>
                (points->first[j].x, points->first[j].y, points->second[j].x, points->second[j].y);

            //  [5]. 残差添加顶点
            std::vector<std::shared_ptr<Vertex>> vec;
            vec.push_back(homovertex);
            homoedge->SetVertex(vec);

            //  [6]. 添加残差
            problem.AddEdge(homoedge);
        }
        // cout << "original:" << homovertex->Parameters().transpose() << endl;
        problem.Solve(30);
        homoSets[i] = homo;
        // cout << "optimized:" << homovertex->Parameters().transpose() << endl;
    }
}



void my_calibr::optimizeAutoH()
{
    ceres::Solver::Options option;
    option.max_num_iterations = 100;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;

    for(int i=0; i<homoSets.size(); i++)
    {
        ceres::Problem problem;
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> homo = homoSets[i];
        double *param = homo.data();
        problem.AddParameterBlock(param, 9);
        mypair *points = &m_corre[i];
        for(int j=0; j<points->first.size(); j++ )
        {
            ceres::CostFunction *mycost = new ceres::AutoDiffCostFunction<myAutoCost, 2, 9>
                    (new myAutoCost(points->second[j].x, points->second[j].y, points->first[j].x,points->first[j].y));
            problem.AddResidualBlock(mycost, NULL, param);
        }
        ceres::Solver::Summary summary;
        // std::cout << "homo1:" << homo << std::endl;
        ceres::Solve(option, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        homoSets[i] = homo;
        // std::cout << "homo2:" << homo << std::endl;
    }
}


//  TODO: 我猜测是雅可比求错了！！！！！果真是雅可比求错了, 但是验证的雅可比没有错
void my_calibr::optimizedH()
{
    // ceres::LossFunction *loss = new ceres::CauchyLoss(1.0);
    // ceres::CostFunction* mycostfunction = new myCostFunctor
    ceres::Solver::Options option;
    option.max_num_iterations = 30;  
    // option.minimizer_type = ceres::TRUST_REGION;
    option.trust_region_strategy_type = ceres::DOGLEG;
    option.linear_solver_type = ceres::DENSE_QR;
    // option.line_search_direction_type = ceres::STEEPEST_DESCENT;
    // option.line_search_type = ceres::ARMIJO;
    option.minimizer_progress_to_stdout = true;

    for(int i=0; i<homoSets.size(); i++)
    {
        ceres::Problem problem;

        Eigen::Matrix<double,3,3,Eigen::RowMajor> homo = homoSets[i];
        double *param = homo.data();

        problem.AddParameterBlock(param, 9);

        mypair *points = &m_corre[i];
        for(int j=0; j<(*points).first.size();j++)
        {
            double X = points->second[j].x;
            double Y = points->second[j].y;
            double u = points->first[j].x;
            double v = points->first[j].y;
            ceres::CostFunction* mycost =  new myCostFunctor(X, Y, u, v);
            problem.AddResidualBlock(mycost, NULL, param);
            if(false)
            {
                myCostFunctor * my = new myCostFunctor(X, Y, u, v);
                double jacobians[1][18];
                double residuals[2];
                double **m_param = new double *[1];
                m_param[0] = param;
                my->check(m_param, residuals, jacobians);
            }
        }
        ceres::Solver::Summary summary;
        ceres::Solve(option, &problem, &summary);
        // std::cout << "homo:" << homo << std::endl;
        // ROS_INFO("final_cost:%lf", summary.final_cost);
        // ROS_INFO("initial_cost:%lf", summary.initial_cost);
        // std::cout << "homo:" << homo << std::endl;
        std::cout << summary.BriefReport() << std::endl;
    }
}

void my_calibr::checkHomo()
{
    std::ofstream ofs("/home/zjj/MyCode/myVinsPrj/src/my_utils/txt/myHomo.txt", std::ios::out);

    int i = 0;
    for(auto& homo: homoSets)
    {
        ofs << i << " original_uv " << " uvhat " << std::endl;
        pixelCordinate* piexlptr = &m_corre[i].first;
        wordlCordinate* worldptr = &m_corre[i].second;
        
        std::vector<cv::Point3d>::iterator itworld = (*worldptr).begin();
        std::vector<cv::Point2f>::iterator itpixel = (*piexlptr).begin();
        int j = 0;
        for(; itworld != (*worldptr).end() ; itworld++, itpixel++)
        {
            Eigen::Vector3d XY;
            XY << itworld->x, itworld->y, itworld->z;

            Eigen::Vector3d uv;
            uv << itpixel->x, itpixel->y, 1;

            Eigen::Vector3d s_uv_hat;
            s_uv_hat = homo * XY;
            // s_uv_hat /= s_uv_hat(2);
            s_uv_hat(0) = s_uv_hat(0) / s_uv_hat(2);
            s_uv_hat(1) = s_uv_hat(1) / s_uv_hat(2);
            s_uv_hat(2) = s_uv_hat(2) / s_uv_hat(2);

            ofs << j++ << " " << "(" << int(uv(0)) << "," << int(uv(1)) << "," << int(uv(2)) << "),"
                << "(" << int(s_uv_hat(0)) << "," << int(s_uv_hat(1)) << "," << s_uv_hat(2) << ")" << std::endl; 
        }
        i++;
    }
    ofs.close();
}

//  求解内参矩阵
void my_calibr::solveInertialMatrix()
{
    if(homoSets.size()<3)
    {
        ROS_ERROR("There is no enough photo!");
        std::abort();
    }

    // [1]. 求解B矩阵
    Eigen::Matrix<double, 6, 6> HB;
    HB.setZero();
    std::vector<Eigen::Matrix3d>::iterator it = homoSets.begin();
    int i = 0;
    for(; i<homoSets.size(); i++, it++ )
    {
        Eigen::Matrix<double, 1, 6> HB1, HB2;
        HB1 << (*it)(0,0)*(*it)(0,1), (*it)(0,0)*(*it)(1,1)+(*it)(1,0)*(*it)(0,1), 
               (*it)(1,0)*(*it)(1,1), (*it)(0,0)*(*it)(2,1)+(*it)(2,0)*(*it)(0,1), 
               (*it)(1,0)*(*it)(2,1)+(*it)(2,0)*(*it)(1,1), (*it)(2,0)*(*it)(2,1);
        //  TODO 真没想到，这里也打错了两个数字。。。
        HB2 << (*it)(0,0)*(*it)(0,0)-(*it)(0,1)*(*it)(0,1), 2*((*it)(0,0)*(*it)(1,0)-(*it)(0,1)*(*it)(1,1)),
              (*it)(1,0)*(*it)(1,0)-(*it)(1,1)*(*it)(1,1), 2*((*it)(0,0)*(*it)(2,0)-(*it)(0,1)*(*it)(2,1)),
             2*((*it)(1,0)*(*it)(2,0)-(*it)(1,1)*(*it)(2,1)), (*it)(2,0)*(*it)(2,0)-(*it)(2,1)*(*it)(2,1);
        // Eigen::Matrix3d unitOne = Eigen::Matrix3d::Identity();
        if(false)
        {
            std::cout << (*it) << std::endl;
            //  (row, col)  第row行，第col列的元素 // TODO 吃了不熟悉eigen访问元素的亏
            std::cout << (*it)(0,0) << std::endl;
            std::cout << (*it)(0,1) << std::endl;
            // std::cout << (*it)(1,0) << std::endl;
            // std::cout << (*it)(1,1) << std::endl;
            // std::cout << (*it)(2,0) << std::endl;
            // std::cout << (*it)(2,1) << std::endl;
            // std::cout << *(it+1) << std::endl;
        }
        // HB.block<1,6>(i*2, 0) = HB1;
        // HB.block<1,6>(i*2+1, 0) = HB2;
        HB += HB1.transpose() * HB1;
        HB += HB2.transpose() * HB2;
    }

    if(false)
    {
        ROS_INFO("%d", i);
        std::cout << "HB:" << HB << std::endl;
    }
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(HB, Eigen::ComputeFullV);
    if(false)
    {
        std::cout << "D:" << svd.singularValues() << std::endl;
        std::cout << "V:" << svd.matrixV() << std::endl;
    }
    Eigen::Matrix<double, 6, 1> lastV = svd.matrixV().rightCols(1);
    Eigen::Matrix<double, 3, 3> B;
    B.setZero();
    B << lastV(0), lastV(1), lastV(3),
         lastV(1), lastV(2), lastV(4),
         lastV(3), lastV(4), lastV(5);

    if(false)
    {
        std::cout << lastV << std::endl;
        std::cout << B << std::endl;
    }

    //  [2]. 对B进行cholesky分解得到内参初值
    double alpha, beta, lamda, gamma, u0, v0;
    double w, d; 

    w = lastV(0)*lastV(2)*lastV(5)-lastV(1)*lastV(1)*lastV(5)-lastV(0)*lastV(4)*lastV(4)
        + 2*lastV(1)*lastV(3)*lastV(4) - lastV(2)*lastV(3)*lastV(3);
    d = lastV(2)*lastV(0) - lastV(1)*lastV(1);

    lamda = w / d;
    alpha = sqrt(lamda / lastV(0) );
    beta = sqrt(lamda * lastV(0) / d );
    gamma = - lastV(1) * sqrt(w / ( lastV(0) * d * d));
    v0 = (lastV(1)*lastV(3)-lastV(4)*lastV(0))/d;
    u0 = (lastV(4)*lastV(1)-lastV(2)*lastV(3))/d;

    // TODO: gamma, 这个gamma是切向畸变的体现？？？
    K << alpha, 0, u0, 0, beta, v0, 0 , 0, 1;
    orignalK << alpha, gamma, u0, 0, beta, v0, 0 , 0, 1;
    if(true)
    {
        std::cout << K << std::endl;
    }
}

void my_calibr::caculateExternalMatrix()
{
    //  本质是PnP问题，但是并这里不通过PnP方法求解，因为PnP求解的矩阵不一定满足旋转矩阵的性质，还需要近似？？？
    //  如果通过PnP，如何近似呢？
    for(auto &homo: homoSets )
    {
        //  [1]. 计算尺度因子 \kappa
        double kappa;
        Eigen::Vector3d h1 = homo.col(0);
        Eigen::Vector3d h2 = homo.col(1);
        Eigen::Vector3d h3 = homo.col(2);
        //  TODO norm返回的是矩阵的范数而不是行列式
        kappa = 1.0 / (K.inverse() * h1).norm();
        
        //  [2]. 计算R1 和 R2
        Eigen::Vector3d R1 = kappa * K.inverse() * h1;
        Eigen::Vector3d R2 = kappa * K.inverse() * h2;

        //  [3]. 计算R3
        Eigen::Vector3d R3 = R1.cross(R2);

        //  [4]. 组成旋转矩阵并且使得其满足旋转矩阵的性质
        Eigen::Matrix3d R;
        R << R1, R2, R3;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU|Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();

        //  [5]. 计算外参的t
        Eigen::Vector3d t;
        t = K.inverse() * kappa * h3;

        Eigen::Matrix<double, 3, 4> T;
        T << R, t;
        externalMatrixSets.push_back(T);

        if(false)
        {
            // std::cout << R << std::endl;
            // std::cout << R.determinant() << std::endl;
            // std::cout << svd.matrixU().determinant() << std::endl;
            // std::cout << svd.matrixV().determinant() << std::endl;
            // std::cout << R1 << std::endl;
            // std::cout << R2 << std::endl;
            // std::cout << K.inverse() << std::endl;
            // std::cout << h1 << std::endl;
            // std::cout << kappa << std::endl;
            // //  很明显不正交
            // std::cout << h1.dot(h2) << std::endl; 
            // std::cout << h1.norm() << std::endl;
            // std::cout << h2.norm() << std::endl;
            std::abort();
        }
    }
    // externalMatrixSets.swap(externalMatrixSets);
}

//  DLT, P3P, EPnP....
void my_calibr::caculateExternalMatrixPnP()
{
    
}

void my_calibr::caculateDistortionFactor()
{
    //  [1]. 把世界坐标投影到像素坐标
    pixelCordinate pixelptr;
    pixelptr.reserve( m_corre.size() * m_corre[0].first.size() );
    for(int i=0; i<m_corre.size(); i++)
    {
        mypair *ptr = &m_corre[i];
        wordlCordinate *worldptr = &ptr->second;
        Eigen::Matrix<double, 3, 4> externalMatrix = externalMatrixSets[i];
        for(auto &worldcor: *worldptr )
        {
            Eigen::Vector4d pw;
            pw << worldcor.x, worldcor.y, 0, 1;

            Eigen::Vector3d pc = K * externalMatrix * pw;
            pc /= pc(2);
            cv::Point2f p(pc(0), pc(1));
            pixelptr.push_back(p);
        }
    }   

    double u0 = K(0,2), v0 = K(1,2), alpha = K(0,0), beta = K(1,1);

    //  [2]. 构建方程组
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    // Eigen::Matrix<double, Eigen::Dynamic, 2> A;
    // Eigen::Matrix<double,Eigen::Dynamic,1> b;
    // A.resize(m_corre.size() * m_corre[0].first.size() *2, 2);
    // b.resize(m_corre.size() * m_corre[0].first.size() *2, 1);
    b.setZero();
    A.setZero();
    pixelCordinate::iterator bar_it = pixelptr.begin();
    for(int i=0; i<m_corre.size(); i++ )
    {
        mypair *ptr = &m_corre[i];
        pixelCordinate::iterator hat_it = ptr->first.begin();
        for(int j=0 ;hat_it!=ptr->first.end() ; bar_it++, hat_it++ , j++)
        {
            Eigen::Matrix2d subA;
            subA.setZero();
            // double r2 = bar_it->x*bar_it->x + bar_it->y*bar_it->y;
            // double r4 = r2*r2;
            double x = (bar_it->x-u0)/alpha;
            double y = (bar_it->y-v0)/beta;
            double r2 = x*x+y*y;
            double r4 = r2*r2;
            double subA11 = (bar_it->x-u0)*r2;
            double subA12 = (bar_it->x-u0)*r4;
            double subA21 = (bar_it->y-v0)*r2;
            double subA22 = (bar_it->y-v0)*r4;
            subA << subA11, subA12, subA21, subA22;

            Eigen::Vector2d subB;
            subB.setZero();
            double subB1 = hat_it->x-bar_it->x;
            double subB2 = hat_it->y-bar_it->y;
            subB << subB1, subB2;

            A += subA.transpose()*subA;
            b += subA.transpose()*subB;
        }
    }

    //  TODO: 你被SVD求解非齐次方程给坑了！！！
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU|Eigen::ComputeFullV);
    //  TODO 终究还是这句代码比较容易出错
    Eigen::VectorXd x = svd.solve(b);
    // Eigen::Vector2d x = svd.solve(b);
    // // Eigen::Vector2d x = A.jacobiSvd().solve(b);
    // Eigen::MatrixXd UT = svd.matrixU().leftCols(2).transpose();
    // Eigen::MatrixXd Dinv = svd.singularValues().asDiagonal().inverse();
    // Eigen::MatrixXd V = svd.matrixV();
    // Eigen::VectorXd x = V * Dinv * UT * b;

    k0 = x(0);
    k1 = x(1);
    if(true)
    {
        // std::cout << "A:" << A << std::endl;
        // std::cout << "b:" << b << std::endl;
        std::cout << x << std::endl;
        // std::cout << "k0:" << k0 << std::endl;
        // std::cout << "k1:" << k1 << std::endl;
    }
}

void my_calibr::optimizeReprojection()
{   
    /*
     //  debug
    double u = m_corre[0].first[10].x;
    double v = m_corre[0].first[10].y;
    double Xw = m_corre[0].second[10].x;
    double Yw = m_corre[0].second[10].y;

    reprojectCost *myfunctor = new reprojectCost(u,v,Xw,Yw);
    double parameters[3][7] = {0};
    // double** parameters = new double[3][7];
    parameters[0][0] = orignalK(0,0);
    parameters[0][1] = orignalK(1,1);
    parameters[0][2] = orignalK(0,1);
    parameters[0][3] = orignalK(0,2);
    parameters[0][4] = orignalK(1,2);
    parameters[1][0] = k0;
    parameters[1][1] = k1;

    Eigen::Vector3d Pcw = externalMatrixSets[0].rightCols(1);
    Eigen::Matrix3d Rcw = externalMatrixSets[0].leftCols(3);
    Eigen::Quaterniond qcw(Rcw);
    parameters[2][0] = Pcw.x();
    parameters[2][1] = Pcw.y();
    parameters[2][2] = Pcw.z();
    parameters[2][3] = qcw.x();
    parameters[2][4] = qcw.y();
    parameters[2][5] = qcw.z();
    parameters[2][6] = qcw.w();

    // double** jacobians = new double[3][7];
    double jacobians[3][14] = {0};
    myfunctor->debug(parameters, jacobians);
    cout << "test" << endl;
    */
   
    //  [1] 
    ceres::Problem problem;

    //  [2]. 添加参数块
    double internal_params[5] = {orignalK(0,0), orignalK(1,1), orignalK(0,1), orignalK(0,2), orignalK(1,2)};
    double distorted_params[2] = {k0, k1};
    double **external_params = new double*[externalMatrixSets.size()];
    problem.AddParameterBlock(internal_params, 5);
    problem.AddParameterBlock(distorted_params, 2);
    for(int i=0; i<externalMatrixSets.size(); i++)
    {
        Eigen::Matrix<double, 3, 4> T = externalMatrixSets[i];
        Eigen::Matrix3d Rcw = T.leftCols(3);
        Eigen::Vector3d Pcw = T.rightCols(1);
        Eigen::Quaterniond Qcw(Rcw);
        external_params[i] = new double[7];
        external_params[i][0] = Pcw[0];
        external_params[i][1] = Pcw[1];
        external_params[i][2] = Pcw[2];
        external_params[i][3] = Qcw.x();
        external_params[i][4] = Qcw.y();
        external_params[i][5] = Qcw.z();
        external_params[i][6] = Qcw.w();
        ceres::LocalParameterization *mylocl = new mypose();
        problem.AddParameterBlock(external_params[i], 7, mylocl);
    }

    //  [3]. 添加残差块
    for(int i=0; i<externalMatrixSets.size(); i++)
    {
        pixelCordinate uvpoints = m_corre[i].first;
        wordlCordinate pwpoints = m_corre[i].second;
        for(int j=0; j<uvpoints.size(); j++)
        {
            double u = uvpoints[j].x;
            double v = uvpoints[j].y;
            double Xw = pwpoints[j].x;
            double Yw = pwpoints[j].y;

            //  [4]. 创建残差块
            ceres::CostFunction *mycost = new reprojectCost(u, v, Xw, Yw);
            //  [5]. 添加残差块
            problem.AddResidualBlock(mycost, NULL, internal_params, distorted_params, external_params[i]);
        }
    }
    //  [6]. 设置求解器参数
    ceres::Solver::Options option;
    option.max_num_iterations = 50;
    option.minimizer_type = ceres::TRUST_REGION;
    option.minimizer_progress_to_stdout = true;
    option.trust_region_strategy_type = ceres::DOGLEG;
    option.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    cout << summary.BriefReport() << endl;
    cout << distorted_params[0] << endl;
    cout << distorted_params[1] << endl;
}

void my_calibr::testRepro()
{
    Eigen::Matrix<double, 3, 4> T = externalMatrixSets[0];
    Eigen::Matrix3d Rcw = T.leftCols(3);
    Eigen::Quaterniond qcw(Rcw);
    cout << "Rcw:" << Rcw << endl;
    cout << "qcw:" << qcw.coeffs() << endl;
    cout << "qcw.w():" << qcw.w() << endl;

    pixelCordinate pixel = m_corre[0].first;
    wordlCordinate world = m_corre[0].second;

    Eigen::Matrix<double, 4, 1> Pw(world[10].x, world[10].y, 0, 1);
    Eigen::Vector2d uv(pixel[10].x, pixel[10].y );

    Eigen::Vector3d Pc = T * Pw;
    cout << "T:" << T << endl;
    cout << "Pc:" << Pc << endl;
    Eigen::Vector3d xyPc = Pc / Pc[2];
    cout << "xyPc:" << xyPc << endl;

    double r2 = xyPc[0]*xyPc[0]+xyPc[1]*xyPc[1];
    double r4 = r2*r2;

    Eigen::Vector3d hat_xyPc(xyPc[0]*(1+k0*r2+k1*r4), xyPc[1]*(1+k0*r2+k1*r4), 1);
    Eigen::Vector3d hat_uv = orignalK * hat_xyPc;
    cout << "hat_uv:" << hat_uv << endl;

    Eigen::Vector3d uv_ = orignalK * T * Pw;
    uv_ /= uv_[2];
    cout << "Pw:" << Pw << endl;
    cout << "uv:" << uv << endl;
    cout << "uv_:" << uv_ << endl;
}

void my_calibr::myCalibrationProcess()
{
    //  [1]. 角点检测
    cornerDectection();
    //  [2]. 单应性矩阵初值计算, 为什么这里的初值误差会那么大
    caculateHomoInit();
    //  检查单应性矩阵的重投影效果, 感觉重投影效果挺差的
    // checkHomo();
    //  [3]. 对单应性矩阵进行非线性优化 
    //       法二: 其实可以考虑分开优化，因为本身就是线性优化，曲线拟合的问题
    // optimizedH();
    //  TODO: 已经验证，雅可比没有问题，但是就是算不对，真的难搞！！！
    myNLPoptimized();
    // optimizeAutoH();
    // nonlinearOptimizeH();
    //  [4]. 求解内参矩阵
    solveInertialMatrix();
    //  [5]. 求解外参矩阵
    caculateExternalMatrix();
    //  [6]. 求解畸变系数
    caculateDistortionFactor();
    //  [7]. 重投影误差优化畸变系数、外参、内参
    optimizeReprojection();
    // testRepro();
}

void my_calibr::officialCalib()
{
    //  [1]. 角点检测
    //      世界坐标存储在 vector<vector<Point3f>>  角点坐标存储在 vector<vetcor<Point2f>>
    std::vector<std::vector<cv::Point3f>> worldpt;
    std::vector<std::vector<cv::Point2f>> pixelpt;
    int N = m_files_path.size();
    int rows = m_points_per_row;
    int cols = m_points_per_col;
    int length = m_square_size;
    cv::Size imgSize;
    std::vector<cv::Point3f> wset;
    for(int i=0; i<cols; i++)
    {
        for(int j=0; j<rows; j++)
        {
            cv::Point3f wp(j*length, i*length, 0);
            wset.push_back(wp);
        }
    }
    for(int i=0; i<N; i++)
    {
        cv::Mat img = cv::imread(m_files_path[i], 1);
        imgSize = img.size();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> point2d;
        bool found = cv::findChessboardCorners
                (gray, cv::Size(rows,cols), point2d, 
                cv::CALIB_CB_ADAPTIVE_THRESH | 
                cv::CALIB_CB_FAST_CHECK | 
                cv::CALIB_CB_NORMALIZE_IMAGE);
        
        if(found)
        {
            cv::cornerSubPix(gray, point2d, cv::Size(11,11), 
                cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
            cv::Mat imgcopy = img.clone();
            cv::drawChessboardCorners(imgcopy, cv::Size(rows,cols),point2d, found);
            cv::namedWindow("img", cv::WINDOW_NORMAL);
            cv::imshow("img", imgcopy);
            cv::waitKey(0);
            pixelpt.push_back(point2d);
            worldpt.push_back(wset);
        }
    }

    //  [2]. 运行标定
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> revc;
    std::vector<cv::Mat> tvec;
    double rms = cv::calibrateCamera(worldpt, pixelpt, imgSize, cameraMatrix, distCoeffs, revc, tvec, 
                cv::CALIB_FIX_PRINCIPAL_POINT|cv::CALIB_FIX_K4|cv::CALIB_FIX_K5);
    
    ROS_INFO("rms:%lf", rms);
    cv::FileStorage cfs("/home/zjj/MyCode/myVinsPrj/src/my_utils/config/IntrinsicMatrix.yaml", cv::FileStorage::WRITE);
    
    cfs << "IntrinsicMatrix" << cameraMatrix;
    cfs << "distCoeffs" << distCoeffs;
    
    // for(int i=0; i<revc.size(); i++)
    // {
    //     cfs << "i" << i;
    //     cfs << "revc" << revc[i];
    //     cfs << "tvec" << tvec[i];
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_calibration");
    ros::NodeHandle n("~");
    // ROS_INFO("%s", CALIBRATION_IMAGE_PATH.c_str());
    std::shared_ptr<my_calibr> m_calibr = std::make_shared<my_calibr>(n);
    m_calibr->myCalibrationProcess();
    // m_calibr->officialCalib();
    // ros::shutdown();
    // ros::spin();
    return 0;
}