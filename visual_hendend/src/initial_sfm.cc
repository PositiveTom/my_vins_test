#include "initial_sfm.h"

void SfM_debug(const std::vector<SfmFeature> &sfm_f, const Eigen::Matrix<double, 3, 4>* Pose, int l)
{
    static int i = 0;
    

    //  [1]. 创建发布者
    static tf::TransformBroadcaster br;
    //  [2]. 创建变换关系
    tf::Transform transform;

    //  [3]. 取出其他帧与参考帧的变换关系
    for(int i=0; i<WINDOW_SIZE; i++)
    {
        if(i == l)
        {
            continue;
        }
        const Eigen::Matrix<double, 3, 4> &Pose_cOther_cl = Pose[i];

        transform.setOrigin(tf::Vector3(Pose_cOther_cl(0,3), Pose_cOther_cl(1,3), Pose_cOther_cl(2,3)));
        
        Eigen::Quaterniond q(Pose_cOther_cl.block<3,3>(0,0));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

        std::string frame_id = std::to_string(i);
        std::string child_frame_id = std::to_string(l);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
    }



}


bool GlobalSfM::construct(int WINDOW_SIZE, Eigen::Quaterniond *Q, Eigen::Vector3d *T, int l,
    const Eigen::Matrix3d &R_cl_cur, const Eigen::Vector3d &T_cl_cur, 
    std::vector<SfmFeature> &sfm_f, std::map<int, Eigen::Vector3d> &sfm_track_points)
{
    //  [1]. 得到参与sfm的特征点个数
    feature_nums = sfm_f.size();

    //  [2]. 设置参考帧为世界坐标系
    Q[l].w() = 1;
    Q[l].x() = 0;
    Q[l].y() = 0;
    Q[l].z() = 0;
    T[l].setZero();

    //  [3]. 设置当前帧的位姿
    Q[WINDOW_SIZE - 1] = Eigen::Quaterniond(R_cl_cur);
    T[WINDOW_SIZE - 1] = T_cl_cur;

    //  [4]. 创建 存储 其他帧 到参考帧 的变换矩阵(旋转矩阵 和 平移向量)
    Eigen::Matrix3d R_cOther_cl[WINDOW_SIZE];
    Eigen::Vector3d T_cOther_cl[WINDOW_SIZE];
    Eigen::Quaterniond Q_cOther_cl[WINDOW_SIZE];
    Eigen::Matrix<double, 3, 4> Pose_cOther_cl[WINDOW_SIZE];

    //  [5]. 创建数组存储上述变量, 用于sfm非线性优化的待优化变量
    double q_cOther_cl[WINDOW_SIZE][4];
    double t_cOther_cl[WINDOW_SIZE][3];

    //  [6]. 对于参考帧位姿, 把 T_cl_cur 转为 T_cur_cl;
    Q_cOther_cl[l] = Q[l].inverse();
    R_cOther_cl[l] = Q_cOther_cl[l].toRotationMatrix();
    T_cOther_cl[l] = -1.0 *( R_cOther_cl[l] * T[l]);
    Pose_cOther_cl[l].block<3,3>(0,0) = R_cOther_cl[l];
    Pose_cOther_cl[l].block<3,1>(0,3) = T_cOther_cl[l];

    //  [7]. 对于当前帧位姿, 把 T_cl_cur 转为 T_cur_cl;
    Q_cOther_cl[WINDOW_SIZE - 1] = Q[WINDOW_SIZE - 1].inverse();
    R_cOther_cl[WINDOW_SIZE - 1] = Q_cOther_cl[WINDOW_SIZE - 1].toRotationMatrix();
    T_cOther_cl[WINDOW_SIZE - 1] = -1.0 *( R_cOther_cl[WINDOW_SIZE - 1] * T[WINDOW_SIZE - 1]);
    Pose_cOther_cl[WINDOW_SIZE - 1].block<3,3>(0,0) = R_cOther_cl[WINDOW_SIZE - 1];
    Pose_cOther_cl[WINDOW_SIZE - 1].block<3,1>(0,3) = T_cOther_cl[WINDOW_SIZE - 1];    

    //  [8]. sfm 开始

    //  [8.1]. 利用 当前帧 和 参考帧 的 位姿 进行三角化, 求出当前帧 和 参考帧 之间共视特征点的世界坐标

    for(int i=l; i<WINDOW_SIZE-1; i++)
    {
        //  (1) 最开始选择参考帧 l 与当前帧进行三角化, 之后再是 l+1 帧与 当前帧 先进行PnP, 再进行三角化
        if( i > l )
        {
            // PnP 步骤
            //  (1.1) 取出最近的并且已知位姿的一帧图像的位姿当作当前PnP算法待求帧位姿的初值
            Eigen::Matrix3d Rinit_lPlus1_cl = R_cOther_cl[i - 1];
            Eigen::Vector3d Tinit_lPlus1_cl = T_cOther_cl[i - 1];
            if(!solverFrameByPnP(Rinit_lPlus1_cl, Tinit_lPlus1_cl, i, sfm_f))
                return false;
            R_cOther_cl[i] = Rinit_lPlus1_cl;
            T_cOther_cl[i] = Tinit_lPlus1_cl;
            Q_cOther_cl[i] = R_cOther_cl[i];
            Pose_cOther_cl[i].block<3,3>(0,0) = R_cOther_cl[i];
            Pose_cOther_cl[i].block<3,1>(0,3) = T_cOther_cl[i];
        }
        //  三角化
        triangulateTwoFrames(i, Pose_cOther_cl[i], WINDOW_SIZE-1, Pose_cOther_cl[WINDOW_SIZE-1], sfm_f);
    }
    
    //  [8.2] 参考帧 与 l+1帧, l+2帧, ..., WINDOW_SIZE-2帧 进行三角化
    for(int i=l+1; i<WINDOW_SIZE-2; i++)
    {
        triangulateTwoFrames(l, Pose_cOther_cl[l], i, Pose_cOther_cl[i], sfm_f);
    }

    //  [8.3] 0, 1, ..., l-1帧 进行PnP, 再与参考l帧进行三角化
    for(int i=l-1; i>=0; i--)
    {
        //  [8.3.1] 取初值 进行pnp
        Eigen::Matrix3d R_lMius1_cl = R_cOther_cl[i+1];
        Eigen::Vector3d T_lMins1_cl = T_cOther_cl[i+1];
        if(!solverFrameByPnP(R_lMius1_cl, T_lMins1_cl, i, sfm_f))
            return false;
        R_cOther_cl[i] = R_lMius1_cl;
        T_cOther_cl[i] = T_lMins1_cl;
        Q_cOther_cl[i] = R_cOther_cl[i];
        Pose_cOther_cl[i].block<3,3>(0,0) = R_cOther_cl[i];
        Pose_cOther_cl[i].block<3,1>(0,3) = T_cOther_cl[i];

        //  [8.3.2] 再进行三角化
        triangulateTwoFrames(i, Pose_cOther_cl[i], l, Pose_cOther_cl[l], sfm_f);
    }

    //  [9]. 再三角化剩余的特征点
    for(int i=0; i<feature_nums; i++)
    {
        if(sfm_f[i].state==true)
            continue;
        //  共视帧超过两个以上的就可以进行三角化
        if((int)sfm_f[i].observation.size() > 2)
        {
            int frame0 = sfm_f[i].observation.front().first;
            Eigen::Vector2d point0(sfm_f[i].observation.front().second);

            int frame1 = sfm_f[i].observation.back().first;
            Eigen::Vector2d point1(sfm_f[i].observation.back().second);

            Eigen::Vector3d Pw;
            if(triangulatePoint(Pose_cOther_cl[frame0], Pose_cOther_cl[frame1], point0, point1, Pw))
            {
                sfm_f[i].state = true;
                sfm_f[i].position = Pw;
            }
        }
    }

    //  [10]. 发布到rviz 查看
    if(SfMDebug)
    {
        SfM_debug(sfm_f, Pose_cOther_cl, l);
    }


}   

void GlobalSfM::triangulateTwoFrames
(int frame0, const Eigen::Matrix<double, 3, 4> &Pose0,
int frame1, const Eigen::Matrix<double, 3, 4> &Pose1, 
std::vector<SfmFeature> &sfm_f)
{
    // Eigen::MatrixXd A;

    //  [1].遍历待SfM的特征点
    for(int i=0; i<feature_nums; i++)
    {   
        //  [2]. 如果特征点已经三角化了, 则跳过
        if(sfm_f[i].state == true)
            continue;
        
        //  [3]. 取出具有共视关系的特征点
        bool has0 = false, has1 = false;
        Eigen::Vector2d point0, point1;
        for(int j=0; j<sfm_f[i].observation.size(); j++)
        {
            if(sfm_f[i].observation[j].first == frame0)
            {
                has0 = true;
                point0 = sfm_f[i].observation[j].second;
            }
            if(sfm_f[i].observation[j].first == frame1)
            {
                has1 = true;
                point1 = sfm_f[i].observation[j].second;
            }            
        }
        if( has0 && has1 )
        {
            Eigen::Vector3d Pw;
            if(triangulatePoint(Pose0, Pose1, point0, point1, Pw))
            {
                sfm_f[i].state = true;
                sfm_f[i].position[0] = Pw[0];
                sfm_f[i].position[1] = Pw[1];
                sfm_f[i].position[2] = Pw[2];
            }
        }
    }

}

bool GlobalSfM::triangulatePoint
(const Eigen::Matrix<double,3,4> &Pose0, const Eigen::Matrix<double,3,4> &Pose1, 
const Eigen::Vector2d &point0, const Eigen::Vector2d &point1, Eigen::Vector3d &Pw)
{
    Eigen::Matrix<double, 4, 4> A;

    A.block<1, 4>(0, 0) = point0.x() * Pose0.block<1, 4>(2, 0) - Pose0.block<1, 4>(0, 0);
    A.block<1, 4>(1, 0) = point0.y() * Pose0.block<1, 4>(2, 0) - Pose0.block<1, 4>(1, 0);
    A.block<1, 4>(2, 0) = point1.x() * Pose1.block<1, 4>(2, 0) - Pose1.block<1, 4>(0, 0);
    A.block<1, 4>(3, 0) = point1.y() * Pose1.block<1, 4>(2, 0) - Pose1.block<1, 4>(1, 0);

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::Vector4d sigma = svd.singularValues();

    Eigen::Vector4d solver = V.col(3);

    if( sigma(3) / sigma(2) < 0.01 )
    {
        Pw(0) = solver(0) / solver(3);
        Pw(1) = solver(1) / solver(3);
        Pw(2) = solver(2) / solver(3);
        return true;
    }
    return false;
}

bool GlobalSfM::solverFrameByPnP
(Eigen::Matrix3d &R_lPLus1_cl, Eigen::Vector3d &T_lPlus1_cl, 
int frame_id, std::vector<SfmFeature> &sfm_f)
{
    //  [1]. 挑选出点对, 需要三角化成功, 并且frame_id帧需要看到此图像帧
    std::vector<cv::Point2f> xy1;
    std::vector<cv::Point3f> XYZ;
    for(auto &feature : sfm_f)
    {
        if(feature.state == false)
            continue;
        for(auto &obser : feature.observation )
        {
            if(obser.first == frame_id)
            {
                xy1.push_back(cv::Point2f(obser.second[0], obser.second[1]));
                XYZ.push_back(cv::Point3f(feature.position(0), feature.position(1), feature.position(2)));
            }
        }
    }

    //  [2]. 至少需要16个点对
    if((int)xy1.size() < 15)
    {
        ROS_WARN("unstable feature tracking, please slowly move your device!");
        if( (int)xy1.size() < 10)
            return false;
    }

    //  [3]. opencv 使用 pnp 算法的步骤
    //  [3.1] 把初始旋转矩阵转为轴角形式
    cv::Mat R_lPlus_cl_mat, shaft_angle;
    //  [3.1.1] 把eigen矩阵转为mat矩阵
    R_lPlus_cl_mat = (cv::Mat_<double>(3,3)<<R_lPLus1_cl(0,0), R_lPLus1_cl(0,1), R_lPLus1_cl(0,2),
                                            R_lPLus1_cl(1,0), R_lPLus1_cl(1,1), R_lPLus1_cl(1,2),
                                            R_lPLus1_cl(2,0), R_lPLus1_cl(2,1), R_lPLus1_cl(2,2));
    //  [3.1.2] 把旋转矩阵转为轴角式
    cv::Rodrigues(R_lPlus_cl_mat, shaft_angle);
    
    //  [3.2] 把初始平移向量eigen 转为 mat向量
    cv::Mat t = (cv::Mat_<double>(3,1)<<T_lPlus1_cl[0], T_lPlus1_cl[1], T_lPlus1_cl[2]);

    //  [3.3] 准备一个伪内参矩阵K, 由于这个函数默认二维齐次坐标是像素坐标
    cv::Mat K = (cv::Mat_<double>(3,3)<<1,0,0,0,1,0,0,0,1);

    //  [3.4] 准备一个畸变系数
    cv::Mat D;

    //  [3.5] 进行求解
    bool pnp_sucess = cv::solvePnP(XYZ, xy1, K, D, shaft_angle, t, true);

    //  [3.6] 判断求解的成功与否
    if(!pnp_sucess)
    {
        return false;
    }

    //  [3.7] 求解成功则进行转换
    cv::Rodrigues(shaft_angle, R_lPlus_cl_mat);

    for(int i=0; i<3; i++)
    {
        T_lPlus1_cl(i) = t.at<double>(i,0);
        for(int j=0; j<3; j++)
        {
            R_lPLus1_cl(i,j) = R_lPlus_cl_mat.at<double>(i,j);
        }
    }
    return true;
}