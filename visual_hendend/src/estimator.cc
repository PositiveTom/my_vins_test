#include "estimator.h"

//  上一帧图像的时间戳
double initial_timestamp = 0.;

Estimator::Estimator()
{
    ROS_DEBUG("Estimator Init!");
    clearState();
}

void Estimator::clearState()
{
    //  [1]. 外参
    ric.setIdentity();
    tic.setZero();
    //  [2]. 时间偏移
    td = 0;
    //  [3]. 第一帧IMU数据标志
    first_imu = true;
    //  [4]. 当前操作的是滑动窗口中的第 frame_count 帧
    frame_count = 0;
    //  [5]. 预积分量指针赋空nullptr, 滑动窗口中所有加速度,加速度随机游走项赋0
    for(int i=0; i<WINDOW_SIZE; i++)
    {
        if(pre_integration[i] != nullptr)
            delete pre_integration[i];
        pre_integration[i] = nullptr;

        ba[i].setZero();
        bg[i].setZero();
    }
    //  [6]. 清空暂存关键帧预积分的临时指针
    if(tmp_pre_integration != nullptr)
        tmp_pre_integration = nullptr;

    //  [7]. 设置程序的状态, 估计器初始化
    solver_flag = INITIAL;

    //  [8]. 设置边缘化参数
    sum_of_back = 0;
    sum_of_new = 0;
}


void Estimator::setParameter()
{
    ROS_DEBUG("Begin Estimator init");
    //  [1]. 获取外参
    ric = RCI.transpose();
    tic = - ric * TCI;
    cout << "ric:" << endl << ric << endl;
    cout << "tic:" << endl << tic << endl;
    //  [2]. 给特征管理器赋值
    f_manager.setRic(ric);
}

void Estimator::processIMU(const double dt, const Eigen::Vector3d& body_acc,const Eigen::Vector3d& body_gyro)
{
    //  [1]. 如果是第一帧
    if(first_imu)
    {
        first_imu = false;
        acc0 = body_acc;
        gyro0 = body_gyro;
    }

    //  [2]. 如果当前 frame_count 所指向的帧 对应 的预积分量还没有实例化对象, 则实例化预积分类对象
    if(!pre_integration[frame_count])
        pre_integration[frame_count] = new PreIntegration(acc0, gyro0, ba[frame_count], bg[frame_count]);

    //  [3]. 如果当前帧不是第一帧
    if(frame_count != 0)
    {
        //  [3.1] 滑动窗口预积分
        pre_integration[frame_count]->Integral(dt, body_acc, body_gyro);
        //  [3.2] 普通图像帧对应的预积分
        tmp_pre_integration->Integral(dt, body_acc, body_gyro);

        //  [3.3] 滑动窗口记录原始IMU数据
        dt_buf[frame_count].push_back(dt);
        acc_buf[frame_count].push_back(body_acc);
        gyro_buf[frame_count].push_back(body_gyro);

        //  [3.4] 记录滑动窗口的预积分量
        Rs[frame_count] = pre_integration[frame_count]->q_bi_bj_;
        Ps[frame_count] = pre_integration[frame_count]->alpha_bi_bj_;
        Vs[frame_count] = pre_integration[frame_count]->beta_bi_bj_;
    }

    //  [4]. 记录加速度, 进入到下一次预积分计算时,当作上一帧的加速度
    acc0 = body_acc;
    gyro0 = body_gyro;
}

int first = 0;
void Estimator::processImage
(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
 const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming.............");
    ROS_DEBUG("adding feature points %lu", image.size());
    //  [1]. 进入关键帧选择函数(依赖于特征管理器类), 
    //       同时告诉特征管理器类, 如果这一帧图像可以看作关键帧, 那么frame_count就是为其在滑动窗口内分配的索引
    if( f_manager.addFeatureCheckParallax(frame_count, image, td) )
    {
        //  [1.1] 如果可以看作关键帧, 那么就要挤出前面的图像帧, 也就是边缘化老的
        marginalization_flag = MarginalizationFlag::MARGIN_OLD;
    }
    else
    {
        //  [1.2] 如果不可以看作关键帧, 那么就要挤出次新帧 TODO:
        marginalization_flag = MarginalizationFlag::MARGIN_SECOND_NEW;
    }
    ROS_DEBUG("this frame is--------------------%s", (bool)marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", (bool)marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());

    if(f_manager.getFeatureCount() == 0)
    {
        if(first == 0)
            first++;
        else
        {        
            ROS_ERROR("f_manager.features_.size():%ld", f_manager.features_.size());
            ROS_ERROR("ERROR!!");
            f_manager.getFeatureCount();
            std::abort();
        }
    }
    // ROS_ERROR("INFO1!!");
    // [2]. 记录此图像帧的时间头
    Headers[frame_count] = header;
    // ROS_ERROR("INFO2!!");
    // [3]. 创建图像帧类, 并且给其临时预积分量变量赋值
    ImageFrame frame(image, header.stamp.toSec());
    frame.pre_integration = tmp_pre_integration;
    // ROS_ERROR("INFO3!!");
    //  [4]. 添加图像帧信息到全局帧容器变量里面
    all_image_frame.insert(std::make_pair(header.stamp.toSec(), frame));
    // ROS_ERROR("INFO4!!");
    //  [5]. 第[3]步中, tmp_pre_integration的使命完成了, 即这一帧图像对应的预积分量已经得到了
    //       下面该考虑下一帧图像的预积分量了
    tmp_pre_integration = new PreIntegration(acc0, gyro0, ba[frame_count], bg[frame_count]);
    // ROS_ERROR("INFO5!!");
    //  [6]. 估计外参, 暂时待定 TODO:


    //  [7]. 如果位于估计器初始化步骤
    if(solver_flag == INITIAL)
    {
        //  估计器初始化是否成功的标志
        bool result = false;
        //  [8]. 如果关键帧数等于了 滑动窗口 应该有的帧数
        if( frame_count == WINDOW_SIZE - 1 )
        {
            //  [8.1] 如果图像帧的时间戳正确, 则进入估计器初始化
            if( header.stamp.toSec() -  initial_timestamp > 0)
            {
                //  [8.2]. 进行估计器初始化
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            //  [8.2] 如果初始化成功, 则进入VIO步骤
            if(result)
            {
                //  TODO:
            }
            //  [8.3] 初始化失败, 则要滑动窗口 slidewindow
            else
            {
                // ROS_INFO("slide!!!");
                slideWindow();
            }


        }
        //  [9]. 否则, 进行frame_count++即可, 构造下一个关键帧
        else
            frame_count++;

    }
    else
    {
        //  TODO:
    }
}

void Estimator::slideWindow()
{
    TicToc t_marg;
    //  [1]. 如果是移除最老的图像帧
    if( marginalization_flag == MarginalizationFlag::MARGIN_OLD )
    {
        // [1.1] 保存最老帧的信息
        double t_0 = Headers[0].stamp.toSec(); // 最老帧的时间戳
        back_R0 = Rs[0];
        back_P0 = Ps[0];

        // [1.2] 如果正在初始化
        if( frame_count == WINDOW_SIZE - 1 )
        {
            // [1.3~1.8] 都是在滑动 滑动窗口内的图像帧数据
            // [1.3] 把滑动窗口中 帧 整体向前 移动一帧
            for(int i=0; i<WINDOW_SIZE-1; i++)
            {
                // [1.4] 交换滑动窗口中 旋转矩阵, 位置向量, 速度向量, ba, bg, 
                //        预积分量, dt_buf, 加速度原始数据, 陀螺仪原始数据, 信息头
                Rs[i].swap(Rs[i+1]);
                Ps[i].swap(Ps[i+1]);
                Vs[i].swap(Vs[i+1]);
                ba[i].swap(ba[i+1]);
                bg[i].swap(ba[i+1]);
                std::swap(pre_integration[i], pre_integration[i+1]);
                dt_buf[i].swap(dt_buf[i+1]);
                acc_buf[i].swap(acc_buf[i+1]);
                gyro_buf[i].swap(gyro_buf[i+1]);
                Headers[i] = Headers[i+1];
            }

            //  [1.5] 上面由于一直交换, 导致第0帧的数据跑到了第10帧, 因此需要复位一下,
            //        让第10帧的数据仍然当作第10帧数据的初值
            Headers[WINDOW_SIZE - 1] = Headers[WINDOW_SIZE - 2];
            Ps[WINDOW_SIZE - 1] = Ps[WINDOW_SIZE - 2];
            Vs[WINDOW_SIZE - 1] = Vs[WINDOW_SIZE - 2];
            Rs[WINDOW_SIZE - 1] = Rs[WINDOW_SIZE - 2];
            ba[WINDOW_SIZE - 1] = ba[WINDOW_SIZE - 2];
            bg[WINDOW_SIZE - 1] = bg[WINDOW_SIZE - 2];

            //  [1.6] 删除第0帧的预积分, 同时由于此变量仅仅在初始化阶段创建, 因此这里需要人为创建
            delete pre_integration[WINDOW_SIZE - 1];
            pre_integration[WINDOW_SIZE - 1] = new PreIntegration(acc0, gyro0, ba[WINDOW_SIZE - 1], bg[WINDOW_SIZE - 1]);

            //  [1.7] 情况第0帧的dt_buf
            dt_buf[WINDOW_SIZE - 1].clear();
            acc_buf[WINDOW_SIZE - 1].clear();
            gyro_buf[WINDOW_SIZE - 1].clear();

            //  [1.8] 开始 删除 特征管理器内 all_image_frame 中的一些信息

            //  [1.9] 删除 all_image_frame 中 对应 前面删除帧的 预积分量
            std::map<double, ImageFrame>::iterator it_0;
            it_0 = all_image_frame.find(t_0); // 根据时间戳当作键值, 查找到对应帧并且删除对应的预积分量
            delete it_0->second.pre_integration;
            it_0->second.pre_integration = nullptr; // 注意先删除预积分信息, 因为其内存是人为创建的
            //  [1.10] 删除 删除帧及其以前帧的预积分量, 为什么先删除预积分量类, 因为其内存是人为创建的
            //  由于map容器按照键的升序排列, 因此, 在这一部分中, 删除的帧之前的图像帧, 其对应的时间戳都是小于当前删除的帧
            for(std::map<double, ImageFrame>::iterator it=all_image_frame.begin(); it!=it_0; it++ )
            {
                //  如果存在, 则删除, 并且置空
                if( it->second.pre_integration )
                    delete it->second.pre_integration; 
                it->second.pre_integration = nullptr;
            }
            //  [1.11] 在前面释放了堆区内存之后, 这里再释放其他的图像帧
            all_image_frame.erase(all_image_frame.begin(), it_0);//根据迭代器删除范围区间
            all_image_frame.erase(t_0);//根据键值删除
        
            //  [1.12] 更新特征点信息
            slideWindowOld();
        }   
    }
    //  [2]. 边缘化次新帧
    else
    {
        if( frame_count == WINDOW_SIZE - 1 )
        {
            //  [2.1] 边缘化次新帧, 意味着当前帧的预积分量值需要增大, 为了保证滑动窗口内预积分量的连贯性
            //  注意是从前往后进行积分
            // ROS_INFO("")
            for(int i=0; i<dt_buf[frame_count].size(); i++)
            {
                //  [2.2] 取出 原始加速度 和 原始速度, 进行预积分
                double dt = dt_buf[frame_count][i];
                Eigen::Vector3d body_acc = acc_buf[frame_count][i];
                Eigen::Vector3d body_gyro = gyro_buf[frame_count][i];

                pre_integration[frame_count-1]->Integral(dt, body_acc, body_gyro);
                dt_buf[frame_count-1].push_back(dt);
                acc_buf[frame_count-1].push_back(body_acc);
                gyro_buf[frame_count-1].push_back(body_gyro);
            }
            //  [2.3] 仅仅只是预积分量改变, 位姿 以及 bias是不变的
            Rs[WINDOW_SIZE - 2] = Rs[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE - 2] = Vs[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE - 2] = Ps[WINDOW_SIZE - 1];
            ba[WINDOW_SIZE - 2] = ba[WINDOW_SIZE - 1];
            bg[WINDOW_SIZE - 2] = bg[WINDOW_SIZE - 1];

            // [2.4] 由于当前帧的预积分量已经融入到前一帧了, 因此下面是delete当前帧的预积分量
            delete pre_integration[frame_count];
            pre_integration[frame_count] = new PreIntegration(acc0, gyro0, ba[frame_count], bg[frame_count]);

            //  [2.5] 继续清除当前帧的预积分量信息
            dt_buf[frame_count].clear();
            acc_buf[frame_count].clear();
            gyro_buf[frame_count].clear();

            //  [2.6] 更新特征点信息
            slideWindowNew();
        } 
    }
}

void Estimator::slideWindowNew()
{
    sum_of_new++;

    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    //  [1]. 如果处于估计器初始化阶段
    if( solver_flag == INITIAL )
    {
        // [1.1] 特征管理器中的特征点 由于 边缘化了最老帧 而导致的影响
        f_manager.removeBack();
    }
    //  [2]. 如果处于VIO阶段
    else if (solver_flag == NON_LINEAR)
    {
        //  TODO:
    }

}


bool Estimator::initialStructure()
{
    //  [1].检测IMU是否有充分地移动, 
    //      通过计算 滑动窗口中 各帧imu预积分量 的 平均加速度 所构成集合的统计特性方差 得到    
    TicToc t_sfm;
    {
        //  [1.1] 之所以跳过第一帧图像的预积分量, 是因为第一帧图像的预积分量没有意义!!!
        std::map<double, ImageFrame>::iterator frame_it;
        //  [1.2] 加速度均值
        Eigen::Vector3d average_g(Eigen::Vector3d::Zero());
        for( frame_it=all_image_frame.begin(), frame_it++; frame_it!=all_image_frame.end();frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->beta_bi_bj_ / dt;
            average_g += tmp_g * (1. / (all_image_frame.size() - 1));
        }

        //  [1.3] 加速度方差
        double var = 0;
        for( frame_it=all_image_frame.begin(), frame_it++; frame_it!=all_image_frame.end();frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->beta_bi_bj_ / dt;
            var = ( tmp_g - average_g ).transpose() * ( tmp_g - average_g );
        }
        ROS_DEBUG("var:%lf", var);
        if( var < 0.25 )
        {
            ROS_INFO("IMU exciting not enough!");
        }
    }
    // ROS_ERROR("INFO1!!");
    //  [2]. 全局SfM, 视觉三维重建, 目地是恢复相机的位姿, 和, 特征点的相机坐标
    //       进入这个函数, frame_count 的值等于 WINDOW_SIZE - 1, 代表滑动窗口数值满了
    Eigen::Quaterniond Q[WINDOW_SIZE];  // 暂存滑动窗口中的图像帧的姿态矩阵
    Eigen::Vector3d T[WINDOW_SIZE];     // 暂存滑动窗口中的图像帧的位置向量
    // ROS_ERROR("INFO2!!");
    //  管理sfm特征点的容器
    std::vector<SfmFeature> sfm_feature_buf;
    //  [2.1] 遍历特征管理器中的特征点容器
    // ROS_INFO("ERROR1");
    for(auto &feature : f_manager.features_)
    {
        //  [2.2] 针对每个特征点都创建一个SfM特征点类
        SfmFeature sfm_feature;
        sfm_feature.state = false;            // 表明此特征点还未三角化
        sfm_feature.id = feature.feature_id_; // 继承此特征点的id
        //  此特征点在滑动窗口中的帧id
        int slidwindow_id = feature.start_frame_ - 1; // 之所以-1, 是为后问填补sfm_feature的相关帧容器作准备
        //  [2.3] 遍历此特征点对应的 相关帧容器
        for(auto &relate_frame : feature.feature_per_frame)
        {
            slidwindow_id++;
            //  [2.4] 取出在相关帧里面此特征点的齐次相机坐标
            // ROS_ERROR("INFO1!!");
            // cout << relate_frame.xy1 << endl;
            Eigen::Vector3d xy1 = relate_frame.xy1; // 这里的代码出问题了 TODO:
            // ROS_ERROR("INFO2!!");
            sfm_feature.observation.push_back(std::make_pair(slidwindow_id, Eigen::Vector2d(xy1.x(), xy1.y())));
            // ROS_ERROR("INFO3!!");
        }
        // ROS_ERROR("INFO4!!");
        sfm_feature_buf.push_back(sfm_feature);
    }
    // ROS_ERROR("INFO3!!");
    //  [2.5] 得到 与 当前帧 共视 特征点数最多的参考帧l id
    Eigen::Matrix3d R_cl_cur; // 当前帧 到 参考帧 的 旋转矩阵
    Eigen::Vector3d T_cl_cur; // 当前帧 到 参考帧 的 平移向量
    int l = 0; // 参考帧id
    ROS_INFO("relativePose");
    if(!relativePose(R_cl_cur, T_cl_cur, l) )
    {
        //  [2.6] 如果没有得到合适的参考帧, 则
        ROS_INFO("Not Enough features or parallax!");
        return false;
    }

    GlobalSfM sfm;
    std::map<int, Eigen::Vector3d> sfm_tracked_points;
    if(!sfm.construct(WINDOW_SIZE, Q, T, l, R_cl_cur, T_cl_cur, sfm_feature_buf,sfm_tracked_points))
    {
        ROS_INFO("SFM Failed!");
        marginalization_flag = MarginalizationFlag::MARGIN_OLD;
        return false;
    }


}

bool Estimator::relativePose
(Eigen::Matrix3d &R_cl_cur, Eigen::Vector3d &T_cl_cur, int &l)
{
    //  [1]. 遍历 滑动 窗口 内的图像帧, 因为当前帧所处于的滑动窗口索引就是WINDOW_SIZE-1, 所以就不遍历WINDOW_SIZE-1帧了
    for( int i=0; i<WINDOW_SIZE-1; i++ )
    {
        //  [2]. 得到指定帧 之间的共视特征点数, 滑动窗口内的帧 与 最后一帧(当前帧)
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE-1);

        //  [3]. 如果共同视特征点个数 > 20, 则被列入候选参考帧, 参考帧是优先选择远的
        if(corres.size() > 20)
        {
            // [4]. 计算平均视差
            double sum_parallax = 0.;
            for(int j=0; j<corres.size(); j++)
            {
                Eigen::Vector2d xy1(corres[j].first.x(), corres[j].first.y());
                Eigen::Vector2d xy2(corres[j].second.x(), corres[j].second.y());
                double parallax = (xy1 - xy2 ).norm();
                sum_parallax += parallax;
            }
            double average_parallax = sum_parallax / corres.size();
            //  [5]. 如果平均视差大于阈值
            // ROS_INFO("average_parallax:%lf", average_parallax);
            if(average_parallax > 0.1 &&  solveRelativeRT(corres, R_cl_cur, T_cl_cur))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose %d and newest frame to triangulate the whole structure", average_parallax, l);
                return true;
            }
            // return false;
        }
    }
    return false;
}

bool Estimator::solveRelativeRT
(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& corres,
Eigen::Matrix3d &relativeR, Eigen::Vector3d &relativeT)
{
    //  [1]. 使用ransac基础矩阵的方法, 实质是本质矩阵

    //  [2]. 匹配点对要大于15才能使用 ransac 基础矩阵方法
    if( corres.size() > 0 )
    {
        //  [3]. 构造 findFundamentalMat 方法所需要的数据
        std::vector<cv::Point2f> p_l, p_r;
        for(int i=0; i<corres.size(); i++ )
        {
            p_l.push_back(cv::Point2f(corres[i].first.x(), corres[i].first.y()));
            p_r.push_back(cv::Point2f(corres[i].second.x(), corres[i].second.y()));
        }

        //  [4]. 求解基础矩阵
        cv::Mat hat_t_R;
        cv::Mat E = cv::findFundamentalMat(p_r, p_l, cv::FM_RANSAC, 0.3 / 460, 0.99, hat_t_R);

        //  [5]. 恢复t和R矩阵
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3)<<1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rot, trans;
        int inlier_cnt = cv::recoverPose(E, p_r, p_l, cameraMatrix, rot, trans, hat_t_R);
        
        //  [6]. 把mat矩阵转eigen矩阵
        for(int i=0; i<3; i++)
        {
            relativeT(i) = trans.at<double>(i, 0);
            for(int j=0; j<3; j++)
            {
                relativeR(i,j) = rot.at<double>(i, j);
            }
        }
        
        // cout << "relativeR:" << endl << relativeR << endl;
        // cout << "relativeT:" << endl << relativeT << endl;

        //  [7]. 如果内点数大于 12
        if(inlier_cnt > 12)
            return true;
        else
            return false;
    }
    else
        return false;
}



