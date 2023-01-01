#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

template<typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker(/* args */)
{
    intrinsicMatrix_ = cv::Mat(3,3,CV_64F);
    cout << "FeatureTracker is created!" << endl;
}

void FeatureTracker::readIntrinsicParameter(const std::string& intrinsic_path)
{
    //  通过opencv的 cv::FileStorage 类从yaml文件中读取, 注意yaml文件内存储矩阵的格式!!!
    //  [1]. 创建文件流类 fs
    // cout << intrinsic_path << endl;
    cv::FileStorage fs;
    //  [2]. 打开yaml文件
    fs.open(intrinsic_path, cv::FileStorage::READ);
    //  [3]. 读出数据 (有五种方式读出数据, 这里对应第3种)
    //  [3.1] 创建fileNode
    cv::FileNode n;
    n = fs["cam0"];
    //  [3.2] 读出相机的内参
    std::vector<double> intrinsics;
    n["intrinsics"] >> intrinsics;
    fx = intrinsics[0];
    fy = intrinsics[1];
    u0 = intrinsics[2];
    v0 = intrinsics[3];
    intrinsicMatrix_.at<double>(0,0) = fx;
    intrinsicMatrix_.at<double>(1,1) = fy;
    intrinsicMatrix_.at<double>(0,2) = u0;
    intrinsicMatrix_.at<double>(1,2) = v0;
    cout << "intrinsicMatrix_:" << endl << intrinsicMatrix_ << endl;
    //  [3.3] 读出相机的畸变系数
    std::vector<double> distortion_coeffs;
    n["distortion_coeffs"] >> distortion_coeffs;
    k1 = distortion_coeffs[0];
    k2 = distortion_coeffs[1];
    p1 = distortion_coeffs[2];
    p2 = distortion_coeffs[3];
    cout << "k1:" << k1 << endl;
    cout << "k2:" << k2 << endl;
    cout << "p1:" << p1 << endl;
    cout << "p2:" << p2 << endl;

    //  [4] 初始化相机模型
    camera_ = std::make_shared<PinholeCamera>(k1, k2, p1, p2, fx, fy, u0, v0, false);

    fs.release();
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{   
    cv::Mat img;
    TicToc t_r;
    cur_time_ = _cur_time;
    //  [1]. 直方图均衡化,提高图像对比度
    if(EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3., cv::Size(8,8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
        if(CLAHE_Debug)
        {   
            cv::namedWindow("CLAHE", cv::WINDOW_NORMAL);
            cv::imshow("CLAHE", img);
            cv::namedWindow("Origin", cv::WINDOW_NORMAL);
            cv::imshow("Origin", _img);
            cv::waitKey(1);  
        }      
    }
    else
        img = _img;
    
    //  [2]. 如果存放当前帧图像的内存是空的
    if(forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
        forw_img = img;
    
    //  [3]. 清空存放当前帧图像特征点的容器, 为存放当前帧的特征点集合作准备
    forw_pts.clear();

    //  [4]. 如果上一帧图像提取到了特征点, 则进行光流跟踪特征点
    // cout << "cur_pts.size():" << cur_pts.size() << endl;
    if(cur_pts.size()>0)
    {
        TicToc t_o;
        //  [4.1]. 直接进行光流跟踪
        std::vector<uchar> status;
        std::vector<float> err;
        //  金字塔光流  的窗口大小选取 21
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21,21), 3);

        // cout << "LK forw_pts.size():" << forw_pts.size() << endl;
        //  [4.2] 判断跟踪的特征点是否在跟踪成功,以及是否在图像内
        for(int i=0; i<(int)forw_pts.size(); i++)
        {
            if(status[i] && !inBorder(forw_pts[i]))
            {
                status[i] = 0;
            }
        }

        //  [4.3]. 根据status剔除掉一些容器里面的元素
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("%d points, temporal optical flow costs: %fms",(int)forw_pts.size(), t_o.toc());
    }

    //  [4-5]. 跟踪成功,则代表此特征点又被看到一次,track_cnt加1
    for(auto &n :track_cnt)
        n++;

    //  [5]. 如果发布此帧
    if(PUB_THIS_FRAME)
    {
        //  [6]. 对上一步跟踪的角点,进一步做鲁棒性处理
        rejectWithF();
        // cout << "RF forw_pts.size():" << forw_pts.size() << endl;
        TicToc t_m;
        //  [7]. 设置掩膜
        setMask();
        // cout << "mask forw_pts.size():" << forw_pts.size() << endl;
        ROS_DEBUG("set mask costs %fms", t_m.toc());
        if(maskDebug)
        {
            cv::namedWindow("mask", cv::WINDOW_NORMAL);
            cv::imshow("mask", mask);
            cv::waitKey(1);
        }

        //  检测特征点 shi-Tomasi
        ROS_DEBUG("detect feature begins");

        TicToc t_t;
        //  补充一些额外的角点, 由于上面有一些特征点光流跟踪失败了
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        // cout << "n_max_cnt:" << n_max_cnt << endl;
        if(n_max_cnt > 0)
        {
            //  [8]. shi-Tomasi角点提取
            cv::goodFeaturesToTrack(img, n_pts, n_max_cnt, 0.01, 30, mask);

            if(SHI_TOMASIDebug)
            {
                // cout << "n_pts.size():" << n_pts.size() << endl;
                cv::Mat image = img.clone();
                for(auto& kp:n_pts)
                {
                    cv::Point pt1, pt2;
                    pt1.x = kp.x - 5;
                    pt1.y = kp.y - 5;
                    pt2.x = kp.x + 5;
                    pt2.y = kp.y + 5;
                    if( (pt1.x >=0) && (pt1.y>=0) && (pt2.x<COL) && (pt2.y<ROW))
                    {
                        cv::rectangle(image, pt1, pt2, cv::Scalar(255));
                    }
                }
                cv::namedWindow("shi-Tomasi", cv::WINDOW_NORMAL);
                cv::imshow("shi-Tomasi", image);
                cv::waitKey(1);
            }
        }
        else
        {
            n_pts.clear();
        }
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());
        ROS_DEBUG("add feature begins");
        TicToc t_a;
        //  [9]. 把新的shi-Tomasi角点添加
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    //  进行一些新老成员交替工作处理
    prev_img = cur_img;
    cur_img = forw_img;
    prev_pts = cur_pts;
    cur_pts = forw_pts;
    prev_un_pts = cur_un_pts;
    //  特征点去畸变
    undistortedPoints();

    prev_time_ = cur_time_;
}

void FeatureTracker::setMask()
{
    //  [1]. 初始化掩膜
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    //  创建一个局部变量, 为排序作准备
    std::vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    //  resize 和就不能用push_back 了
    cnt_pts_id.reserve((int)forw_pts.size());

    for(int i=0; i<(int)forw_pts.size(); i++)
        cnt_pts_id.push_back( std::make_pair(track_cnt[i], 
                                            std::make_pair(forw_pts[i], ids[i])) );
    //  [2]. 排序
    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(), 
        [](std::pair<int, pair<cv::Point2f, int>>&a, 
            std::pair<int, pair<cv::Point2f, int>>&b)
            {return a.first > b.first;});
    
    // cout << "forw_pts.clear() foward:" << forw_pts.size() << endl;
    // cout << "ids.clear() foward:" << ids.size() << endl;
    // cout << "track_cnt.clear() foward:" << track_cnt.size() << endl;

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // cout << "forw_pts.clear() back:" << forw_pts.size() << endl;
    // cout << "ids.clear() back:" << ids.size() << endl;
    // cout << "track_cnt.clear() back:" << track_cnt.size() << endl;

    //  [3]. 设置掩膜
    for(auto &pt:cnt_pts_id)
    {
        forw_pts.push_back(pt.second.first);
        ids.push_back(pt.second.second);
        track_cnt.push_back(pt.first);
        //  黑色, 实心
        cv::circle(mask, pt.second.first, MIN_DIST, 0, -1);
    }
    // cout << "forw_pts.clear() mask back:" << forw_pts.size() << endl;
    // cout << "ids.clear() mask back:" << ids.size() << endl;
    // cout << "track_cnt.clear() mask back:" << track_cnt.size() << endl;


}

bool FeatureTracker::updateID(ulong i)
{
    if(i < ids.size())
    {
        if(ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::addPoints()
{
    for(auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::undistortedPoints()
{
    //  [1]. 清空存储当前帧无畸变特征点的容器
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    //  [2]. 遍历当前帧的特征点
    for(int i=0; i<cur_pts.size(); i++)
    {
        //  [2.1] 取出特征点
        Eigen::Vector2d uv(cur_pts[i].x, cur_pts[i].y); // (像素坐标) 单位像素
        Eigen::Vector3d xy; // (图像坐标), 单位mm
        camera_->liftProjective(uv, xy);

        cur_un_pts.push_back(cv::Point2f(xy(0), xy(1)));
        cur_un_pts_map.insert(std::make_pair(ids[i], cv::Point2f(xy(0), xy(1))));
    }

    //  [3]. 计算特征点的速度 , 需要两帧才能计算速度
    if(!prev_un_pts_map.empty())
    {
        //  两帧的时间差
        double dt = cur_time_ - prev_time_;
        pts_velocity.clear();

        //  [4]. 遍历当前帧特征点,开始计算速度
        for(int i=0; i<cur_un_pts.size(); i++)
        {
            if(ids[-1] != -1)
            {
                //  不等于-1,代表此特征点至少出现两次及其以上
                //  [4.1] 从上一张图片,寻找出对应的特征点索引, map根据键值寻找
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if( it != prev_un_pts_map.end() )
                {
                    //  如果找到了, 则计算光流速度, 这里的单位是不知道的 //TODO
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                {
                    pts_velocity.push_back(cv::Point2f(0., 0.));
                }
            }
            else
            {
                //  ids[-1] = -1代表此特征点, 才刚出现第一次, 令其光流速度为0
                pts_velocity.push_back(cv::Point2f(0., 0.));
            }
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

void FeatureTracker::rejectWithF()
{
    if( forw_pts.size() > 8 )
    {
        ROS_DEBUG("FM ransac begins");
        //  创建 局部变量,存储无畸变的特征点坐标, 上一帧的以及当前帧的, 齐次相机坐标
        TicToc t_f;
        std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size()); 
        //  [1]. 把像素坐标转为齐次相机坐标,并且去畸变
        for(int i=0; i< (int)forw_pts.size(); i++ )
        {
            Eigen::Vector2d uv_cur(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d un_xy_cur;
            camera_->liftProjective(uv_cur, un_xy_cur);
            un_cur_pts.push_back(cv::Point2f(un_xy_cur.x(), un_xy_cur.y()));

            Eigen::Vector2d uv_forw(forw_pts[i].x, forw_pts[i].y);
            Eigen::Vector3d un_xy_forw;
            camera_->liftProjective(uv_forw, un_xy_forw);
            un_forw_pts.push_back(cv::Point2f(un_xy_forw.x(), un_xy_forw.y()));
        }
        //  [2]. 通过RANSAC方法计算基础矩阵(实质是本质矩阵)
        std::vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);

        int size_a = (int)cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());

    }
}