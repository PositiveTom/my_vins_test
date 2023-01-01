#include "visualization.h"

//  [1]. 发布滑动窗口内符合pubPointCloud中要求特征点的客户端
ros::Publisher point_cloud;


void registerPub(ros::NodeHandle& n)
{
    point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_in_slidewindow", 1000);
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    //  [0]. 创建点云数据
    sensor_msgs::PointCloud cloud_points;
    cloud_points.header = header;
    // cloud_points.

    //  [1]. 遍历特征管理器里面的特征点
    for( auto &feature : estimator.f_manager.features_ )
    {
        //  下面的开始帧都是此刻滑动窗口里面的索引
        int used_num;
        //  [2]. 查看此特征点的共视帧个数
        used_num = feature.feature_per_frame.size();
        //  [3]. 共视帧个数>=2 并且 开始的帧 位于 滑动窗口中前 8 帧, 才有机会被发布
        if(! (used_num >= 2 && feature.start_frame_ < WINDOW_SIZE - 3 ))
            continue;
        //  [4]. 特征点必须三角化成功, 并且开始帧不能在后3帧
        if(feature.start_frame_ > WINDOW_SIZE * 3./4. || feature.solver_flag != true)
            continue;
        //  
        int start_frame = feature.start_frame_;
        //  [5]. 计算此特征点的在开始帧下的相机坐标
        Eigen::Vector3d start_frame_point = feature.feature_per_frame[0].xy1 * feature.estimated_depth;
        //  [6]. 计算出在第c0帧下的相机坐标
        Eigen::Vector3d c0_point = estimator.Rs[start_frame] * ( estimator.ric * start_frame_point + estimator.tic ) + estimator.Ps[start_frame];

        //  [7]. 
        geometry_msgs::Point32 p;
        p.x = c0_point.x();
        p.y = c0_point.y();
        p.z = c0_point.z();
        cloud_points.points.push_back(p);
    }
    //  [8]. 发布点云
    point_cloud.publish(cloud_points);
}

void debug(const Estimator &estimator)
{
    cout << estimator.first_imu << endl;
}
