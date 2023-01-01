#include <ros/ros.h>
#include "estimator.h"
#include "parameters.h"
#include "feature_manager.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <mutex>
#include <queue>
#include <iostream>
#include <thread>
#include <vector>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Eigen/Eigen>
#include <map>
#include <std_msgs/Header.h>
#include "visualization.h"


using namespace std;

//  [1]. 创建估计器节点
Estimator estimator;
//  [2]. 上一帧IMU数据的时间戳
double last_imu_time = 0;
//  [3]. 存储IMU信息的容器-普通队列,需要关注一端插入和删除,以及随机访问 
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
//  [4]. 保证只有一个线程操作 存储 imu信息的容器 和 存储 image信息的容器, 为之定做的锁
std::mutex cam_imu_buf_lock;
//  [4]. 存储图像特征信息的容器
std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;
//  [5]. 第一帧图像信息的flag
bool init_feature = false;
//  [6]. 创建一个条件变量, 用来再次激活判断锁
std::condition_variable con;
//  [7]. 等待imu来数据的总次数
int sum_of_wait_imu = 0;
//  [8]. 估计器进程和VIO进程锁
std::mutex estimator_lock;
//  [9]. 上一帧IMU数据的时间戳, 最开始的时候是-1
double last_time = -1;

double last_imu_t = 0, last_img_t = 0;
double first_imu_t = 0;

//  [10]. 存储回环帧的容器
std::queue<sensor_msgs::PointCloudConstPtr> relo_buf;

// void registerPub(ros::NodeHandle& n)
// {

// }

bool first_debug = true;
//  把匹配得到的数据的时间戳写入txt文件,实际查看时间戳数据, 重要的debug
void writeDebug(std::vector<std::pair
               <std::vector<sensor_msgs::ImuConstPtr>, 
                sensor_msgs::PointCloudConstPtr>> &paired_datas)
{
    std::ofstream ofs;
    if(first_debug)
    {
        first_imu_t = paired_datas[0].first[0]->header.stamp.toSec();
        ofs.open("/home/zjj/MyCode/myVinsPrj/src/visual_hendend/debug_log/time.txt", std::ios::out);
        first_debug = false;
    }
    else
        ofs.open("/home/zjj/MyCode/myVinsPrj/src/visual_hendend/debug_log/time.txt", std::ios::out|std::ios::app);
    if(!ofs.is_open())
    {
        ROS_ERROR("error!");
        return;
    }
    for(auto &paired_data : paired_datas)
    {
        ofs << "imu_time_stamp" << endl;
        for(auto &imu_data : paired_data.first)
        {
            ofs << imu_data->header.stamp.toSec()-first_imu_t << endl;
        }
        ofs << "image_time_stamp" << endl;
        ofs << paired_data.second->header.stamp.toSec()+estimator.td-first_imu_t << endl;
    }
    ofs.close();
}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> 
GetMeasurements()
{
    // ROS_DEBUG("Paring begin!");
    // [1]. 创建一个存储返回值的容器
    std::vector<std::pair
        <std::vector<sensor_msgs::ImuConstPtr>, 
        sensor_msgs::PointCloudConstPtr>> paired_datas;

    auto& temp_imu = imu_buf;
    auto& temp_feature = feature_buf;

    /*
    ROS_ERROR("error0");
    if(!temp_imu.empty())
    {
        std::cout << "temp_imu:" << temp_imu.back()->header.stamp.toSec() - last_imu_t << endl;
        last_imu_t = temp_imu.back()->header.stamp.toSec();
    }
    if(!temp_feature.empty())
    {
        std::cout << "temp_feature:" << temp_feature.back()->header.stamp.toSec() - last_img_t << endl;
        last_img_t = temp_feature.back()->header.stamp.toSec();
    }
    */

    //  [2]. 开始进行数据配对
    while(ros::ok())
    {
        //  [3]. 如果 imu_buf 容器 或 feature_buf 容器为空,直接返回空容器
        // ROS_ERROR("error1");
        if( imu_buf.empty() || feature_buf.empty() )
        {
            break;
        }

        //  注意时间偏移模型  t_imu = t_cam + td
        /****IMU和CAM数据存在以下三种情况****/
        //           ------------------------------->  time
        
        //  [4]. imu 队首-------------------队尾
        //       cam                            队首-   -   -   -   -   -队尾
        //  imu 数据不够,等待 imu 数据再来        
        // ROS_ERROR("error2");
        double imu_buf_back_time = imu_buf.back()->header.stamp.toSec();
        double feature_buf_front_time = feature_buf.front()->header.stamp.toSec();
        if( imu_buf_back_time <= feature_buf_front_time + estimator.td)
        {
            sum_of_wait_imu++;
            if(MEASUREMENT_Debug)
                ROS_WARN("IMU data is not enough!");
            break;
        }

        //  [5]. imu          队首-------------------队尾
        //       cam 队首-   -   -   -   -   -队尾
        //  cam数据来的太早了,剔除掉前面的cam数据
        // ROS_ERROR("error3");
        double imu_buf_front_time = imu_buf.front()->header.stamp.toSec();
        if( imu_buf_front_time > feature_buf_front_time + estimator.td )
        {
            if(MEASUREMENT_Debug)
                ROS_WARN("Throw image data!");
            feature_buf.pop();
            continue;
        }

        //  [6]. imu 队首-------------------队尾
        //       cam          队首-   -   -   -   -   -队尾
        //  这种情况是合乎常理的
        //  [6.1] 取出一帧图像数据
        // cout << "feature_buf.size():" << feature_buf.size() << endl;
        // cout << "imu_buf.size():" << imu_buf.size() << endl;

        // ROS_ERROR("error4");
        sensor_msgs::PointCloudConstPtr one_feature_msg = feature_buf.front();
        feature_buf.pop();
        //  [6.2] 取出对应的imu数据
        std::vector<sensor_msgs::ImuConstPtr> Imu_correspondings;
        while( imu_buf.front()->header.stamp.toSec() < one_feature_msg->header.stamp.toSec() + estimator.td )
        {   
            // ROS_ERROR("error5");
            // if(!imu_buf.empty())
            // {
            Imu_correspondings.emplace_back(imu_buf.front());
            imu_buf.pop();
            continue;
            // }
            // else
            //     break;
        }
        // ROS_ERROR("error5");
        // Imu_correspondings.emplace_back(imu_buf.front());
        if(Imu_correspondings.empty())
            ROS_WARN("no IMU messages!");
        
        paired_datas.emplace_back(Imu_correspondings, one_feature_msg);
    }
    // ROS_DEBUG("Paring end!");
    return paired_datas;
}


int cnt = 0;
void Process(ros::NodeHandle n)
{
    //  debug 专用的控制频率的类
    ros::Rate rate(30);

    ROS_DEBUG("Process Begin");
    while(ros::ok())
    {
        //  [1]. IMU 与 CAM 数据进行配对
        std::vector<std::pair
            <std::vector<sensor_msgs::ImuConstPtr>, 
            sensor_msgs::PointCloudConstPtr>> paired_datas;
        //  [2]. 配对的时候要操作 imu_buf 队列容器 和 feature_buf 队列容器, 此时需要锁住
        std::unique_lock<std::mutex> paired_lock(cam_imu_buf_lock);
        //  TODO  果真,程序全程卡在wait这里,在反反复复执行lambda函数? 什么原因呢?
        //        是因为条件不满足,一直等待
        con.wait(paired_lock, [&]{
            paired_datas = GetMeasurements();
            return paired_datas.size() != 0;
        });
        paired_lock.unlock();
        estimator_lock.lock();
        //  [3]. 遍历  配对的IMU和CAM数据, 对IMU数据进行预积分, 得到CAM对应的预积分量
        for(auto &paired_data : paired_datas )
        {
            ROS_DEBUG("^^^^^^^^^^^^^^^^^^^^^cnt:%d IMU BEGIN^^^^^^^^^^^^^^^^^^^^^", cnt);
            auto &feature_data = paired_data.second;
            double image_t = feature_data->header.stamp.toSec() + estimator.td;
            
            double dt;      //  前后两帧IMU数据的时间差
            double cur_ax;  //  当前IMU帧的body系x轴加速度
            double cur_ay;  //  当前IMU帧的body系y轴加速度
            double cur_az;  //  当前IMU帧的body系z轴加速度
            double cur_wx;  //  当前IMU帧的body系绕x轴角速度
            double cur_wy;  //  当前IMU帧的body系绕y轴角速度
            double cur_wz;  //  当前IMU帧的body系绕z轴角速度
            //  [4]. 遍历 IMU 数据, 进行预积分, 每组配对的数据只有一帧 CAM
            int i = -1;
            for(auto &imu_data : paired_data.first)
            {
                i++;
                //  [5]. 由于在VIO过程,还会优化td, 因此, 有可能出现有少数几帧 imu 数据 的时间戳 大于 图像时间戳的情况
                double imu_t = imu_data->header.stamp.toSec();
                if ( image_t >= imu_t )
                {
                    //  [6]. current_time < 0 意味着 当前才刚刚开始进入到slam系统, 这是整个系统的第一帧IMU数据
                    if(last_time < 0)
                        last_time = imu_t;
                    //  [7]. 计算相邻两帧IMU数据的时间差, 当前帧与上一帧
                    dt = imu_t - last_time;
                    //  [8]. 必须要>=0才是正常的,这里的检查是双重检查
                    ROS_ASSERT(dt>=0);
                    last_time = imu_t;
                    //  [9]. 获取原始的基于body系的加速度和角速度
                    cur_ax = imu_data->linear_acceleration.x;
                    cur_ay = imu_data->linear_acceleration.y;
                    cur_az = imu_data->linear_acceleration.z;
                    cur_wx = imu_data->angular_velocity.x;
                    cur_wy = imu_data->angular_velocity.y;
                    cur_wz = imu_data->angular_velocity.z;
                    //  [10]. 进行预积分计算
                    estimator.processIMU(dt, Eigen::Vector3d(cur_ax, cur_ay, cur_az), Eigen::Vector3d(cur_wx, cur_wy, cur_wz));
                }
                else
                {
                    //  这里你修改了,不知道效果好不好? 即使有效果, 也不会特别明显
                    //  TODO: 是建立在这种情况, 每次只会进依次, td不会调整太多, 你可以作一些修改
                    int j = i;
                    while( paired_data.first[--j]->header.stamp.toSec() > image_t );
                    if( j != (i-1) )
                    {
                        cur_ax = paired_data.first[j]->linear_acceleration.x;
                        cur_ay = paired_data.first[j]->linear_acceleration.y;
                        cur_az = paired_data.first[j]->linear_acceleration.z;
                        cur_wx = paired_data.first[j]->angular_velocity.x;
                        cur_wy = paired_data.first[j]->angular_velocity.y;
                        cur_wz = paired_data.first[j]->angular_velocity.z;
                    }
                    //  对这种情况的IMU数据,进行线性插值计算
                    double dt_1 = imu_t - image_t;
                    last_time = imu_t;
                    double dt_2 =  image_t - paired_data.first[j]->header.stamp.toSec();
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double k1 = dt_2 / ( dt_1 + dt_2 );
                    cur_ax = cur_ax + k1 * ( imu_data->linear_acceleration.x - cur_ax );
                    cur_ay = cur_ay + k1 * ( imu_data->linear_acceleration.y - cur_ay );
                    cur_az = cur_az + k1 * ( imu_data->linear_acceleration.z - cur_az );
                    cur_wx = cur_wx + k1 * ( imu_data->angular_velocity.x - cur_wx );
                    cur_wy = cur_wy + k1 * ( imu_data->angular_velocity.y - cur_wy );
                    cur_wz = cur_wz + k1 * ( imu_data->angular_velocity.z - cur_wz );
                    estimator.processIMU(dt_2, Eigen::Vector3d(cur_ax, cur_ay, cur_az), Eigen::Vector3d(cur_wx, cur_wy, cur_wz));
                }
            }
            ROS_DEBUG("^^^^^^^^^^^^^^^^^^^^^cnt:%d IMU END^^^^^^^^^^^^^^^^^^^^^", cnt);
            //  TODO: 暂时待写
            //  [11]. 设置回环帧
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            //  [11.1]. 当回环帧容器里面有消息, 取出最新的回环帧
            while( !relo_buf.empty() )
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            ROS_DEBUG("^^^^^^^^^^^^^^^^^^^^^cnt:%d VISION BEGIN^^^^^^^^^^^^^^^^^^^^^", cnt);
            ROS_DEBUG("processing vision data with stamp %lf", feature_data->header.stamp.toSec());

            //  [12]. 处理图像数据
            //  <特征id, < (相机id, 特征数据) >  >
            std::map<int, std::vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            //  [12.1] 遍历图像里面的特征点
            // ROS_INFO("feature_data->points.size():%lu", feature_data->points.size());
            for( int i=0; i<feature_data->points.size(); i++ )
            {
                //  [12.2] 取出 齐次相机坐标, 特征点像素坐标, 特征点id, 特征点的光流速度
                double x = feature_data->points[i].x;
                double y = feature_data->points[i].y;
                double z = feature_data->points[i].z;

                double u = feature_data->channels[1].values[i];
                double v = feature_data->channels[2].values[i];

                int feature_id = feature_data->channels[0].values[i];

                double x_velocity = feature_data->channels[3].values[i];
                double y_velocity = feature_data->channels[4].values[i];

                //  [12.3] 整合数据, 添加数据
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, u, v, x_velocity, y_velocity;
                image[feature_id].emplace_back(0, xyz_uv_velocity);
            }
            //  [12.4] 正式处理图像数据
            estimator.processImage(image, feature_data->header);

            //  [12.5] 发布点云数据
            pubPointCloud(estimator, feature_data->header);
            //  TODO: 还有发布其他的数据
            ROS_DEBUG("^^^^^^^^^^^^^^^^^^^^^cnt:%d VISION END^^^^^^^^^^^^^^^^^^^^^", cnt++);
        }

        //  [13]. 解锁估计器线程锁
        estimator_lock.unlock();
        
        //  [14]. 进入VIO的 updata TODO:
        
        
        if(MEASUREMENT_Debug)
        {
            writeDebug(paired_datas);
            rate.sleep();
        }
    }
}

void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //  [1]. 如果当前帧IMU数据时间戳 小于 上一帧数据时间戳, 则意味着IMU数据混乱
    double cur_imu_time = imu_msg->header.stamp.toSec();
    if(cur_imu_time <= last_imu_time)
    {
        ROS_WARN("Imu message in disorder!");
        return;
    }
    last_imu_time = cur_imu_time;

    //  [2]. 添加imu信息
    cam_imu_buf_lock.lock();
    imu_buf.push(imu_msg);
    // cout << "imu_buf.size():" << imu_buf.size() << endl;
    cam_imu_buf_lock.unlock();
    con.notify_one();
    // cout << imu_buf.size() << endl;
    // ROS_INFO("Normally!");

}

void ImageCallback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //  [1]. 跳过第一帧图像特征信息,因为其不包含光流数据
    if(!init_feature)
    {
        init_feature = true;
        return;
    }
    cam_imu_buf_lock.lock();
    feature_buf.push(point_msg);
    if(MEASUREMENT_Debug)
        std::cout << "Proportion:" << (double) imu_buf.size() / feature_buf.size()  << endl;
    cam_imu_buf_lock.unlock();
    con.notify_one();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "esitimator_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    //  [1]. 读取参数
    readParameters();
    //  [2]. 估计器初始化
    estimator.setParameter();
    ROS_DEBUG("wait image and imu...");
    //  [3]. 注册发布者,暂时待定...
    registerPub(n);
    //  [4]. 订阅 imu数据,图像特征数据,复位数据
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC_NAME, 2000, ImuCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker_node/feature", 2000,  ImageCallback);
    
    //  [5]. 估计器初始化和VIO核心线程
    std::thread estimator_init_vio(Process, n);

    // debug(estimator);

    ros::spin();
    return 0;
}