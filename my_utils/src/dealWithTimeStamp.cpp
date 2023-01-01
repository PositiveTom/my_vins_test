#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <mutex>
#include <thread>
#include <deque>
#include <condition_variable>

std::mutex accLock, gyroLock;
std::condition_variable conAcc, conGyro;

double IMUFREQ;
double T;
double threshold;
double Tthreshld;

//  监控采样频率, 控制采样频率, 实现线性插值
//  如何高效率实现数据对齐呢？
class DWTS
{
public:
    DWTS(ros::NodeHandle& n)
    {   
        subRawAcc = n.subscribe
            ("/camera/accel/sample", 2000, &DWTS::rawAccCallback, this);
        subRawGyro = n.subscribe
            ("/camera/gyro/sample", 2000, &DWTS::rawGyroCallback, this);
        
        pubNewImu = n.advertise<sensor_msgs::Imu>("/m_imu", 2000);

    }

    void insertAccData( sensor_msgs::Imu& curAcc , double& t_step )
    {
        // [1]. 计算插入的个数
        int nums = static_cast<int>(t_step / T) - 1;

        // [2]. 插入数据
        for(int i=0; i<nums; i++)
        {
            sensor_msgs::ImuPtr data = 
                boost::make_shared<sensor_msgs::Imu>();
            //  插入时间戳
            data->header = curAcc.header;
            data->header.stamp = 
                ros::Time().fromSec(lastAcc.header.stamp.toSec() + (i+1) * T );
            
            //  插入加速度数据
            data->linear_acceleration.x = lastAcc.linear_acceleration.x + 
                ( curAcc.linear_acceleration.x - lastAcc.linear_acceleration.x ) * (i+1)* T / t_step;
        
            data->linear_acceleration.y = lastAcc.linear_acceleration.y + 
                ( curAcc.linear_acceleration.y - lastAcc.linear_acceleration.y ) * (i+1)* T / t_step;

            data->linear_acceleration.z = lastAcc.linear_acceleration.z + 
                ( curAcc.linear_acceleration.z - lastAcc.linear_acceleration.z ) * (i+1)* T / t_step;

            rawAcc_buf.push_back( data );
            // ROS_INFO("hello2");
        }
        // rawAcc_buf.push_back(curAcc);
    }

    void rawAccCallback(const sensor_msgs::ImuConstPtr& acc )
    {
        // ROS_INFO("hello2");
        static bool i = true;
        if(i)
        {
            i = false;
            lastAcc.header = acc->header;
            lastAcc.angular_velocity = acc->angular_velocity;
            // lastAcc = acc;
            return;
        }
        // curAcc = acc;
        curAcc.header = acc->header;
        curAcc.angular_velocity = acc->angular_velocity;
        double t_step = curAcc.header.stamp.toSec() - lastAcc.header.stamp.toSec();
        ROS_INFO("%f", 1.0/t_step);
        // double freq = 1.0 / t_step;
        // if(freq > 1.5 * IMUFREQ)
        //     return;
        if( t_step < Tthreshld )
            return;
        // ROS_INFO("hello1");
        accLock.lock();
        if(t_step < 2 * T )
        {
            rawAcc_buf.push_back(acc);
            // ROS_INFO("hello2");
        }
        else
        {
            // ROS_INFO("hello3");
            insertAccData(curAcc, t_step);
            rawAcc_buf.push_back(acc);
            // ROS_INFO("hello4");
        }
        accLock.unlock();
        conAcc.notify_one();
        // lastAcc = acc;
        lastAcc.header = acc->header;
        lastAcc.angular_velocity = acc->angular_velocity;
    }

    void rawGyroCallback(const sensor_msgs::ImuConstPtr& gyro)
    {
        static bool i = true;
        if(i)
        {
            i = false;
            //  TODO: 很奇怪为什么不能这样赋值
            // lastGyro = gyro;
            lastGyro.header = gyro->header;
            lastGyro.angular_velocity = gyro->angular_velocity;
            return;
        }
        // curGyro = gyro;
        curGyro.header = gyro->header;
        curGyro.angular_velocity = gyro->angular_velocity;
        double t_step = curGyro.header.stamp.toSec() - lastGyro.header.stamp.toSec();
        // double freq = 1.0 / t_step;
        // if(freq > 1.5 * IMUFREQ)
        //     return;
        if( t_step < Tthreshld )
            return;
        gyroLock.lock();
        rawGyro_buf.push_back(gyro);
        gyroLock.unlock();
        conGyro.notify_one();
        // lastGyro = gyro;
        lastGyro.header = gyro->header;
        lastGyro.angular_velocity = gyro->angular_velocity;
        // ROS_INFO("hello");
    }

    void getMeasurements
    (std::deque<sensor_msgs::ImuConstPtr>& target, std::deque<sensor_msgs::ImuConstPtr>& origin)
    {
        for(;origin.begin()!=origin.end();)
        {
            target.push_back(origin.front());
            origin.pop_front();
        }
    }



    void pubNewImuThread()
    {
        while(ros::ok())
        {
            // ROS_INFO("I am coming");
            std::deque<sensor_msgs::ImuConstPtr> mesureAcc;
            std::deque<sensor_msgs::ImuConstPtr> mesureGyro;

            std::unique_lock<std::mutex> accUniqueLock(accLock);
            conAcc.wait(accUniqueLock, [&](){
                if(rawAcc_buf.size() > 5)
                {
                    getMeasurements(mesureAcc, rawAcc_buf);
                    return true;
                }
                else
                    return false;
            });
            // ROS_INFO("hello5");
            accUniqueLock.unlock();  // TODO: 为何这里一定要加？？？
            std::unique_lock<std::mutex> gyroUniqueLock(gyroLock);
            conGyro.wait(gyroUniqueLock, [&](){
                if(rawGyro_buf.size() > 5)
                {
                    getMeasurements(mesureGyro, rawGyro_buf);
                    return true;
                }
                else
                    return false;
            });
            // ROS_INFO("hello6");
            gyroUniqueLock.unlock();
            // ROS_INFO("%d,%d",mesureAcc.size(), mesureGyro.size());
            while( !mesureGyro.empty() && !mesureAcc.empty()  )
            {
                // ROS_INFO("hello7");
                sensor_msgs::ImuConstPtr itGyro = mesureGyro.front();
                mesureGyro.pop_front();
                // ROS_INFO("hello7.5");
                for(;(!mesureGyro.empty() && !mesureAcc.empty()) && (mesureAcc.front()->header.stamp.toSec() > itGyro->header.stamp.toSec() ); )
                {
                    mesureAcc.pop_front();
                }
                if(mesureGyro.empty() || mesureAcc.empty())
                    break;

                // ROS_INFO("hello8");
                sensor_msgs::ImuConstPtr itAcc = mesureAcc.front();
                mesureAcc.pop_front();
                // ROS_INFO("hello9");
                sensor_msgs::Imu data;
                data.header = itAcc->header;
                data.linear_acceleration = itAcc->linear_acceleration;
                // ROS_INFO("hello10");
                sensor_msgs::ImuConstPtr itGyro2 = mesureGyro.front();
                double tStep = itGyro2->header.stamp.toSec() - itGyro->header.stamp.toSec();
                double tGyroAcc = itAcc->header.stamp.toSec() - itGyro->header.stamp.toSec(); 
                // ROS_INFO("hello11");
                data.angular_velocity.x = itGyro->angular_velocity.x + 
                    tGyroAcc / tStep * ( itGyro2->angular_velocity.x - itGyro->angular_velocity.x );
                data.angular_velocity.y = itGyro->angular_velocity.y + 
                    tGyroAcc / tStep * ( itGyro2->angular_velocity.y - itGyro->angular_velocity.y );
                data.angular_velocity.z = itGyro->angular_velocity.z + 
                    tGyroAcc / tStep * ( itGyro2->angular_velocity.z - itGyro->angular_velocity.z );
                // ROS_INFO("hello12");
                pubNewImu.publish(data);
            }
            // while( itAcc->header.stamp.toSec() > rawGyro_buf.front()->header.stamp.toSec() )
            // {
            //     rawGyro_buf.pop_front();
            // }
            // sensor_msgs::ImuConstPtr itGyro = rawGyro_buf.front();
            // rawGyro_buf.pop_front();
        }
    }

private:
    ros::Publisher pubNewImu;
    ros::Subscriber subRawAcc;
    ros::Subscriber subRawGyro;

    // sensor_msgs::ImuConstPtr curAcc, lastAcc;
    // sensor_msgs::ImuConstPtr curGyro, lastGyro;
    sensor_msgs::Imu curAcc, lastAcc;
    sensor_msgs::Imu curGyro, lastGyro;

    std::deque<sensor_msgs::ImuConstPtr> rawAcc_buf;
    std::deque<sensor_msgs::ImuConstPtr> rawGyro_buf;

    std::deque<sensor_msgs::Imu> newImu_buf;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DWTS");
    ros::NodeHandle n;
    n.getParam("/DWTS/ImuSamplingFreq", IMUFREQ);
    n.getParam("/DWTS/threshold", threshold );
    T = 1 / IMUFREQ;
    Tthreshld = threshold * T;

    std::shared_ptr<DWTS> m_DWTS = std::make_shared<DWTS>(n);
    // DWTS my(n);

    std::thread m_thread(&DWTS::pubNewImuThread, m_DWTS);
    ros::spin();

    return 0;
}