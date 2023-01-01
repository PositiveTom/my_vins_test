//  check the timestamp of imu sensor with plotjugger

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

std::string SensorTopic;
float multiple;

class checkSTS
{
private:
    /* data */
    ros::Subscriber subImu;
    ros::Publisher pubTime;
    bool flag;
    sensor_msgs::ImuConstPtr lastMsg, curMsg;

private:
    checkSTS(ros::NodeHandle& n);
    // ~checkSTS();

    void subImuCallBack(const sensor_msgs::ImuConstPtr& msg)
    {
        if(flag)
        {
            flag = false;
            lastMsg = msg;
            return;
        }
        curMsg = msg;
        std_msgs::Float32 data;
        data.data = multiple * ( curMsg->header.stamp.toSec() - lastMsg->header.stamp.toSec() );
        pubTime.publish(data);
    }

public:
    static checkSTS* m_instance;
    static checkSTS*
    getInstance(ros::NodeHandle& n)
    {
        if( m_instance == nullptr )
        {
            m_instance = new checkSTS(n);
        }
        return m_instance;
    }
};
checkSTS* checkSTS::m_instance = nullptr;


checkSTS::checkSTS(ros::NodeHandle& n)
{
    subImu = n.subscribe(SensorTopic, 1000, &checkSTS::subImuCallBack, this);
    pubTime = n.advertise<std_msgs::Float32>("/time", 1000);
    flag = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "checkSensorTime");
    ros::NodeHandle n("~");
    n.getParam("/checkSensorTime/SensorTopic", SensorTopic);
    n.getParam("/checkSensorTime/multiple", multiple);
    ROS_INFO("multiple:%f", multiple);
    std::shared_ptr<checkSTS> m_check 
                = std::shared_ptr<checkSTS>(checkSTS::getInstance(n));
    ros::spin();
}


