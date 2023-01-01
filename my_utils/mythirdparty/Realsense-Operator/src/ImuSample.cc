#include "ImuSample.h"

ros::Publisher pub_camera;
ros::Publisher pub_imu;
sensor_msgs::Image m_image;
sensor_msgs::Imu m_imu;

mySampleRealsense::mySampleRealsense()
{
    pipe = new rs2::pipeline();
    cfg = new rs2::config();
    cfg->enable_stream(RS2_STREAM_ACCEL);
    cfg->enable_stream(RS2_STREAM_GYRO);
    cfg->enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe_profile = pipe->start(*cfg);
}

mySampleRealsense::mySampleRealsense(bool call, ros::NodeHandle& n)
{
    pub_camera = n.advertise<sensor_msgs::Image>("/my_camera", 10);
    pub_imu = n.advertise<sensor_msgs::Imu>("my_imu",10);
    pipe = new rs2::pipeline();
    cfg = new rs2::config();
    cfg->enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg->enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg->enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe->start(*cfg, mySampleRealsense::streamCallback);
}

// mySampleRealsense::~mySampleRealsense()
// {
//     pipe->stop();
// }

bool mySampleRealsense::SampleImu(sensor_msgs::ImuPtr imu)
{
    bool temp = false, temp2 = false;
    rs2::frameset frameset = pipe->wait_for_frames();
    if( rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL) )
    {   
        //  ros header的seq是什么?
        //  milliseconds 毫秒
        // double accel_time = accel_frame.get_timestamp();
        rs2_vector accel_sample = accel_frame.get_motion_data();
        imu->linear_acceleration.x = accel_sample.x;
        imu->linear_acceleration.y = accel_sample.y;
        imu->linear_acceleration.z = accel_sample.z;
        temp = true;
        // cout << "time:"<< accel_time/1000.0 << "s," << "Aceel:" << accel_sample.x << "," << accel_sample.y << "," << accel_sample.z << endl;
    }
    if(rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
        double gyro_time = gyro_frame.get_timestamp();
        rs2_vector gyro_sample = gyro_frame.get_motion_data();
        uint32_t sec, nsec;
        
        fromMsec(sec, nsec, gyro_time);

        imu->header.stamp.sec = sec;
        imu->header.stamp.nsec = nsec;
        ++imu->header.seq;

        imu->angular_velocity.x = gyro_sample.x;
        imu->angular_velocity.y = gyro_sample.y;
        imu->angular_velocity.z = gyro_sample.z;
        temp2 = true;
        // cout <<"time:"<< gyro_time/1000.0 << "s," << "Aceel:" << gyro_sample.x << "," << gyro_sample.y << "," << gyro_sample.z << endl;
    }
    return (temp && temp2);
}

bool mySampleRealsense::SamplePhoto(sensor_msgs::ImagePtr img)
{
    rs2::frameset frameset = pipe->wait_for_frames();
    
    if(rs2::frame color_frame = frameset.get_color_frame())
    {
        cv::Mat color(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::namedWindow("photo", cv::WINDOW_NORMAL);
        cv::imshow("photo", color);
        cv::waitKey(1);
    }

}

//  [1]
void mySampleRealsense::streamCallback(const rs2::frame& frame)
{
    static double frame_fs = 0;
    static double gyro_fs = 0;
    static double accel_fs = 0;
    static bool temp1 = false, temp2 = false;

    // cout << "frame_fs:" << frame_fs << endl;
    // cout << "gyro_fs:" << gyro_fs << endl;
    // cout << "accel_fs:" << accel_fs << endl;

    //  [2]
    if(rs2::frameset frames = frame.as<rs2::frameset>())
    {
        static int i = 1;
        static double last_frame_time = 0;
        //  [3]
        rs2::video_frame color_frame = frames.get_color_frame();
        //  [4]
        double frame_time = frames.get_timestamp();
        //  [5]
        cv::Mat color(cv::Size(640,480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
        int frame_freq = int( 1000 / (frame_time - last_frame_time) );
        // if(last_frame_time > 20)
        // {
        //     cout << "Frame FPS:" << int( 1000 / (frame_time - last_frame_time) ) << endl;            
        // }
        last_frame_time = frame_time;
        i++;
        if(frame_freq > 20 && frame_freq < 40 && (i%4==0))
        {
            i = 1;
            uint32_t sec, nsec;
            fromMsec(sec, nsec, frame_time);
            // m_image.header.stamp.sec = sec;
            // m_image.header.stamp.nsec = nsec;

            cv_bridge::CvImage cv_image;
            cv_image.image = gray;
            cv_image.encoding = "mono8";
            cv_image.header.stamp.sec = sec;
            cv_image.header.stamp.nsec = nsec;
            m_image = *cv_image.toImageMsg();

            pub_camera.publish(m_image);
        }
        // cv::namedWindow("photo", cv::WINDOW_NORMAL);
        // cv::imshow("photo", color);
        // cv::waitKey(1);        
    }
    //  [2]
    else if(rs2::motion_frame imu_frames = frame.as<rs2::motion_frame>())
    {
        //  [3]
        if(imu_frames.get_profile().stream_name() == "Gyro" && !temp1)
        {
            static double gyro_last_time = 0;
            //  [4]
            rs2_vector gyro_data = imu_frames.get_motion_data();
            //  [5]
            double gyro_time = imu_frames.get_timestamp();
            int gyro_freq = int( 1000 / (gyro_time - gyro_last_time) );
            // if(gyro_last_time > 0)
            // {
            //     cout << "Gyro FPS:" << int( 1000 / (gyro_time - gyro_last_time) ) << endl;
            // }
            gyro_last_time = gyro_time;
            if(gyro_freq>150 && gyro_freq<250)
            {
                uint32_t sec, nsec;
                fromMsec(sec, nsec, gyro_time);
                m_imu.header.stamp.sec = sec;
                m_imu.header.stamp.nsec = nsec;
                ++m_imu.header.seq;

                m_imu.angular_velocity.x = gyro_data.x;
                m_imu.angular_velocity.y = gyro_data.y;
                m_imu.angular_velocity.z = gyro_data.z;

                temp1 = true;
            }
        }
        //  [3]
        else if(imu_frames.get_profile().stream_name() == "Accel" && !temp2)
        {
            static double accel_last_time = 0;
            //  [4]
            rs2_vector accel_data = imu_frames.get_motion_data();
            //  [5]
            double accel_time = imu_frames.get_timestamp();
            int accel_freq = int( 1000 / (accel_time - accel_last_time) );
            // if(accel_last_time > 0)
            // {
            //     cout << "Accel FPS:" << int( 1000 / (accel_time - accel_last_time) ) << endl;
            // }
            accel_last_time = accel_time;
            if(accel_freq>150 && accel_freq<250)
            {
                m_imu.linear_acceleration.x = accel_data.x;
                m_imu.linear_acceleration.y = accel_data.y;
                m_imu.linear_acceleration.z = accel_data.z;

                temp2 = true;
            }
        }
        static int imp = 0;
        if((!temp1&&temp2)||(temp1&&!temp2))
        {
            ++imp;
        }
        if(imp==2)
        {
            temp1 = false;
            temp2 = false;
            imp = 0;
        }
        if(temp1 && temp2)
        {
            temp1 = false;
            temp2 = false;
            imp = 0;
            pub_imu.publish(m_imu);
        }
    }
}

void mySampleRealsense::GetIntrinsics()
{
    rs2::stream_profile color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2::video_stream_profile video_profile = color_stream.as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = video_profile.get_intrinsics();
    cout << "fx:" << intrinsics.fx << endl;
    cout << "fy:" << intrinsics.fy << endl;
    cout << "ppx:" << intrinsics.ppx << endl;
    cout << "ppy:" << intrinsics.ppy << endl;
    cout << "k1:" << intrinsics.coeffs[0] << endl;
    cout << "k2:" << intrinsics.coeffs[1] << endl;
    cout << "p1:" << intrinsics.coeffs[2] << endl;
    cout << "p2:" << intrinsics.coeffs[3] << endl;
    cout << "k3:" << intrinsics.coeffs[4] << endl;

}

void mySampleRealsense::GetExtrinsics()
{
    rs2::stream_profile from_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    rs2::stream_profile to_stream = pipe_profile.get_stream(RS2_STREAM_ACCEL);
    try
    {
        // Given two streams, use the get_extrinsics_to() function to get the transformation from the stream to the other stream
        rs2_extrinsics extrinsics = from_stream.get_extrinsics_to(to_stream);
        std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
        std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
        std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
        std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get extrinsics for the given streams. " << e.what() << std::endl;
    }
}

// 查询到的外参
// cam to gyro  Tbc   
// Translation Vector : [-0.020636,0.00489123,0.011653]
// Rotation Matrix    : [0.999811,-0.0193249,-0.00200949]
//                    : [0.0193176,0.999807,-0.00359821]
//                    : [0.00207864,0.00355871,0.999992]

// cam to accel
// Translation Vector : [-0.020636,0.00489123,0.011653]
// Rotation Matrix    : [0.999811,-0.0193249,-0.00200949]
//                    : [0.0193176,0.999807,-0.00359821]
//                    : [0.00207864,0.00355871,0.999992]

// gyro to accel
// Translation Vector : [0,0,0]
// Rotation Matrix    : [1,0,0]
//                    : [0,1,0]
//                    : [0,0,1]
