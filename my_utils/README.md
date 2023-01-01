# Description

```mermaid
graph LR
A(搭建自己的VINS系统依次使用了的模块)-->B(1.采集相机图片,标定相机内参)
A-->C(2.采集IMU数据,利用imu_tils工具标定IMU内参)
A-->D(3.采集相机和IMU数据,利用kalibr工具标定相机和IMU外参)
```

这个ros工具包主要记录搭建VINS系统会用到的工具模块。

```mermaid
graph LR
A(VINS工具包)-->B(检测IMU时间戳my_check_time)
A-->BC(my_samplephoto.cpp)-->B1(采集相机图片)
A-->C(张正友非线性优化相机标定)
A-->D(my_smoothproperty.cpp)-->D1(验证李群的光滑性质)
A-->E(my_sampleRealsense.cpp)-->E1(从官网SDK采集IMU数据)
A-->F(my_sample_cam_imu.cpp)-->F1(从官网SDK采集IMU数据和相机数据)
A-->G(my_visualhendendtest.cpp)-->G1(验证特征点算法,用于采集IMU和相机数据时查看相机数据)
A-->H(mythirdparty/MyOrbExtractor)-->H1(使用四叉树均匀化方法计算ORB角点)
A-->L(mythirdparty/NLP)-->L1(手动实现非线性最小二乘优化算法,LM算法和DogLeg算法)
```

## my_check_time的使用

用于检查IMU传感器的时间戳是否正确, 正常而言应该是线性增长的, 不排除有一些劣质传感器时间戳不对劲的情况

+ 运行对应的 my_check_time.launch文件即可
+ 1. Set the topic of imu message  and the multiple of time in checkSensorTime.launch.
  2. Run the checkSensorTime.launch.
  3. Play your rosbag.
  4. Open Plotjuggler to check the timestamp of sensor.

## 采集相机图片

采集相机图片，目前主要用于采集标定图片

> 源码：
>
> 1. my_samplephoto.cpp
> 2. my_sample.launch

> 需要修改的地方：
>
> 1. my_sample.launch 的4个参数

运行 my_sample.launch文件即可，摁'q'键用于保存图片。

## 张正友非线性优化相机标定

基于张正友标定法, 使用非线性优化框架进行相机标定。

源码：

> 1. my_calibration.cpp 和 my_calibration.h
> 2. my_calibration,launch

> 需要数据：
>
> 1. 以 1,2,3,4....  .jpg 命名的棋盘图像

> 需要修改的地方：
>
> 1. my_calibration.launch 文件的四个参数

> 运行：
>
> ```
> roslaunch my_utils my_calibration.launch
> ```
