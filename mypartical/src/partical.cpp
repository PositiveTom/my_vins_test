#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <vector>
#include <random>
#include <map>
#include <string>
#include <fstream> //[1]
#include <cmath>
#include <boost/math/distributions/normal.hpp>

std::string filepath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/obsever.csv";
std::string groundpath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/ground.csv";
std::string msepath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/mse.csv";
std::string particalpath = "/home/zjj/MyCode/particalFilter/src/mypartical/data/partical.csv";

namespace params
{
    float T = 0.05;
    float freq = 1;
    float time = 4;

    float obs_std = 4;
    float mse_std = 2;

    float x_init = 10;
    //  初始位置置信度
    float init_std = 10;
}

//  粒子滤波需要设置的参数有如下
//  1. 状态方程噪声协方差矩阵
//  2. 观测方差噪声协方差矩阵
//  3. 初始状态（用于状态方程递推）
//  4. 初始状态置信度（如果我们认为初始状态不准确，就把对应的初始状态协方差设置大一点）
//  5. 粒子的个数
//  6. 重采样的频率

//  总结：粒子滤波对状态方程的精度要求不高，即使建模很差，也能依靠观测方程得出稍好的结果
//  有一个精度高的传感器，胜过一切滤波算法，粒子滤波对初值不敏感，前提是你得设置好初值的置信度
//  粒子滤波的本质还是贝叶斯滤波，只不过用粒子来近似概率密度函数而已罢了

class stateEquation
{
public:
    stateEquation(){}
    virtual float transformation(float& xk_1) = 0;
};

//  采样粒子
typedef struct partical
{
    float weight;
    float state;
    int id;
    /* data */
}*ParticalPtr, Partical;

class stateEquation;
//  操作粒子的类
class operatepartical
{
public:

    operatepartical(int nums):partical_nums(nums)
    {
        particalset.reserve(nums);
    }

    //  (1)产生粒子
    void produce_particles(float& init_std)
    {
        std::random_device generator;
        std::normal_distribution<float> noise(0, init_std);

        for(int i=0; i<partical_nums; i++)
        {
            ParticalPtr ptr = new Partical;
            ptr->id = i;
            ptr->state = params::x_init +  noise(generator);
            ptr->weight = 1.0 / partical_nums;
            particalset.push_back(ptr);
        }
    }

    //  (2)预测步更新粒子位置, 得到先验概率密度函数的采样形式
    void prediction_module(float& Qk_std, stateEquation* functor)
    {
        std::random_device generator;
        std::normal_distribution<float> noise(0, Qk_std);

        std::vector<ParticalPtr>::iterator it = particalset.begin();
        for(; it!=particalset.end(); it++)
        {
            (*it)->state = functor->transformation((*it)->state) + noise(generator);
        }
    }

    //  (3)更新步更新粒子权重, 得到后验概率的采样形式，并进行归一化
    void update_module(float& Rk_std, float& obsever_data)
    {
        std::vector<ParticalPtr>::iterator it = particalset.begin();
        float sum_weight = 0;
        for(; it!=particalset.end(); it++)
        {
            float pdfR = 1.0 / ( sqrt(2 * M_PI) * Rk_std ) * 
            exp( - (obsever_data - (*it)->state)*(obsever_data - (*it)->state) /
                    (2 * Rk_std * Rk_std) );
            (*it)->weight *= pdfR;
            sum_weight += (*it)->weight;
        }

        //  归一化
        std::vector<ParticalPtr>::iterator it2 = particalset.begin();
        float sum = 0;
        for(; it2 != particalset.end(); it2++)
        {
            (*it2)->weight /= sum_weight;
            sum += (*it2)->weight;
        }
        // ROS_INFO("sum:%f",sum);
    }

    //  (4) 重采样，缓解粒子退化的问题
    void resamples_module()
    {
        std::random_device generator;
        std::uniform_real_distribution<float> randomnumber;

        //  对粒子进行排序，选择冒泡排序算法
        std::vector<ParticalPtr>::iterator it1 = particalset.begin();
        std::vector<ParticalPtr>::iterator it2;
        for(; it1!=particalset.end(); it1++)
        {
            it2 = it1 + 1;
            for(int j = 0; it2!=particalset.end(); it2++, j++)
            {
                if((*(it1+j))->weight > (*(it2))->weight)
                {
                    std::iter_swap(it1+j, it2);
                }
            }
        }

        //  采样的索引
        std::vector<int> sampleindex; 
        for(int i = 0; i<partical_nums; i++)
        {
            float randomnums = randomnumber(generator);
            float weigt_sum = 0;
            int index = 0;
            for(; index < particalset.size() ; index++)
            {
                weigt_sum += particalset[index]->weight;
                if(randomnums < weigt_sum)
                {
                    sampleindex.push_back(index);
                    break;
                }
            }
        }

        //  创建局部变量用来暂存重采样得到粒子
        std::vector<Partical> localpartical;
        localpartical.reserve(sampleindex.size());
        float sum_weight = 0;
        for(int i=0; i<sampleindex.size(); i++)
        {
            sum_weight += particalset[sampleindex[i]]->weight;
            localpartical.push_back( *particalset[sampleindex[i]] );
        }

        //  把这些粒子的值放回到堆区, 并且归一化权重
        std::vector<ParticalPtr>::iterator myit = particalset.begin();
        for(int i = 0 ;myit!=particalset.end(); myit++, i++)
        {
            localpartical[i].weight /= sum_weight;
            *(*myit) = localpartical[i];
        }

    }

    //  (5) 得到后验期望值
    float posteriorExpected()
    {
        float expected;
        std::vector<ParticalPtr>::iterator it = particalset.begin();
        for(; it!=particalset.end() ; it++)
        {
            expected += (*it)->weight * (*it)->state; 
        }
        return expected;
    }

public:
    int partical_nums;
    std::vector<ParticalPtr> particalset;
};

//  非线性状态空间模型 
class mysinFunctor:public stateEquation
{
public:
    mysinFunctor(float _A, float _B):A(_A),B(_B){}
    virtual float transformation(float& xk_1)
    {
        std::random_device generator;
        std::normal_distribution<float> noise(0, params::mse_std);

        //  TODO C++的pow的第一个参数必须得是正数才行
        float part2 = 0;
        if (xk_1 < 0)
        {
            part2 =  - 3 * std::pow( std::abs(xk_1) , 1.0 / 3.0) * params::T * params::T;
        }
        else
        {
            part2 =  3 * std::pow( xk_1 , 1.0 / 3.0) * params::T * params::T;
        }
        return [&]{return xk_1 + part2 + 
             3 * std::pow( xk_1 * xk_1, 1.0 / 3.0) * params::T +
             params::T * params::T * params::T + noise(generator); }();
    }

public:
    float A;
    float B; 
};

//  观测器 y = t^3 + noise
class observer
{
public:
    observer(){}
    void simulate_observer
    (std::map<float, float>& data,const float& time, 
            const float& T, const float& freq, float std)
    {
        std::default_random_engine generator;
        std::normal_distribution<float> noise(0, std);

        int nums = int(time / T );
        // data.reserve(nums);
        float t = 0;
        for(int i = 0; i<nums; i++)
        {
            // float output = 4 * sin( 2 * M_PI * freq * t ) + noise(generator);
            float output = std::pow(t, 3) + noise(generator);
            data.emplace(t, output);
            t += T;
            // data.emplace_back(output);
        }
    }
public:
    std::vector<float> observer_data;

};

class savedata
{
public:
    static void savefile
    (const std::string& filepath, std::map<float, float>& data)
    {
        //  [2]
        std::ofstream ofs;
        ofs.open(filepath.c_str(), std::ios::out);

        //  [3]
        if(!ofs.is_open())
        {
            ROS_ERROR("Failed to open file");
            return;
        }

        //  [4]
        std::map<float, float>::iterator it = data.begin();

        if(filepath.rfind("ground") != std::string::npos)
        {
            ROS_INFO("ground");
            ofs << "gt" << "," << "gvalue" << "\n";
        }
        else if(filepath.rfind("obsever") != std::string::npos)
        {
            ROS_INFO("noise");
            ofs << "nt" << "," << "nvalue" << "\n";
        }
        else if(filepath.rfind("mse") != std::string::npos)
        {
            ROS_INFO("mse");
            ofs << "mt" << "," << "mvalue" << "\n";
        }
        else if(filepath.rfind("partical") != std::string::npos)
        {
            ROS_INFO("partical");
            ofs << "part" << "," << "parvalue" << "\n";
        }

        for( ; it != data.end(); it++)
        {
            ofs << it->first << "," << it->second << "\n";
        }

        //  [5]
        ofs.close();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypartical");
    ros::NodeHandle n;
    ros::Rate r(1);

    ros::Publisher pub = n.advertise<std_msgs::Float32>("partical_prediction", 1000);
    std::unique_ptr<observer> myobsever = std::unique_ptr<observer>(new observer);
    
    // std::vector<float> obsever_data;
    std::map<float, float> obsever_data;
    myobsever->simulate_observer
        (obsever_data, params::time, params::T , params::freq, params::obs_std);

    std::map<float, float> ground_data;
    myobsever->simulate_observer
        (ground_data, params::time, params::T , params::freq, 0);

    std::map<float, float> mse_data;
    stateEquation* myequation = new mysinFunctor(1, 2);
    if(ros::ok())
    {   
        int nums = int(params::time / params::T );
        float t = 0;
        float xk = params::x_init;
        mse_data.emplace(t, xk);
        for(int i = 0; i< nums-1; i++)
        {
            t += params::T;
            xk = myequation->transformation(xk);
            mse_data.emplace(t, xk);
        }
    }

    std::map<float, float> partical_data;
    //  (1) 创建50个粒子，并且声明初值的置信度
    std::shared_ptr<operatepartical> operate = 
                            std::make_shared<operatepartical> (50);
    operate->produce_particles(params::init_std);
    
    std::map<float, float>::iterator it = obsever_data.begin();

    partical_data.emplace((*it).first, params::x_init);
    it++;
    for(; it!=obsever_data.end(); it++)
    {
        //  (2) 预测步更新粒子位置，得到新粒子的位置
        operate->prediction_module(params::mse_std, myequation);

        //  (3) 更新步更新粒子权重
        operate->update_module(params::obs_std, (*it).second);

        //  (4) 重采样
        operate->resamples_module();

        //  得到后验期望值
        partical_data.emplace((*it).first, operate->posteriorExpected());
    }

    savedata::savefile(filepath, obsever_data);
    savedata::savefile(groundpath, ground_data);
    savedata::savefile(msepath, mse_data);
    savedata::savefile(particalpath, partical_data);

    return 0;
}


//  https://zhuanlan.zhihu.com/p/413281749
//  plotjuggle是一个动态观测数据插件，不适合仿真使用, 如果一定要使用这个插件，就只能通过保存csv文件进行显示
//  如果想绘制任意的xy轴图，则先按住ctrl键，鼠标左键选中想要添加的数据，再用右键放入图中，会提示我们选择哪一个当作x轴或者y轴的



//  c++的错误包括 语法错误，逻辑错误，运行错误
//  语法错误和逻辑错误比较简单，运行错误比较难发现，一般通过c++异常处理机制（Exception）进行处理
//  异常处理机制最根本的在于catch关键字后面的 exceptionType variable  
//  exceptionType是异常类型，它指明了当前的catch可以处理什么样的异常；variable是一个变量，用于接收异常信息
//  会创建一份数据，包含错误信息，异常类型可以是int，char，float，bool等基本类型。
//  c++本身以及标准库中函数抛出的异常，都是exception类或者其子类，也就是说抛出异常，会创建
//  一个exception类或者其子类的对象；
//  catch的异常可以是任何类型，没法提前预测，所以不能在编译阶段判断类型是否正确，只有等到程序正常运行后才能知道
//  catch在程序过程中将实参和形参相匹配
//  如果不希望处理异常数据，可以不用写variable
//  c++中，使用throw显示地抛出异常，throw exceptiondata，exceptiondata可以是int，bool，float等等基本类型