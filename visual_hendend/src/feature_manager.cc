#include "feature_manager.h"

FeatureManager::FeatureManager()
{
    ric_.setIdentity();
}

void FeatureManager::setRic(Eigen::Matrix3d &ric)
{
    ric_ = ric;
}

bool FeatureManager::addFeatureCheckParallax
(int frame_count, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_INFO("input feature: %d", (int)image.size());
    ROS_INFO("num of feature: %d", getFeatureCount());
    // ROS_DEBUG("");
    //  视差, 两个图像帧的视差
    double parallax_sum = 0.;
    //  可以 计算 视差的特征点 数
    int parallax_num = 0;

    //  相邻两帧 共视 特征点的个数
    int last_track_num = 0;

    //  [1]. 遍历特征点
    for(auto &per_feature : image)
    {
        //  [2]. 针对每个特征点都创建一个类来进行管理
        // cout << per_feature.second[0].second.segment<3>(0) << endl;
        // ROS_INFO("per_feature.second[0].size():%ld", per_feature.second.size());
        FeaturePerFrame feature_per_point(per_feature.second[0].second);

        //  [3]. 取出当前特征点的id
        int feature_id = per_feature.first;

        //  [4]. 在特征管理器 容器中 查找此特征点是否存在, 通过特征点id来查看
        auto it = std::find_if(features_.begin(), features_.end(), [feature_id](const FeaturePerId& it)
                {
                    return it.feature_id_ == feature_id;
                });
        //  [5]. 如果存在, 那么就要告知特征管理器, 此特征点在 滑动窗口中的 第frame_count 帧中出现了一次
        if(it != features_.end())
        {
            it->feature_per_frame.emplace_back(feature_per_point);
            last_track_num++;
        }
        //  [6]. 否则如果不存在, 那么就要唯一地添加此特征点 到 特征管理器 唯一存储特征点 的容器中
        else
        {
            FeaturePerId feature_point(feature_id, frame_count);
            feature_point.feature_per_frame.push_back(feature_per_point);
            features_.push_back(feature_point);
        }
    }

    //  [7]. 如果是 整个 slam系统的 前两帧图像, 或者, 相邻两图像帧的共视特征点个数 小于30个, 那么就可以被看作关键帧
    if( frame_count < 2 || last_track_num < 30 )
        return true;

    //  [8]. 计算 两帧图像的 视差
    for(auto &feature : features_)
    {
        //  [8.1] 如果 此特征点的 开始帧 <= frame_count - 2, 即在前帧及以前就出现了
        //   此特征点的结束帧 >= frame_count - 1
        //   综上所述, 需要连续三帧及其以上出现, 当前帧必须要出现
        if( feature.start_frame_ <= frame_count - 2 && 
            feature.start_frame_ + (int)feature.feature_per_frame.size() - 1 >= frame_count - 1 )
        {
            parallax_sum += compensatedParallax2(feature, frame_count);
            parallax_num++;
        }   
    }
    //  [9]. 如果没有可以计算视差的特征点, 则意味着这帧于其他帧差别太大, 当作关键帧
    if( parallax_num == 0 )
    {
        return true;
    }
    else
    {
        //  [10]. TODO: 这里涉及一个视差阈值的问题
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * 613.1862293052357);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }

}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for(auto &feature : features_)
    {
        feature.used_num = feature.feature_per_frame.size();

        //  [1]. 如果看到此特征点的帧数大于等于2, 并且, 开始的图像帧的索引小于 WINDOW_SIZE - 3 (也就是滑动窗口中的前8张图像)
        if( feature.used_num >= 2 && feature.start_frame_ < WINDOW_SIZE - 3 )
        {
            ++cnt;
        }

    }
    return cnt;
}

double FeatureManager::compensatedParallax2
(const FeaturePerId& feature, int frame_count)
{
    double ans = 0.;

    //  [1]. 得到 特征点 分别在 倒数第二帧和倒数第三帧的引用
    const FeaturePerFrame& feature_3 = feature.feature_per_frame[frame_count - feature.start_frame_ - 2 ]; // 倒数第三帧
    const FeaturePerFrame& feature_2 = feature.feature_per_frame[frame_count - feature.start_frame_ - 1 ]; // 倒数第二帧

    //  [2]. 取出特征点 分别 在 倒数第三帧 和 倒数第二帧 相机坐标系上的 齐次相机坐标
    Eigen::Vector3d point_3 = feature_3.xy1;
    Eigen::Vector3d point_2 = feature_2.xy1;

    //  [3]. 只取前两行元素
    Eigen::Vector2d d_xy1 = (point_3 - point_2).segment<2>(0);

    //  [4]. 计算欧式距离
    ans = std::max(ans, d_xy1.norm() );

    return ans;
}

void FeatureManager::removeFront(int frame_count)
{
    //  [1]. 遍历所有特征点
    for( auto it=features_.begin(), it_next=features_.begin(); it!=features_.end(); it=it_next )
    {
        it_next++;
        //  [2]. 如果开始帧等于frame_count, 只需要--就行, 因为前移了一帧
        if( it->start_frame_ == frame_count )
        {
            it->start_frame_--;
        }
        else
        {
            //  [3]. 如果此特征点原本就没有被次新帧看见, 则跳过
            if(it->endFrame() < frame_count-1)
            {
                continue;
            }
            //  [4]. 剔除掉特征点里面相关帧中的次新帧
            //  [4.1]. 得到frame_count-1帧在容器it->feature_per_frame中的索引
            int j = frame_count - 1 - it->start_frame_;

            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);

            //  [4.2]. 如果此时此特征点不被任何帧看见, 则剔除掉
            if( it->feature_per_frame.size() == 0 )
            {
                features_.erase(it);
            }
        }
    }
}

void FeatureManager::removeBack()
{
    //  [1]. 遍历所有特征点
    for( auto it = features_.begin(), it_next = features_.begin(); it!=features_.end(); it=it_next)
    {
        it_next++;
        //  [2]. 如果此特征点的开始帧不是最老帧, 只需要开始帧序号减1即可
        if( it->start_frame_ != 0 )
            it->start_frame_--;
        else
        {
            //  [3]. 如果此特征点的开始帧是最老帧
            //  [3.1] 删除此特征点的相关帧(被marg掉的帧)
            it->feature_per_frame.erase(it->feature_per_frame.begin());

            //  [3.2] 如果特征点此时只有没有共视帧, 那么就删除掉这个特征点
            if(it->feature_per_frame.size() == 0)
                features_.erase(it);
        }
    }
}

int FeaturePerId::endFrame()
{
    return start_frame_ + feature_per_frame.size() - 1;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> 
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    //  [1]. 遍历特征管理器中的 存储 特征点 的容器
    for(auto &feature : features_)
    {
        //  [2]. 如果开始帧小于等于frame_count_l, 并且结束帧大于等于frame_count_r, 则认为是共视特征点
        //  因为特征点都是存在于连续的特征上的
        if( feature.start_frame_ <= frame_count_l && feature.endFrame() >= frame_count_r )
        {
            Eigen::Vector3d point_l(Eigen::Vector3d::Zero());
            Eigen::Vector3d point_r = Eigen::Vector3d::Zero();

            //  [3]. 从指定帧索引上取出对应的特征点
            point_l = feature.feature_per_frame[frame_count_l - feature.start_frame_].xy1;
            point_r = feature.feature_per_frame[frame_count_r - feature.start_frame_].xy1;
            
            corres.emplace_back(std::make_pair(point_l, point_r));
        }
    }
    return corres;
}