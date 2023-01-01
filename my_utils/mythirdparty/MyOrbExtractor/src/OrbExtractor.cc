#include "OrbExtractor.h"

static void computeOrbDescriptor
(cv::KeyPoint& kp, const cv::Mat &img, const cv::Point* pattern, uchar* desc)
{
    //  [1]. 角度制转为弧度制
    float angle = kp.angle * M_PI / 180.;
    float a = (float)cos(angle), b = (float)sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kp.pt.y), cvRound(kp.pt.x));
    int step = (int)img.step; 

    //  c-->w
    //  cos \theta  -sin \theta 
    //  sin \theta   cos \theta

    #define GETVALUE(idx) \
            center[cvRound(((pattern+idx)->x * a - (pattern+idx)->y * b)) \
            + cvRound(( (pattern+idx)->x * b + (pattern+idx)->y * a )) * step]

    //  [2]. 一次计算256位描述子的一个字节8位, 计算32次即计算完成
    //  每一位取一对点,8位取8对点
    for(int i=0; i<32; i++, pattern+=16)
    {
        int t0, t1, val;
        // cout << (pattern)->x << endl;
        // cout << (pattern)->y << endl;
        //  取出点,旋转一定的角度,再到原始图中取灰度值
        t0 = GETVALUE(0); t1 = GETVALUE(1);
        val = t0 < t1;
        t0 = GETVALUE(2); t1 = GETVALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GETVALUE(4); t1 = GETVALUE(5);
        val |= (t0 < t1) << 2;   
        t0 = GETVALUE(6); t1 = GETVALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GETVALUE(8); t1 = GETVALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GETVALUE(10); t1 = GETVALUE(11);
        val |= (t0 < t1) << 5;  
        t0 = GETVALUE(12); t1 = GETVALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GETVALUE(14); t1 = GETVALUE(15);
        val |= (t0 < t1) << 7;
        desc[i] = (uchar) val;
    }
}


/**
 * @brief 
 * @param pattern
 */ 
static void computeDescriptors
(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
const vector<cv::Point>& pattern)
{
    //  [1]. 申请存储描述子的空间内存
    descriptors = cv::Mat::zeros(keypoints.size(), 32, CV_8UC1);

    //  [2]. 针对每一个特征点计算描述子
    for(size_t i=0; i<keypoints.size(); i++)
    {
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
    }
}

//  计算特征点的旋转角度
static float IC_Angle
(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max)
{
    int m10 = 0, m01 = 0;

    //  [1]. 让指针指向圆形块正中间的像素
    const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

    //  [2]. 计算正中间那一行的灰度加权和, u轴
    for(int u=-HALF_PATCH_SIZE; u<=HALF_PATCH_SIZE; u++)
        m10 += u * center[u];
    
    //  [3]. 计算其余行的加权和
    int step = (int)image.step1();  //图像每一行的数据元素的个数 8字节时=step
    for(int v=1; v<HALF_PATCH_SIZE; v++)
    {
        int v_sum = 0;
        int d = u_max[v];
        for( int u=-d; u<=d; u++ )
        {
            int val_plus = center[u+step*d], val_minus = center[u-step*d];
            //  对于v轴来说,对称是相减, 因为本身就是在u轴上进行加减运算
            v_sum += val_plus - val_minus;
            m10 += u*(val_plus + val_minus);
        }
        m01 += v_sum;
    }
    return cv::fastAtan2((float)m01, (float)m10);
}

//  计算特征点的旋转角度
static void computeOrientation(const cv::Mat& img,  
std::vector<cv::KeyPoint>& keypoints, 
const vector<int>& umax)
{
    for(std::vector<cv::KeyPoint>::iterator it=keypoints.begin(); it!=keypoints.end();it++)
    {
        it->angle = IC_Angle(img, it->pt, umax);
    }
}

MyOrbExtractor::MyOrbExtractor()
{
    //  [1]. 初始化金字塔参数
    nlevels_ = 8;   //  金字塔层数
    scale_ = 1.2;   //  金字塔放缩比例
    nfeatures_ = 150;  //  所有金字塔图像总共提取的特征点数量
    mvImagePyramid_.resize(nlevels_);   //  存放金字塔图像的容器
    mvInvScaleFactor_.resize(nlevels_); //  存放每层金字塔图像与原始图像的比例系数
    mvScaleFactor_.resize(nlevels_);    //  存放原始图像与每层金字塔图像的比例系数
    mvScaleFactor_[0] = 1.f;
    for(int i=1; i<nlevels_; i++)
    {
        mvScaleFactor_[i] = mvScaleFactor_[i-1]*scale_;
    }
    for(int i=0; i<nlevels_; i++)
    {
        mvInvScaleFactor_[i] = 1.0f / mvScaleFactor_[i];
    }
    mnFeaturesPerLevel_.resize(nlevels_);   //  存放每层金字塔应该提取的特征数的容器
    float invScale = 1.0f / scale_;
    //  每层金字塔图像期望的特征点数
    float nDesiredFeaturesPerScale = nfeatures_ * (1. - invScale) / (1. - pow(invScale, nlevels_) );
    int sumFeatures = 0;
    for(int level=0; level<nlevels_-1; level++)
    {
        mnFeaturesPerLevel_[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel_[level];
        nDesiredFeaturesPerScale *= invScale;
    }
    //  最后一层特征点数,由减法运算得到
    mnFeaturesPerLevel_[nlevels_-1] = std::max(nfeatures_-sumFeatures, 0);

    //  [2]. 初始FAST角点提取参数
    iniThFAST_ = 20;
    minThFAST_ = 7;

    //  [3]. 初始化计算灰度质心的参数, 这里属实是使用了一些歪门斜道计算
    umax.resize(HALF_PATCH_SIZE + 1);
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));
    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }

    //   [4]. 初始化计算brief描述子的参数
    //  数组类型指针转为Point类型指针
    const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;
    const int npoints = 512;    //  计算一个描述子需要总共 npoints 个点
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern_)); // 把point填入pattern_容器

}

void MyOrbExtractor::FeatureExtractor
(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors)
{
    //  [1]. 得到图像
    cv::Mat color = _image.getMat();

    //  [2]. 转为灰度图
    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    assert(gray.type() == CV_8UC1);

    //  [3]. 计算图像金字塔
    ComputePyramid(gray);

    //  [4]. 计算每层金字塔图像的特征点和旋转角度  , allKeypoints就是用来存放不同金字塔层图像的特征点的
    std::vector<std::vector<cv::KeyPoint>> allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);

    //  [5]. 开始计算描述子
    cv::Mat descriptor;
    //  统计所有特征点个数
    int nKeyPoints = 0;
    for(int level=0; level<nlevels_; level++)
    {
        nKeyPoints += (int)allKeypoints[level].size();
    }
    // cout << "nKeyPoints:" << nKeyPoints << endl;
    if(nKeyPoints == 0)
        descriptor.release();
    else
    {
        //  为存储描述子的mat对象申请内存空间, 需要知道总的特征点数
        _descriptors.create(nKeyPoints, 32, CV_8U);
        descriptor = _descriptors.getMat();
    }
    _keypoints.clear();
    _keypoints.reserve(nKeyPoints);

    int offset = 0;
    for(int level=0; level<nlevels_; level++)
    {
        //  [5.1]. 遍历每层的金字塔图像特征点容器
        std::vector<cv::KeyPoint> &keypoint = allKeypoints[level];
        //  得到当前层图像特征点的个数
        int nkeypointsLevel = keypoint.size();

        if(nkeypointsLevel == 0)
            continue;
        
        //  [5.2]. 对每层金字塔层图像进行高斯模糊化
        cv::Mat workingMat = mvImagePyramid_[level].clone();
        cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2., 2., cv::BORDER_REFLECT_101);

        //  [5.3]. 计算描述子
        cv::Mat desc = descriptor.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoint, desc, pattern_);

        //  偏移量增加,为取出下一次的描述子内存做准备
        offset += nkeypointsLevel;

        //  TODO
        //  [5.4]. 缩放特征点坐标 !!! 非常重要,金字塔图像提取出特征点之后,最后一步又变换到原图像上去
        if(level != 0)
        {
            float scale = mvScaleFactor_[level];
            for(std::vector<cv::KeyPoint>::iterator it=keypoint.begin(); it!=keypoint.end();it++)
            {
                it->pt *= scale;
            }
        }
        //  [5.5]. 最后就可以汇总特征点集合了, 全部位于原始图像上, 应用insert方法直接全部插入,一行代码搞定,而不用一一push_back了
        _keypoints.insert(_keypoints.end(), keypoint.begin(), keypoint.end());
    }   
}

void MyOrbExtractor::ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints)
{
    //  [1]. 外层容器申请内存
    allKeypoints.resize(nlevels_);

    //  定义理想的网格尺寸,只是理想,不一定严格等于,这是ORB采用的思想
    const int W = 30;

    //  [2]. 遍历每一个金字塔图像
    for(int level=0; level<nlevels_; level++)
    {

        cv::Mat imgclone;
        if(ComputeKeyPointDebug)
            imgclone = mvImagePyramid_[level].clone();
        
        //  存放特征点的容器
        std::vector<cv::KeyPoint> vToDistributeKeys_;
        vToDistributeKeys_.reserve(nfeatures_*10);

        //  [3]. 只在金字塔图像的中心区域内进行提取特征点
        const int minBorderX = EDGE_THRESHOLD - 3;
        const int minBorderY = EDGE_THRESHOLD - 3;
        const int maxBorderX = mvImagePyramid_[level].cols - EDGE_THRESHOLD + 3;
        const int maxBorderY = mvImagePyramid_[level].rows - EDGE_THRESHOLD + 3;

        //  [4]. 计算金字塔图像中心区域的宽和高
        const float width = maxBorderX - minBorderX;
        const float height = maxBorderY - minBorderY;

        //  [5]. 利用理想网格尺寸,计算每行每列应该有的网格数量
        const int nCols = width / W;
        const int nRows = height / W;

        //  [6]. 利用应该有网格数量,计算每个标准网格的宽和高
        const int wCell = ceil( width / nCols );
        const int hCell = ceil( height / nRows );

        //  [7]. 按行遍历网格
        for(int i=0; i<nRows; i++)
        {
            //  [8]. 计算网格的像素v轴的开始值 和 这个网格的终止最大值
            //  之所以留6,是为了提取不重复的FAST角点
            const float iniY = minBorderY + hCell * i;
            float maxY = iniY + hCell + 6;

            //  [9]. 限制网格的区域, 有了[6]步骤才会有这里的步骤成立
            if(iniY >= maxBorderY - 3)
                continue;
            if(maxY > maxBorderY)
                maxY = maxBorderY;
            
            //  [10]. 按列遍历网格
            for(int j=0; j<nCols; j++)
            {
                //  [11]. 同上的操作
                const float iniX = minBorderX + wCell * j;
                float maxX = iniX + wCell + 6;
                if(iniX >= maxBorderX - 3)
                    continue;
                if(maxX > maxBorderX)
                    maxX = maxBorderX;

                //  [12]. 创建关键点容器, FAST提取角点, 注意opencv的rowRange用法
                std::vector<cv::KeyPoint> vKeysCell;
                cv::FAST(mvImagePyramid_[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                         vKeysCell, iniThFAST_, true);

                //  [13]. 如果上述阈值(20)没有提取到角点,那么就换用更小的阈值进行特征点提取
                if(vKeysCell.empty())
                {   
                    //  肯定需要这一步,因为有的网格几乎没有特征点
                    cv::FAST(mvImagePyramid_[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                            vKeysCell, minThFAST_, true);                    
                }
                
                if(ComputeKeyPointDebug)
                {
                    cv::Point2f pt1, pt2;
                    pt1.x = iniX;
                    pt1.y = iniY;
                    pt2.x = maxX;
                    pt2.y = maxY;
                    cv::rectangle(imgclone, pt1, pt2, cv::Scalar(255));
                }

                //  [14]. 如果提取到了特征点, 就把特征点坐标进行坐标变换, 变换到原始金字塔图像上
                if(!vKeysCell.empty())
                {
                    for(std::vector<cv::KeyPoint>::iterator it=vKeysCell.begin();
                        it!=vKeysCell.end(); it++)
                    {
                        (*it).pt.x += j*wCell;
                        (*it).pt.y += i*hCell;
                        vToDistributeKeys_.push_back((*it));
                    }
                }
            }
        }
        //  [15]. 提前为每一层分配 nfeatures_ 个特征点的内存
        std::vector<cv::KeyPoint> &keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures_);

        //  [16]. 四叉树均匀化角点
        keypoints = DistributeOctTree(vToDistributeKeys_, minBorderX, maxBorderX, 
            minBorderY, maxBorderY, mnFeaturesPerLevel_[level], level);

        //  [17]. 修正特征点坐标,带上原来忽略的边界
        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor_[level];
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }

        if(ComputeKeyPointDebug)
        {        
            std::string namewindow = std::to_string(level);
            cv::namedWindow(namewindow, cv::WINDOW_NORMAL);
            cv::imshow(namewindow, imgclone);
        }
    }

    //  [18]. 计算特征点的旋转角度
    for(int level=0; level<nlevels_; level++)
    {
        computeOrientation(mvImagePyramid_[level], allKeypoints[level], umax);
        if(OrienteDebug)
        {
            cv::Mat img = mvImagePyramid_[level].clone();
            for(auto& keypoint:allKeypoints[level])
            {
                cv::Point2i pt1, pt2;
                pt1.x = keypoint.pt.x - 5;
                pt1.y = keypoint.pt.y - 5;
                pt2.x = keypoint.pt.x + 5;
                pt2.y = keypoint.pt.y + 5;
                cv::rectangle(img, pt1, pt2, cv::Scalar(255));
            }
            std::string name = std::to_string(level);
            cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
            cv::imshow(name, img);
        }
    }
    if(OrienteDebug || ComputeKeyPointDebug)
    {
        cv::waitKey(1);
    }
}


std::vector<cv::KeyPoint> MyOrbExtractor::DistributeOctTree
(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
{
    // cout << "vToDistributeKeys.size():" << vToDistributeKeys.size() << endl;

    //  [1]. 计算有多少个初始节点
    //  这里的意义在于,如果图像比较宽,则分成w/h份,一般640x480图像一开始只有一个node
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    //  [2]. 计算初始节点的宽
    const float hX = static_cast<double>(maxX-minX)/nIni;

    //  [3]. 创建一个存放节点的list容器, 创建一个存放节点指针的vector容器
    //  TODO: 难道是想利用list的插入删除效率高的优点吗?然后结合vector快速随机查找的优点
    std::list<MyExtractorNode> lNodes;
    std::vector<MyExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    //  [4]. 遍历初始节点,创建初始节点
    for(int i=0; i<nIni; i++)
    {
        MyExtractorNode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(i), 0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1), 0);
        ni.BL = cv::Point2i(ni.UL.x, maxY - minY);   
        ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        //  [5]. 添加节点到list容器, 节点指针添加到vector容器
        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    //  [6]. 往初始节点里面添加特征点, kp.pt.x/hX用来选择是哪个初始节点
    // cout << "vToDistributeKeys.size():" << vToDistributeKeys.size() << endl;
    for(int i=0; i<vToDistributeKeys.size(); i++)
    {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    //  [7]. 遍历节点容器, 查看是否分裂
    std::list<MyExtractorNode>::iterator it = lNodes.begin();
    for(;it!=lNodes.end();)
    {
        //  [7.1] 节点只有一个特征点,就不再分裂了
        if(it->vKeys.size() == 1)
        {
            it->bNoMore = true;
            it++;
        }
        //  [7.2] 节点没有特征点,就删除这个节点
        else if(it->vKeys.empty())
        {
            //  TODO 删除当前节点,返回下一个节点的迭代器
            it = lNodes.erase(it);
        }
        else
        {
            it++;
        }
    }
    //  [8]. 进行四叉树分裂
    bool bFinish = false;
    int iteration = 0;
    //  TODO 创建了一个容器, 用于存放待分裂的节点迭代器,以及对应的特征点数
    vector<pair<int, MyExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);
    while(!bFinish)
    {
        if(OctTreeDebug)
        {
            cv::Mat img = mvImagePyramid_[level].clone();
            cv::Mat temp;
            temp = img(cv::Rect(EDGE_THRESHOLD - 3, EDGE_THRESHOLD - 3, img.cols - EDGE_THRESHOLD + 3, img.rows - EDGE_THRESHOLD + 3));
            for(list<MyExtractorNode>::iterator mit=lNodes.begin();mit!=lNodes.end();mit++)
            {
                cv::Point2i pt1, pt2;
                pt1 = mit->UL;
                pt2 = mit->BR;
                cv::rectangle(temp, pt1, pt2, cv::Scalar(255));
            }
            std::string name = std::to_string(level);
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::imshow(name, img);
            cv::waitKey(0);
        }


        iteration++;

        //  [9]. 得到list节点容器的size, 以及节点容器的首迭代器
        int prevSize = lNodes.size();

        //  [9.1]. 先对父节点进行排序, 优先把含特征点数多的放到前面
        lNodes.sort([](MyExtractorNode &a, MyExtractorNode &b)
                    {return a.vKeys.size()>b.vKeys.size();});

        it = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();
        //  [10]. 开始一次只属于当前父节点的分裂
        while(it != lNodes.end())
        {
            //  [11]. 如果此节点不能分裂了,就跳转
            if(it->bNoMore)
            {
                it++;
                continue;
            }
            //  [12]. 如果此节点可以分裂
            else
            {
                // cout << "it->vKeys.size():" << it->vKeys.size() << endl;
                //  [13]. 创建四个子节点, 并且进行分裂
                MyExtractorNode n1, n2, n3, n4;
                it->DivideNode(n1, n2, n3, n4);

                //  [14]. 如果子节点包含特征点,就添加到 list 容器中, 前入
                if(n1.vKeys.size() > 0 )
                {
                    lNodes.push_front(n1);
                    //  [14.1] 如果此子节点还能继续分裂,则添加lNodes中此节点的迭代器到 vSizeAndPointerToNode 容器中
                    if(n1.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n1.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size() > 0 )
                {
                    lNodes.push_front(n2);
                    //  [14.1] 如果此子节点还能继续分裂,则添加lNodes中此节点的迭代器到 vSizeAndPointerToNode 容器中
                    if(n2.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n2.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size() > 0 )
                {
                    lNodes.push_front(n3);
                    //  [14.1] 如果此子节点还能继续分裂,则添加lNodes中此节点的迭代器到 vSizeAndPointerToNode 容器中
                    if(n3.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n3.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size() > 0 )
                {
                    lNodes.push_front(n4);
                    //  [14.1] 如果此子节点还能继续分裂,则添加lNodes中此节点的迭代器到 vSizeAndPointerToNode 容器中
                    if(n4.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n4.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                //  [15]. 再删除掉被分裂的节点, 分裂把自己给搞没了
                //  这里的erase函数不会返回前面节点的迭代器 !!! 重要
                //  而是只返回后面节点的迭代器,因此这里只的父节点遍历完之后,返回了end迭代器
                it = lNodes.erase(it);
                if(lNodes.size()>N)
                    break;
                continue;
            } 
        }
        //  [16]. 如果分裂的节点数大于等于N或者没有变化了,则分裂结束
        // cout << "lNodes.size():" << lNodes.size() << endl;
        // cout << "N:" <<N << endl;
        // cout << "prevSize:" << prevSize << endl;
        if((int)lNodes.size() >= N || (int)lNodes.size() == prevSize )
        {
            bFinish = true;
            // cout << "lNodes.size():" << lNodes.size() << endl;
        }
        //  [17]. 用于预判下一步的节点数 本人觉得这一步可以省略!!!
        // else if( ((int)lNodes.size()+nToExpand*3)>N )
        // {

        // }
    }

    //  [17]. 在每个节点内获得最好的特征点
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures_);

    //  [17.1] 首先遍历每个节点
    for(std::list<MyExtractorNode>::iterator it=lNodes.begin(); it!=lNodes.end(); it++)
    {
        //  [17.2] 取出每个节点中的特征点容器
        std::vector<cv::KeyPoint> &vNodeKeys = it->vKeys;
        
        //  [17.3] 取出第一个特征点的指针
        cv::KeyPoint* pKP = &vNodeKeys[0];

        //  [17.4] 得到此特征点的响应值, 原来FAST角点还有一个响应值
        float maxResponse = pKP->response;

        //  [17.5] 与其他特征点比较,取最大的响应值
        for(size_t k=1; k<vNodeKeys.size(); k++)
        {
            if(vNodeKeys[k].response > maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }
        //  [17.6] 添加角点
        vResultKeys.push_back(*pKP);
    }
    return vResultKeys;

}

void MyExtractorNode::DivideNode
(MyExtractorNode &n1, MyExtractorNode &n2, MyExtractorNode &n3, MyExtractorNode &n4)
{
    //  [1]. 折半分
    const int halfX = ceil(static_cast<float>(UR.x - UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y - UL.y)/2);

    //  [2]. 定义子节点的边界
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX, UL.y);
    n1.BL = cv::Point2i(UL.x, UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX, UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x, UR.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(BL.x+halfX, BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n1.BR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //  [3]. 把节点分配给子节点
    for(size_t i=0; i<vKeys.size(); i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x < n1.UR.x)
        {
            //  在 1 3 节点内
            if(kp.pt.y < n1.BR.y)
            {
                //  在 1 节点内
                n1.vKeys.push_back(kp);
            }
            else
            {
                //  在 3 节点内
                n3.vKeys.push_back(kp);
            }
        }
        else
        {
            //  在 2 4 节点内
            if(kp.pt.y < n1.BR.y)
            {
                //  在 2 节点内
                n2.vKeys.push_back(kp);
            }
            else
            {
                n4.vKeys.push_back(kp);
            }
        }
    }

    //  [4]. 判断这些子节点是否能继续分裂
    if(n1.vKeys.size() == 1)
        n1.bNoMore = true;
    if(n2.vKeys.size() == 1)
        n2.bNoMore = true;
    if(n3.vKeys.size() == 1)
        n3.bNoMore = true;
    if(n4.vKeys.size() == 1)
        n4.bNoMore = true;    
}


void MyOrbExtractor::ComputePyramid(cv::Mat &image)
{
    for(int level = 0; level < nlevels_; level++)
    {
        //  [1]. 取出每层金字塔图像与原始图像的比例系数
        float scale = mvInvScaleFactor_[level];

        //  [2]. 计算缩放后的图像尺寸  Size(width, height )
        cv::Size zoom(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));

        //  [3]. 创建一个Mat模板, 相当于在堆区申请了一块内存, 用于暂存金字塔图像
        cv::Mat temp(zoom, image.type());
        
        //  [4]. Mat 与 Mat 的直接赋值是引用, 因此操作temp, 相当于操作mvImagePyramid_[level]
        mvImagePyramid_[level] = temp;
        if(level != 0)
        {
            //  [5]. 把上一次的图像,缩放到给定的尺寸, 使用线性插值算法
            cv::resize(mvImagePyramid_[level-1], mvImagePyramid_[level], zoom, 0, 0, cv::INTER_LINEAR);

            //  [6]. 把图像存放到图像金字塔, temp此时指向第二层的图像金字塔
            cv::copyMakeBorder(mvImagePyramid_[level], temp, 0, 0, 0, 0, cv::BORDER_REFLECT101+cv::BORDER_ISOLATED);
        }
        else
        {
            //  [5]. 把原始图像复制到金字塔图像容器
            cv::copyMakeBorder(image, temp, 0, 0, 0, 0, cv::BORDER_REFLECT101);
        }
    }

    if(PyramidDebug)
    {
        for(int i=0; i<nlevels_; i++)
        {
            cv::Mat img = mvImagePyramid_[i];
            cout << "The " << i << "th pyramid photo" << endl;
            cout << "size:" << img.cols << "," << img.rows << endl;
            std::string namewindow = std::to_string(i);        
            cv::namedWindow(namewindow, cv::COLOR_BGR2GRAY);
            cv::imshow(namewindow, img);
            cv::waitKey(1);
        }
    }
}