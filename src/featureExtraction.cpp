/*************************************************
--------------------------------------------------
功能简介:
    对经过运动畸变校正之后的当前帧激光点云，计算每个点的曲率，进而提取角点、平面点（用曲率的大小进行判定）。

订阅：
    1、订阅当前激光帧运动畸变校正后的点云信息，来自ImageProjection。

发布：
    1、发布当前激光帧提取特征之后的点云信息，包括的历史数据有：运动畸变校正，点云数据，初始位姿，姿态角，有效点云数据，角点点云，平面点点云等，发布给MapOptimization；
    2、发布当前激光帧提取的角点点云，用于rviz展示；
    3、发布当前激光帧提取的平面点点云，用于rviz展示。
**************************************************/  
#include "utility.h"
#include "lio_sam/cloud_info.h"
#include "tic_toc.h"

/**
 * 激光点曲率
*/
struct smoothness_t{ 
    float value; // 曲率值
    size_t ind;  // 激光点一维索引
};

/**
 * 曲率比较函数，从小到大排序
*/
struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    // 发布当前激光帧提取特征之后的点云信息
    ros::Publisher pubLaserCloudInfo;
    // 发布当前激光帧提取的角点点云
    ros::Publisher pubCornerPoints;
    // 发布当前激光帧提取的平面点点云
    ros::Publisher pubSurfacePoints;
    
    // 发布当前激光帧提取的GROUND点点云
    ros::Publisher pubGroundPoints;

    // 发布当前激光帧提取的RoadEdge点点云
    ros::Publisher pubRoadEdgePoints;

    // 当前激光帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    // 当前激光帧角点点云集合
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    // 当前激光帧平面点点云集合
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
    // 当前激光帧平面点点云集合
    pcl::PointCloud<PointType>::Ptr groundCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    // 当前激光帧点云信息，包括的历史数据有：运动畸变校正，点云数据，初始位姿，姿态角，有效点云数据，角点点云，平面点点云等
    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    // 当前激光帧点云的曲率
    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    // 特征提取标记，1表示遮挡、平行，或者已经进行特征提取的点，0表示还未进行特征提取处理
    int *cloudNeighborPicked;
    // 1表示角点，-1表示平面点
    int *cloudLabel;
    int groundScanInd;
    float sensorMountAngle;
    
    /**
     * 构造函数
    */
    FeatureExtraction()
    {
        // 初始化
        initializationValue();

        // 订阅当前激光帧运动畸变校正后的点云信息
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布当前激光帧提取特征之后的点云信息
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        // 发布当前激光帧的角点点云
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        // 发布当前激光帧的面点点云
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        // // 发布当前激光帧的地面点点云
        pubGroundPoints  = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_ground", 1);
        //// 发布当前激光帧的路面角点点云
        pubRoadEdgePoints  = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_roadEdge", 1);

    }

    // 初始化
    void initializationValue()
    {

        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        groundCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
        groundScanInd = 7;
        sensorMountAngle = 0.0;
    }

    /**
     * 订阅当前激光帧运动畸变校正后的点云信息
     * 1、计算当前激光帧点云中每个点的曲率
     * 2、标记属于遮挡、平行两种情况的点，不做特征提取
     * 3、点云角点、平面点特征提取
     *   1) 遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合
     *   2) 认为非角点的点都是平面点，加入平面点云集合，最后降采样
     * 4、发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
    */
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; 
        cloudHeader = msgIn->header; 
        // 当前激光帧运动畸变校正后的有效点云
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); 

        // 计算当前激光帧点云中每个点的曲率
        calculateSmoothness();

        // 标记属于遮挡、平行两种情况的点，不做特征提取
        markOccludedPoints();

        //// 分割地面点
        groundRemoval();
        //// 点云角点、平面点特征提取
        // 1、遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合
        // 2、认为非角点的点都是平面点，加入平面点云集合，最后降采样
        extractFeatures();
        
        // 发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
        publishFeatureCloud();
    }

    /**
     * 计算当前激光帧点云中每个点的曲率
    */
    void calculateSmoothness()
    {
        // 遍历当前激光帧运动畸变校正后的有效点云
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 用当前激光点前后5个点计算当前点的曲率，平坦位置处曲率较小，角点处曲率较大；这个方法很简单但有效
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];

            // 距离差值平方作为曲率
            cloudCurvature[i] = diffRange*diffRange;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;            
            // 存储该点曲率值、激光点一维索引
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    /**
     * 标记属于遮挡、平行两种情况的点，不做特征提取
    */
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points

        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // 当前点和下一个点的range值
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            // 两个激光点之间的一维索引差值，如果在一条扫描线上，那么值为1；如果两个点之间有一些无效点(距离筛选等)被剔除了，可能会比1大，但不会特别大
            // 如果恰好前一个点在扫描一周的结束时刻，下一个点是另一条扫描线的起始时刻，那么值会很大
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系（也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
            // 远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
            if (columnDiff < 10){
                
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            
            // 用前后相邻点判断当前点所在平面是否与激光束方向平行
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            // 平行则标记一下
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void groundRemoval(){
        groundCloud->clear();
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // cloudLabel
        // -1, plane points(not ground points)
        //  0, initial value
        //  1, edge points
        //  2, ground points
        TicToc t_ground;
        for (auto j = 0; j < Horizon_SCAN; ++j){
            for (auto i = 0; i < groundScanInd; ++i){
                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                diffX = extractedCloud->points[upperInd].x - extractedCloud->points[lowerInd].x;
                diffY = extractedCloud->points[upperInd].y - extractedCloud->points[lowerInd].y;
                diffZ = extractedCloud->points[upperInd].z - extractedCloud->points[lowerInd].z;
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;              

                if (abs(angle - sensorMountAngle) <= 10 && extractedCloud->points[lowerInd].z < -0.5){
                  cloudLabel[lowerInd] = 2;
                  cloudLabel[upperInd] = 2;
                  cloudNeighborPicked[lowerInd] = 1;
                  cloudNeighborPicked[upperInd] = 1;
                }
            }
        }

        if (pubGroundPoints.getNumSubscribers() != 0)
        {
            for (auto i = 0; i <= groundScanInd; ++i)
            {
                for (auto j = 0; j < Horizon_SCAN; ++j)
                {
                    auto index = j + i * Horizon_SCAN;
                    if (cloudLabel[index] == 2)
                        groundCloud->push_back(extractedCloud->points[index]);
                }
            }
        }
        //ROS_INFO_STREAM("TIME OF T_G: "<< t_ground.toc() << "\n");
    }


    /**
     * 点云角点、平面点特征提取
     * 1、遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合
     * 2、认为非角点的点都是平面点，加入平面点云集合，最后降采样
    */
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();        
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        //// cloudNeighborPicked 是否筛选过的标志
        //// cloudLabel 类别        
        // 遍历扫描线
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            //// 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
            for (int j = 0; j < 6; j++)
            {
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                /*********************************************************
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                //注意：所有的点云在这里都是以"一维数组"的形式保存
                //startRingIndex和 endRingIndex 在imageProjection.cpp中的 cloudExtraction函数里被填入
                //假设 当前ring在一维数组中起始点是m，结尾点为n（不包括n），那么6段的起始点分别为：
                // m + [(n-m)/6]*j   j从0～5
                // 化简为 [（6-j)*m + nj ]/6
                // 6段的终止点分别为：
                // m + (n-m)/6 + [(n-m)/6]*j -1  j从0～5,-1是因为最后一个,减去1
                // 化简为 [（5-j)*m + (j+1)*n ]/6 -1
                //这块不必细究边缘值到底是不是划分的准（例如考虑前五个点是不是都不要，还是说只不要前四个点），
                 *
                *************************************************** * */
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 按照曲率从小到大排序点云
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 按照曲率从大到小遍历，提取角点
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    // 激光点的索引
                    int ind = cloudSmoothness[k].ind;
                    // 当前激光点还未被处理，且曲率大于阈值，则认为是角点
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        // 每段只取20个角点，如果单条扫描线扫描一周是1800个点，则划分6段，每段300个点，从中提取20个角点
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            // 标记为角点
                            cloudLabel[ind] = 1;
                            // 加入角点点云
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;
                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            //// 这说明不是一个 ring
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }                    
                }

                // 按照曲率从小到大遍历，提取面点
                for (int k = sp; k <= ep; k++)
                {
                    // 激光点的索引
                    int ind = cloudSmoothness[k].ind;                                     
                    // 当前激光点还未被处理，且曲率小于阈值，则认为是平面点
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        // 标记为平面点
                        cloudLabel[ind] = -1;
                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;

                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 平面点和未被处理的点，都认为是平面点，加入平面点云集合
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            // 平面点云降采样
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            // 加入平面点云集合
            // 用surfaceCloudScan来装数据，然后放到downSizeFilter里，
            // 再用downSizeFilter进行.filter()操作，把结果输出到*surfaceCloudScanDS里。
            // 最后把DS装到surfaceCloud中。DS指的是DownSample。
            // 同样角点（边缘点）则没有这样的操作，直接就用cornerCloud来装点云。
            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    /**
     * 清理
    */
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    /**
     * 发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
    */
    void publishFeatureCloud()
    {
        // 清理
        freeCloudInfoMemory();
        // 发布角点、面点点云，用于rviz展示
        cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_ground = publishCloud(&pubGroundPoints, groundCloud, cloudHeader.stamp, lidarFrame);

        // 发布当前激光帧点云信息，加入了角点、面点点云数据，发布给mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    TicToc init_start;
    ros::init(argc, argv, "lio_sam");
    FeatureExtraction FE;
    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
    if (FE.isDebug)
    {
      std::cout << "初始化用时: " << init_start.toc() << "毫秒" << std::endl;
    }
    ros::spin();
    return 0;
}
