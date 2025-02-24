#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <rclcpp/rclcpp.hpp> 

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <opencv/cv.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <flann/flann.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2_ros/buffer.h> 
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/transform_broadcaster.h> 
#include <tf2/LinearMath/Quaternion.h> 
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "rolo_ros2_interfaces/msg/cloud_info_stamp.hpp"
#include "rolo_ros2_interfaces/msg/slope.hpp"
#include "rolo_ros2_interfaces/msg/cloud_info.hpp"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 客户端库
#include <pcl/point_types.h>  // PCL 点类型
#include <string>
#include <cmath>  // 用于 FLT_MAX

using namespace std;
// #define PI 3.14159265
// extern const int imuQueLength =200;

typedef pcl::PointXYZI PointType;

enum class lidarType { VELODYNE, OUSTER };

class ParamLoader
{
public:
    rclcpp::Node::SharedPtr node;  // ROS2 中的节点指针

    string robot_id;

    // Topics
    string pointCloudTopic;  // 输入的激光
    string odomTopic;
    // string gpsTopic;
    // bool useGPS;
    // float gpsPublishFreq;

    // string imuTopic;
    // string slopeTopic;
    // bool useCloudRing;

    // Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;
    std::vector<double> initPose;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    lidarType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    float lidarNoiseBound;
    bool deskewEnabled;

    // bool useAutoRing;
    // float ang_res_h;
    // float ang_res_v;
    // float ang_bottom;
    // int groundScanInd;
    // float scanPeriod;
    // int systemDelay;

    // // const int imuQueLength;
    // float sensorMinimumRange;
    // float sensorMountAngle;
    // float segmentTheta;
    // int segmentValidPointNum;
    // int segmentValidLineNum;
    // float segmentAlphaX;
    // float segmentAlphaY;
    // int edgeFeatureNum;
    // int surfFeatureNum;
    // int sectionsTotal;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    float nearestFeatureSearchSqDist;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter params
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Scan Registration
    float CT_lambda;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    // int surroundingKeyframeSearchNum;

    // Loop closure
    bool loopClosureEnableFlag;  // 回环检测使能位
    float loopClosureFrequency;  // 回环检测频率
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // float anglebias;
    // float ransac_thre;
    // std::string robotFrame;
    // float detectionXMin;
    // float detectionYMin;
    // float detectionZMin;
    // float detectionXMax;
    // float detectionYMax;
    // float detectionZMax;
    // bool saveTraj;
    // std::string filepath;
    // std::string fileDirectory;
    // int interTime;
    // float sensorHeight;
    // float minAngle;
    // float maxAngle;
    // float slopeDisThre;
    // float plainThre;

    // 载入param参数
    ParamLoader(rclcpp::Node::SharedPtr node)
    {
        this->node = node;

        node->declare_parameter("robot_id", "roboat");

        node->declare_parameter("rolo/pointCloudTopic", "/velodyne_points");
        // node->declare_parameter("rolo/pointCloudTopic", "/livox/lidar_PointCloud2");
        // node->declare_parameter("rolo/pointCloudTopic", "/cx/lslidar_point_cloud");
        node->declare_parameter("rolo/odomTopic", "/odometry");
        // node->declare_parameter("rolo/gpsTopic", "/gps/fix");
        // node->declare_parameter("rolo/slopeTopic", "/my_slope");
        // node->declare_parameter("rolo/useGPS", true);
        // node->declare_parameter("rolo/gpsPublishFreq", 1.0f);
        
        node->declare_parameter("rolo/lidarFrame", "velodyne");
        node->declare_parameter("rolo/baselinkFrame", "base_link");
        node->declare_parameter("rolo/odometryFrame", "odometry");
        node->declare_parameter("rolo/mapFrame", "map");
        node->declare_parameter("rolo/initPose", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

        node->declare_parameter("rolo/savePCD", false);
        node->declare_parameter("rolo/savePCDDirectory", "/Downloads/LOAM/");

        node->declare_parameter("rolo/sensor", "velodyne");

        node->declare_parameter("rolo/N_SCAN", 16);
        // node->declare_parameter("rolo/useCloudRing", true);
        node->declare_parameter("rolo/Horizon_SCAN", 1800);
        node->declare_parameter("rolo/downsampleRate", 1);
        node->declare_parameter("rolo/lidarMinRange", 0.1f);
        node->declare_parameter("rolo/lidarMaxRange", 1000.0f);
        node->declare_parameter("rolo/lidarNoiseBound", 0.05f);
        node->declare_parameter("rolo/deskewEnabled", false);
        // node->declare_parameter("rolo/useAutoRing", true);
        // node->declare_parameter("rolo/angResH", 0.2f);
        // node->declare_parameter("rolo/angResV", 1.84375f);
        // node->declare_parameter("rolo/angBottom", 0.0f);
        // node->declare_parameter("rolo/groundScanInd", 7);
        // node->declare_parameter("rolo/scanPeriod", 0.1f);
        // node->declare_parameter("rolo/systemDelay", 0);
        // node->declare_parameter("rolo/sensorMinimumRange", 1.0f);
        // node->declare_parameter("rolo/sensorMountAngle", 0.0f);
        // node->declare_parameter("rolo/segmentTheta", 60.0f/180.0f*M_PI);
        // node->declare_parameter("rolo/segmentValidPointNum", 5);
        // node->declare_parameter("rolo/segmentValidLineNum", 3);
        // node->declare_parameter("rolo/segmentAlphaX", node->get_parameter("rolo/angResH").as_double()/180.0*M_PI);
        // node->declare_parameter("rolo/segmentAlphaY", node->get_parameter("rolo/angResV").as_double()/180.0*M_PI);
        // node->declare_parameter("rolo/edgeFeatureNum", 4);
        // node->declare_parameter("rolo/surfFeatureNum", 8);
        // node->declare_parameter("rolo/sectionsTotal", 12);

        node->declare_parameter("rolo/edgeThreshold", 0.8f);
        node->declare_parameter("rolo/surfThreshold", 0.1f);
        node->declare_parameter("rolo/nearestFeatureSearchSqDist", 25.0f);
        node->declare_parameter("rolo/edgeFeatureMinValidNum", 20);
        node->declare_parameter("rolo/surfFeatureMinValidNum", 100);

        node->declare_parameter("rolo/odometrySurfLeafSize", 0.4f);
        node->declare_parameter("rolo/mappingCornerLeafSize", 0.2f);
        node->declare_parameter("rolo/mappingSurfLeafSize", 0.4f);

        node->declare_parameter("rolo/z_tollerance", 1000.0f);
        node->declare_parameter("rolo/rotation_tollerance", 1000.0f);

        node->declare_parameter("rolo/numberOfCores", 8);
        node->declare_parameter("rolo/mappingProcessInterval", 0.15f);

        node->declare_parameter("rolo/continuousTrajectoryWeight", 0.3f);

        node->declare_parameter("rolo/surroundingkeyframeAddingDistThreshold", 0.5f);
        node->declare_parameter("rolo/surroundingkeyframeAddingAngleThreshold", 0.2f);
        node->declare_parameter("rolo/surroundingKeyframeDensity", 2.0f);
        node->declare_parameter("rolo/surroundingKeyframeSearchRadius", 50.0f);

        node->declare_parameter("rolo/loopClosureEnableFlag", false);
        node->declare_parameter("rolo/loopClosureFrequency", 1.0f);
        node->declare_parameter("rolo/surroundingKeyframeSize", 50);
        node->declare_parameter("rolo/historyKeyframeSearchRadius", 30.0f);//默认值不同
        node->declare_parameter("rolo/historyKeyframeSearchTimeDiff", 30.0f);
        node->declare_parameter("rolo/historyKeyframeSearchNum", 25);
        node->declare_parameter("rolo/historyKeyframeFitnessScore", 0.3f);

        node->declare_parameter("rolo/globalMapVisualizationSearchRadius", 1e3f);
        node->declare_parameter("rolo/globalMapVisualizationPoseDensity", 10.0f);
        node->declare_parameter("rolo/globalMapVisualizationLeafSize", 1.0f);

        // node->declare_parameter("rolo/surroundingKeyframeSearchNum", 50);
        // node->declare_parameter("rolo/globalframe", "base_link");
        // node->declare_parameter("rolo/anglebias", 0.5f);
        // node->declare_parameter("rolo/detectionXMin", 3.0f);
        // node->declare_parameter("rolo/detectionXMax", 15.0f);
        // node->declare_parameter("rolo/detectionYMin", -1.0f);
        // node->declare_parameter("rolo/detectionYMax", 1.0f);
        // node->declare_parameter("rolo/detectionZMin", -2.0f);
        // node->declare_parameter("rolo/detectionZMax", 2.0f);

        // node->declare_parameter("rolo/saveTraj", false);
        // node->declare_parameter("rolo/filepath", "/Downloads/LOAM/trajectory.txt");
        // node->declare_parameter("rolo/fileDirectory", "/tmp/");

        // node->declare_parameter("rolo/intervalTime", 7);
        // node->declare_parameter("rolo/ThresholdVTDis", 0.7);
        // node->declare_parameter("rolo/sensorHeight", 1.9);
        // node->declare_parameter("rolo/plainThre", 1.5);
        // node->declare_parameter("rolo/minAngle", -4.0);
        // node->declare_parameter("rolo/maxAngle", 4.0);

        // 获取参数
        robot_id = node->get_parameter("robot_id").as_string();

        pointCloudTopic = node->get_parameter("rolo/pointCloudTopic").as_string();
        odomTopic = node->get_parameter("rolo/odomTopic").as_string();
        // gpsTopic = node->get_parameter("rolo/gpsTopic").as_string();
        // slopeTopic = node->get_parameter("rolo/slopeTopic").as_string();
        // useGPS = node->get_parameter("rolo/useGPS").as_bool();
        // gpsPublishFreq = node->get_parameter("rolo/gpsPublishFreq").as_double();

        lidarFrame = node->get_parameter("rolo/lidarFrame").as_string();
        baselinkFrame = node->get_parameter("rolo/baselinkFrame").as_string();
        odometryFrame = node->get_parameter("rolo/odometryFrame").as_string();
        mapFrame = node->get_parameter("rolo/mapFrame").as_string();
        initPose = node->get_parameter("rolo/initPose").as_double_array();
        for(size_t i = 3; i<initPose.size(); i++){
            initPose[i] = initPose[i] * M_PI / 180.0;
        }

        savePCD = node->get_parameter("rolo/savePCD").as_bool();
        savePCDDirectory = node->get_parameter("rolo/savePCDDirectory").as_string();

        string sensorStr = node->get_parameter("rolo/sensor").as_string();
        if (sensorStr == "velodyne")
        {
            sensor = lidarType::VELODYNE;
        }else if (sensorStr == "ouster")
        {
            sensor = lidarType::OUSTER;
        }else{
            RCLCPP_ERROR(node->get_logger(), "Invalid sensor type (must be either 'velodyne' or 'ouster'): %s", sensorStr.c_str());
            rclcpp::shutdown();
        }

        N_SCAN = node->get_parameter("rolo/N_SCAN").as_int();
        // useCloudRing = node->get_parameter("rolo/useCloudRing").as_bool();
        Horizon_SCAN = node->get_parameter("rolo/Horizon_SCAN").as_int();
        downsampleRate = node->get_parameter("rolo/downsampleRate").as_int();
        lidarMinRange = node->get_parameter("rolo/lidarMinRange").as_double();
        lidarMaxRange = node->get_parameter("rolo/lidarMaxRange").as_double();
        lidarNoiseBound = node->get_parameter("rolo/lidarNoiseBound").as_double();
        deskewEnabled = node->get_parameter("rolo/deskewEnabled").as_bool();
        // useAutoRing = node->get_parameter("rolo/useAutoRing").as_bool();
        // ang_res_h = node->get_parameter("rolo/angResH").as_double();
        // ang_res_v = node->get_parameter("rolo/angResV").as_double();
        // ang_bottom = node->get_parameter("rolo/angBottom").as_double();
        // groundScanInd = node->get_parameter("rolo/groundScanInd").as_int();
        // scanPeriod = node->get_parameter("rolo/scanPeriod").as_double();
        // systemDelay = node->get_parameter("rolo/systemDelay").as_int();
        // // imuQueLength = node->get_parameter("rolo/imuQueLength").as_int();
        // sensorMinimumRange = node->get_parameter("rolo/sensorMinimumRange").as_double();
        // sensorMountAngle = node->get_parameter("rolo/sensorMountAngle").as_double();
        // segmentTheta = node->get_parameter("rolo/segmentTheta").as_double();
        // segmentValidPointNum = node->get_parameter("rolo/segmentValidPointNum").as_int();
        // segmentValidLineNum = node->get_parameter("rolo/segmentValidLineNum").as_int();
        // segmentAlphaX = node->get_parameter("rolo/segmentAlphaX").as_double();
        // segmentAlphaY = node->get_parameter("rolo/segmentAlphaY").as_double();
        // edgeFeatureNum = node->get_parameter("rolo/edgeFeatureNum").as_int();
        // surfFeatureNum = node->get_parameter("rolo/surfFeatureNum").as_int();
        // sectionsTotal = node->get_parameter("rolo/sectionsTotal").as_int();

        edgeThreshold = node->get_parameter("rolo/edgeThreshold").as_double();
        surfThreshold = node->get_parameter("rolo/surfThreshold").as_double();
        edgeFeatureMinValidNum = node->get_parameter("rolo/edgeFeatureMinValidNum").as_int();
        surfFeatureMinValidNum = node->get_parameter("rolo/surfFeatureMinValidNum").as_int();
        nearestFeatureSearchSqDist = node->get_parameter("rolo/nearestFeatureSearchSqDist").as_double();

        odometrySurfLeafSize = node->get_parameter("rolo/odometrySurfLeafSize").as_double();
        mappingCornerLeafSize = node->get_parameter("rolo/mappingCornerLeafSize").as_double();
        mappingSurfLeafSize = node->get_parameter("rolo/mappingSurfLeafSize").as_double();
        
        z_tollerance = node->get_parameter("rolo/z_tollerance").as_double();
        rotation_tollerance = node->get_parameter("rolo/rotation_tollerance").as_double();
        
        numberOfCores = node->get_parameter("rolo/numberOfCores").as_int();
        mappingProcessInterval = node->get_parameter("rolo/mappingProcessInterval").as_double();
        
        CT_lambda = node->get_parameter("rolo/continuousTrajectoryWeight").as_double();
        
        surroundingkeyframeAddingDistThreshold = node->get_parameter("rolo/surroundingkeyframeAddingDistThreshold").as_double();
        surroundingkeyframeAddingAngleThreshold = node->get_parameter("rolo/surroundingkeyframeAddingAngleThreshold").as_double();
        surroundingKeyframeDensity = node->get_parameter("rolo/surroundingKeyframeDensity").as_double();
        surroundingKeyframeSearchRadius = node->get_parameter("rolo/surroundingKeyframeSearchRadius").as_double();
        
        loopClosureEnableFlag = node->get_parameter("rolo/loopClosureEnableFlag").as_bool();
        loopClosureFrequency = node->get_parameter("rolo/loopClosureFrequency").as_double();
        surroundingKeyframeSize = node->get_parameter("rolo/surroundingKeyframeSize").as_int();
        historyKeyframeSearchRadius = node->get_parameter("rolo/historyKeyframeSearchRadius").as_double();
        historyKeyframeSearchTimeDiff = node->get_parameter("rolo/historyKeyframeSearchTimeDiff").as_double();
        historyKeyframeSearchNum = node->get_parameter("rolo/historyKeyframeSearchNum").as_int();
        historyKeyframeFitnessScore = node->get_parameter("rolo/historyKeyframeFitnessScore").as_double();
        
        globalMapVisualizationSearchRadius = node->get_parameter("rolo/globalMapVisualizationSearchRadius").as_double();
        globalMapVisualizationPoseDensity = node->get_parameter("rolo/globalMapVisualizationPoseDensity").as_double();
        globalMapVisualizationLeafSize = node->get_parameter("rolo/globalMapVisualizationLeafSize").as_double();

        // surroundingKeyframeSearchNum = node->get_parameter("rolo/surroundingKeyframeSearchNum").as_int();

        // robotFrame = node->get_parameter("rolo/globalframe").as_string();
        // anglebias = node->get_parameter("rolo/anglebias").as_double();
        // detectionXMin = node->get_parameter("rolo/detectionXMin").as_double();
        // detectionXMax = node->get_parameter("rolo/detectionXMax").as_double();
        // detectionYMin = node->get_parameter("rolo/detectionYMin").as_double();
        // detectionYMax = node->get_parameter("rolo/detectionYMax").as_double();
        // detectionZMin = node->get_parameter("rolo/detectionZMin").as_double();
        // detectionZMax = node->get_parameter("rolo/detectionZMax").as_double();

        // saveTraj = node->get_parameter("rolo/saveTraj").as_bool();
        // filepath = node->get_parameter("rolo/filepath").as_string();
        // fileDirectory = node->get_parameter("rolo/fileDirectory").as_string();

        // interTime = node->get_parameter("rolo/intervalTime").as_int();
        // slopeDisThre = node->get_parameter("rolo/ThresholdVTDis").as_double();
        // sensorHeight = node->get_parameter("rolo/sensorHeight").as_double();
        // plainThre = node->get_parameter("rolo/plainThre").as_double();
        // minAngle = node->get_parameter("rolo/minAngle").as_double();
        // maxAngle = node->get_parameter("rolo/maxAngle").as_double();


        usleep(100);
    }
};
        
template <typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, const T thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
    {
        thisPub->publish(tempCloud);
    }
    return tempCloud;
}

template <typename T>
double GET_ROS_TIMESTAMP(const T& msg)
{
    return msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
}

double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}

double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

float pointDistance(const PointType& p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(const PointType& p1, const PointType& p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

/**
 * Remove one row from matrix.
 * Credit to: https://stackoverflow.com/questions/13290395 
 * @param matrix an Eigen::Matrix.
 * @param rowToRemove index of row to remove. If >= matrix.rows(), no operation will be taken
 */
template <class T, int R, int C>
void removeRow(Eigen::Matrix<T, R, C>& matrix, unsigned int rowToRemove)
{
    if (rowToRemove >= matrix.rows())
    {
        return;
    }
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
    {
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.bottomRows(numRows - rowToRemove);
    }

    matrix.conservativeResize(numRows, numCols);
}

/**
 * Remove one column from matrix.
 * Credit to: https://stackoverflow.com/questions/13290395 
 * @param matrix an Eigen::Matrix.
 * @param colToRemove index of col to remove. If >= matrix.cols(), no operation will be taken
 */
template <class T, int R, int C>
void removeColumn(Eigen::Matrix<T, R, C>& matrix, unsigned int colToRemove)
{
    if (colToRemove >= matrix.cols())
    {
        return;
    }
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;

    if (colToRemove < numCols)
    {
        matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.rightCols(numCols - colToRemove);
    }

    matrix.conservativeResize(numRows, numCols);
}

#endif