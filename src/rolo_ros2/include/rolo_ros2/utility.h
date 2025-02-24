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

        node->declare_parameter("pointCloudTopic", "/velodyne_points");
        // node->declare_parameter("pointCloudTopic", "/livox/lidar_PointCloud2");
        // node->declare_parameter("pointCloudTopic", "/cx/lslidar_point_cloud");
        node->declare_parameter("odomTopic", "/odometry");
        // node->declare_parameter("gpsTopic", "/gps/fix");
        // node->declare_parameter("slopeTopic", "/my_slope");
        // node->declare_parameter("useGPS", true);
        // node->declare_parameter("gpsPublishFreq", 1.0f);
        
        node->declare_parameter("lidarFrame", "base_link");
        node->declare_parameter("baselinkFrame", "base_link");
        node->declare_parameter("odometryFrame", "odometry");
        node->declare_parameter("mapFrame", "map");
        node->declare_parameter("initPose", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

        node->declare_parameter("savePCD", false);
        node->declare_parameter("savePCDDirectory", "/Downloads/LOAM/");

        node->declare_parameter("sensor", "velodyne");

        node->declare_parameter("N_SCAN", 16);
        // node->declare_parameter("useCloudRing", true);
        node->declare_parameter("Horizon_SCAN", 1800);
        node->declare_parameter("downsampleRate", 1);
        node->declare_parameter("lidarMinRange", 0.1f);
        node->declare_parameter("lidarMaxRange", 1000.0f);
        node->declare_parameter("lidarNoiseBound", 0.05f);
        node->declare_parameter("deskewEnabled", false);
        // node->declare_parameter("useAutoRing", true);
        // node->declare_parameter("angResH", 0.2f);
        // node->declare_parameter("angResV", 1.84375f);
        // node->declare_parameter("angBottom", 0.0f);
        // node->declare_parameter("groundScanInd", 7);
        // node->declare_parameter("scanPeriod", 0.1f);
        // node->declare_parameter("systemDelay", 0);
        // node->declare_parameter("sensorMinimumRange", 1.0f);
        // node->declare_parameter("sensorMountAngle", 0.0f);
        // node->declare_parameter("segmentTheta", 60.0f/180.0f*M_PI);
        // node->declare_parameter("segmentValidPointNum", 5);
        // node->declare_parameter("segmentValidLineNum", 3);
        // node->declare_parameter("segmentAlphaX", node->get_parameter("angResH").as_double()/180.0*M_PI);
        // node->declare_parameter("segmentAlphaY", node->get_parameter("angResV").as_double()/180.0*M_PI);
        // node->declare_parameter("edgeFeatureNum", 4);
        // node->declare_parameter("surfFeatureNum", 8);
        // node->declare_parameter("sectionsTotal", 12);

        node->declare_parameter("edgeThreshold", 0.8f);
        node->declare_parameter("surfThreshold", 0.1f);
        node->declare_parameter("nearestFeatureSearchSqDist", 25.0f);
        node->declare_parameter("edgeFeatureMinValidNum", 20);
        node->declare_parameter("surfFeatureMinValidNum", 100);

        node->declare_parameter("odometrySurfLeafSize", 0.4f);
        node->declare_parameter("mappingCornerLeafSize", 0.2f);
        node->declare_parameter("mappingSurfLeafSize", 0.4f);

        node->declare_parameter("z_tollerance", 1000.0f);
        node->declare_parameter("rotation_tollerance", 1000.0f);

        node->declare_parameter("numberOfCores", 8);
        node->declare_parameter("mappingProcessInterval", 0.15f);

        node->declare_parameter("continuousTrajectoryWeight", 0.3f);

        node->declare_parameter("surroundingkeyframeAddingDistThreshold", 0.5f);
        node->declare_parameter("surroundingkeyframeAddingAngleThreshold", 0.2f);
        node->declare_parameter("surroundingKeyframeDensity", 2.0f);
        node->declare_parameter("surroundingKeyframeSearchRadius", 50.0f);

        node->declare_parameter("loopClosureEnableFlag", false);
        node->declare_parameter("loopClosureFrequency", 1.0f);
        node->declare_parameter("surroundingKeyframeSize", 50);
        node->declare_parameter("historyKeyframeSearchRadius", 30.0f);//默认值不同
        node->declare_parameter("historyKeyframeSearchTimeDiff", 30.0f);
        node->declare_parameter("historyKeyframeSearchNum", 25);
        node->declare_parameter("historyKeyframeFitnessScore", 0.3f);

        node->declare_parameter("globalMapVisualizationSearchRadius", 1e3f);
        node->declare_parameter("globalMapVisualizationPoseDensity", 10.0f);
        node->declare_parameter("globalMapVisualizationLeafSize", 1.0f);

        // node->declare_parameter("surroundingKeyframeSearchNum", 50);
        // node->declare_parameter("globalframe", "base_link");
        // node->declare_parameter("anglebias", 0.5f);
        // node->declare_parameter("detectionXMin", 3.0f);
        // node->declare_parameter("detectionXMax", 15.0f);
        // node->declare_parameter("detectionYMin", -1.0f);
        // node->declare_parameter("detectionYMax", 1.0f);
        // node->declare_parameter("detectionZMin", -2.0f);
        // node->declare_parameter("detectionZMax", 2.0f);

        // node->declare_parameter("saveTraj", false);
        // node->declare_parameter("filepath", "/Downloads/LOAM/trajectory.txt");
        // node->declare_parameter("fileDirectory", "/tmp/");

        // node->declare_parameter("intervalTime", 7);
        // node->declare_parameter("ThresholdVTDis", 0.7);
        // node->declare_parameter("sensorHeight", 1.9);
        // node->declare_parameter("plainThre", 1.5);
        // node->declare_parameter("minAngle", -4.0);
        // node->declare_parameter("maxAngle", 4.0);

        // 获取参数
        robot_id = node->get_parameter("robot_id").as_string();

        pointCloudTopic = node->get_parameter("pointCloudTopic").as_string();
        odomTopic = node->get_parameter("odomTopic").as_string();
        // gpsTopic = node->get_parameter("gpsTopic").as_string();
        // slopeTopic = node->get_parameter("slopeTopic").as_string();
        // useGPS = node->get_parameter("useGPS").as_bool();
        // gpsPublishFreq = node->get_parameter("gpsPublishFreq").as_double();

        lidarFrame = node->get_parameter("lidarFrame").as_string();
        baselinkFrame = node->get_parameter("baselinkFrame").as_string();
        odometryFrame = node->get_parameter("odometryFrame").as_string();
        mapFrame = node->get_parameter("mapFrame").as_string();
        initPose = node->get_parameter("initPose").as_double_array();
        for(size_t i = 3; i<initPose.size(); i++){
            initPose[i] = initPose[i] * M_PI / 180.0;
        }

        savePCD = node->get_parameter("savePCD").as_bool();
        savePCDDirectory = node->get_parameter("savePCDDirectory").as_string();

        string sensorStr = node->get_parameter("sensor").as_string();
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

        N_SCAN = node->get_parameter("N_SCAN").as_int();

        RCLCPP_INFO(node->get_logger(), "pointCloudTopic: %s \n sensor: %s \n N_SCAN: %d \n", pointCloudTopic.c_str(), sensorStr.c_str(), N_SCAN);
        // useCloudRing = node->get_parameter("useCloudRing").as_bool();
        Horizon_SCAN = node->get_parameter("Horizon_SCAN").as_int();
        downsampleRate = node->get_parameter("downsampleRate").as_int();
        lidarMinRange = node->get_parameter("lidarMinRange").as_double();
        lidarMaxRange = node->get_parameter("lidarMaxRange").as_double();
        lidarNoiseBound = node->get_parameter("lidarNoiseBound").as_double();
        deskewEnabled = node->get_parameter("deskewEnabled").as_bool();
        // useAutoRing = node->get_parameter("useAutoRing").as_bool();
        // ang_res_h = node->get_parameter("angResH").as_double();
        // ang_res_v = node->get_parameter("angResV").as_double();
        // ang_bottom = node->get_parameter("angBottom").as_double();
        // groundScanInd = node->get_parameter("groundScanInd").as_int();
        // scanPeriod = node->get_parameter("scanPeriod").as_double();
        // systemDelay = node->get_parameter("systemDelay").as_int();
        // // imuQueLength = node->get_parameter("imuQueLength").as_int();
        // sensorMinimumRange = node->get_parameter("sensorMinimumRange").as_double();
        // sensorMountAngle = node->get_parameter("sensorMountAngle").as_double();
        // segmentTheta = node->get_parameter("segmentTheta").as_double();
        // segmentValidPointNum = node->get_parameter("segmentValidPointNum").as_int();
        // segmentValidLineNum = node->get_parameter("segmentValidLineNum").as_int();
        // segmentAlphaX = node->get_parameter("segmentAlphaX").as_double();
        // segmentAlphaY = node->get_parameter("segmentAlphaY").as_double();
        // edgeFeatureNum = node->get_parameter("edgeFeatureNum").as_int();
        // surfFeatureNum = node->get_parameter("surfFeatureNum").as_int();
        // sectionsTotal = node->get_parameter("sectionsTotal").as_int();

        edgeThreshold = node->get_parameter("edgeThreshold").as_double();
        surfThreshold = node->get_parameter("surfThreshold").as_double();
        edgeFeatureMinValidNum = node->get_parameter("edgeFeatureMinValidNum").as_int();
        surfFeatureMinValidNum = node->get_parameter("surfFeatureMinValidNum").as_int();
        nearestFeatureSearchSqDist = node->get_parameter("nearestFeatureSearchSqDist").as_double();

        odometrySurfLeafSize = node->get_parameter("odometrySurfLeafSize").as_double();
        mappingCornerLeafSize = node->get_parameter("mappingCornerLeafSize").as_double();
        mappingSurfLeafSize = node->get_parameter("mappingSurfLeafSize").as_double();
        
        z_tollerance = node->get_parameter("z_tollerance").as_double();
        rotation_tollerance = node->get_parameter("rotation_tollerance").as_double();
        
        numberOfCores = node->get_parameter("numberOfCores").as_int();
        mappingProcessInterval = node->get_parameter("mappingProcessInterval").as_double();
        
        CT_lambda = node->get_parameter("continuousTrajectoryWeight").as_double();
        
        surroundingkeyframeAddingDistThreshold = node->get_parameter("surroundingkeyframeAddingDistThreshold").as_double();
        surroundingkeyframeAddingAngleThreshold = node->get_parameter("surroundingkeyframeAddingAngleThreshold").as_double();
        surroundingKeyframeDensity = node->get_parameter("surroundingKeyframeDensity").as_double();
        surroundingKeyframeSearchRadius = node->get_parameter("surroundingKeyframeSearchRadius").as_double();
        
        loopClosureEnableFlag = node->get_parameter("loopClosureEnableFlag").as_bool();
        loopClosureFrequency = node->get_parameter("loopClosureFrequency").as_double();
        surroundingKeyframeSize = node->get_parameter("surroundingKeyframeSize").as_int();
        historyKeyframeSearchRadius = node->get_parameter("historyKeyframeSearchRadius").as_double();
        historyKeyframeSearchTimeDiff = node->get_parameter("historyKeyframeSearchTimeDiff").as_double();
        historyKeyframeSearchNum = node->get_parameter("historyKeyframeSearchNum").as_int();
        historyKeyframeFitnessScore = node->get_parameter("historyKeyframeFitnessScore").as_double();
        
        globalMapVisualizationSearchRadius = node->get_parameter("globalMapVisualizationSearchRadius").as_double();
        globalMapVisualizationPoseDensity = node->get_parameter("globalMapVisualizationPoseDensity").as_double();
        globalMapVisualizationLeafSize = node->get_parameter("globalMapVisualizationLeafSize").as_double();

        // surroundingKeyframeSearchNum = node->get_parameter("surroundingKeyframeSearchNum").as_int();

        // robotFrame = node->get_parameter("globalframe").as_string();
        // anglebias = node->get_parameter("anglebias").as_double();
        // detectionXMin = node->get_parameter("detectionXMin").as_double();
        // detectionXMax = node->get_parameter("detectionXMax").as_double();
        // detectionYMin = node->get_parameter("detectionYMin").as_double();
        // detectionYMax = node->get_parameter("detectionYMax").as_double();
        // detectionZMin = node->get_parameter("detectionZMin").as_double();
        // detectionZMax = node->get_parameter("detectionZMax").as_double();

        // saveTraj = node->get_parameter("saveTraj").as_bool();
        // filepath = node->get_parameter("filepath").as_string();
        // fileDirectory = node->get_parameter("fileDirectory").as_string();

        // interTime = node->get_parameter("intervalTime").as_int();
        // slopeDisThre = node->get_parameter("ThresholdVTDis").as_double();
        // sensorHeight = node->get_parameter("sensorHeight").as_double();
        // plainThre = node->get_parameter("plainThre").as_double();
        // minAngle = node->get_parameter("minAngle").as_double();
        // maxAngle = node->get_parameter("maxAngle").as_double();


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