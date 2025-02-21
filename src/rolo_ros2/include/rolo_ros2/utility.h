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
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 客户端库
#include <pcl/point_types.h>  // PCL 点类型
#include <string>
#include <cmath>  // 用于 FLT_MAX

using namespace std;

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

    // Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

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

    // LOAM
    float edgeThreshold;
    float surfThreshold;
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

    // 载入param参数
    ParamLoader(rclcpp::Node::SharedPtr node)
    {
        this->node = node;

        // 使用 ROS2 的参数管理方式
        node->declare_parameter("robot_id", "roboat");

        node->declare_parameter("rolo/pointCloudTopic", "points_raw");
        node->declare_parameter("rolo/odomTopic", "odometry/imu");

        node->declare_parameter("rolo/lidarFrame", "base_link");
        node->declare_parameter("rolo/baselinkFrame", "base_link");
        node->declare_parameter("rolo/odometryFrame", "odom");
        node->declare_parameter("rolo/mapFrame", "map");

        node->declare_parameter("rolo/savePCD", false);
        node->declare_parameter("rolo/savePCDDirectory", "/Downloads/LOAM/");

        node->declare_parameter("rolo/sensor", "");

        node->declare_parameter("rolo/N_SCAN", 16);
        node->declare_parameter("rolo/Horizon_SCAN", 1800);
        node->declare_parameter("rolo/downsampleRate", 1);
        node->declare_parameter("rolo/lidarMinRange", 1.0f);
        node->declare_parameter("rolo/lidarMaxRange", 1000.0f);
        node->declare_parameter("rolo/lidarNoiseBound", 0.05f);
        node->declare_parameter("rolo/deskewEnabled", true);

        node->declare_parameter("rolo/edgeThreshold", 0.1f);
        node->declare_parameter("rolo/surfThreshold", 0.1f);
        node->declare_parameter("rolo/edgeFeatureMinValidNum", 10);
        node->declare_parameter("rolo/surfFeatureMinValidNum", 100);

        node->declare_parameter("rolo/odometrySurfLeafSize", 0.2f);
        node->declare_parameter("rolo/mappingCornerLeafSize", 0.2f);
        node->declare_parameter("rolo/mappingSurfLeafSize", 0.2f);

        node->declare_parameter("rolo/z_tollerance", FLT_MAX);
        node->declare_parameter("rolo/rotation_tollerance", FLT_MAX);

        node->declare_parameter("rolo/numberOfCores", 2);
        node->declare_parameter("rolo/mappingProcessInterval", 0.15);

        node->declare_parameter("rolo/continuousTrajectoryWeight", 1.0f);

        node->declare_parameter("rolo/surroundingkeyframeAddingDistThreshold", 1.0f);
        node->declare_parameter("rolo/surroundingkeyframeAddingAngleThreshold", 0.2f);
        node->declare_parameter("rolo/surroundingKeyframeDensity", 1.0f);
        node->declare_parameter("rolo/surroundingKeyframeSearchRadius", 50.0f);

        node->declare_parameter("rolo/loopClosureEnableFlag", true);
        node->declare_parameter("rolo/loopClosureFrequency", 1.0f);
        node->declare_parameter("rolo/surroundingKeyframeSize", 50);
        node->declare_parameter("rolo/historyKeyframeSearchRadius", 10.0f);
        node->declare_parameter("rolo/historyKeyframeSearchTimeDiff", 30.0f);
        node->declare_parameter("rolo/historyKeyframeSearchNum", 25);
        node->declare_parameter("rolo/historyKeyframeFitnessScore", 0.3f);

        node->declare_parameter("rolo/globalMapVisualizationSearchRadius", 1e3f);
        node->declare_parameter("rolo/globalMapVisualizationPoseDensity", 10.0f);
        node->declare_parameter("rolo/globalMapVisualizationLeafSize", 1.0f);

        // 获取参数
        robot_id = node->get_parameter("robot_id").as_string();
        pointCloudTopic = node->get_parameter("rolo/pointCloudTopic").as_string();
        odomTopic = node->get_parameter("rolo/odomTopic").as_string();
        lidarFrame = node->get_parameter("rolo/lidarFrame").as_string();
        baselinkFrame = node->get_parameter("rolo/baselinkFrame").as_string();
        odometryFrame = node->get_parameter("rolo/odometryFrame").as_string();
        mapFrame = node->get_parameter("rolo/mapFrame").as_string();
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
        Horizon_SCAN = node->get_parameter("rolo/Horizon_SCAN").as_int();
        downsampleRate = node->get_parameter("rolo/downsampleRate").as_int();
        lidarMinRange = node->get_parameter("rolo/lidarMinRange").as_double();
        lidarMaxRange = node->get_parameter("rolo/lidarMaxRange").as_double();
        lidarNoiseBound = node->get_parameter("rolo/lidarNoiseBound").as_double();
        deskewEnabled = node->get_parameter("rolo/deskewEnabled").as_bool();
        edgeThreshold = node->get_parameter("rolo/edgeThreshold").as_double();
        surfThreshold = node->get_parameter("rolo/surfThreshold").as_double();
        edgeFeatureMinValidNum = node->get_parameter("rolo/edgeFeatureMinValidNum").as_int();
        surfFeatureMinValidNum = node->get_parameter("rolo/surfFeatureMinValidNum").as_int();
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

        usleep(100);
    }
};
        
template <typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, const T& thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
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