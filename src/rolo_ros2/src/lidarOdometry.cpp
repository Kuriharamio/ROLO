#include "rolo_ros2/utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <rot_gicp/gicp/impl/rot_vgicp_impl.hpp>
#include <omp.h>

using namespace Eigen;
// ofstream tum_file;

class LidarOdometry : public ParamLoader
{
private:
    mutex mtx;
    bool doneFirstOpt;
    bool isFirstFrame;
    bool failureFrameFlag;
    rclcpp::Time cloudTimeStamp;
    double cloudTimeCur;
    double cloudTimeLast;
    double lastOdomTime;
    double lastMappingInterval; // 上一次后端优化的时间间隔（以收到odom消息为准）
    bool doneBackOpt;

    Affine3f lastOdomAffine; // 上一时刻的odom对应的映射
    Affine3f lidarMappingAffine; // 相邻两帧odom之间的变换
    Affine3f transformation_interpolated;

    std::chrono::_V2::system_clock::time_point start_time;

    // ROS wrraper
    rclcpp::Subscription<rolo_ros2_interfaces::msg::CloudInfoStamp>::SharedPtr subCloudInfo;
    //TODO 接受后端的优化位姿
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometryMapped;
    rclcpp::Publisher<rolo_ros2_interfaces::msg::CloudInfoStamp>::SharedPtr pubFrontCloudInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLidarOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLidarPose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRegScan;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubPlotData;
    
    nav_msgs::msg::Path laser_odom_path;
    nav_msgs::msg::Odometry laser_odom_incremental;
    geometry_msgs::msg::PoseStamped laser_pose;
    pcl::PointCloud<PointType>::Ptr RegCloud;
    
    // 当前帧数据
    rolo_ros2_interfaces::msg::CloudInfoStamp laserCloudInfoLast;
    pcl::PointCloud<PointType>::Ptr FullCloudLast;
    pcl::PointCloud<PointType>::Ptr CloudCornerLast;
    pcl::PointCloud<PointType>::Ptr CloudSurfLast;
    pcl::PointCloud<PointType>::Ptr CloudGroundLast;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerLast;
    pcl::PointCloud<PointType>::Ptr featureLast;

    // 上一帧数据
    rolo_ros2_interfaces::msg::CloudInfoStamp laserCloudInfoOld;
    pcl::PointCloud<PointType>::Ptr FullCloudOld;
    pcl::PointCloud<PointType>::Ptr CloudCornerOld;
    pcl::PointCloud<PointType>::Ptr CloudSurfOld;
    pcl::PointCloud<PointType>::Ptr CloudGroundOld;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerOld;
    pcl::PointCloud<PointType>::Ptr featureOld;

    std::queue<rolo_ros2_interfaces::msg::CloudInfoStamp> laserCloudInfoBuf;
    
    Matrix3d Rotation;
    Vector3d Translation;
    Vector3d TranslationOld;
    float LaserOdomPose[6] = {
        static_cast<float>(initPose[0]),
        static_cast<float>(initPose[1]),
        static_cast<float>(initPose[2]),
        static_cast<float>(initPose[3]),
        static_cast<float>(initPose[4]),
        static_cast<float>(initPose[5])
    };// [x, y, z, roll, pitch, yaw]


public:  
    LidarOdometry(rclcpp::Node::SharedPtr node):ParamLoader(node),
    doneFirstOpt(true),
    doneBackOpt(false),
    isFirstFrame(true),
    failureFrameFlag(false)
    {

        // mapOptimization传来的里程计数据
        subOdometryMapped = node->create_subscription<nav_msgs::msg::Odometry>("rolo/mapping/odometry", 10, std::bind(&LidarOdometry::odometryHandler, this, std::placeholders::_1));
        // 接受imu原始数据
        subCloudInfo = node->create_subscription<rolo_ros2_interfaces::msg::CloudInfoStamp>("rolo/feature/cloud_info", 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1));
        // 发布imu预测里程计
        pubFrontCloudInfo = node->create_publisher<rolo_ros2_interfaces::msg::CloudInfoStamp>(odomTopic+"/cloud_info", 2000);
        pubLidarOdometry = node->create_publisher<nav_msgs::msg::Odometry>(odomTopic+"_incremental", 2000);
        pubLidarPose = node->create_publisher<geometry_msgs::msg::PoseStamped>(odomTopic+"_incremental/pose", 2000);
        pubLaserPath = node->create_publisher<nav_msgs::msg::Path>(odomTopic+"_incremental/path", 2000);
        pubRegScan = node->create_publisher<sensor_msgs::msg::PointCloud2>(odomTopic+"/registration_scan", 10);
        pubPlotData = node->create_publisher<std_msgs::msg::Float64MultiArray>("rolo/data_test", 10);

        Init();
    }
    ~LidarOdometry(){}

    void Init(){
        Rotation = Matrix3d::Identity();
        Translation = Vector3d::Zero();
        TranslationOld = Vector3d::Zero();
        lidarMappingAffine = Affine3f::Identity();
        lastOdomAffine = Affine3f::Identity();
        transformation_interpolated = Affine3f::Identity();
        lastOdomTime = -1;
        RegCloud.reset(new pcl::PointCloud<PointType>());
        lastMappingInterval = 9999.0;
        // 当前帧数据
        FullCloudLast.reset(new pcl::PointCloud<PointType>());
        CloudCornerLast.reset(new pcl::PointCloud<PointType>());
        CloudSurfLast.reset(new pcl::PointCloud<PointType>());
        CloudGroundLast.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerLast.reset(new pcl::PointCloud<PointType>());
        featureLast.reset(new pcl::PointCloud<PointType>());

        // 上一帧数据
        FullCloudOld.reset(new pcl::PointCloud<PointType>());
        CloudCornerOld.reset(new pcl::PointCloud<PointType>());
        CloudSurfOld.reset(new pcl::PointCloud<PointType>());
        CloudGroundOld.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerOld.reset(new pcl::PointCloud<PointType>());
        featureOld.reset(new pcl::PointCloud<PointType>());
        start_time = std::chrono::system_clock::now();

    }

    //! 接受后端的里程计消息，并与前端里程计融合
    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr mappedOdom){
        // 当前时刻odom时间
        double currentCorrectionTime = mappedOdom->header.stamp.sec + mappedOdom->header.stamp.nanosec * 1e-9;
        nav_msgs::msg::Odometry mappedOdom_ = *mappedOdom;
        lastOdomTime = currentCorrectionTime;
        doneBackOpt = true;
    }

    void scanRegeistration(){
        // if(featureOld->points.size() == 0 || featureLast->points.size() == 0){
        //     RCLCPP_ERROR(node->get_logger(), "No feature cloud");
        //     return;
        // }
            
        auto start = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end - start;
        // printf("Solver Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

        pcl::PointCloud<PointType>::Ptr feature_propagated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr feature_rotated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);
        feature_propagated->clear();
        feature_rotated->clear();
        // 先平移插值，使中心对齐
        pcl::transformPointCloud(*featureOld, *feature_propagated, transformation_interpolated);
        // std::cout << "transformation_interpolated: " << transformation_interpolated.matrix() << std::endl;
        // fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
        // rot_vgicp.setResolution(1.0);
        // rot_vgicp.setNumThreads(omp_get_max_threads());
        // // rot_vgicp.clearTarget();
        // // rot_vgicp.clearSource();
        // rot_vgicp.setInputTarget(featureLast);
        // rot_vgicp.setInputSource(feature_propagated);
        // rot_vgicp.align(*aligned);
        // Eigen::Matrix4f trans = rot_vgicp.getFinalTransformation(); // 旋转估计
        // // 确保点云的 is_dense 属性为 false
        // featureLast->is_dense = false;
        // feature_propagated->is_dense = false;

        // // 移除无效点
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*featureLast, *featureLast, indices);
        // pcl::removeNaNFromPointCloud(*feature_propagated, *feature_propagated, indices);

        // RCLCPP_INFO(node->get_logger(), "当前帧点云数：%ld, 上一帧点云数：%ld", featureLast->points.size(), feature_propagated->points.size());
        // // 检查点云是否为空
        // if (featureLast->points.empty() || feature_propagated->points.empty()) {
        //     RCLCPP_ERROR(node->get_logger(), "有空点云");
        //     return;
        // }

        // // 检查点云中的点是否有效
        // for (const auto& point : featureLast->points) {
        //     if (!std::isinf(point.x) || !std::isinf(point.y) || !std::isinf(point.z)) {
        //         RCLCPP_ERROR(node->get_logger(), "发现无效点");
        //         return;
        //     }
        // }

        // 执行 ICP 配准
        fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
        rot_vgicp.setResolution(1.0);
        rot_vgicp.setNumThreads(omp_get_max_threads());
        rot_vgicp.clearTarget();
        rot_vgicp.clearSource();
        rot_vgicp.setInputTarget(featureLast);
        rot_vgicp.setInputSource(feature_propagated);
        rot_vgicp.align(*aligned);

        Eigen::Matrix4f trans = rot_vgicp.getFinalTransformation();
        
        // Rotation = trans.block<3, 3>(0, 0).cast<float>() * Rotation.eval();
        Eigen::Affine3f transformStep;
        transformStep.matrix() = trans.cast<float>();
        transformation_interpolated = transformation_interpolated * transformStep;
        Rotation = transformation_interpolated.rotation().cast<double>();
        Translation = transformation_interpolated.translation().cast<double>();
        auto r_end = std::chrono::system_clock::now();
        std::chrono::duration<double> r_elapsed_seconds = r_end - start;
        printf("Rotation Solver Duration: %f ms.\n" ,r_elapsed_seconds.count() * 1000);

        // Eigen::Vector3f rotation_euler;
        // float x, y, z;
        // pcl::getTranslationAndEulerAngles<float>(transformStep, 
        //                                             x, y, z,
        //                                             rotation_euler[0],
        //                                             rotation_euler[1], 
        //                                             rotation_euler[2]); 
        // std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;

        //* 平移配准
        // 首先进行旋转
        aligned->clear();
        pcl::transformPointCloud(*featureOld, *feature_rotated, transformation_interpolated);
        Eigen::Vector3d Reg_translation = Eigen::Vector3d::Zero();

        rot_vgicp.computeTranslation(*aligned, Reg_translation, Translation, TranslationOld, 0.1, 0.1, CT_lambda);

           
        // std::cout << "Reg_translation: " << Reg_translation.transpose() << std::endl;
        auto t_end = std::chrono::system_clock::now();
        std::chrono::duration<double> t_elapsed_seconds = t_end - r_end;
        printf("Translation Solver Duration: %f ms.\n" ,t_elapsed_seconds.count() * 1000);

        Translation += Reg_translation;
    }

    void cloudHandler(const rolo_ros2_interfaces::msg::CloudInfoStamp::SharedPtr cloudIn){
        // 取时间戳,入buffer
        cloudTimeStamp = cloudIn->header.stamp;
        cloudTimeCur = cloudIn->header.stamp.sec + cloudIn->header.stamp.nanosec * 1e-9;
        laserCloudInfoBuf.push(*cloudIn);
        
        
        // 进行时间匹配
        rclcpp::Time TimeCur = node->now();
        for(int i=0; i<laserCloudInfoBuf.size(); i++){
            laserCloudInfoLast = laserCloudInfoBuf.front();
            laserCloudInfoBuf.pop();
            cloudTimeStamp = laserCloudInfoLast.header.stamp;
            cloudTimeCur = laserCloudInfoLast.header.stamp.sec + laserCloudInfoLast.header.stamp.nanosec * 1e-9;
            if(std::fabs((TimeCur-cloudTimeStamp).seconds()) < 0.1){
                break;
            }
        }

        // 提取当前帧特征点云
        pcl::fromROSMsg(laserCloudInfoLast.extracted_corner,  *CloudCornerLast);
        pcl::fromROSMsg(laserCloudInfoLast.extracted_surface, *CloudSurfLast);
        pcl::fromROSMsg(laserCloudInfoLast.cloud_projected, *FullCloudLast);
        *featureLast = *CloudCornerLast + *CloudSurfLast;

        if(isFirstFrame){
            isFirstFrame = false;
            *FullCloudOld = *FullCloudLast;
            *CloudCornerOld = *CloudCornerLast;
            *CloudSurfOld = *CloudSurfLast;
            *featureOld = *featureLast;
            return;
        }


        // 是否完成第一次全图优化
        // if (doneFirstOpt == false)
        if (lastOdomTime == -1.0){
            updateTransform();
            pubMessage();
            return;
        }


        // 状态前向插值
        if(lastOdomTime != -1.0){   // 未初始化则不进行插值
            double latestInterval = cloudTimeCur - cloudTimeLast;
            
            stateLinearPropagation(lidarMappingAffine, lastMappingInterval, latestInterval, transformation_interpolated);
            Rotation = transformation_interpolated.rotation().cast<double>();
            Translation = transformation_interpolated.translation().cast<double>();
            if(Translation.array().maxCoeff() > 5.0){
                std::cout << "Translation: \n" << Translation << std::endl;
            }
            doneBackOpt = false;
            cloudTimeLast = cloudTimeCur; // 仅对相邻两帧之间进行插值
            lastMappingInterval = latestInterval;
        }

        scanRegeistration();

        updateTransform();
        if(!failureFrameFlag){
            // 发布ROS消息和TF
            pubMessage();
            pubTranform();
        }
        else{
            printf(" Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n "); 
            failureFrameFlag = false;
        }

        // auto f_end = std::chrono::system_clock::now();
        // std::chrono::duration<double> r_elapsed_seconds = f_end - f_start;
        // // 保存前段时间消耗
        // tum_file << setprecision(19) << cloudTimeCur << " " 
        //          << r_elapsed_seconds.count() * 1000 << std::endl;
    }

    void updateTransform(){
        // Matrix4d trans = Matrix4d::Identity();
        // trans << Rotation;
        // trans.col(3) << Translation(0,0), Translation(1,0), Translation(2,0), 1.0;
        Matrix4d trans = Matrix4d::Identity();
        trans.block<3, 3>(0, 0) = Rotation;  // 将 Rotation 赋值到前 3x3 子矩阵
        trans.col(3).head<3>() = Translation;  // 将 Translation 赋值到第 4 列的前 3 个元素

        size_t cloudSize = FullCloudLast->points.size();
        // if(!cloudSize){
        //     std::cout << "Cloud size: " << cloudSize << std::endl;
        //     return;
        // }

        
        RegCloud->clear();
        RegCloud->resize(cloudSize);
        RegCloud->points = FullCloudLast->points;

        // if (trans.array().isNaN().any() || trans.array().isInf().any()) {
        //     std::cerr << "Error: Transformation matrix contains NaN or Inf!" << std::endl;
        //     return;
        // }
        
        pcl::transformPointCloud(*FullCloudLast, *RegCloud, trans);
        
        Affine3f transform_affine = pcl::getTransformation(LaserOdomPose[0], 
                                                           LaserOdomPose[1], 
                                                           LaserOdomPose[2], 
                                                           LaserOdomPose[3], 
                                                           LaserOdomPose[4], 
                                                           LaserOdomPose[5]);
        
        // Affine3d transform_affine_double = transform_affine.cast<double>;
        Affine3f transformStep;
        transformStep.matrix() = trans.cast<float>();

        Affine3f transformed_pose = transform_affine * transformStep.inverse(); // 仿射变换遵循右乘原则
        
        lidarMappingAffine = transformStep;
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // if(!failureDetection(transform_affine, transformed_pose, elapsed_seconds.count()*1.0e6)){
        //     failureFrameFlag = true;
        //     return;
        // }

        start_time = end_time;
        // Vector3f rotation_euler;
        // float x, y, z;
        // pcl::getTranslationAndEulerAngles<float>(transformStep, 
        //                                          x, y, z,
        //                                          rotation_euler[0],
        //                                          rotation_euler[1], 
        //                                          rotation_euler[2]); 
        // std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;

        pcl::getTranslationAndEulerAngles<float>(transformed_pose, 
                                          LaserOdomPose[0], 
                                          LaserOdomPose[1], 
                                          LaserOdomPose[2], 
                                          LaserOdomPose[3], 
                                          LaserOdomPose[4], 
                                          LaserOdomPose[5]);                            
        
        // 新旧信息交换
        *FullCloudOld = *FullCloudLast;
        *CloudCornerOld = *CloudCornerLast;
        *CloudSurfOld = *CloudSurfLast;
        *featureOld = *featureLast;
        TranslationOld = Translation;
    }

    //! 检测前端里程计估计是否发生跳变，如果发生跳变，忽略当前帧的估计结果
    bool failureDetection(Affine3f pose_affine, Affine3f pose_affine_transformed, double delt_Time){
        float x, y, z, roll, pitch, yaw;
        float t_x, t_y, t_z, t_roll, t_pitch, t_yaw;
        auto t_sq = pow(delt_Time, 2);
        pcl::getTranslationAndEulerAngles<float>(pose_affine, 
                                          x, y, z, roll, pitch, yaw);
        pcl::getTranslationAndEulerAngles<float>(pose_affine_transformed, 
                                          t_x, t_y, t_z, t_roll, t_pitch, t_yaw); 
        float delt_t = (t_x-x)*(t_x-x) + (t_y-y)*(t_y-y) + (t_z-z)*(t_z-z);
        float delt_r = (t_roll-roll)*(t_roll-roll) + (t_pitch-pitch)*(t_pitch-pitch) + (t_yaw-yaw)*(t_yaw-yaw);
        if(delt_t/t_sq >= 5.0 || delt_r/t_sq >= pow(0.2, 2)){
            return false;
        }
        return true;
    }
    
    void pubTranform(){
        // 发布TF
        // Publish TF
        static std::shared_ptr<tf2_ros::TransformBroadcaster> br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
        tf2::Transform t_odom_to_lidar;
        t_odom_to_lidar.setOrigin(tf2::Vector3(LaserOdomPose[0], LaserOdomPose[1], LaserOdomPose[2]));
        tf2::Quaternion q;
        q.setRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]);
        t_odom_to_lidar.setRotation(q);
        geometry_msgs::msg::TransformStamped pose_stamped;

        pose_stamped.transform = tf2::toMsg(t_odom_to_lidar);
        pose_stamped.header.stamp = cloudTimeStamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.child_frame_id = "lidar";
        
        br->sendTransform(pose_stamped);
    
    }

    void pubMessage(){
        publishCloud(pubRegScan, RegCloud, cloudTimeStamp, baselinkFrame);
        
        // 发布位姿
        laser_pose.header.frame_id = odometryFrame;
        laser_pose.header.stamp = cloudTimeStamp;
        laser_pose.pose.position.x = LaserOdomPose[0];
        laser_pose.pose.position.y = LaserOdomPose[1];
        laser_pose.pose.position.z = LaserOdomPose[2];
        tf2::Quaternion q;
        q.setRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]);
        laser_pose.pose.orientation.x = q.x();
        laser_pose.pose.orientation.y = q.y();
        laser_pose.pose.orientation.z = q.z();
        laser_pose.pose.orientation.w = q.w();        
        pubLidarPose->publish(laser_pose);

        // 发布Path
        laser_odom_path.header.frame_id = odometryFrame;
        laser_odom_path.header.stamp = cloudTimeStamp;
        laser_odom_path.poses.push_back(laser_pose);
        nav_msgs::msg::Path laser_odom_path2 = laser_odom_path;
        std::reverse(laser_odom_path2.poses.begin(), laser_odom_path2.poses.end());
        pubLaserPath->publish(laser_odom_path2);

        // 发布里程计
        laser_odom_incremental.header.frame_id = odometryFrame;
        laser_odom_incremental.header.stamp = cloudTimeStamp; //ros::Time::now();
        laser_odom_incremental.child_frame_id = "lidar_odometry";
        laser_odom_incremental.pose.pose = laser_pose.pose;
        pubLidarOdometry->publish(laser_odom_incremental);

        // 发布初始位姿估计
        rolo_ros2_interfaces::msg::CloudInfoStamp odometry_cloud;
        odometry_cloud = laserCloudInfoLast;
        odometry_cloud.initial_guess_x = LaserOdomPose[0];
        odometry_cloud.initial_guess_y = LaserOdomPose[1];
        odometry_cloud.initial_guess_z = LaserOdomPose[2];
        odometry_cloud.initial_guess_roll = LaserOdomPose[3];
        odometry_cloud.initial_guess_pitch = LaserOdomPose[4];
        odometry_cloud.initial_guess_yaw = LaserOdomPose[5];
        odometry_cloud.odom_available = true;
        pubFrontCloudInfo->publish(odometry_cloud);
    }

    //! 针对上一时刻的后端变换进行线性插值
    void stateLinearPropagation(const Eigen::Affine3f& last_trans, const double& last_interval, const double& curr_interval,
                                Eigen::Affine3f &curr_trans){
        double propagation_ratio = curr_interval / last_interval;
        Eigen::Matrix<float, 6, 1> trans_vec;
        pcl::getTranslationAndEulerAngles(last_trans,
                                          trans_vec(0), trans_vec(1), trans_vec(2),
                                          trans_vec(3), trans_vec(4), trans_vec(5));
        trans_vec.tail(3) = Eigen::Matrix<float, 3, 1>::Zero();
        // std::cout << "transformation: \n" << trans_vec.transpose() << std::endl;
        trans_vec *= propagation_ratio;
        curr_trans = pcl::getTransformation(trans_vec(0), trans_vec(1), trans_vec(2),
                                            trans_vec(3), trans_vec(4), trans_vec(5));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_LO = std::make_shared<rclcpp::Node>("LidarOdometry");
    // auto node_TF = std::make_shared<rclcpp::Node>("TransformFusion");


    LidarOdometry LO(node_LO);
    // TransformFusion TF(node_TF);

    // if(!LO.loopClosureEnableFlag){
    //     tum_file.open("/home/sdu/slam_time/rolo/rolo_front.tum");
    // }
    // else{
    //     tum_file.open("/home/sdu/slam_time/rolo_lc/rolo_lc_front.tum");
    // }
    // tum_file.clear();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_LO);
    // executor.add_node(node_TF);
    RCLCPP_INFO(node_LO->get_logger(), "\033[1;32m----> Laser Odometry Started.\033[0m");
    executor.spin();
    // tum_file.close();
    
    return 0;
}