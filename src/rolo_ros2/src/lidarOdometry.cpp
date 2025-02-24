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
#include <rot_gicp/gicp/rot_vgicp.hpp>
#include <omp.h>

using namespace Eigen;
// ofstream tum_file;


class TransformFusion : public ParamLoader
{
public:
    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLidarOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLidarPath;

    Eigen::Affine3f mappingOdomAffine;
    Eigen::Affine3f lidarOdomAffineFront;
    Eigen::Affine3f lidarOdomAffineBack;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped lidar2Baselink;

    double mappingOdomTime = -1;
    deque<nav_msgs::msg::Odometry> lidarOdomQueue;
    //! 读取base-lidar的TF，声明输入输出
    TransformFusion(rclcpp::Node::SharedPtr node) : ParamLoader(node)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), std::chrono::seconds(3));
		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
	
        if (lidarFrame != baselinkFrame)
        {
            try
            {
                rclcpp::Rate rate(10);  // 10 Hz
                for (int i = 0; i < 30; ++i)  // 等待最多 3 秒
                {
                    if (tf_buffer_->canTransform(lidarFrame, baselinkFrame, rclcpp::Time(0)))
                    {
                        lidar2Baselink = tf_buffer_->lookupTransform(lidarFrame, baselinkFrame, rclcpp::Time(0));
                        break;
                    }
                    rate.sleep();
                }
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
            }
        }

         // 接受后端优化里程和预积分传过来的历程
        subLaserOdometry = node->create_subscription<nav_msgs::msg::Odometry>("rolo/mapping/odometry", 5, std::bind(&TransformFusion::mappingOdometryHandler, this, std::placeholders::_1));
        subImuOdometry = node->create_subscription<nav_msgs::msg::Odometry>(odomTopic + "_incremental", 2000, std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1));
        // 发布IMU里程计
        pubLidarOdometry = node->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 2000);
        pubLidarPath = node->create_publisher<nav_msgs::msg::Path>("rolo/lidar_odometry/path", 1);
    }
    //! 获取给定odom消息所代表的变换矩阵
    Eigen::Affine3f odom2affine(nav_msgs::msg::Odometry odom) // Eigen::Affine3f为仿射变换矩阵，旋转矩阵和平移矩阵的结合
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation;
        orientation.setX(odom.pose.pose.orientation.x);
        orientation.setY(odom.pose.pose.orientation.y);
        orientation.setZ(odom.pose.pose.orientation.z);
        orientation.setW(odom.pose.pose.orientation.w);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }
    //! 存储lidar_odom消息的变换关系
    void mappingOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        mappingOdomAffine = odom2affine(*odomMsg);
        mappingOdomTime = odomMsg->header.stamp.sec + odomMsg->header.stamp.nanosec / 1e9;

    }
    //! imu预积分里程回调函数，根据后端优化后的激光里程消息，结合imu的位姿估计，得到当前时刻的位姿，并发布TF和odom消息，imu path
    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        // static tf
        // 设置map和odom坐标系重合，发布静态TF
        static  std::shared_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom = std::make_shared<tf2_ros::TransformBroadcaster>(node);
        tf2::Quaternion q0;
        q0.setRPY(0, 0, 0);
        static tf2::Transform map_to_odom = tf2::Transform(q0, tf2::Vector3(0, 0, 0));

        geometry_msgs::msg::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = odomMsg->header.stamp;
        static_transformStamped.header.frame_id = mapFrame;  
        static_transformStamped.child_frame_id = odometryFrame;  
        static_transformStamped.transform.translation.x = map_to_odom.getOrigin().x();
        static_transformStamped.transform.translation.y = map_to_odom.getOrigin().y();
        static_transformStamped.transform.translation.z = map_to_odom.getOrigin().z();
        static_transformStamped.transform.rotation.x = map_to_odom.getRotation().x();
        static_transformStamped.transform.rotation.y = map_to_odom.getRotation().y();
        static_transformStamped.transform.rotation.z = map_to_odom.getRotation().z();
        static_transformStamped.transform.rotation.w = map_to_odom.getRotation().w();

        tfMap2Odom->sendTransform(static_transformStamped);

        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (mappingOdomTime == -1){
            return;
        }
        while (!lidarOdomQueue.empty())
        {
            // 前端里程和后端里程进行时间标定
            if (lidarOdomQueue.front().header.stamp.sec + lidarOdomQueue.front().header.stamp.nanosec * 1e-9 <= mappingOdomTime)
                lidarOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f lidarOdomAffineFront = odom2affine(lidarOdomQueue.front());
        Eigen::Affine3f lidarOdomAffineBack = odom2affine(lidarOdomQueue.back());
        // 求前后两帧lidar里程计位姿的变换关系
        Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
        // Eigen::Matrix<float, 6, 1> lidar_pose_front;
        // Eigen::Matrix<float, 6, 1> lidar_pose_back;
        // Eigen::Matrix<float, 6, 1> lidar_pose_incre;
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineFront, lidar_pose_front(0),
        //                                                         lidar_pose_front(1),
        //                                                         lidar_pose_front(2),
        //                                                         lidar_pose_front(3),
        //                                                         lidar_pose_front(4),
        //                                                         lidar_pose_front(5));
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineBack, lidar_pose_back(0),
        //                                                         lidar_pose_back(1),
        //                                                         lidar_pose_back(2),
        //                                                         lidar_pose_back(3),
        //                                                         lidar_pose_back(4),
        //                                                         lidar_pose_back(5));
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineIncre, lidar_pose_incre(0),
        //                                                         lidar_pose_incre(1),
        //                                                         lidar_pose_incre(2),
        //                                                         lidar_pose_incre(3),
        //                                                         lidar_pose_incre(4),
        //                                                         lidar_pose_incre(5));

        // std::cout << "lidar_pose_front: " << lidar_pose_front.transpose() << std::endl;
        // std::cout << "lidar_pose_back: " << lidar_pose_back.transpose() << std::endl;
        // std::cout << "lidar_pose_incre: " << lidar_pose_incre.transpose() << std::endl;
        
        // 对最新的激光里程位姿进行相应变换，得到当前帧的激光里程估计
        Eigen::Affine3f lidarOdomAffineLast = mappingOdomAffine * lidarOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(lidarOdomAffineLast, x, y, z, roll, pitch, yaw);
        // publish latest odometry
        nav_msgs::msg::Odometry laserOdometry = lidarOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        tf2::Quaternion q1;
        q1.setRPY(roll, pitch, yaw);
        laserOdometry.pose.pose.orientation.x = q1.x();
        laserOdometry.pose.pose.orientation.y = q1.y();
        laserOdometry.pose.pose.orientation.z = q1.z();
        laserOdometry.pose.pose.orientation.w = q1.w();
        pubLidarOdometry->publish(laserOdometry);

        // publish tf
        // 发布 odom -> base_link 的 TF
        static std::shared_ptr<tf2_ros::TransformBroadcaster> tfOdom2BaseLink = std::make_shared<tf2_ros::TransformBroadcaster>(node);
        tf2::Transform tCur;
        tCur.setOrigin(tf2::Vector3(laserOdometry.pose.pose.position.x, laserOdometry.pose.pose.position.y, laserOdometry.pose.pose.position.z));
        tCur.setRotation(tf2::Quaternion(laserOdometry.pose.pose.orientation.x, laserOdometry.pose.pose.orientation.y, laserOdometry.pose.pose.orientation.z, laserOdometry.pose.pose.orientation.w));

        if (lidarFrame != baselinkFrame)
        {
            tf2::Transform tf;
            tf.setOrigin(tf2::Vector3(lidar2Baselink.transform.translation.x, lidar2Baselink.transform.translation.y, lidar2Baselink.transform.translation.z));
            tf.setRotation(tf2::Quaternion(lidar2Baselink.transform.rotation.x, lidar2Baselink.transform.rotation.y, lidar2Baselink.transform.rotation.z, lidar2Baselink.transform.rotation.w));
            tCur = tCur * tf;
        }

        geometry_msgs::msg::TransformStamped odom_2_baselink;
        odom_2_baselink.header = odomMsg->header;
        odom_2_baselink.child_frame_id = baselinkFrame;
        odom_2_baselink.transform.translation.x = tCur.getOrigin().x();
        odom_2_baselink.transform.translation.y = tCur.getOrigin().y();
        odom_2_baselink.transform.translation.z = tCur.getOrigin().z();
        odom_2_baselink.transform.rotation.x = tCur.getRotation().x();
        odom_2_baselink.transform.rotation.y = tCur.getRotation().y();
        odom_2_baselink.transform.rotation.z = tCur.getRotation().z();
        odom_2_baselink.transform.rotation.w = tCur.getRotation().w();
        tfOdom2BaseLink->sendTransform(odom_2_baselink);

        // publish Lidar odometry path
        static nav_msgs::msg::Path lidarPath;
        static double last_path_time = -1;
        double lidarTime = lidarOdomQueue.back().header.stamp.sec + lidarOdomQueue.back().header.stamp.nanosec * 1e-9;
        if (lidarTime - last_path_time > 0.05)
        {
            last_path_time = lidarTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = lidarOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            lidarPath.poses.push_back(pose_stamped);
            // 只保留1s的imu里程计轨迹
            while(!lidarPath.poses.empty() && ((lidarPath.poses.front().header.stamp.sec + lidarPath.poses.front().header.stamp.nanosec * 1e-9) < mappingOdomTime - 1.0))
                lidarPath.poses.erase(lidarPath.poses.begin());
            if (pubLidarPath->get_subscription_count() != 0)
            {
                lidarPath.header.stamp = lidarOdomQueue.back().header.stamp;
                lidarPath.header.frame_id = odometryFrame;
                pubLidarPath->publish(lidarPath);
            }
        }
    }
};

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
        std::cout << "transformation_interpolated: " << transformation_interpolated.matrix() << std::endl;
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
        if (!(Translation == Eigen::Vector3d::Zero())) {
            RCLCPP_INFO(node->get_logger(), "Translation: %f, %f, %f", Translation(0), Translation(1), Translation(2));
            try
            {
                rot_vgicp.computeTranslation(*aligned, Reg_translation, Translation, TranslationOld, 0.1, 0.1, CT_lambda);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            
        }
        else{
            std::cout << "Translation: " << Translation.transpose() << std::endl;
            // return;
        }
           
        std::cout << "Reg_translation: " << Reg_translation.transpose() << std::endl;
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
    auto node_LO = std::make_shared<rclcpp::Node>("rolo_ros2");
    auto node_TF = std::make_shared<rclcpp::Node>("tf_broadcaster");


    LidarOdometry LO(node_LO);
    TransformFusion TF(node_TF);

    // if(!LO.loopClosureEnableFlag){
    //     tum_file.open("/home/sdu/slam_time/rolo/rolo_front.tum");
    // }
    // else{
    //     tum_file.open("/home/sdu/slam_time/rolo_lc/rolo_lc_front.tum");
    // }
    // tum_file.clear();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_LO);
    executor.add_node(node_TF);
    RCLCPP_INFO(node_LO->get_logger(), "\033[1;32m----> Laser Odometry Started.\033[0m");
    executor.spin();
    // tum_file.close();
    
    return 0;
}