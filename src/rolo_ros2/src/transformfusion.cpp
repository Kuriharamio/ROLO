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
        tf2::Quaternion quaternion = tf2::Quaternion(map_to_odom.getRotation().x(), map_to_odom.getRotation().y(), map_to_odom.getRotation().z(), map_to_odom.getRotation().w());
        if (std::isnan(quaternion.x()) || std::isnan(quaternion.y()) || std::isnan(quaternion.z()) || std::isnan(quaternion.w())) {
            // 四元数无效，需要处理
            return;
        }
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
        quaternion = tf2::Quaternion(tCur.getRotation().x(), tCur.getRotation().y(), tCur.getRotation().z(), tCur.getRotation().w());
        if (std::isnan(quaternion.x()) || std::isnan(quaternion.y()) || std::isnan(quaternion.z()) || std::isnan(quaternion.w())) {
            // 四元数无效，需要处理
            return;
        }
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // auto node_LO = std::make_shared<rclcpp::Node>("LidarOdometry");
    auto node_TF = std::make_shared<rclcpp::Node>("TransformFusion");


    // LidarOdometry LO(node_LO);
    TransformFusion TF(node_TF);

    // if(!LO.loopClosureEnableFlag){
    //     tum_file.open("/home/sdu/slam_time/rolo/rolo_front.tum");
    // }
    // else{
    //     tum_file.open("/home/sdu/slam_time/rolo_lc/rolo_lc_front.tum");
    // }
    // tum_file.clear();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node_LO);
    executor.add_node(node_TF);
    RCLCPP_INFO(node_TF->get_logger(), "\033[1;32m----> TransformFusion Started.\033[0m");
    executor.spin();
    // tum_file.close();
    
    return 0;
}