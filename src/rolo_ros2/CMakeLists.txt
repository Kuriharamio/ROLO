cmake_minimum_required(VERSION 3.8)
project(rolo_ros2)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rolo_ros2_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
	include
	${rclcpp_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_library(rot_gicp SHARED
  src/rot_gicp/gicp/lsq_registration.cpp
  src/rot_gicp/gicp/rot_vgicp.cpp
)

target_link_libraries(rot_gicp
  ${PCL_LIBRARIES}
)

target_include_directories(rot_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(imageProjection src/imageProjection.cpp)
ament_target_dependencies(imageProjection rclcpp rclcpp_components geometry_msgs nav_msgs pcl_ros std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs rolo_ros2_interfaces PCL pcl_conversions OpenCV cv_bridge GTSAM Eigen3 Boost OpenMP)
target_link_libraries(imageProjection ${OpenMP_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(featureExtraction rclcpp rclcpp_components geometry_msgs nav_msgs pcl_ros std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs rolo_ros2_interfaces PCL pcl_conversions OpenCV cv_bridge GTSAM Eigen3 Boost OpenMP)
target_link_libraries(featureExtraction ${OpenMP_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidarOdometry src/lidarOdometry.cpp)
ament_target_dependencies(lidarOdometry rclcpp rclcpp_components geometry_msgs nav_msgs pcl_ros std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs rolo_ros2_interfaces PCL pcl_conversions OpenCV cv_bridge GTSAM Eigen3 Boost OpenMP)
target_link_libraries(lidarOdometry ${OpenMP_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam rot_gicp)

add_executable(backMapping src/backMapping.cpp)
ament_target_dependencies(backMapping rclcpp rclcpp_components geometry_msgs nav_msgs pcl_ros std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs rolo_ros2_interfaces PCL pcl_conversions OpenCV cv_bridge GTSAM Eigen3 Boost OpenMP)
target_link_libraries(backMapping ${OpenMP_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam)

install(TARGETS
  imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  lidarOdometry
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  backMapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src include launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
