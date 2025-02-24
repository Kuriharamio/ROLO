import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    package_name = 'rolo_ros2'  
    param_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')

    load_params = ExecuteProcess(
        cmd=['ros2', 'param', 'load', param_file_path],
        output='screen'
    )

    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'base_link'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'rolo_ros2.rviz')],
        output='screen'
    )

    imageProjection = Node(
        package=package_name,
        executable='imageProjection',  
        name='imageProjection',  
        parameters=[param_file_path],
        output='screen'
    )

    featureExtraction = Node(
        package=package_name,
        executable='featureExtraction',  
        name='featureExtraction',  
        parameters=[param_file_path],
        output='screen'
    )

    lidarOdometry = Node(
        package=package_name,
        executable='lidarOdometry',  
        name='lidarOdometry',  
        parameters=[param_file_path],
        output='screen'
    )

    transformfusion = Node(
        package=package_name,
        executable='transformfusion',  
        name='transformfusion',  
        parameters=[param_file_path],
        output='screen'
    )


    backMapping = Node(
        package=package_name,
        executable='backMapping',  
        name='backMapping',  
        parameters=[param_file_path],
        output='screen'
    )
    

    return LaunchDescription([
        # tf_broadcaster,
        transformfusion,
        imageProjection,
        featureExtraction,
        lidarOdometry,
        backMapping,
        rviz
    ])