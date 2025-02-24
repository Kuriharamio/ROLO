import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'rolo_ros2'  
    param_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')

    # param_file_arg = DeclareLaunchArgument(
    #     'param_file',
    #     default_value=param_file_path,
    #     description='Path to the parameter file'
    # )

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
        transformfusion,
        imageProjection,
        featureExtraction,
        lidarOdometry,
        backMapping
    ])