import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    prefix = get_package_share_directory("depthai_ros_driver")

    params_file=LaunchConfiguration("params_file")
    
    parameters = [
        {
            "frame_id": name,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings = [
        ("rgb/image", name+"/rgb/image_rect"),
        ("rgb/camera_info", name+"/rgb/camera_info"),
        ("depth/image", name+"/stereo/image_raw"),
    ]

    print(get_package_share_directory("luxonis-camera-launch"))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items()),

        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("rectify_rgb")),
            target_container=name+"_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    remappings=[('image', name+'/rgb/image_raw'),
                                ('camera_info', name+'/rgb/camera_info'),
                                ('image_rect', name+'/rgb/image_rect'),
                                ('image_rect/compressed', name+'/rgb/image_rect/compressed'),
                                ('image_rect/compressedDepth', name+'/rgb/image_rect/compressedDepth'),
                                ('image_rect/theora', name+'/rgb/image_rect/theora')]
                )
            ]),
        
        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("odom")),
            target_container=name+"_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_odom',
                    plugin='rtabmap_odom::RGBDOdometry',
                    name='rgbd_odometry',
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        ),
        
        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("slam")),
            target_container=name+"_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    name='rtabmap',
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("mobilenet")),
            package='depthai_examples', executable='mobilenet_node',
            output='screen',
            parameters=[
                {'tf_prefix': "oak"},
                {'camera_param_uri': 'package://luxonis-camera-launch/params'},
                {'sync_nn': True},
                {'nnName': "x"},
                {'resourceBaseFolder': os.path.join(get_package_share_directory("depthai_examples"), "resources")}
            ])
    ]


def generate_launch_description():
    prefix = get_package_share_directory("luxonis-camera-launch")
    declared_arguments = [
        DeclareLaunchArgument("odom", default_value="False"),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("mobilenet", default_value="False"),
        DeclareLaunchArgument("rtabmap_viz", default_value="False"),
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(prefix, 'config', 'rtabmap.yaml')),
        DeclareLaunchArgument("rectify_rgb", default_value="True"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
