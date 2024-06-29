from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments (parameters) to be passed to the included launch files
    param1 = LaunchConfiguration('param1', default='value1')
    param2 = LaunchConfiguration('param2', default='value2')
    param3 = LaunchConfiguration('param3', default='value3')
    param4 = LaunchConfiguration('param4', default='value4')

    # Declare the launch arguments
    declare_param1 = DeclareLaunchArgument('param1', default_value='value1', description='Parameter for first launch file')
    declare_param2 = DeclareLaunchArgument('param2', default_value='value2', description='Parameter for first launch file')
    declare_param3 = DeclareLaunchArgument('param3', default_value='value3', description='Parameter for second launch file')
    declare_param4 = DeclareLaunchArgument('param4', default_value='value4', description='Parameter for second launch file')
    declare_rviz_config = DeclareLaunchArgument('rvizconfig', default_value='/home/rpros2024ss/navigation_ws/src/octomap_astar_ros/rviz/example_04.rviz', description='Path to the RViz config file')

    rviz_config_file = LaunchConfiguration('rvizconfig')


    # Include the second launch file with a delay and pass parameters
    delayed_launch_node = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            # Add a normal node
            Node(
                package='octomap_astar_ros',
                executable='example_04_node',
                name='astar_planner_node',
                output='screen',
                # parameters=[{
                #     'your_param_name': 'your_param_value'
                # }]
                remappings=[
                    ('/astar_planner_node/in/pose3d/goal', '/interactive_marker_node_goal/out/pose3d/goal'),
                    ('/astar_planner_node/in/pose3d/start', '/interactive_marker_node_start/out/pose3d/start'),
                    ('/astar_planner/in/octomap', '/octomap_mockup_publisher/out/octoMapMockup')
                ]
            )
        ]
    )

    # Add the RViz node with the specified config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        declare_rviz_config,
        rviz_node,
        delayed_launch_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
