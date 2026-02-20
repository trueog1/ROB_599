import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    this_directory = get_package_share_directory('hw2')

    use_stamped_velocity = LaunchConfiguration('use_stamped_velocity')
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='true',
        description='on true stage will accept TwistStamped command messages')
    
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='open'),
        description='World file relative to the project world file, without .world')


    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    
    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!')

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
    
    def stage_world_configuration(context):
        file = os.path.join(
            this_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument('namespace', default_value=TextSubstitution(text=''))

    rviz_config = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='open'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    def rviz_launch_configuration(context):
        file = os.path.join(
            this_directory,
            'config/rviz',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    return LaunchDescription([
        use_stamped_velocity_arg,
        stage_world_arg,
        one_tf_tree_arg, 
        enforce_prefixes_arg, 
        use_static_transformations_arg, 
        stage_world_configuration_arg,
        namespace_arg,
        rviz_config_arg,
        rviz_launch_configuration_arg,
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{'one_tf_tree': one_tf_tree,
                        'enforce_prefixes': enforce_prefixes,
                        'use_stamped_velocity': use_stamped_velocity,
                        'use_static_transformations': use_static_transformations,
                "world_file": [LaunchConfiguration('world_file')]}],
        ),
        Node(
            package='rviz2',
            namespace=namespace,
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config]],
            parameters=[{
                "use_sim_time": use_sim_time}],
        )
    ])
