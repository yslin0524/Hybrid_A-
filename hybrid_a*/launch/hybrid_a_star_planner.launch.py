from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Hybrid A* Planner Node
        Node(
            package='hybrid_a_star_planner',
            executable='hybrid_a_star_planner_node',
            name='hybrid_a_star_planner_node',
            output='screen'
        ),

        # Odometry to Pose Converter Node (Python script)
        Node(
	    package='hybrid_a_star_planner',
	    executable='odometry_to_pose_converter.py',
	    name='odometry_to_pose_converter',
	    output='screen'
        )
    ])

