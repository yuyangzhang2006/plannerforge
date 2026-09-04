import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    """读取项目内配置并创建地图、规划和 RViz 节点。"""

    simple_map_share = get_package_share_directory("simple_map")
    map_config_path = LaunchConfiguration("map_config").perform(context)
    with open(map_config_path, "r", encoding="utf-8") as stream:
        yaml_data = yaml.safe_load(stream)

    cloud_source = yaml_data["/**"]["ros__parameters"]["cloud_bridge"]
    image_filename = cloud_source.get("image_filename", "")
    image_path = os.path.join(simple_map_share, "maps", image_filename)

    simple_map_params = {
        "image_path": image_path,
        "obstacle_gray_threshold": cloud_source["obstacle_gray_threshold"],
        "map_bounds_m": cloud_source["map_bounds_m"],
        "image_origin_m": cloud_source["image_origin_m"],
        "image_resolution_m_per_pixel": cloud_source["image_resolution_m_per_pixel"],
        "map_resolution_m_per_cell": cloud_source["map_resolution_m_per_cell"],
    }

    planner_share = get_package_share_directory("gcopter")
    planner_config = os.path.join(planner_share, "config", "gcopter_param.yaml")
    rviz_config = os.path.join(planner_share, "rviz", "planner_forge.rviz")

    simple_map_node = Node(
        package="simple_map",
        executable="simple_map_node",
        name="simple_map",
        output="screen",
        parameters=[
            {"cloud_bridge": simple_map_params},
            {
                "cloud_bridge.generate_dynamic_obstacles": ParameterValue(
                    LaunchConfiguration("generate_dynamic_obstacles"), value_type=bool
                )
            },
        ],
    )

    planner_node = Node(
        package="gcopter",
        executable="global_planning",
        name="global_planning_node",
        output="screen",
        parameters=[planner_config],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return [simple_map_node, planner_node, rviz_node]


def generate_launch_description():
    """声明 PlannerForge 的启动参数。"""

    simple_map_share = get_package_share_directory("simple_map")
    default_map_config = os.path.join(simple_map_share, "config", "map_param.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_config",
                default_value=default_map_config,
                description="Map configuration installed with simple_map",
            ),
            DeclareLaunchArgument(
                "generate_dynamic_obstacles",
                default_value="false",
                description="Overlay the deterministic moving rectangle examples",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the PlannerForge configuration",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
