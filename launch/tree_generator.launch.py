#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="forest_map_generator",
                executable="forest_map_generator",
                name="forest_map_generator",
                output="screen",
                parameters=[
                    {
                        "heightmap_file": "heightmap.png",
                        "num_trees": 200,
                        "tree_types": [
                            "tree1",
                            "tree2",
                            "tree3",
                            "tree4",
                            "tree5",
                            "tree6",
                            "tree7",
                            "tree8",
                            "tree9",
                            "tree10",
                            "tree11",
                            "tree12",
                            "tree13",
                            "tree14",
                        ],
                        "terrain_size_x": 257,
                        "terrain_size_y": 257,
                        "terrain_size_z": 50,
                        "min_tree_distance": 5.0,
                        "max_slope": 30.0,
                        "output_world_file": "world_with_trees.world",
                    }
                ],
            ),
        ]
    )
