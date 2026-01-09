from setuptools import setup, find_packages
from glob import glob
import os

package_name = "forest_map_generator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Package XML
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Worlds
        ("share/" + package_name + "/worlds", ["worlds/world.world"]),
        # Terrain
        (
            "share/" + package_name + "/models/terrain/heightmaps",
            glob("models/terrain/heightmaps/*.png"),
        ),
        (
            "share/" + package_name + "/models/terrain/materials/textures",
            glob("models/terrain/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/terrain/materials/scripts",
            glob("models/terrain/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/terrain",
            ["models/terrain/model.config", "models/terrain/model.sdf"],
        ),
        # Road
        (
            "share/" + package_name + "/models/road",
            ["models/road/model.config", "models/road/model.sdf"],
        ),
        ("share/" + package_name + "/models/road/meshes", glob("models/road/meshes/*")),
        # Launch files
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        # Scripts
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
        # Oak Tree
        (
            "share/" + package_name + "/models/oak_tree/materials/textures",
            glob("models/oak_tree/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/oak_tree/materials/scripts",
            glob("models/oak_tree/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/oak_tree/meshes",
            glob("models/oak_tree/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/oak_tree",
            ["models/oak_tree/model.config", "models/oak_tree/model.sdf"],
        ),
        # Pine Tree
        (
            "share/" + package_name + "/models/pine_tree/materials/textures",
            glob("models/pine_tree/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/pine_tree/materials/scripts",
            glob("models/pine_tree/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/pine_tree/meshes",
            glob("models/pine_tree/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/pine_tree",
            ["models/pine_tree/model.config", "models/pine_tree/model.sdf"],
        ),
        # tree1
        (
            "share/" + package_name + "/models/tree1/meshes",
            glob("models/tree1/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree1",
            ["models/tree1/model.config", "models/tree1/model.sdf"],
        ),
        # tree2
        (
            "share/" + package_name + "/models/tree2/meshes",
            glob("models/tree2/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree2",
            ["models/tree2/model.config", "models/tree2/model.sdf"],
        ),
        # tree3
        (
            "share/" + package_name + "/models/tree3/meshes",
            glob("models/tree3/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree3",
            ["models/tree3/model.config", "models/tree3/model.sdf"],
        ),
        # tree4
        (
            "share/" + package_name + "/models/tree4/meshes",
            glob("models/tree4/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree4",
            ["models/tree4/model.config", "models/tree4/model.sdf"],
        ),
        # tree5
        (
            "share/" + package_name + "/models/tree5/meshes",
            glob("models/tree5/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree5",
            ["models/tree5/model.config", "models/tree5/model.sdf"],
        ),
        # tree6
        (
            "share/" + package_name + "/models/tree6/meshes",
            glob("models/tree6/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree6",
            ["models/tree6/model.config", "models/tree6/model.sdf"],
        ),
        # tree7
        (
            "share/" + package_name + "/models/tree7/meshes",
            glob("models/tree7/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree7",
            ["models/tree7/model.config", "models/tree7/model.sdf"],
        ),
        # tree8
        (
            "share/" + package_name + "/models/tree8/meshes",
            glob("models/tree8/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree8",
            ["models/tree8/model.config", "models/tree8/model.sdf"],
        ),
        # tree9
        (
            "share/" + package_name + "/models/tree9/meshes",
            glob("models/tree9/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree9",
            ["models/tree9/model.config", "models/tree9/model.sdf"],
        ),
        # tree10
        (
            "share/" + package_name + "/models/tree10/meshes",
            glob("models/tree10/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree10",
            ["models/tree10/model.config", "models/tree10/model.sdf"],
        ),
        # tree11
        (
            "share/" + package_name + "/models/tree11/meshes",
            glob("models/tree11/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree11",
            ["models/tree11/model.config", "models/tree11/model.sdf"],
        ),
        # tree12
        (
            "share/" + package_name + "/models/tree12/meshes",
            glob("models/tree12/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree12",
            ["models/tree12/model.config", "models/tree12/model.sdf"],
        ),
        # tree13
        (
            "share/" + package_name + "/models/tree13/meshes",
            glob("models/tree13/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree13",
            ["models/tree13/model.config", "models/tree13/model.sdf"],
        ),
        # tree14
        (
            "share/" + package_name + "/models/tree14/meshes",
            glob("models/tree14/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree14",
            ["models/tree14/model.config", "models/tree14/model.sdf"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "forest_map_generator = forest_map_generator.forest_map_generator:main",
            "update_heightmap = forest_map_generator.update_heightmap:main",
        ],
    },
)
