# forest_map_generator (ROS2 Humble + Gazebo Fortress)

A ROS 2 package for generating forest simulation environments in Gazebo Fortress, including
terrain heightmaps, procedural tree placement, and road mesh generation.

<p align="center">
  <img src="docs/images/gazebo_overview.png" width="850">
</p>

---

## Overview

This project provides a complete pipeline to build a forest scene for Gazebo from:
1) a terrain heightmap (PNG) used by the `terrain` model, and  
2) tree assets (either built-in models such as `oak_tree` / `pine_tree`, or textured meshes generated from point clouds).

The core workflow is:

- `scripts/update_heightmap/main.py` updates `models/terrain/model.sdf` (heightmap URI, size, blend, pose) and copies the PNG into `models/terrain/materials/textures/` so Gazebo can load it.
- `forest_map_generator/forest_map_generator.py` (ROS 2 node) generates a new `.world` file by inserting:
  - randomly placed tree `<include>` blocks (slope-aware + minimum distance constraints)
  - an automatically generated road mesh (`models/road/meshes/road.stl`) and the corresponding road `<include>`
- `scripts/ply_to_gazebo_textured/main.py` converts `.ply` point clouds into Gazebo-ready tree models under `models/<tree_name>/`:
  - `meshes/tree_mesh.dae` (visual)
  - `meshes/tree_collision.stl` (collision)
  - `meshes/<tree_name>_albedo.png` (texture baked via Blender)

The package is structured as an `ament_python` ROS 2 package and is intended for research workflows where repeatable generation of natural outdoor scenes is required.

---

## Repository Layout


```text
forest_map_generator/
├── forest_map_generator/
│   └── forest_map_generator.py        # ROS 2 node (tree & road generation)
│
├── scripts/
│   ├── update_heightmap/
│   │   └── main.py                    # Heightmap → terrain SDF update
│   │
│   └── ply_to_gazebo_textured/
│       ├── main.py                    # PLY → Gazebo tree model pipeline
│       └── bake_vcol_to_texture.py    # Blender texture baking (called by main.py)
│
├── models/
│   ├── terrain/                       # Heightmap-based terrain model
│   ├── road/                          # Generated road model
│   ├── oak_tree/                      # Predefined tree model
│   ├── pine_tree/                     # Predefined tree model
│   └── tree*/                         # Auto-generated tree instances
│
├── worlds/
│   └── world.world                    # Base Gazebo world
│
├── launch/
│   ├── gazebo.launch.py
│   └── tree_generator.launch.py
│
└── docs/
    └── images/
        └── gazebo_overview.png

```

---

## Key Components

- `ForestMapGenerator` (ROS 2 node): generates a new world file by injecting trees and roads into a base world.
- `TreeGenerator`: slope-aware random tree placement on the heightmap.
- `RoadGenerator`: generates a smooth road mesh (`road.stl`) while respecting slope and tree clearance.
- `update_heightmap` script: updates terrain SDF parameters and ensures the heightmap image is in the correct model path for Gazebo.
- `ply_to_gazebo_textured` pipeline: converts colored point clouds into textured meshes using Open3D + Blender baking.

### 1. ForestMapGenerator (ROS 2 Node)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**
Primary ROS 2 node that procedurally generates a forest simulation world by injecting trees (and optionally roads) into a base Gazebo world.
The node samples valid placements directly on the terrain heightmap, converts heightmap pixels into world-frame poses, and writes a new .world file under worlds/.

**Execution Flow**
1. Load the terrain heightmap and compute local slope information
2. Sample valid tree positions subject to slope and distance constraints
3. Convert heightmap pixels to world-frame poses
4. Inject generated tree instances into a new Gazebo world file

**Launch Command**
```text
ros2 launch forest_map_generator tree_generator.launch.py
```

The node writes a generated world file to the package worlds/ directory (see output_world_file below).

**Paraments**

| Parameter | Type | Description |
|----------|------|-------------|
| `heightmap_file` | `string` | Heightmap image filename under `models/terrain/heightmaps/`. Used for terrain elevation lookup and slope computation. |
| `num_trees` | `int` | Number of trees to generate and inject into the world. |
| `tree_types` | `list[string]` | List of Gazebo model names available under `models/` (e.g., `tree1`–`tree14`). A random type is selected per placement. |
| `terrain_size_x` | `int` | Heightmap resolution in X (pixels). Must match the heightmap image width. |
| `terrain_size_y` | `int` | Heightmap resolution in Y (pixels). Must match the heightmap image height. |
| `terrain_size_z` | `float` | Terrain vertical scale in meters used to convert heightmap values to world Z. |
| `min_tree_distance` | `float` | Minimum allowed distance (meters) between any two trees. |
| `max_slope` | `float` | Maximum allowed slope (degrees) for valid placements. Trees are rejected on steep terrain. |
| `output_world_file` | `string` | Output world filename written to `worlds/` (e.g., `world_with_trees.world`). |

Note: Several of the parameters above are automatically printed during heightmap loading and SDF update for verification and reproducibility.  
These outputs will be explained in detail in a later section.

**Output**
```text
worlds/<output_world_file>
```

What is written into the world

- A list of Gazebo <include> blocks, one per generated tree instance

- Each instance includes a randomized yaw for visual diversity

- Tree placement is slope-aware and respects minimum spacing constraints

**Assumptions**
- The terrain model and heightmap are pre-loaded in Gazebo
- All tree models listed in `tree_types` exist under `models/`

**Example Launch Parameters**
```text
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
                "tree1","tree2","tree3","tree4","tree5","tree6","tree7",
                "tree8","tree9","tree10","tree11","tree12","tree13","tree14",
            ],
            "terrain_size_x": 257,
            "terrain_size_y": 257,
            "terrain_size_z": 50,
            "min_tree_distance": 5.0,
            "max_slope": 30.0,
            "output_world_file": "world_with_trees.world",
        }
    ],
)
```

**Reproducibility**  
For fixed parameters and heightmap input, the generation process is stochastic due to randomized tree placement, orientation, and type selection.  
A fixed random seed is planned to be introduced to enable reproducible map generation for benchmarking and evaluation.

---
