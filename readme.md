# forest_map_generator (ROS2 + Gazebo Fortress)

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

---
