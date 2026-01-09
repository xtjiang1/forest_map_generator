import os
import numpy as np
import open3d as o3d
import trimesh

# === Configuration ===
INPUT_DIR = "trees_ply"
OUTPUT_DIR = "output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

TARGET_POINTS = 550000
VISUAL_MAX_TRIANGLES = 25000
COLLISION_MAX_TRIANGLES = 5000


def process_ply_file(input_path, output_dae, output_stl):
    if not os.path.exists(input_path):
        print(f"Error: File not found: {input_path}")
        return False

    print(f"Loading point cloud: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    if len(pcd.points) == 0:
        print("Error: Point cloud is empty")
        return False

    points_np = np.asarray(pcd.points)

    centroid_xy = np.mean(points_np[:, :2], axis=0)
    points_np[:, 0] -= centroid_xy[0]
    points_np[:, 1] -= centroid_xy[1]

    min_z = np.min(points_np[:, 2])
    points_np[:, 2] -= min_z

    pcd.points = o3d.utility.Vector3dVector(points_np)
    print(
        f"Tree aligned: base at Z=0, XY centered. Original XY center: {centroid_xy}, original min Z: {min_z}"
    )

    if len(pcd.points) > TARGET_POINTS:
        print(f"Downsampling to {TARGET_POINTS} points...")
        pcd = pcd.random_down_sample(TARGET_POINTS / len(pcd.points))

    print("Estimating and aligning normals...")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    try:
        pcd.orient_normals_consistent_tangent_plane(k=10)
    except RuntimeError:
        print("Warning: Normal orientation failed. Proceeding with raw normals.")

    print("Running Poisson reconstruction...")
    mesh_o3d, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=10, scale=1.1, linear_fit=False
    )

    print("Removing low-density vertices...")
    if len(densities) > 0:
        densities = np.asarray(densities)
        vertices_to_remove = densities < np.quantile(densities, 0.05)
        mesh_o3d.remove_vertices_by_mask(vertices_to_remove)

    mesh_o3d.remove_degenerate_triangles()
    mesh_o3d.remove_unreferenced_vertices()

    if len(mesh_o3d.vertices) == 0:
        print("Error: Mesh is empty after reconstruction")
        return False

    original_has_color = len(pcd.colors) > 0
    if original_has_color:
        print("Mapping original point cloud colors to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        mesh_colors = []
        for vertex in mesh_o3d.vertices:
            _, idx, _ = pcd_tree.search_knn_vector_3d(vertex, 1)
            mesh_colors.append(pcd.colors[idx[0]])
        mesh_o3d.vertex_colors = o3d.utility.Vector3dVector(mesh_colors)
    else:
        print("Point cloud has no color, using uniform green.")
        mesh_o3d.paint_uniform_color([106 / 255, 171 / 255, 115 / 255])

    print(f"Simplifying visual mesh to <= {VISUAL_MAX_TRIANGLES} triangles...")
    if len(mesh_o3d.triangles) > VISUAL_MAX_TRIANGLES:
        visual_mesh = mesh_o3d.simplify_quadric_decimation(
            target_number_of_triangles=VISUAL_MAX_TRIANGLES
        )
    else:
        visual_mesh = mesh_o3d

    visual_mesh.remove_degenerate_triangles()
    visual_mesh.remove_unreferenced_vertices()

    print(
        f"Final visual mesh: {len(visual_mesh.vertices)} vertices, "
        f"{len(visual_mesh.triangles)} triangles"
    )

    print("Exporting .dae with original vertex colors...")
    tm_mesh = trimesh.Trimesh(
        vertices=np.asarray(visual_mesh.vertices),
        faces=np.asarray(visual_mesh.triangles),
        vertex_colors=(np.asarray(visual_mesh.vertex_colors) * 255).astype(np.uint8),
        process=False,
    )

    try:
        tm_mesh.export(output_dae, file_type="dae")
        print(f"Success: {output_dae} generated (vertex colors preserved)!")
    except Exception as e:
        print(f"Failed to export .dae: {e}")
        return False

    print("Generating collision mesh...")
    if len(mesh_o3d.triangles) > COLLISION_MAX_TRIANGLES:
        collision_mesh = mesh_o3d.simplify_quadric_decimation(
            target_number_of_triangles=COLLISION_MAX_TRIANGLES
        )
    else:
        collision_mesh = mesh_o3d

    collision_mesh.remove_degenerate_triangles()
    collision_mesh.remove_unreferenced_vertices()
    collision_mesh.compute_triangle_normals()

    success = o3d.io.write_triangle_mesh(output_stl, collision_mesh, write_ascii=False)
    if success:
        print(f"Collision STL saved: {output_stl}")
    else:
        print("Failed to write collision STL")

    return True


def main():
    ply_files = [f for f in os.listdir(INPUT_DIR) if f.lower().endswith(".ply")]

    for ply_file in ply_files:
        input_path = os.path.join(INPUT_DIR, ply_file)

        base_name = os.path.splitext(ply_file)[0]
        output_dae = os.path.join(OUTPUT_DIR, f"{base_name}_mesh.dae")
        output_stl = os.path.join(OUTPUT_DIR, f"{base_name}_collision.stl")

        print(f"\nProcessing {ply_file}...")
        success = process_ply_file(input_path, output_dae, output_stl)

        if success:
            print(f"Successfully processed {ply_file}")
        else:
            print(f"Failed to process {ply_file}")


if __name__ == "__main__":
    main()
