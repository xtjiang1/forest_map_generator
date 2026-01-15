import os
import subprocess
import numpy as np
import open3d as o3d

TARGET_POINTS = 550000
VISUAL_MAX_TRIANGLES = 25000
COLLISION_MAX_TRIANGLES = 5000

BLENDER_BIN = "blender"
TEX_SIZE = 2048

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "trees_ply")
BAKE_SCRIPT = os.path.join(SCRIPT_DIR, "bake_vcol_to_texture.py")


def run_blender_bake(in_colored_ply, out_dae, out_png):
    cmd = [
        BLENDER_BIN,
        "-b",
        "-P",
        BAKE_SCRIPT,
        "--",
        "--in",
        in_colored_ply,
        "--out_dae",
        out_dae,
        "--out_png",
        out_png,
        "--tex",
        str(TEX_SIZE),
    ]
    print("Running Blender bake:", " ".join(cmd))
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        print("Blender failed!\nSTDOUT:\n", r.stdout, "\nSTDERR:\n", r.stderr)
        return False
    print(r.stdout.strip())
    return True


def process_ply_file(
    input_path, out_textured_dae, out_albedo_png, out_stl, base_name, meshes_dir
):
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
        f"Tree aligned: base Z=0, XY centered. orig center={centroid_xy}, orig minZ={min_z}"
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
        mesh_o3d.remove_vertices_by_mask(densities < np.quantile(densities, 0.05))

    mesh_o3d.remove_degenerate_triangles()
    mesh_o3d.remove_unreferenced_vertices()
    if len(mesh_o3d.vertices) == 0:
        print("Error: Mesh empty after reconstruction")
        return False

    original_has_color = len(pcd.colors) > 0
    if original_has_color:
        print("Mapping original point cloud colors to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        pcd_pts = np.asarray(pcd.points)
        pcd_cols = np.asarray(pcd.colors)

        k = 8
        max_dist = 0.5
        avg_col = pcd_cols.mean(axis=0)

        mesh_colors = []
        for v in np.asarray(mesh_o3d.vertices):
            _, idx, _ = pcd_tree.search_knn_vector_3d(v, k)
            neigh_pts = pcd_pts[idx]
            neigh_cols = pcd_cols[idx]

            d = np.linalg.norm(neigh_pts - v, axis=1)
            dmin = float(d.min())

            if dmin > max_dist:
                mesh_colors.append(avg_col)
            else:
                w = 1.0 / (d + 1e-6)
                w /= w.sum()
                mesh_colors.append((neigh_cols * w[:, None]).sum(axis=0))

        mesh_o3d.vertex_colors = o3d.utility.Vector3dVector(np.asarray(mesh_colors))
    else:
        print("Point cloud has no color, using uniform green.")
        mesh_o3d.paint_uniform_color([106 / 255, 171 / 255, 115 / 255])

    print(f"Simplifying visual mesh to <= {VISUAL_MAX_TRIANGLES} triangles...")
    if len(mesh_o3d.triangles) > VISUAL_MAX_TRIANGLES:
        visual_mesh = mesh_o3d.simplify_quadric_decimation(VISUAL_MAX_TRIANGLES)
    else:
        visual_mesh = mesh_o3d

    visual_mesh.remove_degenerate_triangles()
    visual_mesh.remove_unreferenced_vertices()

    print(
        f"Final visual mesh: {len(visual_mesh.vertices)} verts, {len(visual_mesh.triangles)} tris"
    )

    colored_ply = os.path.join(meshes_dir, f"{base_name}_visual_colored.ply")
    ok = o3d.io.write_triangle_mesh(colored_ply, visual_mesh, write_ascii=False)
    if not ok:
        print("Failed to write colored visual PLY:", colored_ply)
        return False
    print("Wrote colored visual PLY:", colored_ply)

    if not os.path.exists(BAKE_SCRIPT):
        print(f"Error: bake script not found: {BAKE_SCRIPT}")
        return False

    baked = run_blender_bake(colored_ply, out_textured_dae, out_albedo_png)
    if not baked:
        return False

    print("Generating collision mesh...")
    if len(mesh_o3d.triangles) > COLLISION_MAX_TRIANGLES:
        collision_mesh = mesh_o3d.simplify_quadric_decimation(COLLISION_MAX_TRIANGLES)
    else:
        collision_mesh = mesh_o3d

    collision_mesh.remove_degenerate_triangles()
    collision_mesh.remove_unreferenced_vertices()
    collision_mesh.compute_triangle_normals()

    ok = o3d.io.write_triangle_mesh(out_stl, collision_mesh, write_ascii=False)
    if ok:
        print("Collision STL saved:", out_stl)
    else:
        print("Failed to write collision STL")
        return False

    return True


def main():
    if not os.path.isdir(INPUT_DIR):
        print("INPUT_DIR not found:", INPUT_DIR)
        return

    ply_files = [f for f in os.listdir(INPUT_DIR) if f.lower().endswith(".ply")]
    if not ply_files:
        print("No .ply files found in:", INPUT_DIR)
        return

    pkg_root = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
    models_dir = os.path.join(pkg_root, "models")

    for ply_file in ply_files:
        input_path = os.path.join(INPUT_DIR, ply_file)
        base_name = os.path.splitext(ply_file)[0]

        model_dir = os.path.join(models_dir, base_name)
        meshes_dir = os.path.join(model_dir, "meshes")
        os.makedirs(meshes_dir, exist_ok=True)

        out_textured_dae = os.path.join(meshes_dir, "tree_mesh.dae")
        out_stl = os.path.join(meshes_dir, "tree_collision.stl")
        out_albedo_png = os.path.join(meshes_dir, f"{base_name}_albedo.png")

        print(f"\n=== Processing {ply_file} -> {meshes_dir} ===")
        success = process_ply_file(
            input_path, out_textured_dae, out_albedo_png, out_stl, base_name, meshes_dir
        )
        print(("OK" if success else "FAILED"), ply_file)


if __name__ == "__main__":
    main()
