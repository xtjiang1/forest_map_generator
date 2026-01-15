import bpy
import sys
import os


def arg_after(flag, default=None):
    argv = sys.argv
    if "--" not in argv:
        return default
    argv = argv[argv.index("--") + 1 :]
    if flag in argv:
        i = argv.index(flag)
        if i + 1 < len(argv):
            return argv[i + 1]
    return default


in_mesh = arg_after("--in")
out_dae = arg_after("--out_dae")
out_png = arg_after("--out_png")
tex_size = int(arg_after("--tex", "2048"))

if not in_mesh or not out_dae or not out_png:
    raise SystemExit(
        "Usage: blender -b -P bake_vcol_to_texture.py -- "
        "--in in.ply --out_dae out.dae --out_png albedo.png [--tex 2048]"
    )

bpy.ops.wm.read_factory_settings(use_empty=True)

try:
    bpy.ops.preferences.addon_enable(module="io_mesh_ply")
except:
    pass
try:
    bpy.ops.preferences.addon_enable(module="io_scene_obj")
except:
    pass

ext = os.path.splitext(in_mesh)[1].lower()
if ext == ".ply":
    bpy.ops.import_mesh.ply(filepath=in_mesh)
elif ext == ".obj":
    bpy.ops.import_scene.obj(filepath=in_mesh)
else:
    raise SystemExit("Unsupported input: " + ext)

obj = bpy.context.selected_objects[0]
bpy.context.view_layer.objects.active = obj
mesh = obj.data

if not mesh.uv_layers:
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=66.0, island_margin=0.05)
    bpy.ops.object.mode_set(mode="OBJECT")

vcol_name = None
if hasattr(mesh, "color_attributes") and len(mesh.color_attributes) > 0:
    vcol_name = mesh.color_attributes.active.name
elif hasattr(mesh, "vertex_colors") and len(mesh.vertex_colors) > 0:
    vcol_name = mesh.vertex_colors.active.name

mat = bpy.data.materials.new("VColMat")
mat.use_nodes = True
nodes = mat.node_tree.nodes
links = mat.node_tree.links
nodes.clear()

out_node = nodes.new(type="ShaderNodeOutputMaterial")
bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")
out_node.location = (450, 0)
bsdf.location = (200, 0)
links.new(bsdf.outputs["BSDF"], out_node.inputs["Surface"])

if vcol_name is not None:
    vcol_node = nodes.new(type="ShaderNodeVertexColor")
    vcol_node.layer_name = vcol_name
    vcol_node.location = (-150, 0)
    links.new(vcol_node.outputs["Color"], bsdf.inputs["Emission"])
    bsdf.inputs["Emission Strength"].default_value = 1.0
else:
    bsdf.inputs["Emission"].default_value = (0.4, 0.6, 0.4, 1.0)
    bsdf.inputs["Emission Strength"].default_value = 1.0

img_name = os.path.splitext(os.path.basename(out_png))[0]
img = bpy.data.images.new(img_name, width=tex_size, height=tex_size)
img.filepath_raw = out_png
img.file_format = "PNG"

avg_color = [0.5, 0.5, 0.5, 1.0]
try:
    cols = []
    if (
        hasattr(mesh, "color_attributes")
        and vcol_name
        and vcol_name in mesh.color_attributes
    ):
        ca = mesh.color_attributes[vcol_name]
        cols = [c.color[:3] for c in ca.data]
    elif (
        hasattr(mesh, "vertex_colors") and vcol_name and vcol_name in mesh.vertex_colors
    ):
        vc = mesh.vertex_colors[vcol_name]
        cols = [c.color[:3] for c in vc.data]
    if cols:
        mean = [sum(c[i] for c in cols) / len(cols) for i in range(3)]
        avg_color = [mean[0], mean[1], mean[2], 1.0]
except:
    pass

img.pixels = avg_color * (tex_size * tex_size)

img_node = nodes.new(type="ShaderNodeTexImage")
img_node.image = img
img_node.location = (-150, -250)
nodes.active = img_node

if obj.data.materials:
    obj.data.materials[0] = mat
else:
    obj.data.materials.append(mat)

bpy.context.scene.render.engine = "CYCLES"
bpy.context.scene.cycles.samples = 1
bpy.context.scene.render.bake.margin = 32
bpy.context.scene.render.bake.use_clear = False

bpy.ops.object.select_all(action="DESELECT")
obj.select_set(True)
bpy.context.view_layer.objects.active = obj
bpy.ops.object.bake(type="EMIT")

links.new(img_node.outputs["Color"], bsdf.inputs["Base Color"])
bsdf.inputs["Emission"].default_value = (0.0, 0.0, 0.0, 1.0)
bsdf.inputs["Emission Strength"].default_value = 0.0

os.makedirs(os.path.dirname(out_png), exist_ok=True)
img.save()

os.makedirs(os.path.dirname(out_dae), exist_ok=True)
bpy.ops.wm.collada_export(filepath=out_dae, apply_modifiers=True, triangulate=True)

print("Baked:", out_png)
print("Exported:", out_dae)
