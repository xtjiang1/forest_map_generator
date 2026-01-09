import os
import sys
import argparse
from PIL import Image
import xml.etree.ElementTree as ET

# python3 update_terrain.py --heightmap /path/to/heightmap.png --height_range 75 --blend1_min 2 --blend1_fade 4 --blend2_min 15 --blend2_fade 5 --pos_x 10 --pos_y 5 --pos_z 0

def main():
    parser = argparse.ArgumentParser(description='Update terrain SDF with new heightmap parameters')
    parser.add_argument('--heightmap', default='heightmap.png', help='Path to heightmap image')
    parser.add_argument('--height_range', type=float, default=50, help='Height range value')
    parser.add_argument('--blend1_min', type=float, default=1, help='Blend1 minimum height')
    parser.add_argument('--blend1_fade', type=float, default=3, help='Blend1 fade distance')
    parser.add_argument('--blend2_min', type=float, default=10, help='Blend2 minimum height')
    parser.add_argument('--blend2_fade', type=float, default=3, help='Blend2 fade distance')
    parser.add_argument('--pos_x', type=float, default=0, help='X position')
    parser.add_argument('--pos_y', type=float, default=0, help='Y position')
    parser.add_argument('--pos_z', type=float, default=0, help='Z position')
    
    args = parser.parse_args()

    try:
        with Image.open(args.heightmap) as img:
            width, height = img.size
        print(f"Heightmap info: size {width}×{height}, height range {args.height_range}")
        print(f"Blend config: blend1(min_height={args.blend1_min}, fade_dist={args.blend1_fade}), blend2(min_height={args.blend2_min}, fade_dist={args.blend2_fade})")
        print(f"Position config: pos({args.pos_x} {args.pos_y} {args.pos_z})")
    except FileNotFoundError:
        print(f"Heightmap file not found: {args.heightmap}")
        sys.exit(1)

    try:
        script_path = os.path.abspath(__file__)
        script_dir = os.path.dirname(script_path)
        parent_dir = os.path.dirname(os.path.dirname(script_dir))
        sdf_path = os.path.join(parent_dir, 'worlds', 'terrain.sdf')

        tree = ET.parse(sdf_path)
        root = tree.getroot()
        sdf_version = root.attrib.get('version', '1.6')
        print(f"SDF file parsed successfully, version: {sdf_version}")

    except Exception as e:
        print(f"Failed to parse SDF file: {str(e)}")
        sys.exit(1)

    try:
        models = root.findall('.//model')
        if not models:
            print("No model elements found")
            sys.exit(1)

        for model in models:
            model_name = model.attrib.get('name', 'Unknown model')
            print(f"Processing model: {model_name}")

            link = model.find('link')
            if not link:
                print(f"No link element in model {model_name}, skipping")
                continue

            visual = link.find('visual')
            if not visual:
                print(f"No visual element in model {model_name}, skipping")
                continue

            geometry = visual.find('geometry')
            if not geometry:
                print(f"No geometry element in model {model_name}, skipping")
                continue

            heightmap = geometry.find('heightmap')
            if not heightmap:
                print(f"No heightmap element in model {model_name}, skipping")
                continue
            
            new_uri = os.path.join('height_maps', os.path.basename(args.heightmap))
            uri_elems = heightmap.findall('uri')
            
            if uri_elems:
                for i, uri_elem in enumerate(uri_elems):
                    if i == 0:
                        old_uri = uri_elem.text
                        uri_elem.text = new_uri
                        print(f"Updated uri: {old_uri} -> {new_uri}")
                    else:
                        heightmap.remove(uri_elem)
            else:
                ET.SubElement(heightmap, 'uri').text = new_uri
                print(f"Added uri: {new_uri}")

            new_size = f"{width} {height} {args.height_range}"
            size_elems = heightmap.findall('size')
            
            if size_elems:
                for i, size_elem in enumerate(size_elems):
                    if i == 0:
                        old_size = size_elem.text
                        size_elem.text = new_size
                        print(f"Updated size: {old_size} -> {new_size}")
                    else:
                        heightmap.remove(size_elem)
            else:
                ET.SubElement(heightmap, 'size').text = new_size
                print(f"Added size: {new_size}")

            blend_elems = heightmap.findall('blend')
            
            if len(blend_elems) > 0:
                blend1 = blend_elems[0]
            else:
                blend1 = ET.SubElement(heightmap, 'blend')
                print("Added first blend element")
            
            min_height1 = blend1.find('min_height')
            if min_height1 is not None:
                old_min_height1 = min_height1.text
                min_height1.text = str(args.blend1_min)
                print(f"Updated blend1 min_height: {old_min_height1} -> {args.blend1_min}")
            else:
                ET.SubElement(blend1, 'min_height').text = str(args.blend1_min)
                print(f"Added blend1 min_height: {args.blend1_min}")
            
            fade_dist1 = blend1.find('fade_dist')
            if fade_dist1 is not None:
                old_fade_dist1 = fade_dist1.text
                fade_dist1.text = str(args.blend1_fade)
                print(f"Updated blend1 fade_dist: {old_fade_dist1} -> {args.blend1_fade}")
            else:
                ET.SubElement(blend1, 'fade_dist').text = str(args.blend1_fade)
                print(f"Added blend1 fade_dist: {args.blend1_fade}")

            if len(blend_elems) > 1:
                blend2 = blend_elems[1]
            else:
                blend2 = ET.SubElement(heightmap, 'blend')
                print("Added second blend element")
            
            min_height2 = blend2.find('min_height')
            if min_height2 is not None:
                old_min_height2 = min_height2.text
                min_height2.text = str(args.blend2_min)
                print(f"Updated blend2 min_height: {old_min_height2} -> {args.blend2_min}")
            else:
                ET.SubElement(blend2, 'min_height').text = str(args.blend2_min)
                print(f"Added blend2 min_height: {args.blend2_min}")
            
            fade_dist2 = blend2.find('fade_dist')
            if fade_dist2 is not None:
                old_fade_dist2 = fade_dist2.text
                fade_dist2.text = str(args.blend2_fade)
                print(f"Updated blend2 fade_dist: {old_fade_dist2} -> {args.blend2_fade}")
            else:
                ET.SubElement(blend2, 'fade_dist').text = str(args.blend2_fade)
                print(f"Added blend2 fade_dist: {args.blend2_fade}")

            for i in range(2, len(blend_elems)):
                heightmap.remove(blend_elems[i])
                print(f"Removed extra blend element {i+1}")

            new_pos = f"{args.pos_x} {args.pos_y} {args.pos_z}"
            pos_elems = heightmap.findall('pos')
            
            if pos_elems:
                for i, pos_elem in enumerate(pos_elems):
                    if i == 0:
                        old_pos = pos_elem.text
                        pos_elem.text = new_pos
                        print(f"Updated pos: {old_pos} -> {new_pos}")
                    else:
                        heightmap.remove(pos_elem)
            else:
                ET.SubElement(heightmap, 'pos').text = new_pos
                print(f"Added pos: {new_pos}")

        tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
        print(f"SDF file updated successfully: {sdf_path}")

    except Exception as e:
        print(f"Error modifying SDF file: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()