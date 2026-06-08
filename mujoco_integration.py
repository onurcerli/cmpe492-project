import xml.etree.ElementTree as ET
from collections import defaultdict
import math

def generate_mujoco_simulation_environment(pts, cells, room_floor_cells, corridor_cells, xml_file_name="mujoco_simulation_environment.xml"):

    # generate mesh files for walkable cells
    generate_mesh_file("corridor.obj", pts, cells, corridor_cells)
    generate_mesh_file("room.obj", pts, cells, room_floor_cells)

    root = ET.Element("mujoco", model="simulation_environment")

    asset = ET.SubElement(root, "asset")

    ET.SubElement(asset, "mesh", name="corridor_mesh", file="corridor.obj")
    ET.SubElement(asset, "mesh", name="room_mesh", file="room.obj")

    worldbody = ET.SubElement(root, "worldbody")

    minxy = pts.min(axis=0)
    maxxy = pts.max(axis=0)

    center = (minxy + maxxy) / 2

    sx = (maxxy[0] - minxy[0]) / 2 + 2
    sy = (maxxy[1] - minxy[1]) / 2 + 2

    # gray background floor
    ET.SubElement(worldbody, "geom", type="plane", pos=f"{center[0]} {center[1]} -0.01", size=f"{sx} {sy} 0.1", rgba="0.8 0.8 0.8 1")

    ET.SubElement(worldbody, "geom", type="mesh", mesh="corridor_mesh", rgba="1.0 1.0 0.0 1")
    ET.SubElement(worldbody, "geom", type="mesh", mesh="room_mesh",rgba="0.0 1.0 0.0 1")

    walkable_cells = set(room_floor_cells) | set(corridor_cells)

    add_boundary_walls(worldbody, pts, cells, walkable_cells)

    tree = ET.ElementTree(root)
    ET.indent(tree)
    tree.write(xml_file_name, encoding="utf-8", xml_declaration=True)

    print(f"Saved {xml_file_name}")



def generate_mesh_file(filename, pts, cells, walkable_cells):

    thickness = 0.02
    walkable_cells = set(walkable_cells)
    n = len(pts)

    vertices = []

    # Top layer
    for x, y in pts:
        vertices.append((float(x), float(y), thickness))

    # Bottom layer
    for x, y in pts:
        vertices.append((float(x), float(y), 0.0))

    # Top and bottom faces
    faces = []

    for cell_idx in walkable_cells:

        cell = cells[cell_idx]

        if len(cell) != 4:
            print("a")
            continue

        a, b, c, d = cell

        # Top
        faces.append((a + 1, b + 1, c + 1))
        faces.append((a + 1, c + 1, d + 1))

        # Bottom
        faces.append((a + n + 1, c + n + 1, b + n + 1))
        faces.append((a + n + 1, d + n + 1, c + n + 1))

    
    # Write .obj file
    with open(filename, "w") as f:
        for x, y, z in vertices:
            f.write(f"v {x} {y} {z}\n")

        for a, b, c in faces:
            f.write(f"f {a} {b} {c}\n")

    print(f"Saved {filename}")



def add_boundary_walls(worldbody, pts, cells, walkable_cells):

    wall_height = 0.5
    wall_thickness = 0.01

    edge_count = defaultdict(int)

    # Find border edges (edges that are only used 1 wallkable cell)

    for cell_idx in walkable_cells:

        cell = cells[cell_idx]

        if len(cell) != 4:
            continue

        for i in range(4):

            v1 = cell[i]
            v2 = cell[(i + 1) % 4]

            edge = tuple(sorted((v1, v2)))

            edge_count[edge] += 1

    # Create wall for every border edge

    for (v1, v2), count in edge_count.items():

        if count != 1:
            continue

        p1 = pts[v1]
        p2 = pts[v2]

        mid = (p1 + p2) / 2

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        length = math.sqrt(dx * dx + dy * dy)

        angle = math.atan2(dy, dx)

        qw = math.cos(angle / 2)
        qz = math.sin(angle / 2)

        ET.SubElement(
            worldbody,
            "geom",
            type="box",
            pos=f"{mid[0]} {mid[1]} {wall_height/2}",
            size=f"{length/2} {wall_thickness/2} {wall_height/2}",
            quat=f"{qw} 0 0 {qz}",
            rgba="0.3 0.3 0.3 1"
        )