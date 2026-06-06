import xml.etree.ElementTree as ET


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