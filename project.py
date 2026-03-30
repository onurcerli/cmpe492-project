import random
import numpy as np
import math
import matplotlib.pyplot as plt
from collections import defaultdict


# --- Configuration ---
EDGE_LENGTH = 16
EDGE_DIV = 8
NUM_ROOMS = 6
RELAX_ITERATIONS = 20
RELAX_LAMBDA = 0.45


def build_triangular_grid(R, n):
    """
        R: hexagon edge length
        n: number of triangles per hexagon edge
    """
    
    # Base triangle (sector of hexagon)
    A = np.array([0.0, 0.0])
    B = np.array([R, 0.0])
    C = np.array([R/2, math.sqrt(3)*R/2])

    # Generate subdivided points inside this triangle
    points = []
    for i in range(n + 1):
        for j in range(n + 1 - i):
            P = A + (B - A) * (i / n) + (C - A) * (j / n)
            points.append(P)
    points = np.array(points)

    def idx(i, j):
        return (i * (n + 1) - i*(i - 1)//2 + j)
    
    # Generate smaller triangles for the base sector
    triangles = []
    for i in range(n):
        for j in range(n - i):
            p1, p2, p3 = idx(i, j), idx(i + 1, j), idx(i, j + 1)
            triangles.append([p1, p2, p3])
            if j + i < n - 1:
                p4 = idx(i + 1, j + 1)
                triangles.append([p2, p4, p3])

    # Rotate the base sector and get smaller triangles for all six sectors of hexagon
    all_points = []
    all_tris = []
    angle_step = math.pi / 3
    for k in range(6):
        angle = k * angle_step
        rot = np.array([[math.cos(angle), -math.sin(angle)],
                        [math.sin(angle),  math.cos(angle)]])
        rotated = points @ rot.T
        offset = len(all_points)
        all_points.extend(rotated)
        all_tris.extend([[a + offset, b + offset, c + offset] for a, b, c in triangles])
    
    # Remove duplicate vertices
    pts = np.round(np.array(all_points), 8)
    uniq_pts, inv = np.unique(pts, axis=0, return_inverse=True)
    all_tris = np.array(inv)[np.array(all_tris)]

    return uniq_pts, all_tris



def pair_triangles_to_quads(tris):
    """ Pair triangles randomly to create quads """
    edge_map = defaultdict(list)
    for ti, t in enumerate(tris):
        for e in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            edge_map[tuple(sorted(e))].append(ti)
    
    paired, quads, leftover = set(), [], []
    tri_neighbors = {i: set() for i in range(len(tris))}
    for edge, tri_list in edge_map.items():
        if len(tri_list) == 2:
            t1, t2 = tri_list
            tri_neighbors[t1].add((t2, edge))
            tri_neighbors[t2].add((t1, edge))
    
    indices = list(range(len(tris)))
    random.shuffle(indices)
    
    for ti in indices:
        if ti in paired: continue

        neigh = [n for n in tri_neighbors[ti] if n[0] not in paired]

        if neigh:
            tj, common_edge = random.choice(neigh)

            a = [v for v in tris[ti] if v not in common_edge]
            b = [v for v in tris[tj] if v not in common_edge]
            
            c, d = common_edge
            
            quads.append([a[0], c, b[0], d]) 
            paired.add(ti)
            paired.add(tj)
        else:
            leftover.append(tris[ti])

    return quads, leftover



def subdivide_all_to_quads(pts, cells):
    """ Subdivides all cells (triangles and quads) into smaller, uniform quads """
    current_pts = list(pts)
    new_quads = []
    
    for cell in cells:
        N = len(cell)
        cell_points = pts[cell]
        centroid_pos = np.mean(cell_points, axis=0)
        
        centroid_idx = len(current_pts)
        current_pts.append(centroid_pos)
        
        edge_midpoints = []
        for i in range(N):
            v_a, v_b = cell[i], cell[(i + 1) % N]
            midpoint_pos = (pts[v_a] + pts[v_b]) / 2.0
            
            mid_idx = len(current_pts)
            current_pts.append(midpoint_pos)
            edge_midpoints.append(mid_idx)

        for i in range(N):
            v_i = cell[i]
            e_prev = edge_midpoints[(i - 1 + N) % N]
            e_curr = edge_midpoints[i]
            # New quad: [Original Corner, Current Midpoint, Center, Previous Midpoint]
            new_quads.append([v_i, e_curr, centroid_idx, e_prev])
            
    return np.array(current_pts), new_quads



def weld_vertices(pts, cells, tol=1e-6):
    """Merge vertices that are within tolerance 'tol' of each other"""

    pts = np.asarray(pts)
    grid, new_pts, old_to_new = {}, [], {}
    
    # remove duplicate points from the array
    for i, p in enumerate(pts):
        vertex = (round(float(p[0]) / tol) * tol, round(float(p[1]) / tol) * tol)

        if vertex in grid:
            old_to_new[i] = grid[vertex]
        else:
            ni = len(new_pts)
            grid[vertex] = ni
            old_to_new[i] = ni
            new_pts.append(p.tolist())
    new_pts = np.array(new_pts)
    new_cells = []
    
    # change point indexes in each cell accordingly
    for c in cells:
        mapped = [old_to_new[v] for v in c]

        # Remove consecutive duplicate vertices
        out = [v for i, v in enumerate(mapped) if i == 0 or v != mapped[i-1]]
        if len(out) > 1 and out[0] == out[-1]: 
            out = out[:-1]
        
        # Take the cell if it is valid(cell with 3 or 4 edges) after welding
        if len(out) >= 3:
            new_cells.append(out)

    return new_pts, new_cells



def apply_relaxation(pts, cells, iterations, lam):
    """Smoothen vertex positions for a more organic grid"""

    pts = pts.copy()
    neighbors = defaultdict(set)
    for c in cells:
        L = len(c)
        for i in range(L):
            a, b = c[i], c[(i + 1) % L]
            neighbors[a].add(b); neighbors[b].add(a)
    

    for _ in range(iterations):
        new_pts = pts.copy()
        for i in range(len(pts)):
            nb = list(neighbors[i])

            if not nb:
                continue

            avg = np.mean(pts[nb], axis=0)

            # Relaxation step: new = old * (1 - lambda) + average * lambda
            new_pts[i] = pts[i] * (1 - lam) + avg * lam
        pts = new_pts

    return pts



def build_vertex_to_cell_map(num_pts, cells):
    """ Maps each vertex index to the list of cell indices that share it """
    
    vertex_to_cells = defaultdict(list)
    for cell_idx, cell_verts in enumerate(cells):
        for vertex_idx in cell_verts:
            if 0 <= vertex_idx < num_pts:
                vertex_to_cells[vertex_idx].append(cell_idx)
    return vertex_to_cells



def build_cell_to_cell_neighbor_map(cells):
    """ Maps each cell index to a set of neighboring cell indices """

    edge_to_cells = defaultdict(list)
    for cell_idx, cell_verts in enumerate(cells):
        N = len(cell_verts)
        for i in range(N):
            v1 = cell_verts[i]
            v2 = cell_verts[(i + 1) % N]
            edge = tuple(sorted((v1, v2)))
            edge_to_cells[edge].append(cell_idx)

    cell_neighbors = defaultdict(set)
    for edge, cell_list in edge_to_cells.items():
        if len(cell_list) == 2:
            c1, c2 = cell_list
            cell_neighbors[c1].add(c2)
            cell_neighbors[c2].add(c1)
    return cell_neighbors



def calculate_cell_centroids(pts, cells):
    """ Calculates the 2D centroid for every cell """
    centroids = []
    for cell_verts in cells:
        cell_points = pts[cell_verts]
        centroid_pos = np.mean(cell_points, axis=0)
        centroids.append(centroid_pos)
    return np.array(centroids)



def create_rooms(num_rooms, vertex_to_cells, cell_neighbors):
    """
    Generates non-overlapping rooms with a 1-cell buffer zone around each room to prevent merging of multiple rooms.
    
    note: the number of rooms created may not be equal to number of rooms requested if it can't fit them in the grid.
    """

    protected_cells = set() 
    available_center_vertices = list(vertex_to_cells.keys())
    random.shuffle(available_center_vertices)
    
    generated_rooms = {}
    room_id_counter = 0

    while room_id_counter < num_rooms and available_center_vertices:
        
        center_vertex_idx = available_center_vertices.pop()
        
        room_cells = set(vertex_to_cells[center_vertex_idx])
        
        # get the buffer cells (neighbors of the room cells)
        buffer_cells = set()
        for cell_idx in room_cells:
            buffer_cells.update(cell_neighbors.get(cell_idx, set()))
            
        buffer_cells.difference_update(room_cells) # buffer cannot include room itself
        
        required_free_cells = room_cells.union(buffer_cells)
        
        # check if room and buffer cells are already occupied
        if required_free_cells.isdisjoint(protected_cells):
            
            # New, valid room
            generated_rooms[room_id_counter] = {
                'center_vertex': center_vertex_idx,
                'cell_indices': room_cells
            }
            
            # mark the required cells as protected for next iteration:
            protected_cells.update(required_free_cells) 
            room_id_counter += 1

    print(f"Generated {len(generated_rooms)} rooms out of a requested {num_rooms}.")
    return generated_rooms, protected_cells



def _AStar(start_cell_idx, goal_cell_idx, cell_neighbors, cell_centroids):
    """ A* search for the shortest path of adjacent quad cells using centroid distance """
    
    def heuristic(a_cell_idx, b_cell_idx):
        pos_a = cell_centroids[a_cell_idx]
        pos_b = cell_centroids[b_cell_idx]
        return np.linalg.norm(pos_a - pos_b)

    def reconstructPath(n):
        """path = []
        while n in cameFrom:
            path.append(n)
            n = cameFrom[n]
        path.append(start_cell_idx)
        return path[::-1]
        """
        if n == start_cell_idx:
            return [n]
        return reconstructPath(cameFrom[n]) + [n]
    
    closed = set()
    open_set = {start_cell_idx}
    cameFrom = {}
    gScore = {start_cell_idx: 0}
    fScore = {start_cell_idx: heuristic(start_cell_idx, goal_cell_idx)}

    while open_set:
        current = None
        for i in open_set:
            if current is None or fScore[i] < fScore[current]:
                current = i

        if current == goal_cell_idx:
            return reconstructPath(goal_cell_idx)

        open_set.remove(current)
        closed.add(current)

        for neighbor in cell_neighbors.get(current, set()):
            if neighbor in closed:
                continue

            g = gScore[current] + 1 

            if neighbor not in open_set or g < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = g
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal_cell_idx)
                if neighbor not in open_set:
                    open_set.add(neighbor)
                    
    return []



def connect_rooms_and_get_corridors(rooms, cell_neighbors, cell_centroids, room_floor_cells):
    """
    Connects all defined rooms using A* paths and MST-like selection.
    Corridor cells are allowed to occupy the buffer zone (protected_cells) to touch the rooms.
    """
    
    room_ids = list(rooms.keys())
    num_rooms = len(room_ids)
    all_room_connections = [] 

    # Calculate the shortest A* path between all pairs of rooms
    for i in range(num_rooms):
        for j in range(i + 1, num_rooms):
            room_A_id, room_B_id = room_ids[i], room_ids[j]
            best_path, shortest_length = None, float('inf')

            # Find path between the closest cells of Room A and Room B
            for start_cell_idx in rooms[room_A_id]['cell_indices']:
                for goal_cell_idx in rooms[room_B_id]['cell_indices']:
                    
                    path = _AStar(start_cell_idx, goal_cell_idx, cell_neighbors, cell_centroids)
                    
                    if path and len(path) < shortest_length:
                        shortest_length = len(path)
                        # Remove the start and end cells from path(which are already room floors)
                        best_path = set(path[1:-1])

            if best_path is not None:
                all_room_connections.append({
                    'rooms': tuple(sorted((room_A_id, room_B_id))),
                    'path_length': shortest_length,
                    'path_cells': best_path
                })

    # Select the connections
    all_room_connections.sort(key=lambda c: c['path_length'])
    
    parent = {i: i for i in room_ids}
    def find(i):
        if parent[i] == i: return i
        parent[i] = find(parent[i])
        return parent[i]
    
    def union(i, j):
        root_i, root_j = find(i), find(j)
        if root_i != root_j:
            parent[root_i] = root_j
            return True
        return False

    final_corridor_cells = set()
    
    for conn in all_room_connections:
        room_a, room_b = conn['rooms']
        
        if union(room_a, room_b):
            final_corridor_cells.update(conn['path_cells'])
            
            # Check if all rooms are connected
            if len({find(i) for i in room_ids}) == 1: break

    # remove cells that are part of the room floors from corridor cells
    final_corridor_cells = final_corridor_cells.difference(room_floor_cells)
    
    return final_corridor_cells



def visualize_dungeon_grid(pts, cells, room_cells, corridor_cells):
    """
    Create a visualization of the grid and cells with rooms and corridors with different colors
    """
    plt.figure(figsize=(12, 10)) 
    
    all_cell_indices = set(range(len(cells)))

    unused_cells = all_cell_indices.difference(room_cells).difference(corridor_cells)

    cell_statuses = {
        'Room': {'indices': room_cells, 'color': '#6aa84f'},     # Green for room
        'Corridor': {'indices': corridor_cells, 'color': '#f1c232'}, # Yellow for corridor
        'Unused': {'indices': unused_cells, 'color': '#8e7cc3'}       # Violet for other cells
    }
    
    # 2D Plotting: Draw and color all cells
    for status, data in cell_statuses.items():
        color = data['color']
        
        for cell_idx in data['indices']:
            cell = cells[cell_idx]
            poly_pts = pts[cell]
            poly_closed = np.vstack([poly_pts, poly_pts[0]])
            
            plt.fill(
                poly_closed[:, 0], 
                poly_closed[:, 1], 
                facecolor=color, 
                alpha=0.7, 
                edgecolor='black', 
                linewidth=0.5
            )

    # Draw vertices on top
    plt.scatter(pts[:, 0], pts[:, 1], color='red', s=5, zorder=3)
    
    plt.title("Grid Structure")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend(handles=[
        plt.Rectangle((0, 0), 1, 1, fc='#6aa84f', alpha=0.7, label='Room'),
        plt.Rectangle((0, 0), 1, 1, fc='#f1c232', alpha=0.7, label='Corridor'),
        plt.Rectangle((0, 0), 1, 1, fc='#8e7cc3', alpha=0.7, label='Empty')
    ], loc='upper right')
    
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show() 



if __name__ == "__main__":
    # Grid generation, subdivision and welding
    pts, tris = build_triangular_grid(EDGE_LENGTH, EDGE_DIV) 
    quads, leftover_triangles = pair_triangles_to_quads(tris)
    pts_s, cells_s = subdivide_all_to_quads(pts, quads + leftover_triangles)
    pts_w, cells_w = weld_vertices(pts_s, cells_s)
    
    # Apply relaxation for more natural grid structure
    pts_relaxed = apply_relaxation(pts_w, cells_w, RELAX_ITERATIONS, RELAX_LAMBDA) 
    cells_final = cells_w 
    
    num_pts = len(pts_relaxed)
    vertex_to_cells = build_vertex_to_cell_map(num_pts, cells_final)
    cell_neighbors = build_cell_to_cell_neighbor_map(cells_final)
    cell_centroids = calculate_cell_centroids(pts_relaxed, cells_final)
    
    # Room generation
    rooms_dict, protected_cells = create_rooms(NUM_ROOMS, vertex_to_cells, cell_neighbors)
    
    room_floor_cells = set()
    for room_data in rooms_dict.values():
        room_floor_cells.update(room_data['cell_indices'])
    
    # Corridor generation and path finding with A*
    corridor_cells = connect_rooms_and_get_corridors(rooms_dict, cell_neighbors, cell_centroids, room_floor_cells)
    
    # Visualization of the grid structure with numpy
    visualize_dungeon_grid(pts_relaxed, cells_final, room_floor_cells, corridor_cells)