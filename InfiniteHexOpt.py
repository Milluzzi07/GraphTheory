from ortools.sat.python import cp_model
import time
from collections import deque
import itertools

# --- CONFIGURATION ---
GRID_SIZE = 16 # The width and height of your graph (must be even for Toroidal Hex graphs)
MAX_COL_NUMBER = 40 # Max coloring number to attempt
TOROIDAL = False # Select Infinite (Toroidal) or Finite Hex Grid

# ENFORCE ____ RESTRICTIONS
PACKING = True # Enforces Packing Constraints: Node 5 must be >5 distance from another 5
DOUBLES = True # Restricts doubling: a -- 2a
SANDWICHES = True # Restricts Sandwiches: a -- b -- a
STAIRS = True # Restricts staircases: a -- a+b -- a+2b
IDENTICAL_NEIGHBORS = False # Restricts a -- a (Redundant if Packing is enabled)

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 12 # Threads to use (0 will use all)
MAX_MEMORY_IN_MB = 16000 # Max memory for solver
MAX_TIME_IN_MINUTES = 600
LOG_SEARCH = True
RANDOM_SEARCH_SEED = 314

def compute_distances_and_edges(N, toroidal):
    """
    Computes precise graph distances following only valid connections,
    ignoring Euclidean matrix geometry to prevent coordinate wrap bugs.
    """
    # 1. Build Adjacency List for the 3-neighbor honeycomb lattice
    adj = {}
    edges = []
    
    for r in range(N):
        for c in range(N):
            # Alternating vertical connections to form a hexagonal/honeycomb topology
            if (r + c) % 2 == 0:
                offsets = [(-1, 0), (0, -1), (0, 1)] # Up, Left, Right
            else:
                offsets = [(1, 0), (0, -1), (0, 1)]  # Down, Left, Right
            
            neighbors = []
            for dy, dx in offsets:
                nr, nc = r + dy, c + dx
                if toroidal:
                    nr, nc = nr % N, nc % N
                    neighbors.append((nr, nc))
                else:
                    if 0 <= nr < N and 0 <= nc < N:
                        neighbors.append((nr, nc))
            
            adj[(r, c)] = neighbors
            
            for nr, nc in neighbors:
                # Add edge uniformly ordering pairs to avoid duplicates
                u, v = (r, c), (nr, nc)
                if u < v:
                    edges.append((u, v))

    # 2. Compute Exact Shortest Paths using BFS
    # distances[start_node][target_node] = integer distance along valid path
    distances = {}
    for r in range(N):
        for c in range(N):
            start_node = (r, c)
            distances[start_node] = {start_node: 0}
            queue = deque([start_node])
            
            while queue:
                curr = queue.popleft()
                curr_dist = distances[start_node][curr]
                
                # Stop expanding if distance exceeds MAX needed
                if curr_dist >= MAX_COL_NUMBER:
                    continue
                    
                for nxt in adj[curr]:
                    if nxt not in distances[start_node]:
                        distances[start_node][nxt] = curr_dist + 1
                        queue.append(nxt)
                        
    return adj, edges, distances

def build_model(N, toroidal):
    model = cp_model.CpModel()
    
    print(f"[{time.strftime('%H:%M:%S')}] Generating model: {N}x{N} {'Toroidal' if toroidal else 'Finite'} Hex Grid.")
    start_time = time.time()
    
    adj, edges, distances = compute_distances_and_edges(N, toroidal)
    
    # 1. CREATE VARIABLES
    print("  > Creating boolean and integer variables...")
    grid = {}
    b_is_z = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')
            z_vars = []
            for z in range(1, MAX_COL_NUMBER + 1):
                b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                b_is_z[r, c, z] = b
                z_vars.append(b)
                model.Add(grid[r, c] == z).OnlyEnforceIf(b)
            model.AddExactlyOne(z_vars)

    # 2. PACKING CONSTRAINTS
    if PACKING:
        print("  > Pre-computing BFS-based Maximal Cliques for Packing Constraints...")
        for z in range(1, MAX_COL_NUMBER + 1):
            if z % 10 == 0 or z == MAX_COL_NUMBER:
                print(f"    ... processing color {z}/{MAX_COL_NUMBER}")
            
            k = z // 2
            
            if z % 2 == 0:
                # Even z (e.g., 4): Ball of radius 2 covers distance <= 4 paths perfectly from every center.
                for r in range(N):
                    for c in range(N):
                        center = (r, c)
                        clique = []
                        for node, dist in distances[center].items():
                            if dist <= k:
                                clique.append(b_is_z[node[0], node[1], z])
                        if len(clique) > 1:
                            model.AddAtMostOne(clique)
            else:
                # Odd z (e.g., 5): Union of balls of radius 2 from adjacent centers covers distance <= 5 perfectly.
                # E.g distance exactly 5 must jump through an edge (m1, m2), ensuring endpoints are trapped in union.
                for u, v in edges:
                    clique_nodes = set()
                    
                    for node, dist in distances[u].items():
                        if dist <= k:
                            clique_nodes.add(node)
                    for node, dist in distances[v].items():
                        if dist <= k:
                            clique_nodes.add(node)
                    
                    clique = [b_is_z[nr, nc, z] for nr, nc in clique_nodes]
                    if len(clique) > 1:
                        model.AddAtMostOne(clique)

    # 3. VERTEX ADJACENCY RULES
    print("  > Generating Local Adjacency Rules...")
    for u, u_neighbors in adj.items():
        current = grid[u]
        valid_neighbors = [grid[n] for n in u_neighbors]

        for neighbor in valid_neighbors:
            # Rule 1: No Doubling
            if DOUBLES:
                model.Add(current != 2 * neighbor)
            
            # Rule 2: Basic Inequality
            if IDENTICAL_NEIGHBORS and not PACKING:
                model.Add(current != neighbor)

        # Rule 3: Arithmetic Progression Prevention
        if STAIRS:
            for i in range(len(valid_neighbors)):
                for j in range(i + 1, len(valid_neighbors)):
                    model.Add(2 * current != valid_neighbors[i] + valid_neighbors[j])
                    
        # Rule 4: Sandwiches Prevention
        if SANDWICHES and len(valid_neighbors) > 1:
            model.AddAllDifferent(valid_neighbors)

    # Performance forcing: Symmetry breaking
    # If the grid is Toroidal, it is vertex-transitive, meaning any valid solution can be 
    # freely translated. We force the max color to the center to instantly break symmetry.
    # Note: We do NOT do this for Finite grids as they have boundaries.
    if toroidal:
        model.Add(grid[N//2, N//2] == MAX_COL_NUMBER)

    load_time = time.time() - start_time
    print(f"  > Constraints generated in {load_time:.2f} seconds.")
    print(f"  > Total raw CP-SAT model constraints natively generated: {len(model.Proto().constraints)}")
    
    return model, grid

def solve():
    if TOROIDAL and GRID_SIZE % 2 != 0:
        print("ERROR: Toroidal Hex layout requires an even GRID_SIZE.")
        return

    model, grid = build_model(GRID_SIZE, TOROIDAL)

    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = NUM_SEARCH_WORKERS
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solver.parameters.max_time_in_seconds = MAX_TIME_IN_MINUTES * 60.0
    solver.parameters.random_seed = RANDOM_SEARCH_SEED
    solver.parameters.log_search_progress = LOG_SEARCH

    print("\nSolving...")
    start_time = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - start_time

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"\nSUCCESS: Solution Found for {GRID_SIZE}x{GRID_SIZE} Hex Grid in {elapsed:.2f}s!")
        for r in range(GRID_SIZE):
            row_str = ""
            for c in range(GRID_SIZE):
                val = solver.Value(grid[r, c])
                row_str += f"{val:02d}"
                if c < GRID_SIZE - 1:
                    row_str += "---"
            print(row_str)
            
            if r < GRID_SIZE - 1:
                vert_str = ""
                for c in range(GRID_SIZE):
                    # In our coordinate grid mapping, down exists if (r+c) is odd
                    if (r + c) % 2 != 0:
                        vert_str += "|    "
                    else:
                        vert_str += "     "
                print(vert_str)
    else:
        print(f"\nFAILURE: No solution found within constraints (Time: {elapsed:.2f}s).")

if __name__ == "__main__":
    solve()
