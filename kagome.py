from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 20              # The width and height of your graph (Must be even for Toroidal wrapping)
MAX_COL_NUMBER = GRID_SIZE - 1  # Max coloring number to attempt

# ENFORCE RESTRICTIONS
PACKING = True              # Enforces Packing Constraints: Node 'z' must be > z distance from another 'z'
DOUBLES = False              # Restricts doubling: a -- 2a
SANDWICHES = False           # Restricts Sandwiches: a -- b -- a
STAIRS = False               # Restricts staircase progressions: a -- a+b -- a+2b
IDENTICAL_NEIGHBORS = False  # Restricts a -- a (Redundant if Packing is enabled)

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 8      # Threads to use (0 will use all available cores)
MAX_MEMORY_IN_MB = 2000     # Max memory for solver
MAX_TIME_IN_MINUTES = 600
LOG_SEARCH = True
RANDOM_SEARCH_SEED = 314

def compute_distances_and_edges(N):
    """
    Computes precise graph distances on a Toroidal Kagome Lattice topology.
    Kagome is modeled by pruning a triangular lattice grid where both coordinates are even.
    """
    adj = {}
    edges = []
    
    # 6 directional offsets of a standard triangular lattice grid
    # Horizontal, Vertical, and the primary Diagonal axis
    offsets = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, -1), (-1, 1)]
    
    # 1. Build Adjacency List for the active Kagome vertices
    for r in range(N):
        for c in range(N):
            # Define structural holes (1/4 of the triangular grid sites are skipped)
            if r % 2 == 0 and c % 2 == 0:
                continue 
                
            neighbors = []
            for dy, dx in offsets:
                nr = (r + dy) % N
                nc = (c + dx) % N
                
                # Only connect if the neighbor target isn't a lattice hole
                if not (nr % 2 == 0 and nc % 2 == 0):
                    neighbors.append((nr, nc))
            
            adj[(r, c)] = neighbors
            
            for nr, nc in neighbors:
                u, v = (r, c), (nr, nc)
                if u < v:
                    edges.append((u, v))

    # 2. Compute Exact Shortest Paths using BFS
    distances = {}
    for start_node in adj.keys():
        distances[start_node] = {start_node: 0}
        queue = deque([start_node])
        
        while queue:
            curr = queue.popleft()
            curr_dist = distances[start_node][curr]
            
            if curr_dist >= MAX_COL_NUMBER:
                continue
                
            for nxt in adj[curr]:
                if nxt not in distances[start_node]:
                    distances[start_node][nxt] = curr_dist + 1
                    queue.append(nxt)
                        
    return adj, edges, distances

def build_model(N):
    model = cp_model.CpModel()
    
    print(f"[{time.strftime('%H:%M:%S')}] Generating model: {N}x{N} Toroidal Kagome Lattice.")
    start_time = time.time()
    
    adj, edges, distances = compute_distances_and_edges(N)
    
    # 1. CREATE VARIABLES (Only for active grid positions)
    print("  > Creating boolean and integer variables...")
    grid = {}
    b_is_z = {}
    
    for r, c in adj.keys():
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
                for center in adj.keys():
                    clique = []
                    for node, dist in distances[center].items():
                        if dist <= k:
                            clique.append(b_is_z[node[0], node[1], z])
                    if len(clique) > 1:
                        model.AddAtMostOne(clique)
            else:
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
            if DOUBLES:
                model.Add(current != 2 * neighbor)
            if IDENTICAL_NEIGHBORS and not PACKING:
                model.Add(current != neighbor)

        if STAIRS:
            for i in range(len(valid_neighbors)):
                for j in range(i + 1, len(valid_neighbors)):
                    model.Add(2 * current != valid_neighbors[i] + valid_neighbors[j])
                    
        if SANDWICHES and len(valid_neighbors) > 1:
            model.AddAllDifferent(valid_neighbors)

    # 4. SYMMETRIC BREAKING (Always applied since it is permanently Toroidal)
    # Find a stable active coordinate near the spatial center of the grid matrix
    center_r, center_c = N // 2, N // 2
    if (center_r, center_c) not in grid:
        center_c += 1  # Shift right if the true center falls directly onto a lattice hole
    model.Add(grid[center_r, center_c] == MAX_COL_NUMBER)

    load_time = time.time() - start_time
    print(f"  > Constraints generated in {load_time:.2f} seconds.")
    print(f"  > Total raw CP-SAT model constraints natively generated: {len(model.Proto().constraints)}")
    
    return model, grid

def solve():
    if GRID_SIZE % 2 != 0:
        print("ERROR: Toroidal Kagome layout requires an even GRID_SIZE to tile seamlessly.")
        return

    model, grid = build_model(GRID_SIZE)

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
        print(f"\nSUCCESS: Solution Found for {GRID_SIZE}x{GRID_SIZE} Kagome Lattice in {elapsed:.2f}s!")
        
        for r in range(GRID_SIZE):
            # 1. Print the primary vertex row text array
            row_chars = [" "] * (4 * GRID_SIZE)
            for c in range(GRID_SIZE):
                if (r, c) in grid:
                    val = solver.Value(grid[r, c])
                    val_str = f"{val:02d}"
                    row_chars[4*c] = val_str[0]
                    row_chars[4*c+1] = val_str[1]
                    
                    # Check horizontal connectivity to the right
                    if c < GRID_SIZE - 1 and (r, c+1) in grid:
                        row_chars[4*c+2] = '-'
                        row_chars[4*c+3] = '-'
            print("".join(row_chars).rstrip())
            
            # 2. Print the intermediate diagonal and vertical connection layer
            if r < GRID_SIZE - 1:
                inter_chars = [" "] * (4 * GRID_SIZE)
                for c in range(GRID_SIZE):
                    # Vertical link (|)
                    if (r, c) in grid and (r+1, c) in grid:
                        inter_chars[4*c] = '|'
                    # Diagonal link (/) from top-right down to bottom-left
                    if c > 0 and (r, c) in grid and (r+1, c-1) in grid:
                        inter_chars[4*c-2] = '/'
                print("".join(inter_chars).rstrip())
    else:
        print(f"\nFAILURE: No solution found within constraints (Time: {elapsed:.2f}s).")

if __name__ == "__main__":
    solve()
