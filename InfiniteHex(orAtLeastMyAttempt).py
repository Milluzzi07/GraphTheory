from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
# GRID_SIZE must be even to ensure a valid toroidal wrap for a 3-regular bipartite graph.
GRID_SIZE = 36  
MAX_COL_NUMBER = GRID_SIZE - 1 
N = GRID_SIZE

def solve_infinite_optimized():
    model = cp_model.CpModel()
    
    # 1. CREATE GRIDVARIABLES
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading Toroidal constraints for {N}x{N}...")
    start_time = time.time()

    print("Pre-computing 3-regular graph adjacency and distances...", end=" ", flush=True)
    
    # 1. Build Adjacency List for the 3-neighbor lattice
    adj = {}
    for r in range(N):
        for c in range(N):
            # Alternating vertical connections to form a hexagonal/honeycomb topology
            if (r + c) % 2 == 0:
                offsets = [(-1, 0), (0, -1), (0, 1)] # Up, Left, Right
            else:
                offsets = [(1, 0), (0, -1), (0, 1)]  # Down, Left, Right
            
            neighbors = []
            for dy, dx in offsets:
                nr, nc = (r + dy) % N, (c + dx) % N
                neighbors.append((nr, nc))
            adj[(r, c)] = neighbors

    # 2. Compute Exact Shortest Paths using BFS
    # distances[(r, c)][(nr, nc)] = integer distance
    distances = {}
    for r in range(N):
        for c in range(N):
            start_node = (r, c)
            distances[start_node] = {start_node: 0}
            queue = deque([start_node])
            
            while queue:
                curr = queue.popleft()
                curr_dist = distances[start_node][curr]
                
                # Stop expanding if the distance exceeds the maximum needed color value
                if curr_dist >= MAX_COL_NUMBER:
                    continue
                    
                for nxt in adj[curr]:
                    if nxt not in distances[start_node]:
                        distances[start_node][nxt] = curr_dist + 1
                        queue.append(nxt)
    print("Done.")

  # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Memory Optimized)
    # ---------------------------------------------------------
    print("  - Generating memory-optimized packing constraints...")

    for r in range(N):
        for c in range(N):
            u = (r, c)
            u_index = r * N + c
            
            # Iterate through all reachable nodes from 'u'
            for v, dist in distances[u].items():
                v_index = v[0] * N + v[1]
                
                # SYMMETRY OPTIMIZATION: Only process each pair once
                if v_index > u_index and 0 < dist <= MAX_COL_NUMBER:
                    
                    # If distance is 'd', they cannot share ANY color 'z' >= 'd'
                    # e.g., if distance is 3, they cannot both be 3, 4, 5...
                    forbidden_pairs = [(z, z) for z in range(dist, MAX_COL_NUMBER + 1)]
                    
                    if forbidden_pairs:
                        model.AddForbiddenAssignments([grid[u], grid[v]], forbidden_pairs)

    # ---------------------------------------------------------
    # B. VERTEX ADJACENCY RULES (Toroidal)
    # ---------------------------------------------------------
    print("  - Generating Adjacency Rules...")

    for r in range(N):
        for c in range(N):
            u = (r, c)
            current = grid[u]
            
            # Pull neighbors directly from pre-computed adjacency list
            neighbors = [grid[nxt] for nxt in adj[u]]

            for neighbor in neighbors:
                # Rule 1: No Doubling
                model.Add(neighbor != 2 * current)
                
                # Rule 2: Basic Inequality
                model.Add(current != neighbor)

            # Rule 3: Arithmetic Progression Prevention
            # "2 * Current != Neighbor1 + Neighbor2"
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    n1 = neighbors[i]
                    n2 = neighbors[j]
                    model.Add(2 * current != n1 + n2)

    load_time = time.time() - start_time
    print(f"Constraints loaded in {load_time:.2f} seconds.")

    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 12
    solver.parameters.max_memory_in_mb = 24000
    solver.parameters.max_time_in_seconds = 28800.0
    solver.parameters.log_search_progress = True

    print("Solving...")
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"\nSUCCESS: Solution Found for {N}x{N} Finite Grid!\n")
        
        for r in range(N):
            # 1. Print the nodes and horizontal edges
            row_str = ""
            for c in range(N):
                val = solver.Value(grid[r, c])
                row_str += f"{val:02d}"
                if c < N - 1:
                    row_str += "---"
            print(row_str)
            
            # 2. Print the alternating vertical edges (Brick Wall layout)
            if r < N - 1:
                vert_str = ""
                for c in range(N):
                    # Based on the script's adjacency, a downward edge exists if (r+c) is odd
                    if (r + c) % 2 != 0:
                        vert_str += "|    "
                    else:
                        vert_str += "     "
                print(vert_str)
    else:
        print("No solution found.")

if __name__ == "__main__":
    if GRID_SIZE%2!=0:
        print("Improper Grid Size.")
    else:
        print("Starting Rule Generation")
        solve_infinite_optimized()