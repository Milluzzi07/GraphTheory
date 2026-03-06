from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 6  # Can now be odd or even, as parity no longer needs to match across a wrap
MAX_COL_NUMBER = 30 
N = GRID_SIZE

def solve_finite_optimized():
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading Finite constraints for {N}x{N}...")
    start_time = time.time()

    # --- PRE-COMPUTATION (Graph Topology & Distances) ---
    print("Pre-computing finite 3-regular graph adjacency and distances...", end=" ", flush=True)
    
    # 1. Build Adjacency List for the finite lattice
    adj = {}
    for r in range(N):
        for c in range(N):
            # Alternating vertical connections
            if (r + c) % 2 == 0:
                offsets = [(-1, 0), (0, -1), (0, 1)] # Up, Left, Right
            else:
                offsets = [(1, 0), (0, -1), (0, 1)]  # Down, Left, Right
            
            neighbors = []
            for dy, dx in offsets:
                nr, nc = r + dy, c + dx
                # Strict boundary check (no toroidal modulo wrap)
                if 0 <= nr < N and 0 <= nc < N:
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
                
                if curr_dist >= MAX_COL_NUMBER:
                    continue
                    
                for nxt in adj[curr]:
                    if nxt not in distances[start_node]:
                        distances[start_node][nxt] = curr_dist + 1
                        queue.append(nxt)
    print("Done.")

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Optimized)
    # ---------------------------------------------------------
    for z in range(1, MAX_COL_NUMBER + 1):
        if z % 5 == 0:
            print(f"  - Generating constraints for Color {z}/{MAX_COL_NUMBER}...")

        for r in range(N):
            for c in range(N):
                u = (r, c)
                u_index = r * N + c
                
                b_is_z = model.NewBoolVar(f'is_{z}_{r}_{c}')
                model.Add(grid[u] == z).OnlyEnforceIf(b_is_z)
                model.Add(grid[u] != z).OnlyEnforceIf(b_is_z.Not())

                for v, dist in distances[u].items():
                    if 0 < dist <= z:
                        v_index = v[0] * N + v[1]
                        
                        # SYMMETRY OPTIMIZATION
                        if v_index > u_index:
                            model.Add(grid[v] != z).OnlyEnforceIf(b_is_z)

    # ---------------------------------------------------------
    # B. VERTEX ADJACENCY RULES (Finite)
    # ---------------------------------------------------------
    print("  - Generating Adjacency Rules...")

    for r in range(N):
        for c in range(N):
            u = (r, c)
            current = grid[u]
            
            neighbors = [grid[nxt] for nxt in adj[u]]

            for neighbor in neighbors:
                # Rule 1: No Doubling
                model.Add(neighbor != 2 * current)
                
                # Rule 2: Basic Inequality
                model.Add(current != neighbor)

            # Rule 3: Arithmetic Progression Prevention
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
    solver.parameters.max_memory_in_mb = 16000
    solver.parameters.max_time_in_seconds = 150
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
    solve_finite_optimized()