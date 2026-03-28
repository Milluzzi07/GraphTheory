from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 24         # Must be a multiple of BRICK_LENGTH
MAX_COL_NUMBER = 22    # The ceiling for our search # 22 works for 24x24 grid, brick length 6

BRICK_LENGTH = 6  # (ideally even) length of bricks

# ENFORCE RESTRICTIONS
PACKING = True
DOUBLES = True
SANDWICHES = True
STAIRS = True
IDENTICAL_NEIGHBORS = True 

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 12
MAX_MEMORY_IN_MB = 4000
MAX_TIME_IN_MINUTES = 60
LOG_SEARCH = True
RANDOM_SEARCH_SEED = 42

def compute_distances_and_edges(N):
    adj = {(r, c): [] for r in range(N) for c in range(N)}
    edges = []
    
    for r in range(N):
        # Offset for vertical joints on the bottom edge of row r
        row_start_offset = (r % 2) * (BRICK_LENGTH // 2)
        
        for c in range(N):
            # 1. HORIZONTAL: Connect to the right with wrapping
            # (c + 1) % N handles the wrap from right-edge to left-edge
            neighbor_h = (r, (c + 1) % N)
            if neighbor_h not in adj[(r, c)]:
                adj[(r, c)].append(neighbor_h)
            if (r, c) not in adj[neighbor_h]:
                adj[neighbor_h].append((r, c))

            # 2. VERTICAL: Connect down with wrapping
            # Check if the current column is a joint for this row
            if (c - row_start_offset) % BRICK_LENGTH == 0:
                # (r + 1) % N handles the wrap from bottom-edge to top-edge
                neighbor_v = ((r + 1) % N, c)
                if neighbor_v not in adj[(r, c)]:
                    adj[(r, c)].append(neighbor_v)
                if (r, c) not in adj[neighbor_v]:
                    adj[neighbor_v].append((r, c))
                    
    # Convert adj to a list of unique edges for the "Odd Color" packing rule
    seen_edges = set()
    for u, neighbors in adj.items():
        for v in neighbors:
            edge = tuple(sorted((u, v)))
            if edge not in seen_edges:
                edges.append(edge)
                seen_edges.add(edge)

    # 3. Compute Exact Shortest Paths using BFS
    # Optimization: We only need distances up to MAX_COL_NUMBER // 2 + 1
    dist_cap = (MAX_COL_NUMBER // 2) + 1
    distances = {}
    for r in range(N):
        for c in range(N):
            start_node = (r, c)
            distances[start_node] = {start_node: 0}
            queue = deque([start_node])
            while queue:
                curr = queue.popleft()
                curr_dist = distances[start_node][curr]
                if curr_dist >= dist_cap: continue
                for nxt in adj[curr]:
                    if nxt not in distances[start_node]:
                        distances[start_node][nxt] = curr_dist + 1
                        queue.append(nxt)
                        
    return adj, edges, distances

def build_model(N):
    model = cp_model.CpModel()
    print(f"[{time.strftime('%H:%M:%S')}] Building {N}x{N} {BRICK_LENGTH}-Edge Brick Model...")
    
    adj, edges, distances = compute_distances_and_edges(N)
    
    grid = {}
    b_is_z = {}
    # NEW: Variable representing the maximum value used in the entire grid
    #max_val = model.NewIntVar(1, MAX_COL_NUMBER, 'max_val')
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')
            # Constraint: max_val must be >= every cell in the grid
            #model.Add(max_val >= grid[r, c])
            z_vars = []
            for z in range(1, MAX_COL_NUMBER + 1):
                b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                b_is_z[r, c, z] = b
                z_vars.append(b)
                model.Add(grid[r, c] == z).OnlyEnforceIf(b)
            model.AddExactlyOne(z_vars)

    if PACKING:
        print(" > Applying Packing Cliques...")
        for z in range(1, MAX_COL_NUMBER + 1):
            k = max(1, z // 2) # Ensures 01s don't touch
            if z % 2 == 0:
                for r in range(N):
                    for c in range(N):
                        center = (r, c)
                        clique = [b_is_z[node[0], node[1], z] for node, dist in distances[center].items() if dist <= k]
                        if len(clique) > 1: model.AddAtMostOne(clique)
            else:
                for u, v in edges:
                    nodes_in_reach = set()
                    for node, dist in distances[u].items():
                        if dist <= k: nodes_in_reach.add(node)
                    for node, dist in distances[v].items():
                        if dist <= k: nodes_in_reach.add(node)
                    clique = [b_is_z[nr, nc, z] for nr, nc in nodes_in_reach]
                    if len(clique) > 1: model.AddAtMostOne(clique)

    print(" > Applying Local Rules...")
    for u, u_neighbors in adj.items():
        curr_val = grid[u]
        neighbor_vals = [grid[n] for n in u_neighbors]

        for n_val in neighbor_vals:
            if DOUBLES: model.Add(curr_val != 2 * n_val)
            if IDENTICAL_NEIGHBORS: model.Add(curr_val != n_val)

        if STAIRS:
            for i in range(len(neighbor_vals)):
                for j in range(i + 1, len(neighbor_vals)):
                    model.Add(2 * curr_val != neighbor_vals[i] + neighbor_vals[j])
                    
        if SANDWICHES and len(neighbor_vals) > 1:
            model.AddAllDifferent(neighbor_vals)

    # OBJECTIVE: Minimize the maximum number used
    #model.Minimize(max_val)
    
    #return model, grid, max_val
    return model, grid

def solve():
    if GRID_SIZE % BRICK_LENGTH != 0:
        print("ERROR: GRID_SIZE must be a multiple of BRICK_LENGTH.")
        return

    #model, grid, max_val = build_model(GRID_SIZE)
    model, grid = build_model(GRID_SIZE)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = NUM_SEARCH_WORKERS
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solver.parameters.max_time_in_seconds = MAX_TIME_IN_MINUTES * 60.0
    solver.parameters.log_search_progress = LOG_SEARCH

    print("\nSolving...")
    start_time = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - start_time

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"\nSUCCESS: Solution Found for {GRID_SIZE}x{GRID_SIZE} {BRICK_LENGTH}-Edge Brick Grid in {elapsed:.2f}s!")
        #print(f"\nMax number used: {solver.Value(max_val)}")
        for r in range(GRID_SIZE):
            # Print the horizontal row
            row_str = ""
            for c in range(GRID_SIZE):
                val = solver.Value(grid[r, c])
                row_str += f"{val:02d}"
                if c < GRID_SIZE - 1:
                    row_str += "---"
            print(row_str)
            
            # Print the vertical connections between rows
            if r < GRID_SIZE - 1:
                vert_str = ""
                for c in range(GRID_SIZE):
                    if r % 2 == 0:
                        is_connection = (c % BRICK_LENGTH == 0)
                    else:
                        is_connection = (c % BRICK_LENGTH == BRICK_LENGTH // 2)

                    if is_connection:
                        vert_str += "|    "
                    else:
                        vert_str += "     "
                print(vert_str)
    else:
        print("\nNo solution found.")

if __name__ == "__main__":
    solve()
