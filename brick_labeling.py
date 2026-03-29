from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 24         # Must be a multiple of BRICK_LENGTH
MAX_COL_NUMBER = 18    # The ceiling for our search # 18 works for 24x24 grid, brick length 6

BRICK_LENGTH = 6  # (ideally even) length of bricks

# ENFORCE RESTRICTIONS
PACKING = True
DOUBLES = True
SANDWICHES = True
STAIRS = True
IDENTICAL_NEIGHBORS = True 

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 8
MAX_MEMORY_IN_MB = 4000
MAX_TIME_IN_MINUTES = 600
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
    dist_cap = MAX_COL_NUMBER
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
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    # --- 1. PACKING RESTRICTIONS ---
    if PACKING:
        print(" > Applying Packing (Distance = Z)...")
        for z in range(1, MAX_COL_NUMBER + 1):
            k = z
            
            for u, targets in distances.items():
                for v, dist in targets.items():
                    if 0 < dist <= k and u < v:
                        model.AddForbiddenAssignments([grid[u], grid[v]], [(z, z)])

    print(" > Applying Local Rules...")
    # --- 2. PRE-CALCULATE TABLES ---
    forbidden_pairs = []
    for x in range(1, MAX_COL_NUMBER + 1):
        for y in range(1, MAX_COL_NUMBER + 1):
            if (IDENTICAL_NEIGHBORS and x == y) or (DOUBLES and (x == 2 * y or y == 2 * x)):
                forbidden_pairs.append((x, y))

    forbidden_triplets = []
    if STAIRS:
        for x in range(1, MAX_COL_NUMBER + 1):
            for y in range(1, MAX_COL_NUMBER + 1):
                for z in range(1, MAX_COL_NUMBER + 1):
                    if 2 * x == y + z:
                        forbidden_triplets.append((x, y, z))

    # --- 3. APPLY EDGE RULES ---
    for u, v in edges:
        model.AddForbiddenAssignments([grid[u], grid[v]], forbidden_pairs)

    # --- 4. APPLY NODE-CENTRIC RULES ---
    for u, u_neighbors in adj.items():
        neighbor_nodes = list(u_neighbors)
        neighbor_vars = [grid[n] for n in u_neighbors]

        if STAIRS and len(neighbor_nodes) > 1:
            for i in range(len(neighbor_nodes)):
                for j in range(i + 1, len(neighbor_nodes)):
                    model.AddForbiddenAssignments(
                        [grid[u], grid[neighbor_nodes[i]], grid[neighbor_nodes[j]]],
                        forbidden_triplets
                    )

        if SANDWICHES and len(neighbor_vars) > 1:
            model.AddAllDifferent(neighbor_vars)

    # --- 5. SEARCH STRATEGY ---
    # This helps the solver decide which cells to fill first
    all_vars = [grid[r, c] for r in range(N) for c in range(N)]
    model.AddDecisionStrategy(all_vars, cp_model.CHOOSE_FIRST, cp_model.SELECT_MIN_VALUE)

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
    
    # 1. Use a fixed search strategy instead of the default automated portfolio
    solver.parameters.search_branching = cp_model.FIXED_SEARCH 

    # 2. Increase the level of "presolve" to catch contradictions early
    solver.parameters.cp_model_presolve = True

    # 3. Tell the solver to stop as soon as it finds the FIRST valid solution
    # (This is the default for solver.Solve, but this parameter helps the search heuristics)
    solver.parameters.stop_after_first_solution = True

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
