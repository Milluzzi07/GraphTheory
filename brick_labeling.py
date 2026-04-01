from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 24         # Must be a multiple of BRICK_LENGTH
MAX_COL_NUMBER = 14    # The ceiling for our search # 15 works for 24x24 grid, brick length 6, tok ~15 min

BRICK_LENGTH = 6  # (ideally even) length of bricks

FORCE_0_0_TO_1 = False  # Forces (0, 0) to be 1 to break symmetry. Speeds up pre-solve and solve. Good for large grids and you just want any solution.

# ENFORCE RESTRICTIONS
PACKING = True
DOUBLES = True
SANDWICHES = True
STAIRS = True
IDENTICAL_NEIGHBORS = True 

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 8
MAX_MEMORY_IN_MB = 4000
MAX_TIME_IN_MINUTES = 60 * 12
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
    dist_cap = MAX_COL_NUMBER // 2
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
    
    grid = { (r, c): model.NewIntVar(1, MAX_COL_NUMBER, f'g_{r}_{c}') for r in range(N) for c in range(N) }

    # --- 1. PACKING INDICATORS (Crucial for Clique Model) ---
    # is_val[(r,c), z] is true if grid[r,c] == z
    is_val = {}
    for (r, c) in grid.keys():
        for z in range(1, MAX_COL_NUMBER + 1):
            is_val[(r, c), z] = model.NewBoolVar(f'is_{r}_{c}_{z}')
            model.Add(grid[r, c] == z).OnlyEnforceIf(is_val[(r, c), z])
            model.Add(grid[r, c] != z).OnlyEnforceIf(is_val[(r, c), z].Not())

    # --- 2. BALL-BASED PACKING (Maximal Cliques) ---
    if PACKING:
        print("  > Constructing Geometric Cliques...")
        for z in range(1, MAX_COL_NUMBER + 1):
            k = z // 2
            
            if z % 2 == 0:
                # Even z: Any two nodes in a ball of radius k are at distance <= 2k
                for center, targets in distances.items():
                    clique = [is_val[node, z] for node, d in targets.items() if d <= k]
                    if len(clique) > 1:
                        model.AddAtMostOne(clique)
            else:
                # Odd z: Any two nodes in the union of balls of radius k 
                # around an edge (u, v) are at distance <= 2k + 1
                for u, v in edges:
                    clique_nodes = set()
                    for node, d in distances[u].items():
                        if d <= k: clique_nodes.add(node)
                    for node, d in distances[v].items():
                        if d <= k: clique_nodes.add(node)
                    
                    clique = [is_val[node, z] for node in clique_nodes]
                    if len(clique) > 1:
                        model.AddAtMostOne(clique)

    print(" > Applying Local Rules...")
    # --- 3. PRE-CALCULATE TABLES ---
    forbidden_pairs = [(x, x) for x in range(1, MAX_COL_NUMBER + 1)] if IDENTICAL_NEIGHBORS else []
    if DOUBLES:
        for x in range(1, MAX_COL_NUMBER + 1):
            if 2 * x <= MAX_COL_NUMBER:
                forbidden_pairs.append((x, 2 * x))
                forbidden_pairs.append((2 * x, x))

    forbidden_triplets = []
    if STAIRS:
        for x in range(1, MAX_COL_NUMBER + 1):
            for y in range(x + 1, MAX_COL_NUMBER + 1):
                z = 2 * y - x
                if z <= MAX_COL_NUMBER:
                    forbidden_triplets.append((x, y, z))
                    forbidden_triplets.append((z, y, x))

    # --- 4. APPLY EDGE RULES ---
    for u, v in edges:
        model.AddForbiddenAssignments([grid[u], grid[v]], forbidden_pairs)

    # --- 5. APPLY NODE-CENTRIC RULES ---
    for u, u_neighbors in adj.items():
        neighbor_nodes = list(u_neighbors)
        neighbor_vars = [grid[n] for n in u_neighbors]

        if STAIRS and len(neighbor_nodes) > 1:
            for i in range(len(neighbor_nodes)):
                for j in range(i + 1, len(neighbor_nodes)):
                    model.AddForbiddenAssignments(
                        [grid[neighbor_nodes[i]], grid[u], grid[neighbor_nodes[j]]],
                        forbidden_triplets
                    )

        if SANDWICHES and len(neighbor_vars) > 1:
            model.AddAllDifferent(neighbor_vars)

    # --- MANUAL SYMMETRY BREAKING ---
    if FORCE_0_0_TO_1:
        model.Add(grid[0, 0] == 1)

    # --- 6. SEARCH STRATEGY ---
    # This helps the solver decide which cells to fill first
    all_vars = [grid[r, c] for r in range(N) for c in range(N)]
    #model.AddDecisionStrategy(all_vars, cp_model.CHOOSE_FIRST, cp_model.SELECT_MIN_VALUE)
    model.AddDecisionStrategy(
        all_vars, 
        cp_model.CHOOSE_MIN_DOMAIN_SIZE, 
        cp_model.SELECT_MIN_VALUE
    )

    return model, grid

def solve():
    if GRID_SIZE % BRICK_LENGTH != 0:
        print("ERROR: GRID_SIZE must be a multiple of BRICK_LENGTH.")
        return

    model, grid = build_model(GRID_SIZE)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = NUM_SEARCH_WORKERS
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solver.parameters.max_time_in_seconds = MAX_TIME_IN_MINUTES * 60.0
    solver.parameters.log_search_progress = LOG_SEARCH
    
    # 1. Use a fixed search strategy instead of the default automated portfolio
    #solver.parameters.search_branching = cp_model.FIXED_SEARCH

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
