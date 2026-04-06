from ortools.sat.python import cp_model
import time
from collections import deque

# --- CONFIGURATION ---
GRID_SIZE = 8         # Total size of the grid (N x N)
MAX_COL_NUMBER = 7    # Max value for the variables

# --- PUNCH CONFIGURATION ---
PUNCH_SIZE = 7         # The size of the hole (M x M)
GAP_SIZE = 1           # The spacing between holes
# The pattern will repeat every (PUNCH_SIZE + GAP_SIZE) units.

# ENFORCE RESTRICTIONS
PACKING = True
DOUBLES = True
SANDWICHES = True
STAIRS = True
IDENTICAL_NEIGHBORS = True 

# SEARCH CONFIG
NUM_SEARCH_WORKERS = 8
LOG_SEARCH = True

def is_valid_vertex(r, c):
    """
    Generalized punch logic:
    If the coordinate falls within the M x M 'punch' zone 
    of the repeating block, it returns False.
    """
    period = PUNCH_SIZE + GAP_SIZE
    if (r % period < PUNCH_SIZE) and (c % period < PUNCH_SIZE):
        return False
    return True

def compute_distances_and_edges(N):
    valid_nodes = [(r, c) for r in range(N) for c in range(N) if is_valid_vertex(r, c)]
    adj = {node: [] for node in valid_nodes}
    
    for r, c in valid_nodes:
        # Standard 4-way neighbors with wrapping
        potentials = [
            (r, (c + 1) % N), (r, (c - 1) % N),
            ((r + 1) % N, c), ((r - 1) % N, c)
        ]
        
        for nr, nc in potentials:
            if is_valid_vertex(nr, nc):
                neighbor = (nr, nc)
                if neighbor not in adj[(r, c)]:
                    adj[(r, c)].append(neighbor)

    edges = []
    seen_edges = set()
    for u, neighbors in adj.items():
        for v in neighbors:
            edge = tuple(sorted((u, v)))
            if edge not in seen_edges:
                edges.append(edge)
                seen_edges.add(edge)

    # BFS for Shortest Paths
    distances = {}
    for start_node in valid_nodes:
        distances[start_node] = {start_node: 0}
        queue = deque([start_node])
        while queue:
            curr = queue.popleft()
            curr_dist = distances[start_node][curr]
            if curr_dist >= MAX_COL_NUMBER: continue
            for nxt in adj[curr]:
                if nxt not in distances[start_node]:
                    distances[start_node][nxt] = curr_dist + 1
                    queue.append(nxt)
                        
    return adj, edges, distances

def build_model(N):
    model = cp_model.CpModel()
    print(f"[{time.strftime('%H:%M:%S')}] Building {N}x{N} Grid ({PUNCH_SIZE}x{PUNCH_SIZE} punches)...")
    
    adj, edges, distances = compute_distances_and_edges(N)
    grid = { (r, c): model.NewIntVar(1, MAX_COL_NUMBER, f'g_{r}_{c}') for r, c in adj.keys() }

    # is_val[u, z] is True if grid[u] == z, False otherwise.
    is_val = {}
    for u in grid.keys():
        for z in range(1, MAX_COL_NUMBER + 1):
            is_val[u, z] = model.NewBoolVar(f'is_{u}_{z}')
            # Link the boolean to the integer variable
            model.Add(grid[u] == z).OnlyEnforceIf(is_val[u, z])
            model.Add(grid[u] != z).OnlyEnforceIf(is_val[u, z].Not())

    # 1. PACKING
    if PACKING:
        for z in range(1, MAX_COL_NUMBER + 1):
            for u, targets in distances.items():
                for v, dist in targets.items():
                    if 0 < dist <= z and u < v:
                        # Enforce that u and v cannot BOTH be z using boolean logic.
                        model.AddAtMostOne([is_val[u, z], is_val[v, z]])

    # 2. TABLES
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

    # 3. CONSTRAINTS
    for u, v in edges:
        model.AddForbiddenAssignments([grid[u], grid[v]], forbidden_pairs)

    for u, neighbors in adj.items():
        if SANDWICHES and len(neighbors) > 1:
            model.AddAllDifferent([grid[n] for n in neighbors])
        if STAIRS and len(neighbors) > 1:
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    model.AddForbiddenAssignments(
                        [grid[neighbors[i]], grid[u], grid[neighbors[j]]],
                        forbidden_triplets
                    )

    model.AddDecisionStrategy(list(grid.values()), cp_model.CHOOSE_FIRST, cp_model.SELECT_MIN_VALUE)
    return model, grid

def solve():
    model, grid = build_model(GRID_SIZE)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 8
    solver.parameters.log_search_progress = LOG_SEARCH

    print("\nSolving...")
    start_time = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - start_time

    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        print(f"\nSolution Found for {PUNCH_SIZE}x{PUNCH_SIZE} blocks in {elapsed:.2f}s:\n")
        for r in range(GRID_SIZE):
            row_str = ""
            for c in range(GRID_SIZE):
                if (r, c) in grid:
                    row_str += f"{solver.Value(grid[r, c]):02d}"
                else:
                    row_str += ".." # Visual representation of the hole
                
                # Simple spacing
                row_str += " "
            print(row_str)
    else:
        print("No solution.")

if __name__ == "__main__":
    solve()
