from ortools.sat.python import cp_model
import time  # Added to track duration

# --- CONFIGURATION ---
GRID_SIZE = 16  # Reduced for faster testing (Change back to 50 if desired)
MAX_MAX_COL_NUMBER = 88
N = GRID_SIZE
MIN_COL_NUMBER = 78
MAX_TIME=250.0

def solve_finite_grid(MAX_COL_NUMBER):
    print(f"\n[{time.strftime('%H:%M:%S')}] Starting setup for Max Colors: {MAX_COL_NUMBER}")
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    print(f"  > Creating variables for {N}x{N} grid...")
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Finite Exclusion Zone)
    # ---------------------------------------------------------
    print("  > Building Packing Constraints (Manhattan Exclusion)...")
    for z in range(1, MAX_COL_NUMBER + 1):
        # Progress update every 10 colors to show activity
        if z % 10 == 0 or z == MAX_COL_NUMBER:
            print(f"    ... processing color {z}/{MAX_COL_NUMBER}")
            
        for r in range(N):
            for c in range(N):
                # Switch variable: Is grid[r,c] == z?
                b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                model.Add(grid[r, c] == z).OnlyEnforceIf(b)
                model.Add(grid[r, c] != z).OnlyEnforceIf(b.Not())

                # If 'b' is True, forbid 'z' in the exclusion zone
                for dy in range(-z, z + 1):
                    for dx in range(-z, z + 1):
                        if dy == 0 and dx == 0: continue
                        
                        # Manhattan check
                        if abs(dy) + abs(dx) <= z:
                            nr, nc = r + dy, c + dx
                            
                            # BOUNDARY CHECK: Only apply if neighbor is inside grid
                            if 0 <= nr < N and 0 <= nc < N:
                                model.Add(grid[nr, nc] != z).OnlyEnforceIf(b)

    # ---------------------------------------------------------
    # B. SPECIAL RULE FOR 1s (Distance 2)
    # ---------------------------------------------------------
    print("  > Building Special Rule for 1s (Distance 2)...")
    for r in range(N):
        for c in range(N):
            is_one = model.NewBoolVar(f'is_one_{r}_{c}')
            model.Add(grid[r, c] == 1).OnlyEnforceIf(is_one)
            model.Add(grid[r, c] != 1).OnlyEnforceIf(is_one.Not())
            
            # Check Distance 2 Diamond
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    if dy == 0 and dx == 0: continue
                    if abs(dy) + abs(dx) <= 2:
                        nr, nc = r + dy, c + dx
                        
                        # BOUNDARY CHECK
                        if 0 <= nr < N and 0 <= nc < N:
                            model.Add(grid[nr, nc] != 1).OnlyEnforceIf(is_one)

    # ---------------------------------------------------------
    # C. VERTEX ADJACENCY RULES
    # ---------------------------------------------------------
    print("  > Building Vertex Adjacency & Arithmetic Rules...")
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(N):
        for c in range(N):
            current = grid[r, c]
            
            # 1. Collect Valid Neighbors (Handle Corners/Edges)
            valid_neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = r + dy, c + dx
                if 0 <= nr < N and 0 <= nc < N:
                    valid_neighbors.append(grid[nr, nc])

            # 2. Apply Rules to Valid Neighbors
            for neighbor in valid_neighbors:
                # Rule 1: No Doubling
                model.Add(neighbor != 2 * current)
                model.Add(current != 2 * neighbor)
                
                # Rule 2: Basic Inequality
                model.Add(current != neighbor)

            # Rule 3: Arithmetic Progression Prevention
            # "2 * Current != Neighbor1 + Neighbor2"
            for i in range(len(valid_neighbors)):
                for j in range(i + 1, len(valid_neighbors)):
                    n1 = valid_neighbors[i]
                    n2 = valid_neighbors[j]
                    model.Add(2 * current != n1 + n2)

    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 0 
    solver.parameters.random_seed = 42
    solver.parameters.max_time_in_seconds = MAX_TIME
    solver.parameters.log_search_progress = False 

    print(f"  > Model built. Starting CP-SAT Solver (Time limit: {MAX_TIME}s)...")
    start_time = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - start_time

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"  >>> SUCCESS: Solution Found in {elapsed:.2f} seconds!")
        
        final_grid = []
        for r in range(N):
            row = [solver.Value(grid[r, c]) for c in range(N)]
            final_grid.append(row)
            print(f"      {row}")
            
        verify_finite_solution(final_grid, N)
        return True
    else:
        print(f"  >>> FAILURE: No solution found within constraints (Time: {elapsed:.2f}s).")
        return False

def verify_finite_solution(grid, N):
    print("    Running Independent Verification...", end=" ")
    
    for r in range(N):
        for c in range(N):
            val = grid[r][c]

            # 1. Finite Packing Test
            for dy in range(-val, val + 1):
                for dx in range(-val, val + 1):
                    if dy == 0 and dx == 0: continue
                    
                    if abs(dy) + abs(dx) <= val:
                        nr, nc = r + dy, c + dx
                        # BOUNDARY CHECK
                        if 0 <= nr < N and 0 <= nc < N:
                            if grid[nr][nc] == val:
                                print(f"\n    [Verification Failed] Packing error at [{r}][{c}] vs [{nr}][{nc}] (Value: {val})")
                                return False

            # 2. Collect Finite Incident Edges
            incident_edges = []
            neighbor_vals = []
            
            # Check Up/Down/Left/Right
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dy, c + dx
                if 0 <= nr < N and 0 <= nc < N:
                    n_val = grid[nr][nc]
                    neighbor_vals.append(n_val)
                    
                    # Calculate Edge Difference
                    diff = abs(val - n_val)
                    incident_edges.append(diff)
                    
                    # Vertex Rules
                    if diff == 0: 
                        print(f"\n    [Verification Failed] Neighbor equal at [{r}][{c}]")
                        return False
                    if diff == val: 
                        print(f"\n    [Verification Failed] Diff equals self at [{r}][{c}]")
                        return False
                    if diff == n_val: 
                        print(f"\n    [Verification Failed] Diff equals neighbor at [{r}][{c}]")
                        return False

            # 3. Incident Edge Uniqueness
            if len(set(incident_edges)) != len(incident_edges):
                print(f"\n    [Verification Failed] Edge collision at [{r}][{c}]: {incident_edges}")
                return False

    print("PASSED.")
    return True

if __name__ == "__main__":
    print("=== Finite Grid SAT Solver ===")
    print(f"Grid Size: {GRID_SIZE}x{GRID_SIZE}")
    print(f"Testing Max Colors from {MAX_MAX_COL_NUMBER} down to {MIN_COL_NUMBER}")
    
    solved = True
    for i in range(MAX_MAX_COL_NUMBER, MIN_COL_NUMBER, -1):
        # Pass the current Max Color to the solver
        if not solve_finite_grid(i):
            print(f"\n!!! Stopping: Could not solve for {i} colors. (Previous {i+1} was likely the minimum) !!!")
            solved = False
            break