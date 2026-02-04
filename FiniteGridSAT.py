from ortools.sat.python import cp_model

# --- CONFIGURATION ---
GRID_SIZE = 9  # Reduced for faster testing (Change back to 50 if desired)
MAX_MAX_COL_NUMBER=37

N = GRID_SIZE
MIN_COL_NUMBER =28

def solve_finite_grid(MAX_COL_NUMBER):
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading Finite Grid constraints for {N}x{N} with {MAX_COL_NUMBER} Colors")
    
    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Finite Exclusion Zone)
    # ---------------------------------------------------------
    for z in range(1, MAX_COL_NUMBER + 1):
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
            # Apply to ALL pairs of valid neighbors
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
    solver.parameters.max_time_in_seconds = 100.0
    solver.parameters.log_search_progress = False #For anyone analyzing this code they can switch to true to watch progress

    print("Solving...")
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"SUCCESS: Solution Found for {N}x{N} Finite Grid!")
        
        final_grid = []
        for r in range(N):
            row = [solver.Value(grid[r, c]) for c in range(N)]
            final_grid.append(row)
            print(row)
            
        verify_finite_solution(final_grid, N)
        return True
    else:
        print("No solution found.")
        return False

def verify_finite_solution(grid, N):
    print("\n--- Running Finite Verification ---")
    
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
                                return print_fail(r, c, f"Packing Failed: Found another {val} at [{nr}][{nc}]")

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
                    if diff == 0: return print_fail(r, c, "Vertex Failed: Equal to neighbor")
                    if diff == val: return print_fail(r, c, "Vertex Failed: Diff equals self")
                    if diff == n_val: return print_fail(r, c, "Vertex Failed: Diff equals neighbor")

            # 3. Incident Edge Uniqueness
            if len(set(incident_edges)) != len(incident_edges):
                return print_fail(r, c, f"Incident Edge Collision: {incident_edges}")

    print("VERIFICATION SUCCESS: Finite grid is valid.")
    return True

def print_fail(r, c, message):
    print(f"VERIFICATION FAILED at [{r}][{c}]: {message}")
    return False

if __name__ == "__main__":
    solved=True
    for i in range(MAX_MAX_COL_NUMBER, MIN_COL_NUMBER,-1):
        if(solved==True): 
            if not solve_finite_grid(i):
                solved=False