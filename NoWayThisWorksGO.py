from ortools.sat.python import cp_model
import time

# --- CONFIGURATION ---
GRID_SIZE = 20  # Set to 50 for your full run
MAX_COL_NUMBER = GRID_SIZE - 1 
N = GRID_SIZE

def solve_infinite_optimized():
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading Toroidal constraints for {N}x{N}...")
    start_time = time.time()

    # --- PRE-COMPUTATION (The Speed Fix) ---
    # We calculate the relative offsets for every distance 'z' ONCE.
    # offsets_by_z[z] = set of (dr, dc)
    print("Pre-computing diamond shapes...", end=" ")
    offsets_by_z = {}
    
    for z in range(1, MAX_COL_NUMBER + 1):
        unique_offsets = set()
        for dy in range(-z, z + 1):
            remaining = z - abs(dy)
            for dx in range(-remaining, remaining + 1):
                if dy == 0 and dx == 0: continue
                # We store just the offsets. We handle modulo wrapping later.
                unique_offsets.add((dy, dx))
        offsets_by_z[z] = unique_offsets
    print("Done.")

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Optimized)
    # ---------------------------------------------------------
    # We loop Z first to minimize context switching
    for z in range(1, MAX_COL_NUMBER + 1):
        if z % 5 == 0: # Progress bar
            print(f"  - Generating constraints for Color {z}/{MAX_COL_NUMBER}...")

        diamond_offsets = offsets_by_z[z]
        
        for r in range(N):
            for c in range(N):
                # 1. Create the Switch: Is grid[r,c] == z?
                b_is_z = model.NewBoolVar(f'is_{z}_{r}_{c}')
                model.Add(grid[r, c] == z).OnlyEnforceIf(b_is_z)
                model.Add(grid[r, c] != z).OnlyEnforceIf(b_is_z.Not())

                # 2. Apply Exclusion Zone
                current_index = r * N + c # Unique ID for the current cell
                
                for dy, dx in diamond_offsets:
                    # Toroidal Wrap
                    nr = (r + dy) % N
                    nc = (c + dx) % N
                    neighbor_index = nr * N + nc

                    # SYMMETRY OPTIMIZATION:
                    # Only add the constraint if the neighbor's ID is greater.
                    # Logic: If A forbids B, B forbids A. We only need to say it once.
                    # This cuts 'model.Add' calls by 50%.
                    if neighbor_index > current_index:
                        model.Add(grid[nr, nc] != z).OnlyEnforceIf(b_is_z)

    # ---------------------------------------------------------
    # B. VERTEX ADJACENCY RULES (Toroidal)
    # ---------------------------------------------------------
    print("  - Generating Adjacency Rules...")
    # Offsets: Up, Down, Left, Right
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(N):
        for c in range(N):
            current = grid[r, c]
            
            # Collect Neighbors (All valid due to torus)
            neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = (r + dy) % N, (c + dx) % N
                neighbors.append(grid[nr, nc])

            for neighbor in neighbors:
                # Rule 1: No Doubling
                model.Add(neighbor != 2 * current)
                # Note: We don't check 'current != 2 * neighbor' here because 
                # that constraint will be added when we iterate to 'neighbor'.
                
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
    solver.parameters.num_search_workers = 0 
    solver.parameters.max_memory_in_mb = 16000
    solver.parameters.max_time_in_seconds = 28800.0
    solver.parameters.log_search_progress = True

    print("Solving...")
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"SUCCESS: Solution Found for {N}x{N} Toroidal Grid!")
        for r in range(N):
            row = [solver.Value(grid[r, c]) for c in range(N)]
            print(row)
    else:
        print("No solution found.")

if __name__ == "__main__":
    solve_infinite_optimized()