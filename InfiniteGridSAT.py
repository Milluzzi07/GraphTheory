from ortools.sat.python import cp_model

# --- CONFIGURATION ---
# The size of the N x N grid (50x50 = 2,500 variables).
GRID_SIZE = 100

# The maximum allowed value for any cell (Color Limit).
# We set this to N-1 because usually, these graphs require close to N colors.
MAX_COL_NUMBER = GRID_SIZE - 1 
N = GRID_SIZE

def solve_with_ortools():
    """
    Main function to define the CP-SAT model and solve the Graph Labeling problem.
    """
    model = cp_model.CpModel()
    
    # ---------------------------------------------------------
    # 1. CREATE VARIABLES
    # ---------------------------------------------------------
    # We create a grid of Integer Variables.
    # Each cell (r, c) can hold a value from 1 to MAX_COL_NUMBER.
    grid = {}
    for r in range(N):
        for c in range(N):
            # 'grid_r_c' is the internal name used by the solver for debugging.
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    # Helper function to handle Toroidal Wrapping (wrapping around edges).
    # If we ask for column -1, it returns column N-1.
    def get_var(r, c):
        return grid[r % N, c % N]

    print("Loading optimized constraints... (This uses minimal RAM!)")
    
    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (The "Exclusion Zone" Rule)
    # Rule: If a cell has value 'z', no other cell within Manhattan distance 'z'
    #       can also have the value 'z'.
    # ---------------------------------------------------------
    
    # We loop through every possible color 'z' to enforce its specific exclusion zone.
    for z in range(1, MAX_COL_NUMBER + 1):
        for r in range(N):
            for c in range(N):
                # 1. Create a "Switch" variable (b) that turns on if this cell == z
                b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                model.Add(grid[r, c] == z).OnlyEnforceIf(b)
                model.Add(grid[r, c] != z).OnlyEnforceIf(b.Not())

                # 2. If the switch 'b' is ON, forbid 'z' in the surrounding diamond
                for dy in range(-z, z + 1):
                    for dx in range(-z, z + 1):
                        if dy == 0 and dx == 0: continue # Don't check self
                        
                        # Manhattan Distance Check: |dy| + |dx| <= z
                        if abs(dy) + abs(dx) <= z:
                            # "get_var" handles the wrap-around (torus) automatically
                            neighbor = get_var(r + dy, c + dx)
                            
                            # CONSTRAINT: Neighbor cannot be 'z' if 'b' is True
                            model.Add(neighbor != z).OnlyEnforceIf(b)
    # "If there is a 1, there cannot be another 1 in distance 2"
    # We can handle this efficiently by finding all cells that COULD be 1.
    for r in range(N):
        for c in range(N):
            # Create a boolean: Is this cell a 1?
            is_one = model.NewBoolVar(f'is_one_{r}_{c}')
            model.Add(grid[r, c] == 1).OnlyEnforceIf(is_one)
            model.Add(grid[r, c] != 1).OnlyEnforceIf(is_one.Not())
            
            # If is_one is True, forbid 1s in the Distance 2 Diamond
            # Dist 2 offsets: (±1, ±1), (±2, 0), (0, ±2) + (±1, 0), (0, ±1) from Dist 1
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    if dy == 0 and dx == 0: continue
                    if abs(dy) + abs(dx) <= 2:
                        neighbor = get_var(r + dy, c + dx)
                        model.Add(neighbor != 1).OnlyEnforceIf(is_one)

   # --- C. VERTEX ADJACENCY RULES ---
    # We iterate over every cell and define its relationship with neighbors.
    # Offsets for immediate neighbors (Up, Down, Left, Right)
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(N):
        for c in range(N):
            current = grid[r, c]
            
            # Get list of neighbor variables
            neighbors = [get_var(r + dy, c + dx) for dy, dx in neighbor_offsets]

            for neighbor in neighbors:
                # Rule 1: No Doubling (If A adjacent to B, B != 2A)
                # Since adjacency is symmetric, we check both ways or just B != 2A for all edges
                model.Add(neighbor != 2 * current)
                model.Add(current != 2 * neighbor) # Symmetric check
                
                # Rule 2: Basic Vertex Rule (Neighbors cannot be equal)
                model.Add(current != neighbor)

            # Rule 3: The "a, a+b, a+2b" Rule (Arithmetic Progression Prevention)
            # Logic: If A -> B -> C, then C cannot be a+2b.
            # This simplifies to: 2*B != A + C
            # We must check this for ALL pairs of neighbors connected to 'current' (B).
            
            # We iterate through all unique pairs of neighbors around 'current'
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    n1 = neighbors[i]
                    n2 = neighbors[j]
                    
                    # Constraint: 2 * Current != Neighbor1 + Neighbor2
                    model.Add(2 * current != n1 + n2)

    # ---------------------------------------------------------
    # 3. SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    
    # Solver Parameters
    solver.parameters.num_search_workers = 0   # 0 = Use all available CPU cores
    solver.parameters.random_seed = 42         # Ensure reproducible results
    solver.parameters.max_time_in_seconds = 28800.0 # 8 Hours timeout
    solver.parameters.log_search_progress = True    # Print logs to console

    print("Solving...")
    status = solver.Solve(model)

    # Check Results
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"SUCCESS: OR-Tools found a valid graph!")
        print("Optimal:", cp_model.OPTIMAL)
        
        # Extract the solution into a standard Python list
        final_grid = []
        for r in range(N):
            row = [solver.Value(grid[r, c]) for c in range(N)]
            final_grid.append(row)
            print(row)
            
        # Double-check the math independently
        verify_solution(final_grid, N)
        
    else:
        print("No solution found by the solver.")

def verify_solution(grid, N):
    """
    Rigorously tests a completed grid to ensure it obeys all Graph Labeling constraints.
    Returns True if valid, raises an AssertionError with details if invalid.
    """
    print("\n--- Running Independent Verification ---")
    
def verify_solution(grid, N):
    """
    Rigorously tests a completed grid to ensure it obeys all Graph Labeling constraints.
    Returns True if valid, raises an AssertionError with details if invalid.
    """
    print("\n--- Running Independent Verification ---")
    
    for r in range(N):
        for c in range(N):
            val = grid[r][c]

           # 1. Toroidal Packing Test (Manhattan Exclusion Zone)
            # Check all cells within distance 'val'
            for dy in range(-val, val + 1):
                for dx in range(-val, val + 1):
                    if dy == 0 and dx == 0: continue
                    
                    if abs(dy) + abs(dx) <= val:
                        check_r = (r + dy) % N
                        check_c = (c + dx) % N
                        if grid[check_r][check_c] == val:
                            distance = abs(dy) + abs(dx)
                            return print_fail(r, c, f"Packing Failed: Found another {val} at Grid[{check_r}][{check_c}] (Distance: {distance})")
            # Calculate the 4 incident edge differences for this vertex (Toroidal)
            edge_right = abs(val - grid[r][(c + 1) % N])
            edge_down  = abs(val - grid[(r + 1) % N][c])
            edge_left  = abs(val - grid[r][(c - 1) % N])
            edge_up    = abs(val - grid[(r - 1) % N][c])
            
            incident_edges = [edge_right, edge_down, edge_left, edge_up]

            # 2. Vertex Test: Difference cannot be 0 or equal to the vertex
            if 0 in incident_edges: return print_fail(r, c, "Vertex Failed: Difference is 0")
            if val in incident_edges: return print_fail(r, c, "Vertex Failed: Difference equals vertex value")

            # Check neighbors to ensure difference doesn't equal their value either
            if edge_right == grid[r][(c + 1) % N]: return print_fail(r, c, "Vertex Failed: Equals right neighbor")
            if edge_down == grid[(r + 1) % N][c]:  return print_fail(r, c, "Vertex Failed: Equals down neighbor")

            # 3. Incident Edge Test: All 4 edges touching the vertex must be unique
            if len(set(incident_edges)) != 4:
                return print_fail(r, c, f"Incident Edge Collision: Edges are {incident_edges}")

    print("VERIFICATION SUCCESS: The grid is 100% mathematically valid.")
    return True

def print_fail(r, c, message):
    print(f"VERIFICATION FAILED at Grid[{r}][{c}]: {message}")
    return False
if __name__ == "__main__":
    solve_with_ortools()