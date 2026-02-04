from ortools.sat.python import cp_model

# --- CONFIGURATION ---
MIN_WIDTH = 12        # Start checking at this width
MAX_WIDTH = 128        # End checking at this width (inclusive)
MAX_COLOR = 11        # The available palette 
HEIGHT = 2            # Fixed height for the ladder

def solve_ladder_cylinder(width, max_color):
    """
    Solves the T-coloring problem for a 2 x Width grid.
    Topology: Cylinder (Wraps horizontally, hard edges vertically).
    """
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(HEIGHT):
        for c in range(width):
            grid[r, c] = model.NewIntVar(1, max_color, f'cell_{r}_{c}')

    # Helper: Handle Cylinder Topology
    def get_var(r, c):
        if 0 <= r < HEIGHT:
            return grid[r, c % width] # Horizontal Wrap
        return None # Vertical Hard Edge

    # 2. APPLY CONSTRAINTS
    for r in range(HEIGHT):
        for c in range(width):
            current = grid[r, c]
            
            # --- A. PACKING CONSTRAINTS (Exclusion Zone) ---
            for z in range(1, max_color + 1):
                # SPECIAL RULE RE-ADDED: 
                # If z=1, exclusion distance is 2. Otherwise, exclusion is z.
                exclusion_dist = 2 if z == 1 else z
                
                # Switch: True if this cell == z
                is_z = model.NewBoolVar(f'is_val_{z}_{r}_{c}')
                model.Add(current == z).OnlyEnforceIf(is_z)
                model.Add(current != z).OnlyEnforceIf(is_z.Not())

                # Enforce exclusion zone
                for dy in range(-exclusion_dist, exclusion_dist + 1):
                    for dx in range(-exclusion_dist, exclusion_dist + 1):
                        if dy == 0 and dx == 0: continue
                        
                        if abs(dy) + abs(dx) <= exclusion_dist:
                            neighbor = get_var(r + dy, c + dx)
                            
                            # Explicit check for None (Vertical edges)
                            if neighbor is not None:
                                model.Add(neighbor != z).OnlyEnforceIf(is_z)

            # --- B. ADJACENCY & ARITHMETIC RULES ---
            potential_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            neighbors = []
            for dy, dx in potential_offsets:
                n_var = get_var(r + dy, c + dx)
                if n_var is not None:
                    neighbors.append(n_var)

            for n in neighbors:
                # Rule 1: Neighbors cannot be equal
                model.Add(n != current)
                
                # Rule 2: No Doubling (Neighbor != 2*Current)
                model.Add(n != 2 * current)
                model.Add(current != 2 * n)

            # Rule 3: Arithmetic Progression Prevention
            # (A, B, C) -> 2*B != A + C
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    model.Add(2 * current != neighbors[i] + neighbors[j])

    # 3. SOLVE
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 30.0 
    
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        return "SOLVED", solver, grid
    else:
        return "FAILED", None, None
    
def main():
    print(f"--- STARTING SEARCH ---")
    print(f"Height: {HEIGHT} (Fixed)")
    print(f"Width Range: {MIN_WIDTH} - {MAX_WIDTH}")
    print(f"Max Color: {MAX_COLOR}")
    print("-" * 30)

    # Loop through the requested widths
    for w in range(MIN_WIDTH, MAX_WIDTH + 1):
        print(f"Testing Width {w}...", end=" ", flush=True)
        
        result_status, solver, grid = solve_ladder_cylinder(w, MAX_COLOR)
        
        if result_status == "SOLVED":
            print(f"SUCCESS!")
            print(f"\n--- SOLUTION FOUND (Width {w}) ---")
            
            # Print the full grid for the successful width
            for r in range(HEIGHT):
                # Extract the row values
                row_values = [solver.Value(grid[r, c]) for c in range(w)]
                print(row_values)
            
            print("\nStopping search as requested.")
            break  # <--- This stops the loop immediately!
            
        else:
            print(f"NO SOLUTION (or Timeout)")

if __name__ == "__main__":
    main()

